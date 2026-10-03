// ============================================================================
//  drv_lampir.c  --  OpenBeken driver: NEC infrared transmitter
//
//  The 38 kHz carrier is made in software: a HW timer ticks every 13 us and the
//  ISR toggles a plain GPIO while an NEC "mark" is on. (OBK's hardware PWM path
//  puts nothing on the pin on this LN882H board, plain GPIO output works.)
//
//  Hardware: GPIO -> 1k -> NPN base, LED + resistor from 3V3 to the collector,
//  so pin HIGH = LED on. Leave the pin's OBK role as None.
//
//  Console:
//      LampIR_Setup <pin>
//      LampIR_Send  <addrHex> <cmdHex> [repeats]      e.g.  LampIR_Send 80 1D
//      LampIR_Carrier <ms>      steady 38 kHz burst (10..5000 ms): a multimeter on the
//                               pin reads ~half of 3.3 V, the LED glows steadily on a camera
//      LampIR_Status
//
//  'repeats' = NEC repeat frames after the first one (what a remote sends while
//  its key is held, used for brightness / speed steps), max 20.
//
//  ponytail: one send at a time, a new LampIR_Send while busy is rejected.
//            Add a queue when commands from HA start to collide.
//  ponytail: 13 us tick -> 38.46 kHz (+1.2 %). Change LIR_TICK_US if a receiver
//            turns out to be picky.
// ============================================================================

#include "../new_common.h"
#include "../logging/logging.h"
#include "../cmnds/cmd_public.h"
#include "../hal/hal_pins.h"
#include "../hal/hal_hwtimer.h"
#include "drv_local.h"

#define LIR_TICK_US      13         // half a carrier period
#define LIR_FRAME_US     108000     // NEC repeat period, start of frame to start of frame
#define LIR_MAX_REPEATS  20
#define LIR_MAX_ENTRIES  160        // 67 for a frame + 4 per repeat

static int8_t lir_timer = -1;
static int    lir_pin = -1;

// durations in timer ticks, alternating mark / space, index 0 = mark
static uint32_t lir_ticks[LIR_MAX_ENTRIES];
static int      lir_count;
static volatile int      lir_idx;
static volatile int      lir_level;
static volatile int      lir_busy;
static volatile uint32_t lir_left;
static volatile uint32_t lir_isr;      // ISR calls, for LampIR_Status

static void lir_add(uint32_t us) {
	if (lir_count >= LIR_MAX_ENTRIES)
		return;
	// marks are whole carrier cycles (2 ticks), so the pin always ends low
	lir_ticks[lir_count] = (lir_count & 1) ? (us + LIR_TICK_US / 2) / LIR_TICK_US
	                                       : (us + LIR_TICK_US) / (2 * LIR_TICK_US) * 2;
	lir_count++;
}

// NEC: 9 ms + 4.5 ms header, 32 bits LSB first (addr, ~addr, cmd, ~cmd),
// bit = 560 us mark + 560 (0) / 1690 (1) us space, 560 us stop mark.
static void lir_build(int addr, int cmd, int repeats) {
	uint32_t d = (addr & 0xFF) | ((~addr & 0xFF) << 8) |
	             ((cmd & 0xFF) << 16) | ((uint32_t)(~cmd & 0xFF) << 24);
	uint32_t t = 9000 + 4500 + 560;
	int i;

	lir_count = 0;
	lir_add(9000);
	lir_add(4500);
	for (i = 0; i < 32; i++) {
		uint32_t sp = ((d >> i) & 1) ? 1690 : 560;
		lir_add(560);
		lir_add(sp);
		t += 560 + sp;
	}
	lir_add(560);

	while (repeats-- > 0) {
		lir_add(LIR_FRAME_US - t);
		lir_add(9000);
		lir_add(2250);
		lir_add(560);
		t = 9000 + 2250 + 560;
	}
}

// Timer ISR: no logging, no allocation in here.
static void lir_tick(void *arg) {
	lir_isr++;
	if (lir_idx >= lir_count) {
		HAL_PIN_SetOutputValue(lir_pin, 0);
		HAL_HWTimerStop(lir_timer);
		lir_busy = 0;
		return;
	}
	lir_level = (lir_idx & 1) ? 0 : !lir_level;     // mark: carrier, space: low
	HAL_PIN_SetOutputValue(lir_pin, lir_level);
	if (--lir_left == 0 && ++lir_idx < lir_count)
		lir_left = lir_ticks[lir_idx];
}

static commandResult_t lir_start(void) {
	if (lir_pin < 0 || lir_timer < 0) {
		addLogAdv(LOG_ERROR, LOG_FEATURE_CMD, "LampIR: call LampIR_Setup <pin> first");
		return CMD_RES_ERROR;
	}
	if (lir_busy) {
		addLogAdv(LOG_WARN, LOG_FEATURE_CMD, "LampIR: busy, command dropped");
		return CMD_RES_ERROR;
	}
	return CMD_RES_OK;
}

static void lir_go(void) {
	lir_idx = 0;
	lir_left = lir_ticks[0];
	lir_level = 0;
	lir_busy = 1;
	HAL_HWTimerStart(lir_timer);
}

static commandResult_t CMD_LampIR_Setup(const void *context, const char *cmd,
                                        const char *args, int cmdFlags) {
	Tokenizer_TokenizeString(args, 0);
	if (Tokenizer_GetArgsCount() < 1)
		return CMD_RES_NOT_ENOUGH_ARGUMENTS;

	lir_pin = Tokenizer_GetArgInteger(0);
	HAL_PIN_Setup_Output(lir_pin);
	HAL_PIN_SetOutputValue(lir_pin, 0);

	addLogAdv(LOG_INFO, LOG_FEATURE_CMD, "LampIR: IR LED on pin %i", lir_pin);
	return CMD_RES_OK;
}

static commandResult_t CMD_LampIR_Send(const void *context, const char *cmd,
                                       const char *args, int cmdFlags) {
	int addr, code, repeats = 0;

	Tokenizer_TokenizeString(args, 0);
	if (Tokenizer_GetArgsCount() < 2)
		return CMD_RES_NOT_ENOUGH_ARGUMENTS;
	if (lir_start() != CMD_RES_OK)
		return CMD_RES_ERROR;

	addr = strtol(Tokenizer_GetArg(0), 0, 16);
	code = strtol(Tokenizer_GetArg(1), 0, 16);
	if (Tokenizer_GetArgsCount() > 2)
		repeats = Tokenizer_GetArgInteger(2);
	if (repeats < 0)
		repeats = 0;
	if (repeats > LIR_MAX_REPEATS)
		repeats = LIR_MAX_REPEATS;

	lir_build(addr, code, repeats);
	lir_go();

	addLogAdv(LOG_INFO, LOG_FEATURE_CMD, "LampIR: NEC addr 0x%X cmd 0x%X repeats %i",
	          addr & 0xFF, code & 0xFF, repeats);
	return CMD_RES_OK;
}

static commandResult_t CMD_LampIR_Carrier(const void *context, const char *cmd,
                                          const char *args, int cmdFlags) {
	int ms;

	Tokenizer_TokenizeString(args, 0);
	if (Tokenizer_GetArgsCount() < 1)
		return CMD_RES_NOT_ENOUGH_ARGUMENTS;
	if (lir_start() != CMD_RES_OK)
		return CMD_RES_ERROR;

	ms = Tokenizer_GetArgInteger(0);
	if (ms < 10)
		ms = 10;
	if (ms > 5000)
		ms = 5000;
	lir_count = 0;
	lir_add((uint32_t)ms * 1000);
	lir_go();

	addLogAdv(LOG_INFO, LOG_FEATURE_CMD, "LampIR: carrier for %i ms", ms);
	return CMD_RES_OK;
}

static commandResult_t CMD_LampIR_Status(const void *context, const char *cmd,
                                         const char *args, int cmdFlags) {
	addLogAdv(LOG_INFO, LOG_FEATURE_CMD,
	          "LampIR: pin %i, timer %i, busy %i, entry %i/%i, ISR calls %u",
	          lir_pin, (int)lir_timer, (int)lir_busy, (int)lir_idx, lir_count, (unsigned)lir_isr);
	return CMD_RES_OK;
}

void LampIR_Init(void) {
	float real = 0;

	lir_pin = -1;
	lir_busy = 0;
	if (lir_timer < 0)
		lir_timer = HAL_RequestHWTimer(LIR_TICK_US, &real, lir_tick, NULL);

	CMD_RegisterCommand("LampIR_Setup", CMD_LampIR_Setup, NULL);
	CMD_RegisterCommand("LampIR_Send",  CMD_LampIR_Send,  NULL);
	CMD_RegisterCommand("LampIR_Carrier", CMD_LampIR_Carrier, NULL);
	CMD_RegisterCommand("LampIR_Status", CMD_LampIR_Status, NULL);

	addLogAdv(LOG_INFO, LOG_FEATURE_CMD, "LampIR driver started, HW timer %i, tick %.1f us",
	          (int)lir_timer, real);
}

void LampIR_StopDriver(void) {
	if (lir_timer >= 0) {
		HAL_HWTimerStop(lir_timer);
		HAL_HWTimerDeinit(lir_timer);
		lir_timer = -1;
	}
	if (lir_pin >= 0)
		HAL_PIN_SetOutputValue(lir_pin, 0);
	lir_pin = -1;
	lir_busy = 0;
}
