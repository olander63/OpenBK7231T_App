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
//      LampIR_Code  <on|off|bright_up|bright_down|warm|cold|fan|fan_up|fan_down> [repeats]
//                                                     replay a frame recorded from the lamp's remote
//      LampIR_Raw   <us> <us> ...                     replay a recorded frame: mark, space,
//                                                     mark ... (signs ignored), <= 160 values;
//                                                     the web console cuts commands at ~127 chars
//      LampIR_Carrier <ms> [dc] steady 38 kHz burst (10..5000 ms): a multimeter on the
//                               pin reads ~half of 3.3 V, the LED glows steadily on a camera.
//                               With a 2nd argument the pin is held HIGH instead (up to 30 s):
//                               LED test without the carrier, measure the output stage
//                               (pin / collector / voltage across the LED resistor)
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
static volatile int      lir_dc;       // marks held high instead of 38 kHz (LED test)
static volatile int      lir_busy;
static volatile uint32_t lir_left;
static volatile uint32_t lir_isr;      // ISR calls, for LampIR_Status

static void lir_add(uint32_t us) {
	uint32_t t;

	if (lir_count >= LIR_MAX_ENTRIES)
		return;
	// marks are whole carrier cycles (2 ticks), so the pin always ends low
	t = (lir_count & 1) ? (us + LIR_TICK_US / 2) / LIR_TICK_US
	                    : (us + LIR_TICK_US) / (2 * LIR_TICK_US) * 2;
	lir_ticks[lir_count++] = t ? t : 2;       // a 0 would underflow the ISR countdown
}

// NEC repeat frames, sent while a remote key is held; t = length of the frame before.
static void lir_repeats(uint32_t t, int repeats) {
	while (repeats-- > 0) {
		lir_add(LIR_FRAME_US > t ? LIR_FRAME_US - t : 1);
		lir_add(9000);
		lir_add(2250);
		lir_add(560);
		t = 9000 + 2250 + 560;
	}
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
	lir_repeats(t, repeats);
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
	lir_level = (lir_idx & 1) ? 0 : (lir_dc || !lir_level);     // mark: carrier (or high), space: low
	HAL_PIN_SetOutputValue(lir_pin, lir_level);
	if (--lir_left == 0 && ++lir_idx < lir_count)
		lir_left = lir_ticks[lir_idx];
}

// Frames recorded from the lamp's own remote (mark, space, mark ... 67 values),
// replayed exactly instead of re-encoding them.
static const uint16_t lir_code_bright_down[67] = {
	9009, 4445, 606, 544, 580, 545, 558, 567, 579, 545, 580, 546, 557, 566, 583, 542, 579, 1673, 603, 1645, 558, 1693, 556, 1694, 580, 1671, 558, 1695, 554, 1695, 580, 1672, 558, 568, 559, 1692, 560, 566, 580, 1672, 581, 547, 555, 568, 556, 569, 580, 545, 581, 544, 555, 570, 557, 1693, 557, 569, 557, 1695, 559, 1693, 558, 1697, 579, 1648, 583, 1694, 558
};

static const uint16_t lir_code_bright_up[67] = {
	8984, 4470, 580, 566, 556, 569, 556, 570, 557, 566, 557, 567, 558, 567, 562, 563, 583, 1644, 583, 1670, 604, 1644, 608, 1645, 605, 1647, 604, 1645, 583, 1668, 605, 1645, 607, 521, 601, 520, 580, 1668, 607, 519, 579, 544, 580, 1671, 601, 522, 579, 544, 579, 544, 602, 1648, 580, 543, 578, 1672, 579, 1671, 601, 526, 575, 1673, 583, 1671, 574, 1675, 576
};

static const uint16_t lir_code_fan[67] = {
	9006, 4446, 606, 543, 581, 543, 606, 519, 581, 544, 581, 543, 583, 544, 579, 546, 580, 1671, 583, 1669, 558, 1698, 575, 1672, 556, 1697, 554, 1695, 557, 1697, 554, 1674, 579, 570, 554, 1673, 583, 1672, 579, 1668, 612, 1641, 585, 1669, 605, 543, 583, 543, 582, 543, 584, 542, 581, 523, 602, 522, 604, 522, 603, 523, 579, 1671, 590, 1663, 578, 1672, 605
};

static const uint16_t lir_code_fan_down[67] = {
	9028, 4424, 603, 545, 580, 544, 582, 543, 581, 545, 579, 547, 579, 544, 579, 546, 581, 1668, 585, 1667, 559, 1696, 578, 1670, 581, 1672, 577, 1672, 581, 1671, 582, 1670, 580, 546, 579, 1673, 580, 547, 574, 548, 558, 567, 580, 1673, 555, 568, 579, 546, 579, 547, 555, 568, 578, 1672, 557, 1696, 578, 1670, 557, 569, 557, 1693, 555, 1701, 574, 1671, 558
};

static const uint16_t lir_code_fan_up[67] = {
	9036, 4430, 604, 544, 581, 544, 605, 522, 580, 544, 582, 543, 583, 543, 586, 539, 582, 1670, 583, 1673, 602, 1647, 584, 1669, 605, 1650, 579, 1671, 582, 1670, 582, 1671, 581, 542, 582, 1671, 581, 1674, 602, 520, 582, 1671, 584, 1667, 583, 544, 582, 545, 580, 545, 582, 543, 582, 543, 586, 1667, 583, 545, 581, 548, 576, 1670, 583, 1671, 579, 1670, 582
};

static const uint16_t lir_code_off[67] = {
	9017, 4451, 605, 542, 582, 544, 581, 543, 582, 544, 582, 543, 583, 549, 579, 543, 581, 1671, 585, 1667, 583, 1673, 579, 1670, 582, 1673, 580, 1671, 582, 1673, 580, 1670, 581, 545, 583, 542, 581, 543, 582, 1674, 578, 1671, 583, 1669, 581, 545, 582, 546, 579, 545, 581, 1671, 584, 1668, 582, 544, 579, 546, 559, 567, 556, 1696, 557, 1698, 554, 1696, 561
};

static const uint16_t lir_code_on[67] = {
	9008, 4447, 603, 542, 582, 543, 584, 540, 581, 544, 581, 544, 582, 543, 581, 546, 580, 1669, 584, 1668, 581, 1671, 582, 1670, 580, 1672, 578, 1670, 582, 1671, 580, 1670, 582, 544, 582, 1671, 579, 545, 579, 1671, 580, 1673, 577, 1672, 581, 545, 564, 561, 578, 546, 580, 544, 579, 1677, 556, 566, 579, 546, 578, 548, 555, 1693, 580, 1676, 554, 1695, 561
};

static const uint16_t lir_code_cold[67] = {
	8995, 4471, 583, 567, 557, 568, 557, 568, 557, 568, 582, 546, 557, 567, 558, 568, 581, 1646, 582, 1669, 584, 1669, 605, 1645, 610, 1643, 607, 1649, 580, 1670, 607, 1647, 606, 521, 604, 520, 582, 1670, 603, 521, 604, 1648, 603, 1647, 604, 522, 607, 518, 604, 521, 579, 1674, 602, 522, 603, 1648, 606, 520, 579, 546, 579, 1676, 575, 1674, 600, 1653, 576
};

static const uint16_t lir_code_warm[67] = {
	9015, 4444, 608, 540, 583, 542, 583, 542, 582, 542, 582, 545, 580, 542, 582, 543, 582, 1668, 582, 1670, 580, 1668, 583, 1670, 581, 1668, 582, 1670, 580, 1669, 582, 1671, 580, 542, 583, 542, 584, 1666, 582, 1673, 579, 1669, 584, 1668, 585, 540, 585, 540, 583, 544, 581, 1668, 583, 542, 583, 544, 581, 542, 583, 542, 582, 1668, 582, 1669, 585, 1666, 582
};

static const struct { const char *name; const uint16_t *d; } lir_codes[] = {
	{ "bright_down", lir_code_bright_down },
	{ "bright_up", lir_code_bright_up },
	{ "fan", lir_code_fan },
	{ "fan_down", lir_code_fan_down },
	{ "fan_up", lir_code_fan_up },
	{ "off", lir_code_off },
	{ "on", lir_code_on },
	{ "cold", lir_code_cold },
	{ "warm", lir_code_warm },
};

static commandResult_t lir_start(void) {
	if (lir_pin < 0 || lir_timer < 0) {
		addLogAdv(LOG_ERROR, LOG_FEATURE_CMD, "LampIR: call LampIR_Setup <pin> first");
		return CMD_RES_ERROR;
	}
	if (lir_busy) {
		addLogAdv(LOG_WARN, LOG_FEATURE_CMD, "LampIR: busy, command dropped");
		return CMD_RES_ERROR;
	}
	lir_dc = 0;
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

static commandResult_t CMD_LampIR_Code(const void *context, const char *cmd,
                                       const char *args, int cmdFlags) {
	int i, k, repeats = 0;
	uint32_t t = 0;
	const char *name;

	Tokenizer_TokenizeString(args, 0);
	name = Tokenizer_GetArgsCount() > 0 ? Tokenizer_GetArg(0) : "";
	for (i = 0; i < (int)(sizeof(lir_codes) / sizeof(lir_codes[0])); i++)
		if (!strcmp(name, lir_codes[i].name))
			break;
	if (i == (int)(sizeof(lir_codes) / sizeof(lir_codes[0]))) {
		addLogAdv(LOG_ERROR, LOG_FEATURE_CMD,
		          "LampIR_Code <on|off|bright_up|bright_down|warm|cold|fan|fan_up|fan_down> [repeats]");
		return CMD_RES_BAD_ARGUMENT;
	}
	if (lir_start() != CMD_RES_OK)
		return CMD_RES_ERROR;

	if (Tokenizer_GetArgsCount() > 1)
		repeats = Tokenizer_GetArgInteger(1);
	if (repeats < 0)
		repeats = 0;
	if (repeats > LIR_MAX_REPEATS)
		repeats = LIR_MAX_REPEATS;

	lir_count = 0;
	for (k = 0; k < 67; k++) {
		lir_add(lir_codes[i].d[k]);
		t += lir_codes[i].d[k];
	}
	lir_repeats(t, repeats);
	lir_go();

	addLogAdv(LOG_INFO, LOG_FEATURE_CMD, "LampIR: code '%s' repeats %i", name, repeats);
	return CMD_RES_OK;
}

static commandResult_t CMD_LampIR_Raw(const void *context, const char *cmd,
                                      const char *args, int cmdFlags) {
	const char *p = args;
	char *e;

	if (lir_start() != CMD_RES_OK)
		return CMD_RES_ERROR;

	lir_count = 0;
	while (*p) {
		long v = strtol(p, &e, 10);
		if (e == p) {
			p++;
			continue;
		}
		lir_add((uint32_t)(v < 0 ? -v : v));
		p = e;
	}
	if (lir_count < 2)
		return CMD_RES_BAD_ARGUMENT;
	lir_go();

	addLogAdv(LOG_INFO, LOG_FEATURE_CMD, "LampIR: raw frame, %i entries", lir_count);
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

	lir_dc = Tokenizer_GetArgsCount() > 1;
	ms = Tokenizer_GetArgInteger(0);
	if (ms < 10)
		ms = 10;
	if (ms > (lir_dc ? 30000 : 5000))
		ms = lir_dc ? 30000 : 5000;
	lir_count = 0;
	lir_add((uint32_t)ms * 1000);
	lir_go();

	addLogAdv(LOG_INFO, LOG_FEATURE_CMD, "LampIR: %s for %i ms (a 38 kHz carrier = %i ISR/s)",
	          lir_dc ? "pin HIGH" : "carrier", ms, 1000000 / LIR_TICK_US);
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
	CMD_RegisterCommand("LampIR_Code",  CMD_LampIR_Code,  NULL);
	CMD_RegisterCommand("LampIR_Raw",   CMD_LampIR_Raw,   NULL);
	CMD_RegisterCommand("LampIR_Carrier", CMD_LampIR_Carrier, NULL);
	CMD_RegisterCommand("LampIR_Status", CMD_LampIR_Status, NULL);

	addLogAdv(LOG_INFO, LOG_FEATURE_CMD, "LampIR driver started, HW timer %i, tick %.1f us",
	          (int)lir_timer, real);
}

// Real timer rate: a full second of carrier must log ~76923 ISR/s, otherwise the
// carrier is not 38 kHz whatever the pin average says.
void LampIR_OnEverySecond(void) {
	static uint32_t prev;
	uint32_t n = lir_isr;

	if (n != prev)
		addLogAdv(LOG_INFO, LOG_FEATURE_CMD, "LampIR: %u ISR in the last second", (unsigned)(n - prev));
	prev = n;
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
