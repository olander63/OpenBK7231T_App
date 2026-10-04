// ============================================================================
//  drv_lampir.c  --  OpenBeken driver: IR remote of the ceiling lamp + fan,
//                    and a model of the lamp's state
//
//  The 38 kHz carrier is made in software: a HW timer ticks every 13 us and the
//  ISR toggles a plain GPIO while an IR "mark" is on. (OBK's hardware PWM path
//  puts nothing on the pin on this LN882H board, plain GPIO output works.)
//  The lamp's own remote frames (NEC, addr 0x80) are replayed from recordings.
//
//  Hardware: GPIO -> 1k -> NPN base, IR LED + resistor from 3V3 to the collector,
//  so pin HIGH = LED on. Leave the pin's OBK role as None.
//
//  The lamp (what the physical remote and the relay do to it):
//    - mains relay (OBK channel 1) off  -> light and fan are off
//    - mains relay back on              -> the light comes on by itself, the fan stays off
//    - light: discrete on / off codes, 10 brightness steps, 10 colour steps
//    - fan: ONE toggle code, 6 speed steps; both light and fan remember their settings
//  IR is one way, so light/fan/brightness/colour/speed are ASSUMED states:
//  every step is a separate remote press. Values that may be wrong are fixed by
//  Lamp_Set (tell the model) or Lamp_Sync (drive the lamp to its stops).
//
//  Console (also usable from MQTT:  cmnd/<clientId>/<command>  payload = argument):
//      LampIR_Setup <pin>             IR LED pin
//      Lamp_Light <on|off|toggle>
//      Lamp_Brightness <1..10>        turns the light on if needed
//      Lamp_Temp <1..10>              1 = 2700K warm ... 10 = 6500K cold
//      Lamp_Fan <on|off|toggle>
//      Lamp_Speed <1..6>              turns the fan on if needed
//      Lamp_Click <1|2|3>             what a wall-switch click does (MultiButton calls it)
//      Lamp_Hold                      relay off, light + fan off
//      Lamp_Set <light|fan|bright|temp|speed> <value>   fix the model, sends nothing
//      Lamp_Sync                      drive brightness, colour, speed to their lowest stop
//      Lamp_Tune <gapMs> [repeats] [bootMs]   time between remote presses, NEC repeat
//                                     frames per press, wait after the lamp gets power
//      Lamp_Status
//      LampIR_Code <name> [repeats]   send one recorded remote button (see lir_codes)
//      LampIR_Carrier <ms> [dc]       38 kHz burst / pin held HIGH, for checking the LED
//      LampIR_Status
//
//  State is published retained to <clientId>/lamp/get as JSON
//  {"light":1,"bright":5,"temp":3,"fan":0,"speed":2} (0 = unknown).
//
//  ponytail: no IR receiver, so a press on the real remote is invisible; tell the
//            model with Lamp_Set or Lamp_Sync. Add a receiver pin to track it.
//  ponytail: 13 us tick -> 38.46 kHz (+1.2 %). Change LIR_TICK_US if a receiver
//            turns out to be picky.
// ============================================================================

#include "../new_common.h"
#include "../new_pins.h"
#include "../logging/logging.h"
#include "../cmnds/cmd_public.h"
#include "../hal/hal_pins.h"
#include "../hal/hal_hwtimer.h"
#include "../hal/hal_flashVars.h"
#include "../mqtt/new_mqtt.h"
#include "drv_local.h"

extern int g_deltaTimeMS;              // ms elapsed since previous quick tick

#define LIR_TICK_US      13         // half a carrier period
#define LIR_FRAME_US     108000     // NEC repeat period, start of frame to start of frame
#define LIR_MAX_REPEATS  20
#define LIR_MAX_ENTRIES  160        // 67 for a frame + 4 per repeat

#define LAMP_RELAY_CH    1
#define LAMP_STEPS       10         // brightness and colour positions
#define LAMP_SPEEDS      6          // fan speed positions
#define LAMP_OVERSHOOT   2          // extra presses at a stop, heals a drifted model
#define LAMP_QUEUE       128
#define LAMP_FV_BRIGHT   5          // flash-var slots (0..11): kept across reboots
#define LAMP_FV_TEMP     6
#define LAMP_FV_SPEED    7

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

// order of lir_codes[]
enum { C_BRIGHT_DOWN, C_BRIGHT_UP, C_FAN, C_FAN_DOWN, C_FAN_UP, C_OFF, C_ON, C_COLD, C_WARM, C_COUNT };

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

static void lir_send(int code, int repeats) {
	int k;
	uint32_t t = 0;

	lir_count = 0;
	for (k = 0; k < 67; k++) {
		lir_add(lir_codes[code].d[k]);
		t += lir_codes[code].d[k];
	}
	lir_repeats(t, repeats);
	lir_go();
}

// ---------------------------------------------------------------------------
//  Lamp model
// ---------------------------------------------------------------------------
static struct {
	int light, fan;                     // assumed on / off
	int bright, temp, speed;            // 1..max, 0 = unknown
	int dirty;                          // needs publishing
} L;

static struct { uint8_t c[LAMP_QUEUE]; int h, n; } lq;      // remote presses to send
static int lir_wait;                    // ms until the next press may start
static int lir_gap = 250;               // ms between presses (start to start)
static int lir_press_repeats;           // NEC repeat frames after each press
static int lir_boot = 2000;             // ms the lamp needs after it gets power
static int lamp_saved[3];               // flash copies of bright / temp / speed

static int relay(void) {
	return CHANNEL_Get(LAMP_RELAY_CH) > 0;
}

static void q_add(int code) {
	if (lir_pin < 0)
		return;
	if (lq.n >= LAMP_QUEUE) {
		addLogAdv(LOG_WARN, LOG_FEATURE_CMD, "Lamp: IR queue full, press dropped");
		return;
	}
	lq.c[(lq.h + lq.n++) % LAMP_QUEUE] = (uint8_t)code;
}

static void light_set(int on) {
	q_add(on ? C_ON : C_OFF);
	L.light = on;
	L.dirty = 1;
}

// Relay on powers the lamp: it lights up by itself, the fan stays off.
// Called from the OBK channel callback, so HA / web / button / script all end up here.
void LampIR_OnChannelChanged(int ch, int val) {
	if (ch != LAMP_RELAY_CH)
		return;
	if (val) {
		L.light = 1;
		L.fan = 0;
		if (lir_wait < lir_boot)
			lir_wait = lir_boot;
	} else {
		L.light = L.fan = 0;
		lq.n = 0;                       // the lamp has no power, drop pending presses
	}
	L.dirty = 1;
}

// true if the lamp had no power (it then lights up by itself)
static int lamp_power(void) {
	if (relay())
		return 0;
	CHANNEL_Set(LAMP_RELAY_CH, 1, 0);
	return 1;
}

static void lamp_light_on(void) {
	if (!lamp_power() && !L.light)
		light_set(1);
}

// Move a stepped value to target with remote presses. Unknown (0): hit the
// lower stop first. A stop gets extra presses so a drifted model heals itself.
static void step_to(int *cur, int target, int max, int up, int down) {
	int n;

	if (*cur == 0) {
		for (n = 0; n < max; n++)
			q_add(down);
		*cur = 1;
	}
	for (n = *cur; n < target; n++)
		q_add(up);
	for (n = *cur; n > target; n--)
		q_add(down);
	if (target == max || target == 1)
		for (n = 0; n < LAMP_OVERSHOOT; n++)
			q_add(target == max ? up : down);
	*cur = target;
	L.dirty = 1;
}

static void lamp_fan(int on) {
	if (on == L.fan)
		return;
	if (on && lamp_power())
		light_set(0);                   // the lamp lit up by itself, a fan request keeps the light off
	q_add(C_FAN);
	L.fan = on;
	if (on && !L.speed)
		step_to(&L.speed, 1, LAMP_SPEEDS, C_FAN_UP, C_FAN_DOWN);
	L.dirty = 1;
}

static void lamp_save(void) {
	int v[3] = { L.bright, L.temp, L.speed };
	static const int slot[3] = { LAMP_FV_BRIGHT, LAMP_FV_TEMP, LAMP_FV_SPEED };
	int i;

	for (i = 0; i < 3; i++)
		if (lamp_saved[i] != v[i]) {
			lamp_saved[i] = v[i];
			HAL_FlashVars_SaveChannel(slot[i], v[i]);
		}
}

static void lamp_publish(void) {
	char s[80];

	snprintf(s, sizeof(s), "{\"light\":%i,\"bright\":%i,\"temp\":%i,\"fan\":%i,\"speed\":%i}",
	         L.light, L.bright, L.temp, L.fan, L.speed);
	if (MQTT_PublishMain_StringString("lamp", s, OBK_PUBLISH_FLAG_RETAIN) == OBK_PUBLISH_OK)
		L.dirty = 0;
}

// Called from the OBK main loop: paces the remote presses and publishes the state.
void LampIR_RunQuickTick(void) {
	static int wasReady;
	int dt = g_deltaTimeMS, ready;

	if (dt <= 0)
		dt = 1;
	if (dt > 200)
		dt = 200;                       // ignore huge stalls (OTA, scan, ...)
	if (lir_wait > 0)
		lir_wait -= dt;

	if (lq.n && lir_wait <= 0 && !lir_busy && lir_pin >= 0 && lir_timer >= 0) {
		lir_send(lq.c[lq.h], lir_press_repeats);
		lq.h = (lq.h + 1) % LAMP_QUEUE;
		lq.n--;
		lir_wait = lir_gap;
	} else if (!lq.n && !lir_busy) {
		lamp_save();                    // one flash write after a burst of presses
	}

	ready = MQTT_IsReady();
	if (ready && !wasReady)
		L.dirty = 1;
	wasReady = ready;
	if (L.dirty && ready)
		lamp_publish();
}

// on / off / toggle -> 1 / 0, anything else -> -1
static int onoff_arg(const char *args, int cur) {
	const char *a;

	Tokenizer_TokenizeString(args, 0);
	if (Tokenizer_GetArgsCount() < 1)
		return -1;
	a = Tokenizer_GetArg(0);
	if (!strcmp(a, "toggle"))
		return !cur;
	if (!strcmp(a, "on") || !strcmp(a, "1"))
		return 1;
	if (!strcmp(a, "off") || !strcmp(a, "0"))
		return 0;
	return -1;
}

static commandResult_t CMD_Lamp_Light(const void *context, const char *cmd,
                                      const char *args, int cmdFlags) {
	int on = onoff_arg(args, L.light);

	if (on < 0)
		return CMD_RES_BAD_ARGUMENT;
	if (on)
		lamp_light_on();
	else if (relay())
		light_set(0);
	return CMD_RES_OK;
}

static commandResult_t CMD_Lamp_Fan(const void *context, const char *cmd,
                                    const char *args, int cmdFlags) {
	int on = onoff_arg(args, L.fan);

	if (on < 0)
		return CMD_RES_BAD_ARGUMENT;
	lamp_fan(on);
	return CMD_RES_OK;
}

// shared by Lamp_Brightness / Lamp_Temp / Lamp_Speed
static commandResult_t lamp_value(const char *args, int *cur, int max, int up, int down, int fan) {
	int v;

	Tokenizer_TokenizeString(args, 0);
	if (Tokenizer_GetArgsCount() < 1)
		return CMD_RES_NOT_ENOUGH_ARGUMENTS;
	v = Tokenizer_GetArgInteger(0);
	if (v < 1 || v > max)
		return CMD_RES_BAD_ARGUMENT;
	if (fan)
		lamp_fan(1);
	else
		lamp_light_on();
	step_to(cur, v, max, up, down);
	return CMD_RES_OK;
}

static commandResult_t CMD_Lamp_Brightness(const void *context, const char *cmd,
                                           const char *args, int cmdFlags) {
	return lamp_value(args, &L.bright, LAMP_STEPS, C_BRIGHT_UP, C_BRIGHT_DOWN, 0);
}

static commandResult_t CMD_Lamp_Temp(const void *context, const char *cmd,
                                     const char *args, int cmdFlags) {
	return lamp_value(args, &L.temp, LAMP_STEPS, C_COLD, C_WARM, 0);
}

static commandResult_t CMD_Lamp_Speed(const void *context, const char *cmd,
                                      const char *args, int cmdFlags) {
	return lamp_value(args, &L.speed, LAMP_SPEEDS, C_FAN_UP, C_FAN_DOWN, 1);
}

// Wall switch: 1 click = relay on / light toggle, 2 clicks = relay on (light stays off) / fan toggle.
static commandResult_t CMD_Lamp_Click(const void *context, const char *cmd,
                                      const char *args, int cmdFlags) {
	Tokenizer_TokenizeString(args, 0);
	if (Tokenizer_GetArgsCount() < 1)
		return CMD_RES_NOT_ENOUGH_ARGUMENTS;
	switch (Tokenizer_GetArgInteger(0)) {
	case 1:
		if (!lamp_power())
			light_set(!L.light);
		break;
	case 2:
		lamp_fan(!L.fan);
		break;
	}
	return CMD_RES_OK;
}

static commandResult_t CMD_Lamp_Hold(const void *context, const char *cmd,
                                     const char *args, int cmdFlags) {
	if (relay())
		CHANNEL_Set(LAMP_RELAY_CH, 0, 0);
	return CMD_RES_OK;
}

static commandResult_t CMD_Lamp_Set(const void *context, const char *cmd,
                                    const char *args, int cmdFlags) {
	const char *n;
	int v;

	Tokenizer_TokenizeString(args, 0);
	if (Tokenizer_GetArgsCount() < 2)
		return CMD_RES_NOT_ENOUGH_ARGUMENTS;
	n = Tokenizer_GetArg(0);
	v = Tokenizer_GetArgInteger(1);
	if (!strcmp(n, "light"))
		L.light = v > 0;
	else if (!strcmp(n, "fan"))
		L.fan = v > 0;
	else if (!strcmp(n, "bright") && v >= 0 && v <= LAMP_STEPS)
		L.bright = v;
	else if (!strcmp(n, "temp") && v >= 0 && v <= LAMP_STEPS)
		L.temp = v;
	else if (!strcmp(n, "speed") && v >= 0 && v <= LAMP_SPEEDS)
		L.speed = v;
	else
		return CMD_RES_BAD_ARGUMENT;
	L.dirty = 1;
	return CMD_RES_OK;
}

static commandResult_t CMD_Lamp_Sync(const void *context, const char *cmd,
                                     const char *args, int cmdFlags) {
	lamp_light_on();
	L.bright = L.temp = 0;
	step_to(&L.bright, 1, LAMP_STEPS, C_BRIGHT_UP, C_BRIGHT_DOWN);
	step_to(&L.temp, 1, LAMP_STEPS, C_COLD, C_WARM);
	L.speed = 0;                        // the fan is synced when it is next switched on
	if (L.fan)
		step_to(&L.speed, 1, LAMP_SPEEDS, C_FAN_UP, C_FAN_DOWN);
	return CMD_RES_OK;
}

static commandResult_t CMD_Lamp_Tune(const void *context, const char *cmd,
                                     const char *args, int cmdFlags) {
	Tokenizer_TokenizeString(args, 0);
	if (Tokenizer_GetArgsCount() < 1)
		return CMD_RES_NOT_ENOUGH_ARGUMENTS;
	lir_gap = Tokenizer_GetArgInteger(0);
	if (Tokenizer_GetArgsCount() > 1) {
		lir_press_repeats = Tokenizer_GetArgInteger(1);
		if (lir_press_repeats < 0)
			lir_press_repeats = 0;
		if (lir_press_repeats > LIR_MAX_REPEATS)
			lir_press_repeats = LIR_MAX_REPEATS;
	}
	if (Tokenizer_GetArgsCount() > 2)
		lir_boot = Tokenizer_GetArgInteger(2);
	return CMD_RES_OK;
}

static commandResult_t CMD_Lamp_Status(const void *context, const char *cmd,
                                       const char *args, int cmdFlags) {
	addLogAdv(LOG_INFO, LOG_FEATURE_CMD,
	          "Lamp: relay %i light %i fan %i bright %i temp %i speed %i, queue %i, gap %i repeats %i boot %i",
	          relay(), L.light, L.fan, L.bright, L.temp, L.speed, lq.n,
	          lir_gap, lir_press_repeats, lir_boot);
	return CMD_RES_OK;
}

// ---------------------------------------------------------------------------
//  Hardware commands
// ---------------------------------------------------------------------------
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

static commandResult_t CMD_LampIR_Code(const void *context, const char *cmd,
                                       const char *args, int cmdFlags) {
	int i, repeats = 0;
	const char *name;

	Tokenizer_TokenizeString(args, 0);
	name = Tokenizer_GetArgsCount() > 0 ? Tokenizer_GetArg(0) : "";
	for (i = 0; i < C_COUNT; i++)
		if (!strcmp(name, lir_codes[i].name))
			break;
	if (i == C_COUNT) {
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

	lir_send(i, repeats);
	addLogAdv(LOG_INFO, LOG_FEATURE_CMD, "LampIR: code '%s' repeats %i", name, repeats);
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

	addLogAdv(LOG_INFO, LOG_FEATURE_CMD, "LampIR: %s for %i ms", lir_dc ? "pin HIGH" : "carrier", ms);
	return CMD_RES_OK;
}

static commandResult_t CMD_LampIR_Status(const void *context, const char *cmd,
                                         const char *args, int cmdFlags) {
	addLogAdv(LOG_INFO, LOG_FEATURE_CMD,
	          "LampIR: pin %i, timer %i, busy %i, entry %i/%i, ISR calls %u",
	          lir_pin, (int)lir_timer, (int)lir_busy, (int)lir_idx, lir_count, (unsigned)lir_isr);
	return CMD_RES_OK;
}

static int fv_get(int slot, int max) {
	int v = HAL_FlashVars_GetChannelValue(slot);

	return (v >= 1 && v <= max) ? v : 0;
}

void LampIR_Init(void) {
	float real = 0;

	lir_pin = -1;
	lir_busy = 0;
	if (lir_timer < 0)
		lir_timer = HAL_RequestHWTimer(LIR_TICK_US, &real, lir_tick, NULL);

	memset(&L, 0, sizeof(L));
	memset(&lq, 0, sizeof(lq));
	lir_wait = 0;
	L.bright = lamp_saved[0] = fv_get(LAMP_FV_BRIGHT, LAMP_STEPS);
	L.temp = lamp_saved[1] = fv_get(LAMP_FV_TEMP, LAMP_STEPS);
	L.speed = lamp_saved[2] = fv_get(LAMP_FV_SPEED, LAMP_SPEEDS);
	L.light = relay();                  // a powered lamp is assumed lit, fan off
	L.dirty = 1;

	CMD_RegisterCommand("LampIR_Setup", CMD_LampIR_Setup, NULL);
	CMD_RegisterCommand("LampIR_Code",  CMD_LampIR_Code,  NULL);
	CMD_RegisterCommand("LampIR_Carrier", CMD_LampIR_Carrier, NULL);
	CMD_RegisterCommand("LampIR_Status", CMD_LampIR_Status, NULL);
	CMD_RegisterCommand("Lamp_Light",      CMD_Lamp_Light,      NULL);
	CMD_RegisterCommand("Lamp_Brightness", CMD_Lamp_Brightness, NULL);
	CMD_RegisterCommand("Lamp_Temp",       CMD_Lamp_Temp,       NULL);
	CMD_RegisterCommand("Lamp_Fan",        CMD_Lamp_Fan,        NULL);
	CMD_RegisterCommand("Lamp_Speed",      CMD_Lamp_Speed,      NULL);
	CMD_RegisterCommand("Lamp_Click",      CMD_Lamp_Click,      NULL);
	CMD_RegisterCommand("Lamp_Hold",       CMD_Lamp_Hold,       NULL);
	CMD_RegisterCommand("Lamp_Set",        CMD_Lamp_Set,        NULL);
	CMD_RegisterCommand("Lamp_Sync",       CMD_Lamp_Sync,       NULL);
	CMD_RegisterCommand("Lamp_Tune",       CMD_Lamp_Tune,       NULL);
	CMD_RegisterCommand("Lamp_Status",     CMD_Lamp_Status,     NULL);

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
	lq.n = 0;
}
