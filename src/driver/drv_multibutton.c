// ============================================================================
//  drv_multibutton.c  --  OpenBeken driver: 1 / 2 / 3 clicks + hold
//  v2: hold toggles the relay locally, clicks 1/2/3 are fully programmable
//
//  Written for: Tuya wall switch, board LSPS5CBA V2.0, Lightning LN882HKI
//      button    PA4   input, internal pull-up, active LOW
//      relay     PB5   output push-pull, active HIGH   -> OBK channel 1
//      backlight PA3   output push-pull, active LOW    (also UART0 RX!)
//      net LED   PA6   output push-pull, active LOW
//  The driver itself is platform independent.
//
//  Behaviour (default):
//      hold  (>= holdMs)  -> toggles the relay channel locally, in firmware,
//                            so the switch keeps working with no Wi-Fi / broker
//      1 / 2 / 3 clicks   -> MQTT event + optional local OBK command,
//                            the relay is NOT touched, so the light never blinks
//
//  Which event drives the relay is configurable: MB_LocalToggle <none|1|2|3|hold>
//
//  Console commands (put them into autoexec.bat):
//      MB_Setup   <pinIndex> [channel] [activeLow]
//      MB_Timings <debounceMs> <gapMs> <holdMs> <holdRepeatMs>
//      MB_Action  <1|2|3|hold> <command ...>          ("-" clears)
//      MB_LocalToggle <none|1|2|3|hold>
//      MB_Status
//      MB_Test    <pinIndex>      // configure as input pull-up and print level
//
//  MQTT: publishes to  <clientId>/button/get  one of:
//        single | double | triple | hold
// ============================================================================

#include "../new_common.h"
#include "../new_pins.h"
#include "../new_cfg.h"
#include "../logging/logging.h"
#include "../cmnds/cmd_public.h"
#include "../hal/hal_pins.h"
#include "../mqtt/new_mqtt.h"
#include "drv_local.h"

// ---------------------------------------------------------------------------
//  PORTING SHIM - the only places that touch OpenBeken internals.
//  If something here does not compile/link in your OBK version, fix it HERE.
// ---------------------------------------------------------------------------
extern int g_deltaTimeMS;              // ms elapsed since previous quick tick
// If your OBK build has no g_deltaTimeMS, delete the line above and use:
//     #define MB_FIXED_TICK_MS 10
// ---------------------------------------------------------------------------

#define MB_MAX_CLICKS   3
#define MB_EV_HOLD      0
#define MB_EV_NONE     (-1)

typedef struct mbState_s {
	int   inited;
	int   pin;
	int   channel;
	int   activeLow;

	int   debounceMs;
	int   gapMs;
	int   holdMs;
	int   holdRepeatMs;
	int   localEvent;               // which event toggles the relay locally

	char *act[MB_MAX_CLICKS + 1];   // [0] = hold, [1..3] = click count

	// runtime
	int   lastRaw;
	int   stable;
	int   debTimer;
	int   pressTimer;
	int   gapTimer;                 // -1 = idle
	int   clicks;
	int   holdFired;
	int   holdRepeatTimer;
	int   totalEvents;
} mbState_t;

static mbState_t mb;

static const char *g_mbNames[MB_MAX_CLICKS + 1] = { "hold", "single", "double", "triple" };

static const char *MB_EventName(int idx) {
	if (idx < 0 || idx > MB_MAX_CLICKS)
		return "none";
	return g_mbNames[idx];
}

// Accepts: none / 0 / hold / 1 / 2 / 3 / single / double / triple
static int MB_ParseEvent(const char *s) {
	if (!s || !s[0])
		return MB_EV_NONE;
	if (!strcmp(s, "none") || !strcmp(s, "off") || !strcmp(s, "-1"))
		return MB_EV_NONE;
	if (!strcmp(s, "hold") || !strcmp(s, "0"))
		return MB_EV_HOLD;
	if (!strcmp(s, "single"))
		return 1;
	if (!strcmp(s, "double"))
		return 2;
	if (!strcmp(s, "triple"))
		return 3;
	if (s[0] >= '1' && s[0] <= '3' && s[1] == 0)
		return s[0] - '0';
	return MB_EV_NONE;
}

// ---------------------------------------------------------------------------

static void MB_SetAction(int idx, const char *cmd) {
	if (idx < 0 || idx > MB_MAX_CLICKS)
		return;
	if (mb.act[idx]) {
		free(mb.act[idx]);
		mb.act[idx] = 0;
	}
	if (cmd && cmd[0] && !(cmd[0] == '-' && cmd[1] == 0)) {
		int len = strlen(cmd);
		mb.act[idx] = (char *)malloc(len + 1);
		if (mb.act[idx])
			memcpy(mb.act[idx], cmd, len + 1);
	}
}

// isRepeat = 1 for the auto-repeated 'hold' ticks; those must never toggle
// the relay again, otherwise holding the key would flap it.
static void MB_Fire(int idx, int isRepeat) {
	if (idx < 0 || idx > MB_MAX_CLICKS)
		return;

	mb.totalEvents++;

	// 1) local relay action first - fastest possible reaction of the light
	if (idx == mb.localEvent && !isRepeat) {
		CHANNEL_Toggle(mb.channel);
		addLogAdv(LOG_INFO, LOG_FEATURE_CMD,
		          "MultiButton: '%s' -> local toggle of channel %i",
		          MB_EventName(idx), mb.channel);
	}

	addLogAdv(LOG_INFO, LOG_FEATURE_CMD, "MultiButton: event '%s'%s",
	          MB_EventName(idx), isRepeat ? " (repeat)" : "");

	// 2) tell Home Assistant
	MQTT_PublishMain_StringString("button", MB_EventName(idx), 0);

	// 3) run the user command bound to this event, if any.
	//    Independent of the local toggle above - both can be active.
	if (mb.act[idx] && mb.act[idx][0])
		CMD_ExecuteCommand(mb.act[idx], COMMAND_FLAG_SOURCE_SCRIPT);
}

// ---------------------------------------------------------------------------
//  State machine, called from the OBK main loop
// ---------------------------------------------------------------------------
void MultiButton_RunQuickTick(void) {
	int raw, dt;

	if (!mb.inited)
		return;

#ifdef MB_FIXED_TICK_MS
	dt = MB_FIXED_TICK_MS;
#else
	dt = g_deltaTimeMS;
#endif
	if (dt <= 0)
		dt = 1;
	if (dt > 200)
		dt = 200;                       // ignore huge stalls (OTA, scan, ...)

	raw = HAL_PIN_ReadDigitalInput(mb.pin);
	if (mb.activeLow)
		raw = !raw;                     // from here: 1 = pressed

	// ---- debounce -------------------------------------------------------
	if (raw != mb.lastRaw) {
		mb.lastRaw = raw;
		mb.debTimer = 0;
	} else if (mb.debTimer < mb.debounceMs) {
		mb.debTimer += dt;
		if (mb.debTimer >= mb.debounceMs && raw != mb.stable) {
			mb.stable = raw;
			if (raw) {
				// ---- pressed ----
				mb.pressTimer = 0;
				mb.holdFired = 0;
				mb.holdRepeatTimer = 0;
				mb.gapTimer = -1;       // freeze the click window while held
			} else {
				// ---- released ----
				if (mb.holdFired) {
					mb.clicks = 0;      // a hold is not a click
					mb.gapTimer = -1;
				} else {
					if (mb.clicks < MB_MAX_CLICKS)
						mb.clicks++;
					mb.gapTimer = 0;    // start / restart the click window
				}
			}
		}
	}

	// ---- held down ------------------------------------------------------
	if (mb.stable) {
		mb.pressTimer += dt;
		if (!mb.holdFired) {
			if (mb.pressTimer >= mb.holdMs) {
				mb.holdFired = 1;
				mb.clicks = 0;          // cancel any pending click sequence
				mb.holdRepeatTimer = 0;
				MB_Fire(MB_EV_HOLD, 0);
			}
		} else if (mb.holdRepeatMs > 0) {
			mb.holdRepeatTimer += dt;
			if (mb.holdRepeatTimer >= mb.holdRepeatMs) {
				mb.holdRepeatTimer = 0;
				MB_Fire(MB_EV_HOLD, 1);
			}
		}
		return;
	}

	// ---- released: wait for the multi-click window to expire -------------
	if (mb.gapTimer >= 0) {
		mb.gapTimer += dt;
		if (mb.gapTimer >= mb.gapMs) {
			int c = mb.clicks;
			mb.gapTimer = -1;
			mb.clicks = 0;
			if (c > 0)
				MB_Fire(c, 0);
		}
	}
}

// ---------------------------------------------------------------------------
//  Commands
// ---------------------------------------------------------------------------
static commandResult_t CMD_MB_Setup(const void *context, const char *cmd,
                                    const char *args, int cmdFlags) {
	Tokenizer_TokenizeString(args, 0);
	if (Tokenizer_GetArgsCount() < 1) {
		addLogAdv(LOG_ERROR, LOG_FEATURE_CMD,
		          "MB_Setup: usage MB_Setup <pin> [channel] [activeLow]");
		return CMD_RES_NOT_ENOUGH_ARGUMENTS;
	}
	mb.pin       = Tokenizer_GetArgInteger(0);
	mb.channel   = (Tokenizer_GetArgsCount() > 1) ? Tokenizer_GetArgInteger(1) : 1;
	mb.activeLow = (Tokenizer_GetArgsCount() > 2) ? Tokenizer_GetArgInteger(2) : 1;

	HAL_PIN_Setup_Input_Pullup(mb.pin);

	mb.lastRaw = mb.stable = 0;
	mb.debTimer = mb.pressTimer = mb.clicks = mb.holdFired = 0;
	mb.gapTimer = -1;
	mb.inited = 1;

	addLogAdv(LOG_INFO, LOG_FEATURE_CMD,
	          "MultiButton: pin %i, channel %i, activeLow %i",
	          mb.pin, mb.channel, mb.activeLow);
	return CMD_RES_OK;
}

static commandResult_t CMD_MB_Timings(const void *context, const char *cmd,
                                      const char *args, int cmdFlags) {
	Tokenizer_TokenizeString(args, 0);
	if (Tokenizer_GetArgsCount() < 3)
		return CMD_RES_NOT_ENOUGH_ARGUMENTS;
	mb.debounceMs   = Tokenizer_GetArgInteger(0);
	mb.gapMs        = Tokenizer_GetArgInteger(1);
	mb.holdMs       = Tokenizer_GetArgInteger(2);
	mb.holdRepeatMs = (Tokenizer_GetArgsCount() > 3) ? Tokenizer_GetArgInteger(3) : 0;

	if (mb.debounceMs < 5)
		mb.debounceMs = 5;
	if (mb.gapMs < 80)
		mb.gapMs = 80;
	if (mb.holdMs < mb.debounceMs + 100)
		mb.holdMs = mb.debounceMs + 100;

	if (mb.localEvent == MB_EV_HOLD && mb.holdRepeatMs > 0)
		addLogAdv(LOG_WARN, LOG_FEATURE_CMD,
		          "MultiButton: hold drives the relay, holdRepeat only re-sends "
		          "the MQTT event - the relay is toggled once per press");

	addLogAdv(LOG_INFO, LOG_FEATURE_CMD,
	          "MultiButton: debounce %i, gap %i, hold %i, holdRepeat %i",
	          mb.debounceMs, mb.gapMs, mb.holdMs, mb.holdRepeatMs);
	return CMD_RES_OK;
}

static commandResult_t CMD_MB_Action(const void *context, const char *cmd,
                                     const char *args, int cmdFlags) {
	int idx;

	Tokenizer_TokenizeString(args, 0);
	if (Tokenizer_GetArgsCount() < 1)
		return CMD_RES_NOT_ENOUGH_ARGUMENTS;

	idx = MB_ParseEvent(Tokenizer_GetArg(0));
	if (idx == MB_EV_NONE)
		return CMD_RES_BAD_ARGUMENT;

	MB_SetAction(idx, (Tokenizer_GetArgsCount() > 1) ? Tokenizer_GetArgFrom(1) : 0);
	addLogAdv(LOG_INFO, LOG_FEATURE_CMD, "MultiButton: action '%s' = '%s'",
	          MB_EventName(idx), mb.act[idx] ? mb.act[idx] : "(none)");
	return CMD_RES_OK;
}

static commandResult_t CMD_MB_LocalToggle(const void *context, const char *cmd,
                                          const char *args, int cmdFlags) {
	Tokenizer_TokenizeString(args, 0);
	if (Tokenizer_GetArgsCount() < 1)
		return CMD_RES_NOT_ENOUGH_ARGUMENTS;
	mb.localEvent = MB_ParseEvent(Tokenizer_GetArg(0));
	addLogAdv(LOG_INFO, LOG_FEATURE_CMD,
	          "MultiButton: local relay toggle on event '%s'",
	          MB_EventName(mb.localEvent));
	return CMD_RES_OK;
}

static commandResult_t CMD_MB_Status(const void *context, const char *cmd,
                                     const char *args, int cmdFlags) {
	int i;
	addLogAdv(LOG_INFO, LOG_FEATURE_CMD,
	          "MultiButton: inited %i, pin %i, ch %i, activeLow %i, rawLevel %i, pressed %i, events %i",
	          mb.inited, mb.pin, mb.channel, mb.activeLow,
	          mb.inited ? HAL_PIN_ReadDigitalInput(mb.pin) : -1,
	          mb.stable, mb.totalEvents);
	addLogAdv(LOG_INFO, LOG_FEATURE_CMD,
	          "MultiButton: debounce %i, gap %i, hold %i, holdRepeat %i, localToggle '%s'",
	          mb.debounceMs, mb.gapMs, mb.holdMs, mb.holdRepeatMs,
	          MB_EventName(mb.localEvent));
	for (i = 0; i <= MB_MAX_CLICKS; i++)
		addLogAdv(LOG_INFO, LOG_FEATURE_CMD, "MultiButton: [%s] cmd -> %s%s",
		          g_mbNames[i], mb.act[i] ? mb.act[i] : "(none)",
		          (i == mb.localEvent) ? "   + local relay toggle" : "");
	return CMD_RES_OK;
}

static commandResult_t CMD_MB_Test(const void *context, const char *cmd,
                                   const char *args, int cmdFlags) {
	int p;
	Tokenizer_TokenizeString(args, 0);
	if (Tokenizer_GetArgsCount() < 1)
		return CMD_RES_NOT_ENOUGH_ARGUMENTS;
	p = Tokenizer_GetArgInteger(0);
	HAL_PIN_Setup_Input_Pullup(p);
	addLogAdv(LOG_INFO, LOG_FEATURE_CMD, "MB_Test: pin %i level = %i", p,
	          HAL_PIN_ReadDigitalInput(p));
	return CMD_RES_OK;
}

// ---------------------------------------------------------------------------
void MultiButton_Init(void) {
	memset(&mb, 0, sizeof(mb));
	mb.pin          = -1;
	mb.channel      = 1;
	mb.activeLow    = 1;
	mb.debounceMs   = 30;
	mb.gapMs        = 400;          // clicks no longer gate the light -> can be generous
	mb.holdMs       = 500;          // hold now switches the light, keep it snappy
	mb.holdRepeatMs = 0;
	mb.localEvent   = MB_EV_HOLD;   // <-- hold toggles the relay
	mb.gapTimer     = -1;

	CMD_RegisterCommand("MB_Setup",       CMD_MB_Setup,       NULL);
	CMD_RegisterCommand("MB_Timings",     CMD_MB_Timings,     NULL);
	CMD_RegisterCommand("MB_Action",      CMD_MB_Action,      NULL);
	CMD_RegisterCommand("MB_LocalToggle", CMD_MB_LocalToggle, NULL);
	CMD_RegisterCommand("MB_Status",      CMD_MB_Status,      NULL);
	CMD_RegisterCommand("MB_Test",        CMD_MB_Test,        NULL);

	addLogAdv(LOG_INFO, LOG_FEATURE_CMD,
	          "MultiButton driver started, call MB_Setup <pin> to bind a pin");
}

void MultiButton_StopDriver(void) {
	int i;
	mb.inited = 0;
	for (i = 0; i <= MB_MAX_CLICKS; i++)
		MB_SetAction(i, 0);
}
