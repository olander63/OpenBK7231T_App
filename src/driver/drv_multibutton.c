// ============================================================================
//  drv_multibutton.c  --  OpenBeken driver: 1 / 2 / 3 clicks + hold + status LED
//
//  Written for: Tuya wall switch, board LSPS5CBA V2.0, Lightning LN882HKI
//      button    PA4   input, internal pull-up, active LOW
//      relay     PB5   OBK channel 1 (role Rel, set in OBK itself)
//      net LED   PA6   output push-pull, active LOW -> owned by this driver,
//                                                      leave its OBK pin role as None
//
//  Behaviour:
//      hold  (>= 500 ms)    -> 'Lamp_Hold' (relay toggle) + MQTT 'hold'
//      hold  (>= 10 s)      -> forces config/AP mode (built-in 'OpenAP')
//      1 / 2 / 3 clicks     -> 'Lamp_Click <n>' + MQTT 'single' / 'double' / 'triple'
//      The Lamp_* commands come from the LampIR driver (drv_lampir.c); without it
//      a hold still toggles the relay and clicks only publish.
//
//  Status LED (only if LED_Setup was called): off when WiFi+MQTT are up,
//  slow blink while not connected, fast blink in config/AP mode.
//  Relay power-on-restore is native OBK config ('Configure Startup').
//
//  Console commands:
//      MB_Setup  <pinIndex>
//      LED_Setup <pinIndex>
//      MB_Status
//
//  MQTT: publishes to <clientId>/button/get one of: single|double|triple|hold
// ============================================================================

#include "../new_common.h"
#include "../new_pins.h"
#include "../new_cfg.h"
#include "../logging/logging.h"
#include "../cmnds/cmd_public.h"
#include "../hal/hal_pins.h"
#include "../mqtt/new_mqtt.h"
#include "drv_local.h"

extern int g_deltaTimeMS;              // ms elapsed since previous quick tick

#define MB_MAX_CLICKS      3
#define MB_EV_HOLD         0
#define MB_RELAY_CHANNEL   1
#define MB_DEBOUNCE_MS     30
#define MB_GAP_MS          400          // multi-click window
#define MB_HOLD_MS         500          // relay toggle
#define MB_CONFIG_HOLD_MS  10000        // force config/AP mode

static const char *g_mbNames[MB_MAX_CLICKS + 1] = { "hold", "single", "double", "triple" };

static struct {
	int inited;
	int pin;
	int lastRaw;
	int stable;
	int debTimer;
	int pressTimer;
	int gapTimer;                       // -1 = idle
	int clicks;
	int holdFired;
	int configHoldFired;
	int totalEvents;
} mb;

static struct {
	int pin;                            // -1 = disabled
	int blinkTimer;
	int blinkOn;
} led;

static void MB_LedWrite(int lit) {
	HAL_PIN_SetOutputValue(led.pin, !lit);      // active low
}

static void MB_LedTick(int dt) {
	int period;

	if (led.pin < 0)
		return;

	if (Main_IsOpenAccessPointMode()) {
		period = 200;
	} else if (!Main_HasWiFiConnected() || !MQTT_IsReady()) {
		period = 500;
	} else {
		MB_LedWrite(0);
		led.blinkTimer = led.blinkOn = 0;
		return;
	}

	led.blinkTimer += dt;
	if (led.blinkTimer >= period) {
		led.blinkTimer = 0;
		led.blinkOn = !led.blinkOn;
		MB_LedWrite(led.blinkOn);
	}
}

static void MB_Fire(int idx) {
	char cmd[16];
	commandResult_t res;

	mb.totalEvents++;

	// local, works without WiFi/broker
	if (idx == MB_EV_HOLD)
		strcpy(cmd, "Lamp_Hold");
	else
		snprintf(cmd, sizeof(cmd), "Lamp_Click %i", idx);
	res = CMD_ExecuteCommand(cmd, COMMAND_FLAG_SOURCE_SCRIPT);
	if (res == CMD_RES_UNKNOWN_COMMAND && idx == MB_EV_HOLD)
		CHANNEL_Toggle(MB_RELAY_CHANNEL);   // LampIR driver not started

	addLogAdv(LOG_INFO, LOG_FEATURE_CMD, "MultiButton: event '%s'", g_mbNames[idx]);
	MQTT_PublishMain_StringString("button", g_mbNames[idx], 0);
}

// ---------------------------------------------------------------------------
//  State machine, called from the OBK main loop
// ---------------------------------------------------------------------------
void MultiButton_RunQuickTick(void) {
	int raw, dt = g_deltaTimeMS;

	if (dt <= 0)
		dt = 1;
	if (dt > 200)
		dt = 200;                       // ignore huge stalls (OTA, scan, ...)

	MB_LedTick(dt);                     // independent of the button being configured

	if (!mb.inited)
		return;

	raw = !HAL_PIN_ReadDigitalInput(mb.pin);    // 1 = pressed (active low)

	// ---- debounce -------------------------------------------------------
	if (raw != mb.lastRaw) {
		mb.lastRaw = raw;
		mb.debTimer = 0;
	} else if (mb.debTimer < MB_DEBOUNCE_MS) {
		mb.debTimer += dt;
		if (mb.debTimer >= MB_DEBOUNCE_MS && raw != mb.stable) {
			mb.stable = raw;
			if (raw) {
				mb.pressTimer = 0;
				mb.holdFired = 0;
				mb.configHoldFired = 0;
				mb.gapTimer = -1;       // freeze the click window while held
			} else if (mb.holdFired) {
				mb.clicks = 0;          // a hold is not a click
				mb.gapTimer = -1;
			} else {
				if (mb.clicks < MB_MAX_CLICKS)
					mb.clicks++;        // 4th+ click collapses into 'triple'
				mb.gapTimer = 0;        // start / restart the click window
			}
		}
	}

	// ---- held down ------------------------------------------------------
	if (mb.stable) {
		mb.pressTimer += dt;
		if (!mb.holdFired && mb.pressTimer >= MB_HOLD_MS) {
			mb.holdFired = 1;
			mb.clicks = 0;              // cancel any pending click sequence
			MB_Fire(MB_EV_HOLD);
		}
		if (!mb.configHoldFired && mb.pressTimer >= MB_CONFIG_HOLD_MS) {
			mb.configHoldFired = 1;
			addLogAdv(LOG_WARN, LOG_FEATURE_CMD,
			          "MultiButton: held %ims -> forcing config/AP mode", mb.pressTimer);
			CMD_ExecuteCommand("OpenAP", COMMAND_FLAG_SOURCE_SCRIPT);
		}
		return;
	}

	// ---- released: wait for the multi-click window to expire -------------
	if (mb.gapTimer >= 0) {
		mb.gapTimer += dt;
		if (mb.gapTimer >= MB_GAP_MS) {
			int c = mb.clicks;
			mb.gapTimer = -1;
			mb.clicks = 0;
			if (c > 0)
				MB_Fire(c);
		}
	}
}

// ---------------------------------------------------------------------------
//  Commands
// ---------------------------------------------------------------------------
static commandResult_t CMD_MB_Setup(const void *context, const char *cmd,
                                    const char *args, int cmdFlags) {
	Tokenizer_TokenizeString(args, 0);
	if (Tokenizer_GetArgsCount() < 1)
		return CMD_RES_NOT_ENOUGH_ARGUMENTS;

	mb.pin = Tokenizer_GetArgInteger(0);
	HAL_PIN_Setup_Input_Pullup(mb.pin);

	mb.lastRaw = mb.stable = 0;
	mb.debTimer = mb.pressTimer = mb.clicks = mb.holdFired = 0;
	mb.gapTimer = -1;
	mb.inited = 1;

	addLogAdv(LOG_INFO, LOG_FEATURE_CMD, "MultiButton: button on pin %i", mb.pin);
	return CMD_RES_OK;
}

static commandResult_t CMD_LED_Setup(const void *context, const char *cmd,
                                     const char *args, int cmdFlags) {
	Tokenizer_TokenizeString(args, 0);
	if (Tokenizer_GetArgsCount() < 1)
		return CMD_RES_NOT_ENOUGH_ARGUMENTS;

	led.pin = Tokenizer_GetArgInteger(0);
	led.blinkTimer = led.blinkOn = 0;
	HAL_PIN_Setup_Output(led.pin);
	MB_LedWrite(0);

	addLogAdv(LOG_INFO, LOG_FEATURE_CMD, "MultiButton: status LED on pin %i", led.pin);
	return CMD_RES_OK;
}

static commandResult_t CMD_MB_Status(const void *context, const char *cmd,
                                     const char *args, int cmdFlags) {
	addLogAdv(LOG_INFO, LOG_FEATURE_CMD,
	          "MultiButton: inited %i, pin %i, rawLevel %i, pressed %i, events %i",
	          mb.inited, mb.pin, mb.inited ? HAL_PIN_ReadDigitalInput(mb.pin) : -1,
	          mb.stable, mb.totalEvents);
	addLogAdv(LOG_INFO, LOG_FEATURE_CMD,
	          "MultiButton: status LED pin %i, AP mode %i, wifi %i, mqtt %i",
	          led.pin, Main_IsOpenAccessPointMode(), Main_HasWiFiConnected(), MQTT_IsReady());
	return CMD_RES_OK;
}

// ---------------------------------------------------------------------------
void MultiButton_Init(void) {
	memset(&mb, 0, sizeof(mb));
	mb.pin = -1;
	mb.gapTimer = -1;

	memset(&led, 0, sizeof(led));
	led.pin = -1;                       // disabled until LED_Setup is called

	CMD_RegisterCommand("MB_Setup",  CMD_MB_Setup,  NULL);
	CMD_RegisterCommand("MB_Status", CMD_MB_Status, NULL);
	CMD_RegisterCommand("LED_Setup", CMD_LED_Setup, NULL);

	addLogAdv(LOG_INFO, LOG_FEATURE_CMD,
	          "MultiButton driver started, call MB_Setup <pin> to bind a pin");
}

void MultiButton_StopDriver(void) {
	mb.inited = 0;
	if (led.pin >= 0)
		MB_LedWrite(0);
	led.pin = -1;
}
