/**
 * @file GarageControl2.ino
 * @brief Main Arduino sketch for the Garage Control System with Home Assistant MQTT integration.
 *
 * Board: Arduino UNO R4 WiFi (production) / Arduino UNO R4 Minima (dev)
 *
 * @section Hardware
 * - Door open sensor:        digital pin 2  (HIGH when fully open)
 * - Door closed sensor:      digital pin 3  (HIGH when fully closed)
 * - PIR motion sensor:       digital pin 4  (active HIGH)
 * - Garage door relay:       digital pin 5  (active HIGH)
 * - HVAC heater relay:       digital pin 6  (active HIGH)
 * - DS18B20 temp sensor (OneWire): digital pin 7
 * - Menu navigation buttons: pins 8/9/10   (active LOW, INPUT_PULLUP)
 * - HVAC cooler relay:       digital pin 11 (active HIGH)
 * - Garage light relay:      digital pin 13 (active HIGH)
 *
 * @section NV (Non-Volatile) Settings Design
 * Four values are persisted in EEPROM as "NV" (non-volatile) settings:
 *   - nvHeatSet       : HVAC heat setpoint
 *   - nvCoolSet       : HVAC cool setpoint
 *   - nvDoorTimeout   : Door auto-close duration (ms)
 *   - nvLightTimeout  : Light auto-off duration (ms)
 *
 * Rules governing NV vs current settings:
 *  1. On boot, loadNV() reads EEPROM and sets BOTH the NV members AND the
 *     live subsystem values (hvac.heatSet, door.autoCloseDuration, etc.).
 *  2. Changes to the CURRENT settings (menu or /cmd MQTT topics) do NOT
 *     write to EEPROM and do NOT touch the NV member variables.
 *  3. Changes to the NV settings (/nv/…/cmd MQTT topics) update ONLY the
 *     NV member variables and save to EEPROM; they do NOT touch the live
 *     subsystem values.
 *  4. reloadNV() (menu or /nv/reload/cmd) copies all four NV members back
 *     into the live subsystems, exactly as loadNV() does on boot.
 *
 * @section TemperatureSampling
 * The DS18B20 is read asynchronously to keep the main loop non-blocking:
 *   - sensors.setWaitForConversion(false) is called once in begin().
 *   - requestTemperatures() is issued every TEMP_INTERVAL_MS, returning
 *     immediately (no 750 ms stall).
 *   - After TEMP_CONVERSION_MS the result is collected with getTempFByIndex().
 * Motion detection and light activation run at the very top of every loop
 * iteration so they are never delayed by WiFi reconnection or temperature
 * conversion.
 *
 * @section Menu
 * - Main: status screen (door/light/HVAC/motion)
 * - SET: enter submenu
 *   - HVAC: enable/disable heat, adjust setpoint
 *   - Light: set auto-off timeout
 *   - Door: set auto-close timeout and retry attempts
 *   - Config: show network status, trigger reconnect
 */

#include "src/Utility.h"
#include <EEPROM.h>
const char *GC_VERSION = "2.13";

#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <OneWire.h>
#include <DallasTemperature.h>

#include "src/MQTT.h" // guards itself with #if ENABLE_WIFI

#include "src/HVAC.h"
#include "src/Motion.h"
#include "src/GarageDoor.h"
#include "src/GarageLight.h"
#include "src/MenuController.h"
#include "src/LcdController.h"

// ============================================================
//  Pin definitions
// ============================================================
const byte DoorOpenPin        = 2;  ///< HIGH when door fully open
const byte DoorClosedPin      = 3;  ///< HIGH when door fully closed
const byte PIRPin             = 4;  ///< PIR OUT – active HIGH on motion
const byte DoorButtonPin      = 5;  ///< Relay → garage door opener (active HIGH)
const byte HVACHeatPin        = 6;  ///< Relay → heater (active HIGH)
const byte HVACCoolPin        = 11; ///< Relay → cooler (active HIGH)
const byte HVACTempSensorPin  = 7;  ///< OneWire data (DS18B20)
const byte MenuBtnDownPin     = 8;  ///< Menu navigation (active LOW)
const byte MenuBtnSetPin      = 9;  ///< Menu select (active LOW)
const byte MenuBtnUpPin       = 10; ///< Menu navigation (active LOW)
const byte LightSwitchPin     = 13; ///< Relay → garage light (active HIGH)

// ============================================================
//  I2C LCD (4x20)
// ============================================================
const byte LCD_ADDR = 0x27;
const byte LCD_COLS = 20;
const byte LCD_ROWS = 4;

// ============================================================
//  Temperature sampling timing
// ============================================================
/** Interval between temperature requests (30 seconds). */
static constexpr unsigned long TEMP_INTERVAL_MS   = 30000UL;
/**
 * DS18B20 conversion time for 12-bit resolution.
 * The datasheet specifies ≤ 750 ms; 800 ms adds a small safety margin.
 */
static constexpr unsigned long TEMP_CONVERSION_MS = 800UL;

// ============================================================
//  Utility helpers (defined here; declared extern in Utility.h)
// ============================================================

/** @brief Returns millis() – thin wrapper used throughout subsystems. */
unsigned long now() { return millis(); }

/**
 * @brief Returns true when (now() - last) >= interval.
 *
 * Safe across the millis() 49.7-day rollover because unsigned subtraction
 * wraps correctly for intervals shorter than 2^31 ms.
 *
 * @param last     Reference timestamp from a prior now() call.
 * @param interval Desired period in milliseconds.
 */
bool expired(unsigned long last, unsigned long interval)
{
  return (now() - last) >= interval;
}

/**
 * @brief Encodes the current GarageDoor state as a compact uint8_t.
 *
 * Avoids heap-allocating a String in the main loop when publishing MQTT state.
 *
 * @param door  GarageDoor instance to query.
 * @return      0=open, 1=closed, 2=moving, 3=error, 4=disabled.
 */
uint8_t doorStateCode(const GarageDoor &door)
{
  switch (door.getState())
  {
  case GarageDoor::Open:     return 0;
  case GarageDoor::Closed:   return 1;
  case GarageDoor::Moving:   return 2;
  case GarageDoor::Error:    return 3;
  default:                   return 4; // Disabled
  }
}

// ============================================================
//  Forward declaration
// ============================================================
class GarageController;
GarageController *g_controller = nullptr;

// ============================================================
//  MAIN SYSTEM CONTROLLER
// ============================================================

/**
 * @class GarageController
 * @brief Top-level controller that owns and coordinates all subsystems.
 *
 * Subsystems:
 *   - MotionSensor  – PIR debouncing and relay-spike rejection
 *   - GarageLight   – relay-controlled light with auto-off
 *   - GarageDoor    – state machine, auto-close, retry logic
 *   - GarageHVAC    – heating/cooling with hysteresis and lockout
 *   - MenuController – debounced 3-button navigation
 *   - LcdController  – 4×20 I2C display rendering
 *   - MQTTManager   – WiFi + MQTT + HA discovery (ENABLE_WIFI only)
 *
 * @section NV Storage Layout
 * EEPROM layout (byte offsets):
 *   [0..3]  NV_MAGIC          – validity sentinel (0x47434E56)
 *   [4..7]  nvHeatSet         – float, heat setpoint (°F)
 *   [8..11] nvCoolSet         – float, cool setpoint (°F)
 *   [12..13] nvDoorTimeout    – uint16_t, door auto-close (minutes)
 *   [14..15] nvLightTimeout   – uint16_t, light auto-off  (minutes)
 *
 * @note All NV EEPROM reads/writes go through loadNV() / saveNV() only.
 *       No subsystem accesses EEPROM directly.
 */
class GarageController : public IMenuHost
{
public:
  // ── Subsystem objects ────────────────────────────────────────────────────
  MotionSensor          motion;
  GarageLight           lights;
  GarageDoor            door;
  GarageHVAC            hvac;
  MenuController        menu;
  LiquidCrystal_I2C     lcd;
  LcdController         lcdDisplay;
  OneWire               oneWire;
  DallasTemperature     sensors;

  // ── NV (EEPROM) settings ─────────────────────────────────────────────────
  /**
   * @defgroup NVSettings Non-volatile settings
   * These four members are the authoritative NV copies.  They are only
   * modified by saveNV() (write) and loadNV() (read).  Changing a live
   * subsystem value (e.g. hvac.heatSet) has NO effect on these members
   * and vice versa, except when reloadNV() is explicitly invoked.
   * @{
   */
  float         nvHeatSet     = 65.0f;          ///< NV HVAC heat setpoint (°F)
  float         nvCoolSet     = 85.0f;          ///< NV HVAC cool setpoint (°F)
  unsigned long nvDoorTimeout = 30UL * 60000UL; ///< NV door auto-close duration (ms)
  unsigned long nvLightTimeout= 20UL * 60000UL; ///< NV light auto-off duration (ms)
  /** @} */

  // ── Temperature state ─────────────────────────────────────────────────────
  float         tempF          = 0.0f;  ///< Last valid temperature reading (°F)
  unsigned long lastTempPoll   = 0;     ///< Timestamp of last requestTemperatures() call
  unsigned long lastTempRequest= 0;     ///< Timestamp when the last conversion was started
  bool          tempPending    = false; ///< True while waiting for DS18B20 conversion

  // ── NV EEPROM layout ──────────────────────────────────────────────────────
  static constexpr uint32_t NV_MAGIC          = 0x47434E56U; ///< 'GCNV' sentinel
  static constexpr int      NV_ADDR_MAGIC     = 0;
  static constexpr int      NV_ADDR_HEAT_SET  = NV_ADDR_MAGIC    + sizeof(NV_MAGIC);
  static constexpr int      NV_ADDR_COOL_SET  = NV_ADDR_HEAT_SET + sizeof(float);
  static constexpr int      NV_ADDR_DOOR_TIMEOUT  = NV_ADDR_COOL_SET    + sizeof(float);
  static constexpr int      NV_ADDR_LIGHT_TIMEOUT = NV_ADDR_DOOR_TIMEOUT + sizeof(uint16_t);

  // ── MQTT command debounce timestamps (ENABLE_WIFI only) ──────────────────
#if ENABLE_WIFI
  unsigned long lastDoorCmd  = 0;
  unsigned long lastLightCmd = 0;
  unsigned long lastHvacCmd  = 0;
#endif

#if ENABLE_WIFI
  MQTTManager mqttManager;
#endif

  // ── Constructor ───────────────────────────────────────────────────────────

  /**
   * @brief Constructs all subsystems with their hardware pin bindings.
   *
   * Subsystems are initialised in member-initialiser order.  Actual hardware
   * setup (pinMode, lcd.init, etc.) is deferred to begin().
   */
  GarageController()
      : motion(PIRPin),
        lights(LightSwitchPin, motion),
        door(DoorButtonPin, DoorOpenPin, DoorClosedPin, motion),
        hvac(HVACHeatPin, HVACCoolPin, motion),
        menu(MenuBtnUpPin, MenuBtnDownPin, MenuBtnSetPin),
        lcd(LCD_ADDR, LCD_COLS, LCD_ROWS),
        lcdDisplay(lcd, menu),
        oneWire(HVACTempSensorPin),
        sensors(&oneWire)
  {
  }

  // ── begin() ───────────────────────────────────────────────────────────────

  /**
   * @brief Initialises all subsystems and kicks off the first temperature read.
   *
   * Call once from Arduino setup().  Order matters:
   *   1. Serial + sensor hardware
   *   2. Menu and LCD (shows splash)
   *   3. NV load (or defaults + save)
   *   4. Async temperature request – result available after TEMP_CONVERSION_MS
   *   5. WiFi/MQTT init (ENABLE_WIFI only)
   */
  void begin()
  {
    Serial.println(F("Controller:Starting v2+HA"));

    // ── Temperature sensor ──────────────────────────────────────────────────
    sensors.begin();
    // Disable blocking wait so requestTemperatures() returns immediately.
    // The main loop collects the result after TEMP_CONVERSION_MS has elapsed.
    sensors.setWaitForConversion(false);

    // Kick off the very first conversion so tempF is valid within ~800 ms
    // rather than waiting the full TEMP_INTERVAL_MS (30 s) before boot.
    sensors.requestTemperatures();
    lastTempRequest = now();
    tempPending     = true;
    lastTempPoll    = now(); // start the 30-s interval from boot

    // ── Menu + LCD ───────────────────────────────────────────────────────────
    menu.begin();
    lcdDisplay.begin();
    lcdDisplay.SetDirty(true);
    Serial.println(F("Controller:LCD ready"));

    // ── NV settings ──────────────────────────────────────────────────────────
    if (loadNV())
    {
      Serial.println(F("Controller:Loaded NV settings"));
    }
    else
    {
      Serial.println(F("Controller:NV missing/invalid – using defaults"));
      saveNV();
    }

    // ── Network (production builds only) ─────────────────────────────────────
#if ENABLE_WIFI
    mqttManager.init(this, GarageController::mqttCallback);
#endif
  }

  // ── NV helpers ────────────────────────────────────────────────────────────

  /**
   * @brief Reads NV settings from EEPROM and applies them to all subsystems.
   *
   * Sets BOTH the four NV member variables AND the live subsystem values
   * (hvac.heatSet, hvac.coolSet, door.autoCloseDuration, lights.duration).
   * Called once on boot; also called by reloadNV().
   *
   * @return true  if EEPROM contained a valid NV block,
   *         false if magic was absent or values were out of range.
   */
  bool loadNV()
  {
    uint32_t magic = 0;
    EEPROM.get(NV_ADDR_MAGIC, magic);
    if (magic != NV_MAGIC)
      return false;

    float    heatSetNV       = 0.0f;
    float    coolSetNV       = 0.0f;
    uint16_t doorTimeoutMins = 0;
    uint16_t lightTimeoutMins= 0;

    EEPROM.get(NV_ADDR_HEAT_SET,      heatSetNV);
    EEPROM.get(NV_ADDR_COOL_SET,      coolSetNV);
    EEPROM.get(NV_ADDR_DOOR_TIMEOUT,  doorTimeoutMins);
    EEPROM.get(NV_ADDR_LIGHT_TIMEOUT, lightTimeoutMins);

    // Sanity-check all four values before accepting the NV block.
    if (heatSetNV      < 30.0f || heatSetNV      > 100.0f ||
        coolSetNV      < 30.0f || coolSetNV       > 100.0f ||
        doorTimeoutMins  < 1   || doorTimeoutMins  > 120   ||
        lightTimeoutMins < 1   || lightTimeoutMins > 120)
    {
      return false;
    }

    // ── Store the four NV members ─────────────────────────────────────────
    nvHeatSet      = heatSetNV;
    nvCoolSet      = coolSetNV;
    nvDoorTimeout  = (unsigned long)doorTimeoutMins  * 60000UL;
    nvLightTimeout = (unsigned long)lightTimeoutMins * 60000UL;

    // ── Apply NV values to live subsystems (Rule 1) ───────────────────────
    hvac.heatSet          = nvHeatSet;
    hvac.coolSet          = nvCoolSet;
    door.autoCloseDuration = nvDoorTimeout;
    lights.duration        = nvLightTimeout;

    return true;
  }

  /**
   * @brief Writes the four NV member variables to EEPROM.
   *
   * Saves nvHeatSet, nvCoolSet, nvDoorTimeout, and nvLightTimeout.
   * This function deliberately does NOT read live subsystem values
   * (hvac.heatSet, door.autoCloseDuration, etc.) – only the NV members
   * are persisted (Rule 2 / Rule 3).
   *
   * @return true always (future-proofed for write-error detection).
   */
  bool saveNV()
  {
    // Convert milliseconds to minutes for compact EEPROM storage.
    uint16_t doorMins  = (uint16_t)(nvDoorTimeout  / 60000UL);
    uint16_t lightMins = (uint16_t)(nvLightTimeout / 60000UL);

    // Clamp to valid range before writing.
    doorMins  = constrain(doorMins,  1, 120);
    lightMins = constrain(lightMins, 1, 120);

    EEPROM.put(NV_ADDR_MAGIC,         NV_MAGIC);
    EEPROM.put(NV_ADDR_HEAT_SET,      nvHeatSet);
    EEPROM.put(NV_ADDR_COOL_SET,      nvCoolSet);
    EEPROM.put(NV_ADDR_DOOR_TIMEOUT,  doorMins);
    EEPROM.put(NV_ADDR_LIGHT_TIMEOUT, lightMins);

#if defined(ESP8266) || defined(ESP32)
    EEPROM.commit();
#endif

    return true;
  }

  /**
   * @brief Restores all live subsystem values from the NV member variables.
   *
   * Implements Rule 4: behaves identically to the "apply" step inside
   * loadNV(), copying nvHeatSet, nvCoolSet, nvDoorTimeout, and
   * nvLightTimeout into the live subsystems.  Does NOT re-read EEPROM.
   *
   * Called by the ReloadNV menu screen (SET button) and the
   * /nv/reload/cmd MQTT topic.
   */
  void reloadNV()
  {
    hvac.heatSet           = nvHeatSet;
    hvac.coolSet           = nvCoolSet;
    door.autoCloseDuration = nvDoorTimeout;
    lights.duration        = nvLightTimeout;
    Serial.println(F("Controller:NV reloaded to live subsystems"));
  }

  // ── MQTT command handler (ENABLE_WIFI only) ───────────────────────────────
#if ENABLE_WIFI

  /**
   * @brief Dispatches an inbound MQTT command to the appropriate subsystem.
   *
   * This function is called from the static mqttCallback() after building a
   * String from the raw byte payload.  It uses strcmp() against pre-built
   * topic strings so no heap allocations occur here.
   *
   * NV-vs-current topic separation:
   *   /cmd    topics  → modify LIVE subsystem values only (no saveNV).
   *   /nv/…/cmd topics → modify NV member variables + saveNV (no live change).
   *
   * @param topic    Null-terminated MQTT topic string.
   * @param payload  Command payload as an Arduino String.
   */
  void handleMQTT(const char *topic, const String &payload)
  {
    Serial.print(F("MQTT IN ["));
    Serial.print(topic);
    Serial.print(F("] "));
    Serial.println(payload);

    // ── Door command (LIVE only) ──────────────────────────────────────────
    if (strcmp(topic, mqttManager.getTopic(F("/door/cmd"))) == 0)
    {
      // Debounce rapid toggles (2-second window).
      if (expired(lastDoorCmd, 2000UL))
      {
        door.manualActivate();
        lastDoorCmd = now();
        lcdDisplay.SetDirty(true);
      }
    }
    // ── Door current timeout (LIVE only – Rule 2) ─────────────────────────
    else if (strcmp(topic, mqttManager.getTopic(F("/door/duration/cmd"))) == 0)
    {
      int mins = payload.toInt();
      if (mins > 0 && mins <= 120)
      {
        // Update the live auto-close duration; NV is NOT changed.
        door.autoCloseDuration = (unsigned long)mins * 60000UL;
        lcdDisplay.SetDirty(false);
      }
    }
    // ── Light command (LIVE only) ─────────────────────────────────────────
    else if (strcmp(topic, mqttManager.getTopic(F("/light/cmd"))) == 0)
    {
      if (expired(lastLightCmd, 500UL))
      {
        if (payload == F("on"))
        {
          lights.turnOn();
          lcdDisplay.SetDirty(true);
        }
        else if (payload == F("off"))
        {
          lights.turnOff();
          lcdDisplay.SetDirty(false);
          lcdDisplay.setBacklight(false);
        }
        lastLightCmd = now();
      }
    }
    // ── Light current timeout (LIVE only – Rule 2) ────────────────────────
    else if (strcmp(topic, mqttManager.getTopic(F("/light/duration/cmd"))) == 0)
    {
      int mins = payload.toInt();
      if (mins > 0 && mins <= 120)
      {
        // Update the live light-off duration; NV is NOT changed.
        lights.duration = (unsigned long)mins * 60000UL;
        lcdDisplay.SetDirty(false);
      }
    }
    // ── HVAC heat setpoint (LIVE only – Rule 2) ───────────────────────────
    else if (strcmp(topic, mqttManager.getTopic(F("/hvac/heat_set/cmd"))) == 0)
    {
      if (expired(lastHvacCmd, 1000UL))
      {
        float val = payload.toFloat();
        if (val > 30 && val < 100)
        {
          // Update live setpoint only; nvHeatSet is NOT changed.
          hvac.heatSet = val;
          lcdDisplay.SetDirty(false);
        }
        lastHvacCmd = now();
      }
    }
    // ── HVAC mode (LIVE only) ─────────────────────────────────────────────
    else if (strcmp(topic, mqttManager.getTopic(F("/hvac/mode/cmd"))) == 0)
    {
      if (expired(lastHvacCmd, 1000UL))
      {
        if      (payload == "off")       hvac.mode = GarageHVAC::Off;
        else if (payload == "heat")      hvac.mode = GarageHVAC::Heat;
        else if (payload == "heat_cool") hvac.mode = GarageHVAC::Heat_Cool;
        else if (payload == "cool")      hvac.mode = GarageHVAC::Cool;
        lcdDisplay.SetDirty(false);
        lastHvacCmd = now();
      }
    }
    // ── HVAC cool setpoint (LIVE only – Rule 2) ───────────────────────────
    else if (strcmp(topic, mqttManager.getTopic(F("/hvac/cool_set/cmd"))) == 0)
    {
      if (expired(lastHvacCmd, 1000UL))
      {
        float val = payload.toFloat();
        if (val > 30 && val < 100)
        {
          // Update live setpoint only; nvCoolSet is NOT changed.
          hvac.coolSet = val;
          lcdDisplay.SetDirty(false);
        }
        lastHvacCmd = now();
      }
    }

    // ════════════════════════════════════════════════════════════════════════
    //  NV topics – update NV members + EEPROM ONLY (Rule 3).
    //  Live subsystem values are intentionally NOT touched here.
    //  Use /nv/reload/cmd (or the ReloadNV menu) to propagate to live values.
    // ════════════════════════════════════════════════════════════════════════

    // ── NV heat setpoint (NV only – Rule 3) ──────────────────────────────
    else if (strcmp(topic, mqttManager.getTopic(F("/nv/hvac/heat_set/cmd"))) == 0)
    {
      float val = payload.toFloat();
      if (val > 30 && val < 100)
      {
        nvHeatSet = val;  // NV member updated; hvac.heatSet is NOT changed.
        saveNV();
        lcdDisplay.SetDirty(false);
      }
    }
    // ── NV cool setpoint (NV only – Rule 3) ──────────────────────────────
    else if (strcmp(topic, mqttManager.getTopic(F("/nv/hvac/cool_set/cmd"))) == 0)
    {
      float val = payload.toFloat();
      if (val > 30 && val < 100)
      {
        nvCoolSet = val;  // NV member updated; hvac.coolSet is NOT changed.
        saveNV();
        lcdDisplay.SetDirty(false);
      }
    }
    // ── NV door timeout (NV only – Rule 3) ───────────────────────────────
    else if (strcmp(topic, mqttManager.getTopic(F("/nv/door_timeout/cmd"))) == 0)
    {
      int mins = payload.toInt();
      if (mins > 0 && mins <= 120)
      {
        // Update NV member and persist; door.autoCloseDuration is NOT touched.
        nvDoorTimeout = (unsigned long)mins * 60000UL;
        saveNV();
        lcdDisplay.SetDirty(false);
      }
    }
    // ── NV light timeout (NV only – Rule 3) ──────────────────────────────
    else if (strcmp(topic, mqttManager.getTopic(F("/nv/light_timeout/cmd"))) == 0)
    {
      int mins = payload.toInt();
      if (mins > 0 && mins <= 120)
      {
        // Update NV member and persist; lights.duration is NOT touched.
        nvLightTimeout = (unsigned long)mins * 60000UL;
        saveNV();
        lcdDisplay.SetDirty(false);
      }
    }
    // ── Reload NV → live subsystems (Rule 4) ─────────────────────────────
    else if (strcmp(topic, mqttManager.getTopic(F("/nv/reload/cmd"))) == 0)
    {
      reloadNV();
      lcdDisplay.SetDirty(false);
    }
  }

  /**
   * @brief Static PubSubClient callback – forwards to the controller instance.
   *
   * PubSubClient requires a static (or free) function pointer.  This shim
   * builds a String from the raw byte payload once and delegates to the
   * instance method handleMQTT().
   *
   * @param topic   Null-terminated topic string from broker.
   * @param payload Raw payload bytes (not null-terminated).
   * @param length  Payload byte count.
   */
  static void mqttCallback(char *topic, byte *payload, unsigned int length)
  {
    String p;
    p.reserve(length);
    for (unsigned int i = 0; i < length; i++)
      p += (char)payload[i];
    if (g_controller)
      g_controller->handleMQTT(topic, p);
  }
#endif // ENABLE_WIFI

  // ── Main loop ─────────────────────────────────────────────────────────────

  /**
   * @brief Executes one iteration of the main control loop.
   *
   * Called repeatedly from Arduino loop().  The order of operations is
   * deliberately chosen to minimise latency for time-critical paths:
   *
   *  1. Motion poll + light activation  ← FIRST, never delayed
   *  2. Light timeout check
   *  3. Backlight auto-off on light-off transition
   *  4. MQTT loop (may reconnect WiFi – can block up to ~5 s)
   *  5. Door state machine
   *  6. HVAC lockout update
   *  7. Async temperature collection (non-blocking)
   *  8. Menu button poll
   *  9. MQTT state publish (change-detected, no heap allocations)
   * 10. LCD display update (dirty-flag gated)
   *
   * @note *this is passed to menu.poll() – do NOT use the global
   *       `controller` variable here; that would be a scope error even
   *       though both refer to the same object with the current single-
   *       instance design.
   */
  void loop()
  {
    // ── 1. Motion detection + light activation ────────────────────────────
    // Poll the PIR sensor at the very top of every loop so that relay
    // switching, WiFi reconnection, or temperature conversion never delays
    // the light-on response when someone enters the garage.
    bool motionDetected = motion.poll();

    if (motionDetected)
    {
      lights.turnOn();
      Serial.println(F("Controller:Motion->light on"));
      if (!lcdDisplay.isBacklightOn())
        lcdDisplay.setBacklight(true);
    }

    // ── 2. Light auto-off timeout check ──────────────────────────────────
    bool lightWasOn = lights.isOn();
    lights.poll();

    // ── 3. Backlight follows light state ─────────────────────────────────
    // If the garage light just timed out or was turned off, dim the LCD.
    if (lightWasOn && !lights.isOn())
    {
      Serial.println(F("Controller:Light off->backlight off"));
      lcdDisplay.setBacklight(false);
    }

    // ── 4. Network / MQTT heartbeat ──────────────────────────────────────
    // Placed AFTER motion/light so a WiFi reconnect (up to ~5 s) does not
    // delay the PIR-to-light response.
#if ENABLE_WIFI
    mqttManager.loop();
#endif

    // ── 5. Door state machine ─────────────────────────────────────────────
    GarageDoor::State doorState = door.poll(motionDetected);

    // ── 6. HVAC lockout: block heating/cooling when door is open ─────────
    hvac.lockout = (doorState != GarageDoor::Closed);

    // ── 7. Async temperature sampling ────────────────────────────────────
    //
    // Phase A – Start a new conversion every TEMP_INTERVAL_MS.
    //   requestTemperatures() returns immediately because
    //   setWaitForConversion(false) was called in begin().
    if (!tempPending && expired(lastTempPoll, TEMP_INTERVAL_MS))
    {
      sensors.requestTemperatures();
      lastTempRequest = now();
      tempPending     = true;
    }

    // Phase B – Collect the result once the DS18B20 has finished converting.
    //   12-bit conversion takes ≤ 750 ms; TEMP_CONVERSION_MS adds margin.
    if (tempPending && expired(lastTempRequest, TEMP_CONVERSION_MS))
    {
      float newTemp = sensors.getTempFByIndex(0);
      // Reject sentinel error values (DallasTemperature returns –196.6 °F
      // for DEVICE_DISCONNECTED and similar constants for faults).
      if (!isnan(newTemp) && newTemp > -100.0f)
        tempF = newTemp;

      lastTempPoll = now(); // restart the 30-s interval from this read
      tempPending  = false;

      // Run HVAC switching logic with the fresh reading.
      hvac.poll(tempF);
      lcdDisplay.SetDirty(false);
    }

    // ── 8. Menu button poll ───────────────────────────────────────────────
    // Pass *this (not the global `controller`) to avoid a scope ambiguity:
    // inside a GarageController member function, `controller` resolves to
    // the file-scope global, not the current instance.  Using *this is
    // explicit and correct regardless of how many instances exist.
    bool menuEvent = menu.poll(*this, hvac, lights, door);
    if (menuEvent)
      lcdDisplay.SetDirty(true);

    // ── 9. MQTT state publish ─────────────────────────────────────────────
    // Pass NV timeouts separately so HA sees independent current vs NV
    // sensor entities and they can diverge when the user changes one without
    // immediately reloading.
#if ENABLE_WIFI
    mqttManager.publishStateChanges(
        lights.isOn(),
        lights.duration        / 60000UL,  // current light timeout (min)
        lights.lightRemaining() / 60000UL,  // time until auto-off (min)
        doorStateCode(door),                // uint8_t – no String allocation
        door.autoCloseDuration / 60000UL,  // current door timeout (min)
        door.getDoorRemainingTime() / 60000UL,
        tempF,
        hvac.heatSet,
        hvac.coolSet,
        nvHeatSet,
        nvCoolSet,
        nvDoorTimeout  / 60000UL,          // NV door timeout (min)
        nvLightTimeout / 60000UL,          // NV light timeout (min)
        (uint8_t)hvac.mode,
        (uint8_t)hvac.state,
        motion.isActive(),
        hvac.lockout);
#endif

    // ── 10. LCD display update ────────────────────────────────────────────
    lcdDisplay.updateDisplay(hvac, door, lights, tempF);
  }
};

// ── Global instance ──────────────────────────────────────────────────────────
GarageController controller;

// ============================================================
//  Arduino entry points
// ============================================================

/** @brief One-time hardware and subsystem initialisation. */
void setup()
{
  Serial.begin(9600);
  delay(2000);
  g_controller = &controller;
  controller.begin();
}

/** @brief Delegates to GarageController::loop() every iteration. */
void loop()
{
  controller.loop();
}
