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
 * Seven values are persisted in EEPROM as "NV" (non-volatile) settings:
 *   - nvHeatSet         : HVAC heat setpoint
 *   - nvCoolSet         : HVAC cool setpoint
 *   - nvHVACSwing       : HVAC hysteresis swing
 *   - nvHVACMinRunTime  : HVAC minimum runtime (minutes)
 *   - nvHVACMinRestTime : HVAC minimum rest time (minutes)
 *   - nvDoorTimeout     : Door auto-close duration (ms)
 *   - nvLightTimeout    : Light auto-off duration (ms)
 *
 * @section NVModel NV vs Live settings — the five rules
 *
 *  Rule 1 – Boot:
 *    loadNV() reads EEPROM → writes NV member variables → writes live subsystem
 *    values.  All three layers are in sync immediately after boot.
 *
 *  Rule 2 – Live changes:
 *    Any update to a live subsystem value (via menu edit or MQTT /cmd topics)
 *    touches ONLY that subsystem.  NV members and EEPROM are NOT affected.
 *
 *  Rule 3 – NV changes:
 *    MQTT /nv/ topics and the new SetNV menu screens update ONLY the NV member
 *    variables (in RAM).  Live subsystem values and EEPROM are NOT affected.
 *    Changes accumulate in RAM until an explicit Save is issued.
 *
 *  Rule 4 – Saving NV:
 *    SaveNV() (menu) copies the current LIVE values → NV members → EEPROM.
 *    saveNV() (MQTT /nv/save/cmd or boot default) writes the current NV
 *    member variables to EEPROM without touching live values.
 *    Neither path alters live subsystem values.
 *
 *  Rule 5 – Reload NV:
 *    LoadNV() copies NV members → live subsystem values only; EEPROM is not
 *    re-read and NV members are not modified.
 *
 * @section NVMenu SetNV Values menu (v2.17.0)
 * A new "Set NV Values" sub-menu was added inside the Config section,
 * accessible via DOWN from NetworkInfo.  It provides one edit screen per NV
 * parameter (heat setpoint, cool setpoint, swing, min run time, min rest time,
 * door timeout, light timeout).  Changes go through the IMenuHost adjNv*()
 * mutators which clamp values to valid ranges and update in-RAM NV members only.
 * EEPROM is not written until the user explicitly navigates to SaveNV or sends
 * the MQTT /nv/save/cmd.
 *
 * @section TemperatureSampling
 * The DS18B20 is read asynchronously to keep the main loop non-blocking:
 *   - sensors.setWaitForConversion(false) is called once in begin().
 *   - requestTemperatures() is issued every TEMP_INTERVAL_MS, returning immediately.
 *   - After TEMP_CONVERSION_MS the result is collected with getTempFByIndex().
 *
 */

#include "src/Utility.h"
#include <EEPROM.h>
const char *GC_VERSION = "2.19.3";

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

/** @brief Returns millis() — thin wrapper used throughout subsystems. */
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
 * @param door GarageDoor instance to query.
 * @return 0=open, 1=closed, 2=moving, 3=error, 4=disabled.
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

// Forward declare pirISR() so it can be used in begin()
void pirISR();

// ============================================================
//  MAIN SYSTEM CONTROLLER
// ============================================================

/**
 * @class GarageController
 * @brief Top-level controller that owns and coordinates all subsystems.
 *
 * Subsystems:
 *   - MotionSensor   – PIR debouncing and relay-spike rejection
 *   - GarageLight    – relay-controlled light with auto-off
 *   - GarageDoor     – state machine, auto-close, retry logic
 *   - GarageHVAC     – heating/cooling with hysteresis and lockout
 *   - MenuController – debounced 3-button navigation
 *   - LcdController  – 4×20 I2C display rendering
 *   - MQTTManager    – WiFi + MQTT + HA discovery (ENABLE_WIFI only)
 *
 * @section IMenuHostImpl IMenuHost implementation
 * GarageController implements IMenuHost to expose:
 *   - LoadNV() / SaveNV() for menu commit operations
 *   - getNv*() getters used by LcdController to display NV values on
 *     the SetNV edit screens
 *   - adjNv*() mutators used by MenuController to increment/decrement
 *     NV members in edit mode without touching live values or EEPROM
 *
 * @section NV Storage Layout
 * EEPROM layout (byte offsets):
 *   [0..3]   NV_MAGIC           – validity sentinel ('GCNV')
 *   [4..7]   nvHeatSet          – float, heat setpoint (°F)
 *   [8..11]  nvCoolSet          – float, cool setpoint (°F)
 *   [12..13] nvDoorTimeout      – uint16_t, door auto-close (minutes)
 *   [14..15] nvLightTimeout     – uint16_t, light auto-off  (minutes)
 *   [16..17] nvHVACSwing        – uint16_t, HVAC hysteresis swing (°F)
 *   [18..19] nvHVACMinRunTime   – uint16_t, HVAC minimum run time (minutes)
 *   [20..21] nvHVACMinRestTime  – uint16_t, HVAC minimum rest time (minutes)
 */
class GarageController : public IMenuHost
{
public:
  // ── Subsystem objects ─────────────────────────────────────────────────────
  MotionSensor      motion;
  GarageLight       lights;
  GarageDoor        door;
  GarageHVAC        hvac;
  MenuController    menu;
  LiquidCrystal_I2C lcd;
  LcdController     lcdDisplay;
  OneWire           oneWire;
  DallasTemperature sensors;

  // ── NV (EEPROM) settings ─────────────────────────────────────────────────
  /**
   * @defgroup NVSettings Non-volatile settings (RAM mirrors of EEPROM)
   *
   * Updated by:
   *   - loadNV()            on boot (reads from EEPROM)
   *   - adjNv*() mutators   from the SetNV menu screens (Rule 3 — RAM only)
   *   - MQTT /nv/ topics    (Rule 3 — RAM only)
   *   - SaveNV()            from the menu (copies live → NV → EEPROM)
   *
   * NOT updated by live subsystem changes (Rule 2).
   * NOT auto-saved to EEPROM on change (Rule 3).
   * EEPROM written only by saveNV() (Rule 4).
   * @{
   */
  float         nvHeatSet         = 65.0f;          ///< NV HVAC heat setpoint (°F)
  float         nvCoolSet         = 85.0f;          ///< NV HVAC cool setpoint (°F)
  uint16_t      nvHVACSwing       = 1;              ///< NV HVAC hysteresis swing (°F)
  uint16_t      nvHVACMinRunTime  = 2;              ///< NV HVAC minimum runtime (minutes)
  uint16_t      nvHVACMinRestTime = 5;              ///< NV HVAC minimum rest time (minutes)
  unsigned long nvDoorTimeout     = 30UL * 60000UL; ///< NV door auto-close duration (ms)
  unsigned long nvLightTimeout    = 20UL * 60000UL; ///< NV light auto-off duration (ms)
  /** @} */

  // ── Temperature state ─────────────────────────────────────────────────────
  float         tempF           = 0.0f;  ///< Last valid temperature reading (°F)
  unsigned long lastTempPoll    = 0;     ///< Timestamp of last requestTemperatures() call
  unsigned long lastTempRequest = 0;     ///< Timestamp when the last conversion started
  bool          tempPending     = false; ///< True while waiting for DS18B20 conversion

  // ── NV EEPROM layout ──────────────────────────────────────────────────────
  static constexpr uint32_t NV_MAGIC              = 0x47434E56U; ///< 'GCNV' sentinel
  static constexpr int      NV_ADDR_MAGIC         = 0;
  static constexpr int      NV_ADDR_HEAT_SET      = NV_ADDR_MAGIC         + sizeof(NV_MAGIC);
  static constexpr int      NV_ADDR_COOL_SET      = NV_ADDR_HEAT_SET      + sizeof(float);
  static constexpr int      NV_ADDR_DOOR_TIMEOUT  = NV_ADDR_COOL_SET      + sizeof(float);
  static constexpr int      NV_ADDR_LIGHT_TIMEOUT = NV_ADDR_DOOR_TIMEOUT  + sizeof(uint16_t);
  static constexpr int      NV_ADDR_HVAC_SWING    = NV_ADDR_LIGHT_TIMEOUT + sizeof(uint16_t);
  static constexpr int      NV_ADDR_HVAC_MIN_RUN  = NV_ADDR_HVAC_SWING    + sizeof(uint16_t);
  static constexpr int      NV_ADDR_HVAC_MIN_REST = NV_ADDR_HVAC_MIN_RUN  + sizeof(uint16_t);

  // ── MQTT command debounce timestamps (ENABLE_WIFI only) ──────────────────
#if ENABLE_WIFI
  unsigned long lastDoorCmd  = 0;
  unsigned long lastLightCmd = 0;
  unsigned long lastHvacCmd  = 0;
#endif

  // ── Auto-revert tracking ──────────────────────────────────────────────────
  /**
   * @brief Timestamp when a live value last diverged from its NV counterpart.
   * 
   * When any of the four adjustable live values (heatSet, coolSet, doorTimeout,
   * lightTimeout) changes via menu or MQTT, this timestamp is recorded. If 24
   * hours pass without another change, all four values revert to their NV
   * counterparts. Zero means no active divergence.
   */
  unsigned long lastLiveChangeTime = 0;
  static constexpr unsigned long AUTO_REVERT_INTERVAL_MS = 24UL * 60UL * 60UL * 1000UL; ///< 24 hours

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
   * @brief Initialises all subsystems.
   *
   * Call once from Arduino setup().  Order matters:
   *   1. Serial + sensor hardware
   *   2. Menu and LCD (shows splash)
   *   3. Wire LcdController to this (IMenuHost) for NV value display
   *   4. NV load (or defaults + save)
   *   5. Async temperature request
   *   6. WiFi/MQTT init (ENABLE_WIFI only)
   */
  void begin()
  {
    Serial.println(F("Controller:Starting v2+HA"));

    // ── Temperature sensor ──────────────────────────────────────────────────
    sensors.begin();
    // Disable blocking wait; main loop collects result after TEMP_CONVERSION_MS.
    sensors.setWaitForConversion(false);

    // Kick off the first conversion immediately so tempF is valid within ~800 ms
    sensors.requestTemperatures();
    lastTempRequest = now();
    tempPending     = true;
    lastTempPoll    = now();

    // ── Menu + LCD ───────────────────────────────────────────────────────────
    menu.begin();
    lcdDisplay.begin();

    // Wire the IMenuHost pointer so LcdController can query NV values for
    // the SetNV edit screens.  Must be called before the first updateDisplay().
    lcdDisplay.setHost(this);

    lcdDisplay.SetDirty(true);
    Serial.println(F("Controller:LCD ready"));

    // ── PIR hardware interrupt ──────────────────────────────────────────────
    // Attach hardware interrupt to force lights on immediately, even during
    // MQTT reconnection. The 15-second cooldown after manual turn-off blocks
    // interrupt processing to allow occupants to exit without reactivation.
    attachInterrupt(digitalPinToInterrupt(PIRPin), pirISR, RISING);
    Serial.println(F("Controller:PIR interrupt attached"));

    // ── NV settings ──────────────────────────────────────────────────────────
    if (loadNV())
    {
      Serial.println(F("Controller:Loaded NV settings"));
    }
    else
    {
      Serial.println(F("Controller:NV missing/invalid - using defaults"));
      saveNV(); // persist compiled defaults
    }

    // ── Network (production builds only) ─────────────────────────────────────
#if ENABLE_WIFI
    mqttManager.init(this, GarageController::mqttCallback);
#endif
  }

  // ══════════════════════════════════════════════════════════════════════════
  //  IMenuHost – NV getter implementations
  //  Used by LcdController::updateDisplay() to display NV values on
  //  the SetNV edit screens without coupling LcdController to GarageController.
  // ══════════════════════════════════════════════════════════════════════════

  /** @brief Returns the current in-RAM NV heat setpoint (°F). */
  float getNvHeatSet() const override { return nvHeatSet; }

  /** @brief Returns the current in-RAM NV cool setpoint (°F). */
  float getNvCoolSet() const override { return nvCoolSet; }

  /** @brief Returns the current in-RAM NV HVAC hysteresis swing (°F). */
  int getNvSwing() const override { return (int)nvHVACSwing; }

  /** @brief Returns the current in-RAM NV minimum HVAC run time (minutes). */
  uint16_t getNvMinRunTime() const override { return nvHVACMinRunTime; }

  /** @brief Returns the current in-RAM NV minimum HVAC rest time (minutes). */
  uint16_t getNvMinRestTime() const override { return nvHVACMinRestTime; }

  /** @brief Returns the current in-RAM NV door auto-close duration (ms). */
  unsigned long getNvDoorTimeout() const override { return nvDoorTimeout; }

  /** @brief Returns the current in-RAM NV light auto-off duration (ms). */
  unsigned long getNvLightTimeout() const override { return nvLightTimeout; }

  // ══════════════════════════════════════════════════════════════════════════
  //  Auto-revert helpers – track and apply 24h timeout for live value changes
  // ══════════════════════════════════════════════════════════════════════════

  /**
   * @brief Records that a live value has been modified (diverged from NV).
   *
   * Called whenever hvac.heatSet, hvac.coolSet, door.autoCloseDuration, or
   * lights.duration are changed via menu or MQTT. Starts the 24-hour auto-revert
   * timer. Once 24 hours pass without another change, all four values revert to
   * their NV counterparts via checkAndRevertAutoValues().
   *
   * @note NOT called during LoadNV() or initialization.
   */
  void notifyLiveValueChanged() override
  {
    lastLiveChangeTime = millis();
  }

  /**
   * @brief Checks if 24 hours have passed since the last live value change.
   *
   * If the timeout has expired AND any of the four values (heatSet, coolSet,
   * doorTimeout, lightTimeout) differ from their NV counterparts, all four are
   * reverted to NV values. The timer is then cleared.
   *
   * Called periodically from loop() to detect and apply the auto-revert.
   */
  void checkAndRevertAutoValues()
  {
    // Skip if timer is not active
    if (lastLiveChangeTime == 0)
      return;

    unsigned long now = millis();
    
    // Check if 24 hours have passed
    if (now - lastLiveChangeTime > AUTO_REVERT_INTERVAL_MS)
    {
      // Check if any live value differs from NV
      if (hvac.heatSet != nvHeatSet ||
          hvac.coolSet != nvCoolSet ||
          door.autoCloseDuration != nvDoorTimeout ||
          lights.duration != nvLightTimeout)
      {
        // Revert all to NV values
        hvac.heatSet           = nvHeatSet;
        hvac.coolSet           = nvCoolSet;
        door.autoCloseDuration = nvDoorTimeout;
        lights.duration        = nvLightTimeout;
        
        Serial.println(F("Auto-revert: Live values reverted to NV after 24h"));
        lcdDisplay.SetDirty(true);
      }
      
      // Clear the timer
      lastLiveChangeTime = 0;
    }
  }

  // ══════════════════════════════════════════════════════════════════════════
  //  IMenuHost – NV mutator implementations (Rule 3: RAM only, no EEPROM)
  //  Called by MenuController::handleUp() / handleDown() when a SetNV
  //  screen is in EditMode.  Each mutator clamps to valid bounds.
  // ══════════════════════════════════════════════════════════════════════════

  /**
   * @brief Adjusts the NV heat setpoint by delta °F; clamps to [30, 100].
   *
   * Does NOT touch hvac.heatSet (live value) or EEPROM.
   * @param delta Amount to add (+1 for UP, -1 for DOWN from menu).
   */
  void adjNvHeatSet(float delta) override
  {
    nvHeatSet = constrain(nvHeatSet + delta, 30.0f, 100.0f);
    Serial.println(F("NV:heat_set adjusted (menu, unsaved)"));
  }

  /**
   * @brief Adjusts the NV cool setpoint by delta °F; clamps to [30, 100].
   *
   * Does NOT touch hvac.coolSet (live value) or EEPROM.
   * @param delta Amount to add (+1 for UP, -1 for DOWN from menu).
   */
  void adjNvCoolSet(float delta) override
  {
    nvCoolSet = constrain(nvCoolSet + delta, 30.0f, 100.0f);
    Serial.println(F("NV:cool_set adjusted (menu, unsaved)"));
  }

  /**
   * @brief Adjusts the NV HVAC swing by delta °F; clamps to [0, 20].
   *
   * Does NOT touch hvac.HVACSwing (live value) or EEPROM.
   * @param delta Amount to add (+1 or -1 from menu).
   */
  void adjNvSwing(int delta) override
  {
    nvHVACSwing = (uint16_t)constrain((int)nvHVACSwing + delta, 0, 20);
    Serial.println(F("NV:swing adjusted (menu, unsaved)"));
  }

  /**
   * @brief Adjusts the NV minimum HVAC run time by delta minutes; clamps to [1, 120].
   *
   * Does NOT touch hvac.minRunTimeMins (live value) or EEPROM.
   * @param delta Amount to add (+1 or -1 from menu).
   */
  void adjNvMinRunTime(int delta) override
  {
    int v = (int)nvHVACMinRunTime + delta;
    nvHVACMinRunTime = (uint16_t)constrain(v, 1, 120);
    Serial.println(F("NV:minRunTime adjusted (menu, unsaved)"));
  }

  /**
   * @brief Adjusts the NV minimum HVAC rest time by delta minutes; clamps to [0, 120].
   *
   * Does NOT touch hvac.minRestTimeMins (live value) or EEPROM.
   * @param delta Amount to add (+1 or -1 from menu).
   */
  void adjNvMinRestTime(int delta) override
  {
    int v = (int)nvHVACMinRestTime + delta;
    nvHVACMinRestTime = (uint16_t)constrain(v, 0, 120);
    Serial.println(F("NV:minRestTime adjusted (menu, unsaved)"));
  }

  /**
   * @brief Adjusts the NV door auto-close timeout by deltaMs; clamps to [1, 120] minutes.
   *
   * deltaMs should be a multiple of 60 000 (one minute step).
   * Does NOT touch door.autoCloseDuration (live value) or EEPROM.
   *
   * @param deltaMs Milliseconds to add (+60000 for UP, -60000 for DOWN from menu).
   */
  void adjNvDoorTimeout(long deltaMs) override
  {
    long mins = (long)(nvDoorTimeout / 60000UL) + (deltaMs / 60000L);
    mins = constrain(mins, 1L, 120L);
    nvDoorTimeout = (unsigned long)mins * 60000UL;
    Serial.println(F("NV:doorTimeout adjusted (menu, unsaved)"));
  }

  /**
   * @brief Adjusts the NV light auto-off timeout by deltaMs; clamps to [1, 120] minutes.
   *
   * deltaMs should be a multiple of 60 000 (one minute step).
   * Does NOT touch lights.duration (live value) or EEPROM.
   *
   * @param deltaMs Milliseconds to add (+60000 for UP, -60000 for DOWN from menu).
   */
  void adjNvLightTimeout(long deltaMs) override
  {
    long mins = (long)(nvLightTimeout / 60000UL) + (deltaMs / 60000L);
    mins = constrain(mins, 1L, 120L);
    nvLightTimeout = (unsigned long)mins * 60000UL;
    Serial.println(F("NV:lightTimeout adjusted (menu, unsaved)"));
  }

  // ── NV helpers ────────────────────────────────────────────────────────────

  /**
   * @brief Reads NV settings from EEPROM and applies them to all subsystems.
   *
   * Implements Rule 1 (boot sync): EEPROM → NV members → live subsystem values.
   *
   * @return true  if EEPROM contained a valid NV block with in-range values.
   * @return false if magic was absent or any value failed the sanity check.
   */
  bool loadNV()
  {
    uint32_t magic = 0;
    EEPROM.get(NV_ADDR_MAGIC, magic);
    if (magic != NV_MAGIC) return false;

    float    heatSetNV        = 0.0f;
    float    coolSetNV        = 0.0f;
    uint16_t hvacSwingNV      = 0;
    uint16_t minRunNV         = 0;
    uint16_t minRestNV        = 0;
    uint16_t doorTimeoutMins  = 0;
    uint16_t lightTimeoutMins = 0;

    EEPROM.get(NV_ADDR_HEAT_SET,      heatSetNV);
    EEPROM.get(NV_ADDR_COOL_SET,      coolSetNV);
    EEPROM.get(NV_ADDR_DOOR_TIMEOUT,  doorTimeoutMins);
    EEPROM.get(NV_ADDR_LIGHT_TIMEOUT, lightTimeoutMins);
    EEPROM.get(NV_ADDR_HVAC_SWING,    hvacSwingNV);
    EEPROM.get(NV_ADDR_HVAC_MIN_RUN,  minRunNV);
    EEPROM.get(NV_ADDR_HVAC_MIN_REST, minRestNV);

    // Sanity-check all values before accepting the NV block.
    if (heatSetNV        < 30.0f || heatSetNV        > 100.0f ||
        coolSetNV        < 30.0f || coolSetNV        > 100.0f ||
        doorTimeoutMins  < 1     || doorTimeoutMins  > 120    ||
        lightTimeoutMins < 1     || lightTimeoutMins > 120    ||
        hvacSwingNV      > 20    ||
        minRunNV         < 1     || minRunNV         > 120    ||
        minRestNV        > 120)
    {
      return false;
    }

    // ── Write NV member variables (Rule 1 – NV layer) ────────────────────
    nvHeatSet         = heatSetNV;
    nvCoolSet         = coolSetNV;
    nvHVACSwing       = hvacSwingNV;
    nvHVACMinRunTime  = minRunNV;
    nvHVACMinRestTime = minRestNV;
    nvDoorTimeout     = (unsigned long)doorTimeoutMins  * 60000UL;
    nvLightTimeout    = (unsigned long)lightTimeoutMins * 60000UL;

    // ── Apply NV values to live subsystems (Rule 1 – live layer) ─────────
    hvac.heatSet           = nvHeatSet;
    hvac.coolSet           = nvCoolSet;
    hvac.HVACSwing         = nvHVACSwing;
    hvac.minRunTimeMins    = nvHVACMinRunTime;
    hvac.minRestTimeMins   = nvHVACMinRestTime;
    door.autoCloseDuration = nvDoorTimeout;
    lights.duration        = nvLightTimeout;

    return true;
  }

  /**
   * @brief Writes the current NV member variables to EEPROM.
   *
   * Implements Rule 4 (NV→EEPROM path for MQTT /nv/save/cmd and boot defaults).
   * Does NOT read live subsystem values; only NV members are written.
   * EEPROM.put() skips the physical write if the value is unchanged, preserving
   * EEPROM write-cycle lifetime.
   *
   * @return true always (reserved for future write-error detection).
   */
  bool saveNV()
  {
    // Convert milliseconds to minutes for compact EEPROM storage.
    uint16_t doorMins    = (uint16_t)constrain((long)(nvDoorTimeout  / 60000UL), 1L, 120L);
    uint16_t lightMins   = (uint16_t)constrain((long)(nvLightTimeout / 60000UL), 1L, 120L);
    uint16_t hvacSwing   = (uint16_t)constrain((int)nvHVACSwing,        0,  20);
    uint16_t minRunMins  = (uint16_t)constrain((int)nvHVACMinRunTime,   1, 120);
    uint16_t minRestMins = (uint16_t)constrain((int)nvHVACMinRestTime,  0, 120);

    EEPROM.put(NV_ADDR_MAGIC,         NV_MAGIC);
    EEPROM.put(NV_ADDR_HEAT_SET,      nvHeatSet);
    EEPROM.put(NV_ADDR_COOL_SET,      nvCoolSet);
    EEPROM.put(NV_ADDR_DOOR_TIMEOUT,  doorMins);
    EEPROM.put(NV_ADDR_LIGHT_TIMEOUT, lightMins);
    EEPROM.put(NV_ADDR_HVAC_SWING,    hvacSwing);
    EEPROM.put(NV_ADDR_HVAC_MIN_RUN,  minRunMins);
    EEPROM.put(NV_ADDR_HVAC_MIN_REST, minRestMins);

#if defined(ESP8266) || defined(ESP32)
    EEPROM.commit();
#endif

    Serial.println(F("Controller:NV saved to EEPROM"));
    return true;
  }

  /**
   * @brief Copies current live subsystem values → NV members → EEPROM.
   *
   * IMenuHost::SaveNV() implementation — called by the SaveNV menu screen.
   * This is the natural "commit what I just configured via the live menus" action.
   * Live subsystem values are NOT modified by this call (Rule 4).
   */
  void SaveNV() override
  {
    // Snapshot every live subsystem value into the corresponding NV member.
    nvHeatSet         = hvac.heatSet;
    nvCoolSet         = hvac.coolSet;
    nvHVACSwing       = (uint16_t)constrain(hvac.HVACSwing,           0,  20);
    nvHVACMinRunTime  = (uint16_t)constrain((int)hvac.minRunTimeMins,  1, 120);
    nvHVACMinRestTime = (uint16_t)constrain((int)hvac.minRestTimeMins, 0, 120);
    nvDoorTimeout     = door.autoCloseDuration;
    nvLightTimeout    = lights.duration;

    // Persist the freshly-updated NV members to EEPROM.
    saveNV();
    Serial.println(F("Controller:Live->NV->EEPROM (menu Save)"));
  }

  /**
   * @brief Restores all live subsystem values from the NV member variables.
   *
   * IMenuHost::LoadNV() implementation — called by the LoadNV menu screen
   * and the /nv/reload/cmd MQTT topic.
   * Does NOT re-read EEPROM and does NOT modify NV members (Rule 5).
   */
  void LoadNV() override
  {
    hvac.heatSet           = nvHeatSet;
    hvac.coolSet           = nvCoolSet;
    hvac.HVACSwing         = nvHVACSwing;
    hvac.minRunTimeMins    = nvHVACMinRunTime;
    hvac.minRestTimeMins   = nvHVACMinRestTime;
    door.autoCloseDuration = nvDoorTimeout;
    lights.duration        = nvLightTimeout;
    Serial.println(F("Controller:NV->live (reload)"));
  }

  // ── MQTT command handler (ENABLE_WIFI only) ───────────────────────────────
#if ENABLE_WIFI

  /**
   * @brief Dispatches an inbound MQTT command to the appropriate subsystem.
   *
   * Topic routing follows the NV model strictly:
   *   /cmd    topics  → modify LIVE subsystem values only (Rule 2).
   *   /nv/…/cmd       → modify NV member variables in RAM only (Rule 3).
   *   /nv/save/cmd    → persist NV members to EEPROM (Rule 4).
   *   /nv/reload/cmd  → copy NV members → live subsystems (Rule 5).
   */
  void handleMQTT(const char *topic, const String &payload)
  {
    Serial.print(F("MQTT IN ["));
    Serial.print(topic);
    Serial.print(F("] "));
    Serial.println(payload);

    // ════════════════════════════════════════════════════════════════════════
    //  LIVE topics – update live subsystem values ONLY (Rule 2).
    // ════════════════════════════════════════════════════════════════════════

    if (strcmp(topic, mqttManager.getTopic(F("/door/cmd"))) == 0)
    {
      if (expired(lastDoorCmd, 2000UL))
      {
        door.manualActivate();
        lastDoorCmd = now();
        lcdDisplay.SetDirty(true);
      }
    }
    else if (strcmp(topic, mqttManager.getTopic(F("/door/duration/cmd"))) == 0)
    {
      int mins = payload.toInt();
      if (mins > 0 && mins <= 120)
      {
        door.autoCloseDuration = (unsigned long)mins * 60000UL;
        notifyLiveValueChanged();
        lcdDisplay.SetDirty(false);
      }
    }
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
    else if (strcmp(topic, mqttManager.getTopic(F("/light/duration/cmd"))) == 0)
    {
      int mins = payload.toInt();
      if (mins > 0 && mins <= 120)
      {
        lights.duration = (unsigned long)mins * 60000UL;
        notifyLiveValueChanged();
        lcdDisplay.SetDirty(false);
      }
    }
    else if (strcmp(topic, mqttManager.getTopic(F("/hvac/heat_set/cmd"))) == 0)
    {
      if (expired(lastHvacCmd, 1000UL))
      {
        float val = payload.toFloat();
        if (val > 30 && val < 100)
        {
          hvac.heatSet = val;
          notifyLiveValueChanged();
          lcdDisplay.SetDirty(false);
        }
        lastHvacCmd = now();
      }
    }
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
    else if (strcmp(topic, mqttManager.getTopic(F("/hvac/cool_set/cmd"))) == 0)
    {
      if (expired(lastHvacCmd, 1000UL))
      {
        float val = payload.toFloat();
        if (val > 30 && val < 100)
        {
          hvac.coolSet = val;
          notifyLiveValueChanged();
          lcdDisplay.SetDirty(false);
        }
        lastHvacCmd = now();
      }
    }

    // ════════════════════════════════════════════════════════════════════════
    //  NV topics – update NV member variables in RAM ONLY (Rule 3).
    // ════════════════════════════════════════════════════════════════════════

    else if (strcmp(topic, mqttManager.getTopic(F("/nv/hvac/heat_set/cmd"))) == 0)
    {
      float val = payload.toFloat();
      if (val > 30 && val < 100)
      {
        nvHeatSet = val;
        Serial.println(F("NV:heat_set updated (unsaved)"));
        lcdDisplay.SetDirty(false);
      }
    }
    else if (strcmp(topic, mqttManager.getTopic(F("/nv/hvac/cool_set/cmd"))) == 0)
    {
      float val = payload.toFloat();
      if (val > 30 && val < 100)
      {
        nvCoolSet = val;
        Serial.println(F("NV:cool_set updated (unsaved)"));
        lcdDisplay.SetDirty(false);
      }
    }
    else if (strcmp(topic, mqttManager.getTopic(F("/nv/door_timeout/cmd"))) == 0)
    {
      int mins = payload.toInt();
      if (mins > 0 && mins <= 120)
      {
        nvDoorTimeout = (unsigned long)mins * 60000UL;
        Serial.println(F("NV:door_timeout updated (unsaved)"));
        lcdDisplay.SetDirty(false);
      }
    }
    else if (strcmp(topic, mqttManager.getTopic(F("/nv/light_timeout/cmd"))) == 0)
    {
      int mins = payload.toInt();
      if (mins > 0 && mins <= 120)
      {
        nvLightTimeout = (unsigned long)mins * 60000UL;
        Serial.println(F("NV:light_timeout updated (unsaved)"));
        lcdDisplay.SetDirty(false);
      }
    }

    // ════════════════════════════════════════════════════════════════════════
    //  NV control topics
    // ════════════════════════════════════════════════════════════════════════

    else if (strcmp(topic, mqttManager.getTopic(F("/nv/save/cmd"))) == 0)
    {
      // Persist current NV members to EEPROM (Rule 4 – MQTT path).
      saveNV();
      Serial.println(F("Controller:NV saved via MQTT"));
      lcdDisplay.SetDirty(false);
    }
    else if (strcmp(topic, mqttManager.getTopic(F("/nv/reload/cmd"))) == 0)
    {
      // Copy NV members → live subsystems (Rule 5).
      LoadNV();
      lcdDisplay.SetDirty(false);
    }
  }

  /**
   * @brief Static PubSubClient callback — forwards to the controller instance.
   *
   * PubSubClient requires a static (or free) function pointer.  This shim
   * builds a String from the raw byte payload and delegates to handleMQTT().
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
   * Called repeatedly from Arduino loop().  Operations are ordered to
   * minimise latency for time-critical paths:
   *
   *  1. Motion poll + light activation  ← FIRST, never delayed
   *  2. Light timeout check
   *  3. Backlight auto-off on light-off transition
   *  4. MQTT loop (may reconnect WiFi — can block up to ~5 s)
   *  5. Door state machine
   *  6. HVAC lockout update
   *  7. Async temperature collection (non-blocking)
   *  8. Menu button poll
   *  9. MQTT state publish (change-detected)
   * 10. LCD display update (dirty-flag gated)
   */
  void loop()
  {
    // ── 1. Motion detection + light activation ────────────────────────────
    bool motionDetected = motion.poll();

    // Turn on lights if motion detected, unless we're in the 15-second cooldown
    // period after manual turn-off (allows occupants to exit without reactivation)
    if (motionDetected && !lights.isInCooldown())
    {
      lights.turnOn();
      Serial.println(F("Controller:Motion->lights on"));
      if (!lcdDisplay.isBacklightOn())
        lcdDisplay.setBacklight(true);
    }
    else if (motionDetected)
    {
      Serial.println(F("Controller:Motion detected but blocked by cooldown"));
    }

    // ── 2. Light auto-off timeout check ──────────────────────────────────
    bool lightWasOn = lights.isOn();
    lights.poll();

    // ── 3. Backlight follows light state ─────────────────────────────────
    if (lightWasOn && !lights.isOn())
    {
      Serial.println(F("Controller:Light off->backlight off"));
      lcdDisplay.setBacklight(false);
    }

    // ── 4. Auto-revert check (live values → NV after 24h) ─────────────────
    checkAndRevertAutoValues();

    // ── 5. Network / MQTT heartbeat ──────────────────────────────────────
#if ENABLE_WIFI
    mqttManager.loop();
#endif

    // ── 6. Door state machine ─────────────────────────────────────────────
    GarageDoor::State doorState = door.poll(motionDetected);

    // ── 7. HVAC lockout: block heating/cooling when door is open ─────────
    hvac.lockout = (doorState != GarageDoor::Closed);

    // ── 8. Async temperature sampling ─────────────────────────────────────
    // Phase A – Start a new conversion every TEMP_INTERVAL_MS.
    if (!tempPending && expired(lastTempPoll, TEMP_INTERVAL_MS))
    {
      sensors.requestTemperatures();
      lastTempRequest = now();
      tempPending     = true;
    }

    // Phase B – Collect result once DS18B20 has finished converting.
    if (tempPending && expired(lastTempRequest, TEMP_CONVERSION_MS))
    {
      float newTemp = sensors.getTempFByIndex(0);
      // Reject sentinel error values (e.g. DEVICE_DISCONNECTED = -196.6 °F).
      if (!isnan(newTemp) && newTemp > -100.0f)
        tempF = newTemp;

      lastTempPoll = now();
      tempPending  = false;

      hvac.poll(tempF);
      lcdDisplay.SetDirty(false);
    }

    // ── 8. Menu button poll ───────────────────────────────────────────────
    // Pass *this (not the global `controller`) to avoid scope ambiguity:
    // inside a GarageController member function, `controller` resolves to
    // the file-scope global, not the current instance.
    bool menuEvent = menu.poll(*this, hvac, lights, door);
    if (menuEvent)
      lcdDisplay.SetDirty(true);

    // ── 9. MQTT state publish ─────────────────────────────────────────────
#if ENABLE_WIFI
    mqttManager.publishStateChanges(
        lights.isOn(),
        lights.duration         / 60000UL,
        lights.lightRemaining() / 60000UL,
        doorStateCode(door),
        door.autoCloseDuration  / 60000UL,
        door.getDoorRemainingTime() / 60000UL,
        tempF,
        hvac.heatSet,
        hvac.coolSet,
        nvHeatSet,
        nvCoolSet,
        hvac.HVACSwing,
        nvDoorTimeout  / 60000UL,
        nvLightTimeout / 60000UL,
        (uint8_t)hvac.mode,
        (uint8_t)hvac.state,
        motion.isActive(),
        hvac.lockout);
#endif

    // ── 10. LCD display update ────────────────────────────────────────────
    lcdDisplay.updateDisplay(hvac, door, lights, tempF);
  }
};

// ============================================================
//  PIR Hardware Interrupt Handler Implementation
// ============================================================

/**
 * @brief Hardware interrupt service routine for PIR motion sensor.
 *
 * Triggered by RISING signal on PIR pin (hardware interrupt).
 * Immediately records motion timestamp and activates lights, even if MQTT is
 * reconnecting. Provides sub-millisecond response for immediate light activation.
 *
 * Actions:
 *   1. recordMotion() – Updates lastMotion timestamp for subsystems needing
 *      occupancy-based timeout extension (e.g., door auto-close)
 *   2. turnOn() – Activates lights unless in cooldown period
 *
 * The 15-second cooldown after manual turn-off is respected: if lights were just
 * manually turned off, the interrupt is blocked allowing occupants to exit without
 * unwanted reactivation.
 *
 * @note This runs at interrupt level with minimal context,
 *       so we keep ISR operations simple and fast.
 */
void pirISR()
{
  // Record the motion event for occupancy-based timeout extension
  if (g_controller)
    g_controller->motion.recordMotion();

  // Check if we have a valid controller and if we're not in cooldown
  if (g_controller && !g_controller->lights.isInCooldown())
  {
    g_controller->lights.turnOn();
    Serial.println(F("Controller:PIR interrupt->lights on"));
  }
  else if (g_controller)
  {
    Serial.println(F("Controller:PIR interrupt blocked (cooldown)"));
  }
}

// ── Global instance ──────────────────────────────────────────────────────────
GarageController controller;

// ============================================================
//  Arduino entry points
// ============================================================

/** @brief One-time hardware and subsystem initialization. */
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
