/**
 * @file GarageControl2.ino
 * @brief Main Arduino sketch for the Garage Control System with Home Assistant MQTT integration.
 *
 * Board: Arduino UNO R4 WiFi (production) / Arduino UNO R4 Minima (dev)
 *
 * @section Hardware
 * - Door open sensor: digital pin 2 (HIGH when fully open)
 * - Door closed sensor: digital pin 3 (HIGH when fully closed)
 * - PIR motion sensor: digital pin 4 (active HIGH)
 * - Garage door relay: digital pin 5 (active HIGH)
 * - HVAC heater relay: digital pin 6 (active HIGH)
 * - DS18B20 temp sensor (OneWire): digital pin 7
 * - Menu navigation buttons: pins 8/9/10 (active LOW, using INPUT_PULLUP)
 * - Garage light relay: digital pin 13 (active HIGH)
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
const char *GC_VERSION = "2.11";

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
const byte DoorOpenPin = 2;       // HIGH when door fully open
const byte DoorClosedPin = 3;     // HIGH when door fully closed
const byte PIRPin = 4;            // PIR OUT - active HIGH on motion
const byte DoorButtonPin = 5;     // Relay → garage door opener (active HIGH)
const byte HVACHeatPin = 6;       // Relay → heater (active HIGH)
const byte HVACCoolPin = 11;      // Relay → cooler (active HIGH)
const byte HVACTempSensorPin = 7; // OneWire data (DS18B20)
const byte MenuBtnDownPin = 8;    // Menu navigation (active LOW)
const byte MenuBtnSetPin = 9;     // Menu select (active LOW)
const byte MenuBtnUpPin = 10;     // Menu navigation (active LOW)
const byte LightSwitchPin = 13;   // Relay → garage light (active HIGH)

// ============================================================
//  I2C LCD (4x20)
// ============================================================
const byte LCD_ADDR = 0x27;
const byte LCD_COLS = 20;
const byte LCD_ROWS = 4;

// ============================================================
//  Utility helpers
// ============================================================

unsigned long now() { return millis(); }
bool expired(unsigned long last, unsigned long interval) { return (now() - last) >= interval; }

/**
 * @brief Converts a GarageDoor state into a compact uint8_t code.
 *
 * This avoids heap allocations in the main loop when publishing MQTT state.
 *
 * @param door Reference to the GarageDoor instance.
 * @return State code: 0=open, 1=closed, 2=moving, 3=error, 4=disabled.
 */
uint8_t doorStateCode(const GarageDoor &door)
{
  switch (door.getState())
  {
  case GarageDoor::Open:
    return 0;
  case GarageDoor::Closed:
    return 1;
  case GarageDoor::Moving:
    return 2;
  case GarageDoor::Error:
    return 3;
  default:
    return 4; // Disabled
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
class GarageController
{
public:
  MotionSensor motion;
  GarageLight lights;
  GarageDoor door;
  GarageHVAC hvac;
  MenuController menu;
  LiquidCrystal_I2C lcd;
  LcdController lcdDisplay;
  OneWire oneWire;
  DallasTemperature sensors;

  // NV values (stored in EEPROM)
  float nvHeatSet = 65;
  float nvCoolSet = 85;

  unsigned long lastTempPoll = 0;
  float tempF = 0;

  // NV (EEPROM) storage layout
  static constexpr uint32_t NV_MAGIC = 0x47434E56U; // 'GCNV'
  static constexpr int NV_ADDR_MAGIC = 0;
  static constexpr int NV_ADDR_HEAT_SET = NV_ADDR_MAGIC + sizeof(NV_MAGIC);
  static constexpr int NV_ADDR_COOL_SET = NV_ADDR_HEAT_SET + sizeof(float);
  static constexpr int NV_ADDR_DOOR_TIMEOUT = NV_ADDR_COOL_SET + sizeof(float);
  static constexpr int NV_ADDR_LIGHT_TIMEOUT = NV_ADDR_DOOR_TIMEOUT + sizeof(uint16_t);

  bool loadNV();
  bool saveNV();
  void reloadNV();

#if ENABLE_WIFI
  unsigned long lastDoorCmd = 0;
  unsigned long lastLightCmd = 0;
  unsigned long lastHvacCmd = 0;
#endif

#if ENABLE_WIFI
  MQTTManager mqttManager;
#endif

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

  void begin()
  {
    Serial.println(F("Controller:Starting v2+HA"));
    sensors.begin();
    menu.begin();
    lcdDisplay.begin();
    lcdDisplay.SetDirty(true);
    Serial.println(F("Controller:LCD ready"));

    if (loadNV())
    {
      Serial.println(F("Controller:Loaded NV settings"));
    }
    else
    {
      Serial.println(F("Controller:NV settings missing or invalid, using defaults"));
      saveNV();
    }

#if ENABLE_WIFI
    mqttManager.init(this, GarageController::mqttCallback);
#endif
  }

#if ENABLE_WIFI
  void handleMQTT(const char *topic, const String &payload)
  {
    Serial.print(F("MQTT IN ["));
    Serial.print(topic);
    Serial.print(F("] "));
    Serial.println(payload);

    if (strcmp(topic, mqttManager.getTopic(F("/door/cmd"))) == 0)
    {
      if (expired(lastDoorCmd, 2000UL)) // Debounce door commands to prevent rapid toggling
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
        lcdDisplay.SetDirty(false);
      }
    }
    else if (strcmp(topic, mqttManager.getTopic(F("/light/cmd"))) == 0)
    {
      if (expired(lastLightCmd, 500UL)) // Debounce light commands
      {
        if (payload == F("on"))
        {
          lights.turnOn();
          lcdDisplay.SetDirty(true); // wake backlight when light turns on
        }
        if (payload == F("off"))
        {
          lights.turnOff();
          lcdDisplay.SetDirty(false); // update display but don't wake backlight
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
        lcdDisplay.SetDirty(false);
      }
    }
    else if (strcmp(topic, mqttManager.getTopic(F("/hvac/heat_set/cmd"))) == 0)
    {
      if (expired(lastHvacCmd, 1000UL)) // Debounce HVAC commands
      {
        float val = payload.toFloat();
        if (val > 30 && val < 100)
        {
          hvac.heatSet = val;
          lcdDisplay.SetDirty(false);
        }
        lastHvacCmd = now();
      }
    }
    else if (strcmp(topic, mqttManager.getTopic(F("/hvac/mode/cmd"))) == 0)
    {
      if (expired(lastHvacCmd, 1000UL)) // Debounce HVAC commands
      {
        if (payload == "off")
          hvac.mode = GarageHVAC::Off;
        else if (payload == "heat")
          hvac.mode = GarageHVAC::Heat;
        else if (payload == "heat_cool")
          hvac.mode = GarageHVAC::Heat_Cool;
        else if (payload == "cool")
          hvac.mode = GarageHVAC::Cool;
        lcdDisplay.SetDirty(false);
        lastHvacCmd = now();
      }
    }
    else if (strcmp(topic, mqttManager.getTopic(F("/hvac/cool_set/cmd"))) == 0)
    {
      if (expired(lastHvacCmd, 1000UL)) // Debounce HVAC commands
      {
        float val = payload.toFloat();
        if (val > 30 && val < 100)
        {
          hvac.coolSet = val;
          lcdDisplay.SetDirty(false);
        }
        lastHvacCmd = now();
      }
    }
    else if (strcmp(topic, mqttManager.getTopic(F("/nv/hvac/heat_set/cmd"))) == 0)
    {
      float val = payload.toFloat();
      if (val > 30 && val < 100)
      {
        nvHeatSet = val;
        saveNV();
        lcdDisplay.SetDirty(false);
      }
    }
    else if (strcmp(topic, mqttManager.getTopic(F("/nv/hvac/cool_set/cmd"))) == 0)
    {
      float val = payload.toFloat();
      if (val > 30 && val < 100)
      {
        nvCoolSet = val;
        saveNV();
        lcdDisplay.SetDirty(false);
      }
    }
    else if (strcmp(topic, mqttManager.getTopic(F("/nv/door_timeout/cmd"))) == 0)
    {
      int mins = payload.toInt();
      if (mins > 0 && mins <= 120)
      {
        door.autoCloseDuration = (unsigned long)mins * 60000UL;
        saveNV();
        lcdDisplay.SetDirty(false);
      }
    }
    else if (strcmp(topic, mqttManager.getTopic(F("/nv/light_timeout/cmd"))) == 0)
    {
      int mins = payload.toInt();
      if (mins > 0 && mins <= 120)
      {
        lights.duration = (unsigned long)mins * 60000UL;
        saveNV();
        lcdDisplay.SetDirty(false);
      }
    }
    else if (strcmp(topic, mqttManager.getTopic(F("/nv/reload/cmd"))) == 0)
    {
      reloadNV();
      lcdDisplay.SetDirty(false);
    }
  }

  static void mqttCallback(char *topic, byte *payload, unsigned int length)
  {
    // Build payload String only once here (PubSubClient requires char* API)
    String p;
    p.reserve(length);
    for (unsigned int i = 0; i < length; i++)
      p += (char)payload[i];
    if (g_controller)
      g_controller->handleMQTT(topic, p);
  }
#endif

  void loop()
  {
#if ENABLE_WIFI
    mqttManager.loop();
#endif

    bool motionDetected = motion.poll();

    if (motionDetected)
    {
      lights.turnOn();
      Serial.println(F("Controller:Motion->light on"));
      if (!lcdDisplay.isBacklightOn())
        lcdDisplay.setBacklight(true);
    }

    bool lightWasOn = lights.isOn();
    lights.poll();

    // Turn backlight off when the garage light turns off (timeout or manual)
    if (lightWasOn && !lights.isOn())
    {
      Serial.println(F("Controller:Light off->backlight off"));
      lcdDisplay.setBacklight(false);
    }

    GarageDoor::State doorState = door.poll(motionDetected);
    hvac.lockout = (doorState != GarageDoor::Closed);

    GarageHVAC::State hvacState = hvac.state;
    if (expired(lastTempPoll, 30000UL))
    {
      lastTempPoll = now();
      sensors.requestTemperatures();
      tempF = sensors.getTempFByIndex(0);
      hvacState = hvac.poll(tempF);
      lcdDisplay.SetDirty(false);
    }

    bool menuEvent = menu.poll(controller, hvac, lights, door);
    if (menuEvent)
      lcdDisplay.SetDirty(true);

#if ENABLE_WIFI
    mqttManager.publishStateChanges(
        lights.isOn(),
        lights.duration / 60000UL,
        lights.lightRemaining() / 60000UL,
        doorStateCode(door), // uint8_t code - no String allocation
        door.autoCloseDuration / 60000UL,
        door.getDoorRemainingTime() / 60000UL,
        tempF,
        hvac.heatSet,
        hvac.coolSet,
        nvHeatSet,
        nvCoolSet,
        (uint8_t)hvac.mode, // uint8_t mode
        (uint8_t)hvacState, // uint8_t runtime state
        motion.isActive(),
        hvac.lockout);
#endif

    lcdDisplay.updateDisplay(hvac, door, lights, tempF);
  }
};

bool GarageController::loadNV()
{
  uint32_t magic = 0;
  EEPROM.get(NV_ADDR_MAGIC, magic);
  if (magic != NV_MAGIC)
    return false;

  float heatSetNV = 0.0f;
  float coolSetNV = 0.0f;
  uint16_t doorTimeoutMins = 0;
  uint16_t lightTimeoutMins = 0;

  EEPROM.get(NV_ADDR_HEAT_SET, heatSetNV);
  EEPROM.get(NV_ADDR_COOL_SET, coolSetNV);
  EEPROM.get(NV_ADDR_DOOR_TIMEOUT, doorTimeoutMins);
  EEPROM.get(NV_ADDR_LIGHT_TIMEOUT, lightTimeoutMins);

  if (heatSetNV < 30.0f || heatSetNV > 100.0f ||
      coolSetNV < 30.0f || coolSetNV > 100.0f ||
      doorTimeoutMins < 1 || doorTimeoutMins > 120 ||
      lightTimeoutMins < 1 || lightTimeoutMins > 120)
  {
    return false;
  }

  // Store NV values
  nvHeatSet = heatSetNV;
  nvCoolSet = coolSetNV;
  
  // Set current values from NV
  hvac.heatSet = nvHeatSet;
  hvac.coolSet = nvCoolSet;
  door.autoCloseDuration = (unsigned long)doorTimeoutMins * 60000UL;
  lights.duration = (unsigned long)lightTimeoutMins * 60000UL;
  return true;
}

bool GarageController::saveNV()
{
  uint16_t doorTimeoutMins = (uint16_t)(door.autoCloseDuration / 60000UL);
  uint16_t lightTimeoutMins = (uint16_t)(lights.duration / 60000UL);

  // Sanity trim in case in-memory values are out of expected limits
  if (doorTimeoutMins < 1)
    doorTimeoutMins = 1;
  if (doorTimeoutMins > 120)
    doorTimeoutMins = 120;
  if (lightTimeoutMins < 1)
    lightTimeoutMins = 1;
  if (lightTimeoutMins > 120)
    lightTimeoutMins = 120;

  EEPROM.put(NV_ADDR_MAGIC, NV_MAGIC);
  EEPROM.put(NV_ADDR_HEAT_SET, nvHeatSet);
  EEPROM.put(NV_ADDR_COOL_SET, nvCoolSet);
  EEPROM.put(NV_ADDR_DOOR_TIMEOUT, doorTimeoutMins);
  EEPROM.put(NV_ADDR_LIGHT_TIMEOUT, lightTimeoutMins);

#if defined(ESP8266) || defined(ESP32)
  EEPROM.commit();
#endif

  return true;
}

void GarageController::reloadNV()
{
  hvac.heatSet = nvHeatSet;
  hvac.coolSet = nvCoolSet;
}

GarageController controller;

void setup()
{
  Serial.begin(9600);
  delay(2000);
  g_controller = &controller;
  controller.begin();
}

void loop()
{
  controller.loop();
}
