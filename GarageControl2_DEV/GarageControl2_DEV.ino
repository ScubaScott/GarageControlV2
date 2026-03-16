/**
 * @file GarageControl2_DEV.ino
 * @brief Main Arduino sketch for the Garage Control System with Home Assistant MQTT integration.
 *
 * This sketch implements a comprehensive garage control system featuring:
 * - WiFi and MQTT connectivity for Home Assistant integration

 * - Garage door control with safety features
 * - Lighting system with motion-activated auto-off
 * - HVAC temperature control with lockout protection
 * - LCD display with hierarchical menu system
 * - Motion detection and door state monitoring
 *
 * Board: Arduino UNO R4 WiFi
 *   - In Arduino IDE: Tools → Board → "Arduino UNO R4 WiFi"
 *   - Board package: "Arduino UNO R4 Boards" (install via Boards Manager)
 *
 * The system can be configured to disable WiFi/MQTT for debugging purposes
 * by setting ENABLE_WIFI to 0.
 */

#include "src/Utility.h"

#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <OneWire.h>
#include <DallasTemperature.h>

#include "src/MQTT.h"

#include "src/HVAC.h"
#include "src/Motion.h"
#include "src/GarageDoor.h"
#include "src/GarageLight.h"
#include "src/MenuController.h"
#include "src/LcdController.h"

// ============================================================
//  Pin definitions  (UNO R4 - same physical numbers as classic Uno)
// ============================================================
const byte DoorOpenPin = 2;       // HIGH when door fully open
const byte DoorClosedPin = 3;     // HIGH when door fully closed
const byte PIRPin = 4;            // PIR OUT - active HIGH on motion
const byte DoorButtonPin = 5;     // Relay → garage door opener (active HIGH)
const byte HVACHeatPin = 6;       // Relay → heater (active HIGH)
const byte HVACTempSensorPin = 7; // OneWire data (DS18B20)
const byte MenuBtnDownPin = 9;    // Menu navigation (active LOW)
const byte MenuBtnSetPin = 10;    // Menu select (active LOW)
const byte MenuBtnUpPin = 11;     // Menu navigation (active LOW)
const byte LightSwitchPin = 13;   // Relay → garage light (active HIGH)

// ============================================================
//  USER CONFIGURATION  ← edit these before flashing
// ============================================================
const char *WIFI_SSID = "ScubaSpot";
const char *WIFI_PASSWORD = "ScubaNet";

const char *MQTT_SERVER = "192.168.0.130"; // IP of your HA MQTT broker
const int MQTT_PORT = 1883;
const char *MQTT_USER = "mqtt_user";     // leave "" if no auth
const char *MQTT_PASS = "mqtt_password"; // leave "" if no auth

// Unique device ID – change if you have multiple garage controllers
const char *DEVICE_ID = "garage_ctrl_01";
const char *DEVICE_NAME = "Garage Controller";

// ============================================================
//  MQTT TOPIC HELPERS  (built from DEVICE_ID)
// ============================================================
// All topics follow:  garage/<DEVICE_ID>/<entity>/<direction>
//   state   = Arduino → HA
//   command = HA      → Arduino

String topicBase; // filled in setup()

// ── Door ──────────────────────────────────────────────────
String DOOR_STATE_TOPIC; // publishes: open / closed / opening / closing / stopped
String DOOR_CMD_TOPIC;   // receives:  any payload → toggle (single button)

// ── Light ─────────────────────────────────────────────────
String LIGHT_STATE_TOPIC;          // publishes: ON / OFF
String LIGHT_CMD_TOPIC;            // receives:  ON / OFF
String LIGHT_DURATION_STATE_TOPIC; // publishes current timeout (minutes)
String LIGHT_DURATION_CMD_TOPIC;   // receives new timeout value (minutes)

// ── Thermostat ────────────────────────────────────────────
String HEAT_SET_STATE_TOPIC; // publishes current setpoint (float)
String HEAT_SET_CMD_TOPIC;   // receives new setpoint (float string)
String TEMP_STATE_TOPIC;     // publishes current temperature (float)
String MODE_STATE_TOPIC;     // publishes: heat / off
String MODE_CMD_TOPIC;       // receives:  heat / off

// ── Sensors ───────────────────────────────────────────────
String MOTION_STATE_TOPIC;  // publishes: ON / OFF
String LOCKOUT_STATE_TOPIC; // publishes: ON / OFF

// ── Availability ─────────────────────────────────────────
String AVAIL_TOPIC; // publishes: online / offline (LWT)

// ── Discovery prefix (must match HA config, default is "homeassistant")
const char *DISCOVERY_PREFIX = "homeassistant";

// ============================================================
//  I2C LCD Configuration (4x20 display)
// ============================================================
const byte LCD_ADDR = 0x27; // I2C address of the LCD display (adjust if needed)
const byte LCD_COLS = 20;   // 20 character width
const byte LCD_ROWS = 4;    // 4 rows

// ============================================================
//  Utilitys
// ============================================================

/**
 * @brief Gets the current time in milliseconds.
 * @return Current time in milliseconds since program start.
 */
unsigned long now()
{
  return millis();
}

/**
 * @brief Checks if a time interval has expired.
 * @param last Timestamp of the last event.
 * @param interval Interval in milliseconds.
 * @return True if the interval has expired since last, false otherwise.
 */
bool expired(unsigned long last, unsigned long interval)
{
  return (now() - last) >= interval;
}

/**
 * @brief Converts door state to string for MQTT publishing.
 * @param door Reference to the GarageDoor instance.
 * @return String representation of door state.
 */
String doorStateString(const GarageDoor &door)
{
  switch (door.getState())
  {
  case GarageDoor::Open:
    return "open";
  case GarageDoor::Closed:
    return "closed";
  case GarageDoor::Moving:
    return "opening"; // Door is moving, assume opening
  case GarageDoor::Error:
    return "error";
  case GarageDoor::Disabled:
    return "disabled";
  }
  return "unknown";
}

// ============================================================
//  Forward declaration of g_controller so GarageController
//  member functions can reference it before it is defined.
// ============================================================
// FIX: Forward-declared g_controller here so it is visible inside
//      GarageController (specifically mqttCallback) before the
//      class definition is complete.
class GarageController;
GarageController *g_controller = nullptr;

// ============================================================
//  MAIN SYSTEM CONTROLLER
// ============================================================
/**
 * @class GarageController
 * @brief Main controller class that orchestrates all garage system components.
 *
 * This class integrates motion sensing, door control, lighting, HVAC, LCD display,
 * and optional MQTT connectivity to provide a complete garage automation system.
 * It handles the main program loop, state management, and communication between
 * all subsystems.
 */
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

  unsigned long lastTempPoll = 0;
  float tempF = 0;

#if ENABLE_WIFI
  MQTTManager mqttManager;
#endif

  /* I think I can drop these
  // Track previous states to publish only on change

    String prevDoorState = "";
    float prevTemp = -999;
    float prevHeatSet = -999;
    bool prevMotion = false;
    bool prevLockout = false;
    String prevHvacMode = "";
    GarageHVAC::State prevHvacState = GarageHVAC::Waiting; // track for display updates

  */
  GarageController()
      : motion(PIRPin),
        lights(LightSwitchPin, motion),
        door(DoorButtonPin, DoorOpenPin, DoorClosedPin, motion),
        hvac(HVACHeatPin, motion),
        menu(MenuBtnUpPin, MenuBtnDownPin, MenuBtnSetPin),
        lcd(LCD_ADDR, LCD_COLS, LCD_ROWS),
        lcdDisplay(lcd, menu),
        oneWire(HVACTempSensorPin),
        sensors(&oneWire)
  {
  }

  /**
   * @brief Initializes all system components.
   *
   * Sets up sensors, menu system, LCD display, and MQTT connectivity (if enabled).
   * This method should be called once in Arduino's setup() function.
   */
  void begin()
  {
    Serial.println("Controller:Garage Control Starting: Version 2 + HA");

    sensors.begin();
    menu.begin();
    lcdDisplay.begin();
    lcdDisplay.SetDirty(true); // wake backlight

    Serial.println("Controller:LCD initialized...");

#if ENABLE_WIFI
    mqttManager.init(this, GarageController::mqttCallback);
#endif
  }

#if ENABLE_WIFI
  /**
   * @brief Handles incoming MQTT commands from Home Assistant.
   * @param topic The MQTT topic of the received message.
   * @param payload The payload of the MQTT message.
   *
   * Processes commands for door control, lighting, HVAC settings, and light timeout.
   * Updates the LCD display when commands are received.
   */
  void handleMQTT(const String &topic, const String &payload)
  {
    Serial.print("Controller:MQTT IN [");
    Serial.print(topic);
    Serial.print("] ");
    Serial.println(payload);

    if (topic == mqttManager.DOOR_CMD_TOPIC)
    {
      // any command simply toggles the door relay (single button behaviour)
      door.manualActivate();
      lcdDisplay.SetDirty(true); // wake backlight
    }
    else if (topic == mqttManager.LIGHT_CMD_TOPIC)
    {
      if (payload == "ON")
        lights.turnOn();
      if (payload == "OFF")
        lights.turnOff();
      lcdDisplay.SetDirty(true); // wake backlight
    }
    else if (topic == mqttManager.LIGHT_DURATION_CMD_TOPIC)
    {
      // payload is treated as minutes
      int mins = payload.toInt();
      if (mins > 0 && mins <= 120)
      {
        lights.duration = (unsigned long)mins * 60000UL;
        Serial.print("Controller:Light timeout → ");
        Serial.print(mins);
        Serial.println(" min");
        lcdDisplay.SetDirty(false); // dont wake backlight
      }
    }
    else if (topic == mqttManager.HEAT_SET_CMD_TOPIC)
    {
      float val = payload.toFloat();
      if (val > 30 && val < 100)
      {
        hvac.heatSet = val;
        Serial.print("Controller:Heat setpoint → ");
        Serial.println(val);
        lcdDisplay.SetDirty(false); // dont wake backlight
      }
    }
    else if (topic == mqttManager.MODE_CMD_TOPIC)
    {
      hvac.enabled = (payload == "heat");
      Serial.print("Controller:HVAC mode → ");
      Serial.println(payload);
      lcdDisplay.SetDirty(false); // dont wake backlight
    }
  }

  /**
   * @brief MQTT callback function that routes messages to the controller.
   * @param topic MQTT topic of the received message.
   * @param payload Message payload bytes.
   * @param length Length of the payload.
   */
  static void mqttCallback(char *topic, byte *payload, unsigned int length)
  {
    String t(topic);
    String p;
    for (unsigned int i = 0; i < length; i++)
      p += (char)payload[i];
    if (g_controller)
      g_controller->handleMQTT(t, p);
  }
#endif

  /**
   * @brief Main program loop that handles all system logic.
   *
   * Processes motion detection, door control, lighting, HVAC, menu interaction,
   * LCD updates, and MQTT communication. This method should be called repeatedly
   * in Arduino's loop() function.
   */
  void loop()
  {

#if ENABLE_WIFI
    // Keep WiFi alive
    mqttManager.loop();
#endif
    // Garage logic (unchanged)
    bool motionDetected = motion.poll();

    // if the sensor is currently active we treat it as activity for the menu timer
    if (motion.isActive())
    {
      menu.noteActivity();
    }

    // turn on lights on the first detection event
    if (motionDetected)
    {
      lights.turnOn();
      Serial.println("Controller:Motion Activated light.");
      if (!lcdDisplay.isBacklightOn())
      {
        lcdDisplay.setBacklight(true);
      }
    }
    lights.poll();

    GarageDoor::State doorState = door.poll(motionDetected);
    hvac.lockout = (doorState != GarageDoor::Closed);

    if (expired(lastTempPoll, 30000UL))
    {
      lastTempPoll = now();
      sensors.requestTemperatures();
      tempF = sensors.getTempFByIndex(0);
      hvac.poll(tempF);

      // temperature has been refreshed – update screen so main page shows new value
      lcdDisplay.SetDirty(false); // dont wake backlight
    }

    bool menuEvent = menu.poll(hvac, lights, door);
    if (menuEvent)
      lcdDisplay.SetDirty(true); // wake backlight

#if ENABLE_WIFI
    // Publish state changes to MQTT
    mqttManager.publishStateChanges(lights.isOn(), lights.duration / 60000UL, doorStateString(door), tempF, hvac.heatSet, hvac.enabled ? "heat" : "off", motion.isActive(), hvac.lockout);
#endif

    lcdDisplay.updateDisplay(hvac, door, lights, tempF);
  }
};

GarageController controller;

/**
 * @brief Arduino setup function called once at startup.
 */
void setup()
{
  Serial.begin(9600);         // USB serial for debug
  delay(2000);                // let serial start
  g_controller = &controller; // Initialize the global pointer

  controller.begin();
}

/**
 * @brief Arduino main loop function called repeatedly.
 */
void loop()
{
  controller.loop();
}
