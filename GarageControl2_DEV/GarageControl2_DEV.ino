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
 * The system can be configured to disable WiFi/MQTT for debugging purposes
 * by setting ENABLE_WIFI to 0.
 */

/********************************************************************
 * GARAGE CONTROL SYSTEM – WITH HOME ASSISTANT MQTT AUTO-DISCOVERY
 *                          (Arduino UNO R4 WiFi edition)
 *
 * features:
 *  - WiFi connectivity (Arduino UNO R4 WiFi, via built-in WiFiS3)
 *  - MQTT client with Home Assistant auto-discovery
 *  - HA entities auto-created:
 *      button  → Garage Door (toggle)
 *      light   → Garage Light (on/off/state)
 *      number  → Light timeout (minutes)
 *      climate → Garage Heating (heat setpoint + current temp)
 *      sensor  → Temperature
 *      binary_sensor → Motion, HVAC Lockout
 *  - Motion → lights on, auto-off after timeout
 *  - Heat to setpoint, lockout when door open
 *  - Auto-close door after timeout if no motion
 *  - Door error detection and disable after repeated failures
 *  - Manual door open/close via long-press on Up/Down (Main screen)
 *  - Hierarchical menu for tuning parameters
 *  - Menu timeout: returns to Main after 1 minute of inactivity
 *  - Relay-spike suppression: any relay switch forces PIR ACK
 *
 * *** CONFIGURE YOUR SETTINGS IN THE "USER CONFIGURATION" SECTION ***
 *
 * Board: Arduino UNO R4 WiFi
 *   - In Arduino IDE: Tools → Board → "Arduino UNO R4 WiFi"
 *   - Board package: "Arduino UNO R4 Boards" (install via Boards Manager)
 *
 * Required libraries (install via Library Manager):
 *   - PubSubClient  (by Nick O'Leary)
 *   - OneWire
 *   - DallasTemperature
 *   NOTE: WiFiS3 is built into the UNO R4 board package – no separate install needed.
 *
 ********************************************************************/
#define ENABLE_WIFI 0 // Set to 0 to disable WiFi and MQTT for debugging
// ============================================================
//  UNO R4 WiFi – uses the built-in WiFiS3 library
// ============================================================
#include <PubSubClient.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <OneWire.h>
#include <DallasTemperature.h>

#if ENABLE_WIFI
// #include <WiFiS3.h> // Built into the "Arduino UNO R4 Boards" package
#include <PubSubClient.h>
#endif

#include "src/Utility.h"
#include "src/HVAC.h"
#include "src/Motion.h"
#include "src/GarageDoor.h"
#include "src/GarageLight.h"
#include "src/MenuController.h"
#include "src/LcdController.h"
#include "src/MQTT.h"
// NOTE: SoftwareSerial is NOT used on the UNO R4.
// The LCD is wired to hardware Serial1 (pin 1 = TX).
// Serial1 is a native hardware UART on the R4 – no extra library needed.

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
//  Pin definitions  (UNO R4 WiFi – same physical pin numbers as Uno)
// ============================================================
// const byte LCDPin = 1;             // TX of hardware Serial1 – LCD data line
const byte DoorOpenPin = 2;       // reads HIGH when door fully open
const byte DoorClosedPin = 3;     // reads HIGH when door fully closed
const byte PIRPin = 4;            // motion sensor output (PIR OUT) – active HIGH when motion detected
const byte DoorButtonPin = 5;     // output to relay controlling garage door opener (active HIGH)
const byte HVACHeatPin = 6;       // output to relay controlling heater (active HIGH)
const byte HVACTempSensorPin = 7; // OneWire data line for temperature sensor (e.g. DS18B20)
const byte MenuBtnDownPin = 9;    // menu navigation button (active LOW when pressed)
const byte MenuBtnSetPin = 10;    // menu select button (active LOW when pressed)
const byte MenuBtnUpPin = 11;     // menu navigation button (active LOW when pressed)
const byte LightSwitchPin = 13;   // output to relay controlling garage light (active HIGH)

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

// Door state string for MQTT publishing

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
//  Forward declarations for MQTT callback
// ============================================================
class GarageController;
GarageController *g_controller = nullptr;
#if ENABLE_WIFI
void mqttCallback(char *topic, byte *payload, unsigned int length);
#endif

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

  // Track previous states to publish only on change
  String prevDoorState = "";
  float prevTemp = -999;
  float prevHeatSet = -999;
  bool prevMotion = false;
  bool prevLockout = false;
  String prevHvacMode = "";
  GarageHVAC::State prevHvacState = GarageHVAC::Waiting; // track for display updates

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
  void begin();

  /**
   * @brief Main program loop that handles all system logic.
   *
   * Processes motion detection, door control, lighting, HVAC, menu interaction,
   * LCD updates, and MQTT communication. This method should be called repeatedly
   * in Arduino's loop() function.
   */
  void loop();

#if ENABLE_WIFI
  /**
   * @brief Handles incoming MQTT commands from Home Assistant.
   * @param topic The MQTT topic of the received message.
   * @param payload The payload of the MQTT message.
   *
   * Processes commands for door control, lighting, HVAC settings, and light timeout.
   * Updates the LCD display when commands are received.
   */
  void handleMQTT(const String &topic, const String &payload);
#endif

private:
};

#if ENABLE_WIFI
// ============================================================
//  MQTT callback (routes to controller)
// ============================================================

/**
 * @brief MQTT callback function that routes messages to the controller.
 * @param topic MQTT topic of the received message.
 * @param payload Message payload bytes.
 * @param length Length of the payload.
 */
void mqttCallback(char *topic, byte *payload, unsigned int length)
{
  String t(topic);
  String p;
  for (unsigned int i = 0; i < length; i++)
    p += (char)payload[i];
  if (g_controller)
    g_controller->handleMQTT(t, p);
}
#endif

// ============================================================
//  ARDUINO SETUP / LOOP
// ============================================================
GarageController controller;

/**
 * @brief Arduino setup function called once at startup.
 */
void setup()
{
  Serial.begin(9600); // USB serial for debug
  delay(2000);        // let serial start
  g_controller = &controller;
  controller.begin();
}


// ============================================================
//  GarageController method implementations
// ============================================================

/**
 * @brief Initializes all system components.
 */
void GarageController::begin()
{
  sensors.begin();
  menu.begin();
  lcdDisplay.begin();
  lcdDisplay.SetDirty(true); // wake backlight

  Serial.println("Controller:LCD initialized...");

#if ENABLE_WIFI
  mqttManager.init(this);
#endif

  Serial.println("Controller:Garage Control Starting: Version 2 + HA");
}

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
#endif
/**
 * @brief Arduino setup function called once at startup.
 */
void
setup()
{
  Serial.begin(9600); // USB serial for debug
  delay(2000);        // let serial start
  g_controller = &controller;
  controller.begin();
}

/**
 * @brief Arduino main loop function called repeatedly.
 */
void loop()
{
  controller.loop();
}
