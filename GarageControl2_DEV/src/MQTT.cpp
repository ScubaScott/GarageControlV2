/**
 * @file MQTT.cpp
 * @brief MQTTManager implementation - compiled only when ENABLE_WIFI is 1.
 *
 * Memory strategy
 * ───────────────
 * 1. F() macro  – all string literals are placed in flash (PROGMEM) and
 *    never copied to SRAM.  This includes every Serial.print message,
 *    every MQTT topic fragment, and every discovery payload fragment.
 *
 * 2. No persistent topic Strings – topics are built on-demand into a single
 *    shared 64-byte char buffer (_topicBuf).  The previous design held 14
 *    separate String members totalling ~400 bytes of heap.
 *
 * 3. Compact state cache – prevDoorState (was a String) is now a uint8_t
 *    door code; prevHvacMode (was a String) is now a bool.  Saves ~60 bytes
 *    of heap plus fragmentation overhead.
 *
 * 4. Discovery payloads – assembled in a single 512-byte stack buffer
 *    (reused for every entity) rather than heap-concatenated Strings.
 *    The buffer is declared once in publishDiscovery() and reused.
 */
#include <ArduinoJson.h>
#include "Utility.h"
#include "MQTT.h"
#include "../config/Config.h"

// Global for reporting network status to UI/menu.
MQTTManager *g_mqttManager = nullptr;

#if ENABLE_WIFI

// ============================================================
//  Constructor
// ============================================================
/**
 * @file MQTT.cpp
 * @brief Implementation of MQTTManager class for WiFi and MQTT handling.
 */

/**
 * @brief Constructor for MQTTManager.
 *
 * Initializes WiFi and MQTT configuration from Config.h.
 * All configuration values are stored in flash memory (PROGMEM) via F() macro,
 * minimizing SRAM footprint of configuration pointers.
 */
MQTTManager::MQTTManager()
    : mqtt(wifiClient)
{
  // All literals go to flash; the pointers themselves cost 2 bytes each.
  WIFI_SSID = CONFIG_WIFI_SSID;
  WIFI_PASSWORD = CONFIG_WIFI_PASSWORD;
  MQTT_SERVER = CONFIG_MQTT_SERVER;
  MQTT_PORT = CONFIG_MQTT_PORT;
  MQTT_USER = CONFIG_MQTT_USER;
  MQTT_PASS = CONFIG_MQTT_PASSWORD;
  DEVICE_ID = CONFIG_DEVICE_ID;
  DEVICE_NAME = CONFIG_DEVICE_NAME;
  DISCOVERY_PREFIX = CONFIG_DISCOVERY_PREFIX;
}

// ============================================================
//  Topic helpers
// ============================================================
// Helper Functions (Internal DSL-style)
// ============================================================

void MQTTManager::addDevice(JsonObject dev)
{
  JsonArray ids = dev.createNestedArray("ids");
  ids.add(DEVICE_ID);
  dev["name"] = DEVICE_NAME;
  dev["mf"] = "Arduino";
  dev["mdl"] = "UNO R4 WiFi";
}

void MQTTManager::makeTopic(char *out, size_t len, const char *base, const char *suffix)
{
  snprintf(out, len, "%s/%s", base, suffix);
}

// ============================================================
// Refactored Discovery Publisher
// ============================================================
/**
 * Builds "garage/<DEVICE_ID><suffix>" into _topicBuf.
 * suffix is a flash string, e.g. F("/door/state").
 * Returns pointer to _topicBuf.
 */
const char *MQTTManager::buildTopic(const __FlashStringHelper *suffix)
{
  // Build topic into shared buffer using flash-safe concatenation.
  // snprintf_P with %S is unreliable on some toolchains (it may emit "S").
  strcpy(_topicBuf, "garage/");
  strncat(_topicBuf, DEVICE_ID, sizeof(_topicBuf) - strlen(_topicBuf) - 1);
  strncat_P(_topicBuf, (PGM_P)suffix,
            sizeof(_topicBuf) - strlen(_topicBuf) - 1);
  return _topicBuf;
}

/**
 * Builds "<DISCOVERY_PREFIX>/<component>/<DEVICE_ID><entitySuffix>/config"
 * Returns pointer to _topicBuf.
 */
const char *MQTTManager::buildDiscoveryTopic(const __FlashStringHelper *component,
                                             const __FlashStringHelper *entitySuffix)
{
  // Build discovery topic into shared buffer using flash-safe concatenation.
  strcpy(_topicBuf, DISCOVERY_PREFIX);
  strncat(_topicBuf, "/", sizeof(_topicBuf) - strlen(_topicBuf) - 1);
  strncat_P(_topicBuf, (PGM_P)component,
            sizeof(_topicBuf) - strlen(_topicBuf) - 1);
  strncat(_topicBuf, "/", sizeof(_topicBuf) - strlen(_topicBuf) - 1);
  strncat(_topicBuf, DEVICE_ID, sizeof(_topicBuf) - strlen(_topicBuf) - 1);
  strncat_P(_topicBuf, (PGM_P)entitySuffix,
            sizeof(_topicBuf) - strlen(_topicBuf) - 1);
  strncat(_topicBuf, "/config", sizeof(_topicBuf) - strlen(_topicBuf) - 1);
  return _topicBuf;
}

/** Public accessor used by GarageController::handleMQTT(). */
const char *MQTTManager::getTopic(const __FlashStringHelper *suffix)
{
  return buildTopic(suffix);
}

// ============================================================
//  Public API
// ============================================================

void MQTTManager::init(GarageController *ctrl,
                       void (*callback)(char *, byte *, unsigned int))
{
  controller = ctrl;
  g_mqttManager = this;

  // Ensure we start in a state where we attempt to connect.
  netStatus = NetStatus::Connecting;
  consecutiveFailures = 0;

  connectWiFi();
  mqtt.setServer(MQTT_SERVER, MQTT_PORT);
  mqtt.setCallback(callback);
  mqtt.setBufferSize(1280); // Increased from 768 to accommodate large climate discovery payload
  connectMQTT();
  publishDiscovery();
}

void MQTTManager::loop()
{
  // When disabled due to repeated failures, attempt a full reconnect once per hour.
  if (netStatus == NetStatus::Disabled)
  {
    if (millis() - lastHourlyRetry >= 3600000UL)
    {
      lastHourlyRetry = millis();
      Serial.println(F("MQTT:Hourly reconnect attempt"));
      consecutiveFailures = 0;
      netStatus = NetStatus::Connecting;
      connectWiFi();
      if (WiFi.status() == WL_CONNECTED)
        connectMQTT();
    }
    return;
  }

  if (WiFi.status() != WL_CONNECTED)
    connectWiFi();

  if (!mqtt.connected())
  {
    netStatus = NetStatus::Connecting;
    if (millis() - lastMqttReconnect >= 5000UL)
    {
      lastMqttReconnect = millis();
      connectMQTT();
    }
  }

  if (netStatus != NetStatus::Disabled && mqtt.connected())
    mqtt.loop();
}

/**
 * @brief Publishes state changes to MQTT topics.
 * @param lightOn Current light state.
 * @param lightDurationMins Light timeout in minutes.
 * @param lightRemainingMins Light timeout remaining in minutes.
 * @param doorCode Current door state code.
 * @param doorDurationMins Door timeout in minutes.
 * @param doorRemainingMins Door timeout remaining in minutes.
 * @param tempF Current temperature.
 * @param heatSet Heat setpoint.
 * @param coolSet Cool setpoint.
 * @param mode HVAC mode code.
 * @param hvacState HVAC state code.
 * @param motionActive Motion sensor state.
 * @param lockout HVAC lockout state.
 */
void MQTTManager::publishStateChanges(bool lightOn,
                                      unsigned long lightDurationMins,
                                      unsigned long lightRemainingMins,
                                      uint8_t doorCode,
                                      unsigned long doorDurationMins,
                                      unsigned long doorRemainingMins,
                                      float tempF,
                                      float heatSet,
                                      float coolSet,
                                      uint8_t mode,
                                      uint8_t hvacState,
                                      bool motionActive,
                                      bool lockout)
{
  if (netStatus == NetStatus::Disabled)
    return;
  if (!mqtt.connected())
    return;

  // After every (re)connect, force all prev-state sentinels to differ from
  // current state so that the next call publishes everything immediately.
  // This overwrites any stale retained values HA has from a previous session
  // (e.g. heatSet 70 retained in HA while device boots with heatSet 65).
  // Temperature is excluded here – it is gated separately below until valid.
  if (pendingFullPublish)
  {
    prevLightState = !lightOn;
    prevLightDuration = lightDurationMins + 1;
    prevLightRemaining = lightRemainingMins + 1;
    prevDoorCode = 0xFF;
    prevDoorDuration = doorDurationMins + 1;
    prevDoorRemaining = doorRemainingMins + 1;
    prevHeatSet = heatSet + 10.0f;
    prevCoolSet = coolSet + 10.0f;
    prevMode = 0xFF;
    prevHvacState = 0xFF;
    prevMotion = !motionActive;
    prevLockout = !lockout;
    // prevTemp intentionally NOT reset – temperature waits for hasValidTemp below
    pendingFullPublish = false;
  }

  // Use a small stack buffer for numeric payloads (replaces heap String temps)
  char val[12];

  if (lightOn != prevLightState)
  {
    mqtt.publish(buildTopic(F("/light/state")),
                 lightOn ? "on" : "off", true);
    prevLightState = lightOn;
  }

  if (lightDurationMins != prevLightDuration)
  {
    snprintf(val, sizeof(val), "%lu", lightDurationMins);
    mqtt.publish(buildTopic(F("/light/duration/state")), val, true);
    prevLightDuration = lightDurationMins;
  }

  if (lightRemainingMins != prevLightRemaining)
  {
    snprintf(val, sizeof(val), "%lu", lightRemainingMins);
    mqtt.publish(buildTopic(F("/light/remaining/state")), val, true);
    prevLightRemaining = lightRemainingMins;
  }

  if (doorCode != prevDoorCode)
  {
    // Convert code to payload string without heap allocation.
    // Use canonical values (uppercase) so Home Assistant can consume them
    // cleanly via discovery mappings.
    const char *ds;
    switch (doorCode)
    {
    case 0:
      ds = "open";
      break;
    case 1:
      ds = "closed";
      break;
    case 2:
      ds = "opening";
      break;
    case 3:
      ds = "error";
      break;
    default:
      ds = "disabled";
      break;
    }
    mqtt.publish(buildTopic(F("/door/state")), ds, true);
    prevDoorCode = doorCode;
  }

  if (doorDurationMins != prevDoorDuration)
  {
    snprintf(val, sizeof(val), "%lu", doorDurationMins);
    mqtt.publish(buildTopic(F("/door/duration/state")), val, true);
    prevDoorDuration = doorDurationMins;
  }

  if (doorRemainingMins != prevDoorRemaining)
  {
    snprintf(val, sizeof(val), "%lu", doorRemainingMins);
    mqtt.publish(buildTopic(F("/door/remaining/state")), val, true);
    prevDoorRemaining = doorRemainingMins;
  }

  // Temperature: validate before first publish.  Only this value is gated –
  // setpoints and modes must publish immediately so HA does not retain stale
  // values while waiting for the DS18B20 to return a valid reading.
  if (!hasValidTemp)
  {
    if (!isnan(tempF) && tempF > -40.0f && tempF < 150.0f)
      hasValidTemp = true;
  }
  if (hasValidTemp && abs(tempF - prevTemp) >= 0.2f)
  {
    dtostrf(tempF, 4, 1, val);
    mqtt.publish(buildTopic(F("/hvac/temp/state")), val, true);
    prevTemp = tempF;
  }

  if (abs(heatSet - prevHeatSet) >= 0.5f)
  {
    dtostrf(heatSet, 4, 1, val);
    mqtt.publish(buildTopic(F("/hvac/heat_set/state")), val, true);
    prevHeatSet = heatSet;
  }

  if (abs(coolSet - prevCoolSet) >= 0.5f)
  {
    dtostrf(coolSet, 4, 1, val);
    mqtt.publish(buildTopic(F("/hvac/cool_set/state")), val, true);
    prevCoolSet = coolSet;
  }

  if (mode != prevMode)
  {
    const char *modeStr;
    switch (mode)
    {
    case 0:
      modeStr = "off";
      break; // Off
    case 1:
      modeStr = "heat";
      break; // Heat
    case 2:
      modeStr = "heat_cool";
      break; // Heat_Cool
    case 3:
      modeStr = "cool";
      break; // Cool
    default:
      modeStr = "off";
      break;
    }
    mqtt.publish(buildTopic(F("/hvac/mode/state")), modeStr, true);
    prevMode = mode;
  }

  if (hvacState != prevHvacState)
  {
    const char *action;
    switch (hvacState)
    {
    case 0:
      action = "idle";
      break; // Waiting
    case 1:
      action = "heating";
      break;
    case 2:
      action = "cooling";
      break;
    case 3:
      action = "pending";
      break;
    default:
      action = "unknown";
      break;
    }
    mqtt.publish(buildTopic(F("/hvac/action/state")), action, true);
    prevHvacState = hvacState;
  }

  if (motionActive != prevMotion)
  {
    mqtt.publish(buildTopic(F("/motion/state")),
                 motionActive ? "on" : "off", true);
    prevMotion = motionActive;
  }

  if (lockout != prevLockout)
  {
    mqtt.publish(buildTopic(F("/lockout/state")),
                 lockout ? "Lockout" : "OK", true);
    prevLockout = lockout;
  }
}

// ============================================================
//  Private helpers
// ============================================================

void MQTTManager::connectWiFi()
{
  if (netStatus == NetStatus::Disabled)
    return;

  Serial.print(F("Connecting to WiFi: "));
  Serial.println(WIFI_SSID);
  netStatus = NetStatus::Connecting;
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < 15000UL)
  {
    delay(500);
    Serial.print(F("."));
  }
  if (WiFi.status() == WL_CONNECTED)
  {
    Serial.print(F("\nWiFi connected, IP: "));
    Serial.println(WiFi.localIP());
    consecutiveFailures = 0;
  }
  else
  {
    consecutiveFailures++;
    Serial.println(F("\nWiFi connect failed"));
    if (consecutiveFailures >= MAX_FAILURES)
    {
      netStatus = NetStatus::Disabled;
      Serial.println(F("Network disabled due to repeated failures"));
      return;
    }
    Serial.println(F("- will retry"));
  }
}

void MQTTManager::connectMQTT()
{
  if (netStatus == NetStatus::Disabled)
    return;

  // clientId on the stack - no heap String
  char clientId[32];
  snprintf(clientId, sizeof(clientId), "%s_%04x", DEVICE_ID, (unsigned)random(0xffff));

  // Build availability topic once for the connect call
  const char *availTopic = buildTopic(F("/availability"));
  // Copy it - buildTopic() reuses the buffer, connectMQTT uses it below
  char avail[64];
  strncpy(avail, availTopic, sizeof(avail));

  Serial.print(F("Connecting MQTT... "));
  bool ok;
  if (strlen(MQTT_USER) > 0)
  {
    ok = mqtt.connect(clientId, MQTT_USER, MQTT_PASS,
                      avail, 1, true, "offline");
  }
  else
  {
    ok = mqtt.connect(clientId, nullptr, nullptr,
                      avail, 1, true, "offline");
  }

  if (ok)
  {
    Serial.println(F("connected"));
    mqtt.publish(avail, "online", true);

    mqtt.subscribe(buildTopic(F("/door/cmd")));
    mqtt.subscribe(buildTopic(F("/door/duration/cmd")));
    mqtt.subscribe(buildTopic(F("/light/cmd")));
    mqtt.subscribe(buildTopic(F("/light/duration/cmd")));
    mqtt.subscribe(buildTopic(F("/hvac/heat_set/cmd")));
    mqtt.subscribe(buildTopic(F("/hvac/cool_set/cmd")));
    mqtt.subscribe(buildTopic(F("/hvac/mode/cmd")));

    publishDiscovery();

    // Force a full state publish on the next publishStateChanges() call so that
    // any stale retained values in HA are immediately overwritten with the
    // device's current state.
    pendingFullPublish = true;

    netStatus = NetStatus::Connected;
    consecutiveFailures = 0;
  }
  else
  {
    consecutiveFailures++;
    Serial.print(F("MQTT failed rc="));
    Serial.println(mqtt.state());
    if (consecutiveFailures >= MAX_FAILURES)
    {
      netStatus = NetStatus::Disabled;
      Serial.println(F("Network disabled due to repeated failures"));
      return;
    }
  }
}

/**
 * Publishes all HA auto-discovery configs.
 *
 * A single 1024-byte stack buffer is reused for every payload, replacing
 * the heap-concatenated String approach that allocated and freed ~400 bytes
 * six times in a row (causing heap fragmentation).
 *
 * NOTE: Buffer size increased from 512 to 1024 bytes to accommodate the
 * climate (HVAC) entity payload, which contains 9 device ID substitutions
 * and multiple topic references.
 */
void MQTTManager::publishDiscovery()
{

  // Clear stale retained 'number' discovery entries for read-only remaining sensors.
  // These were previously published as 'number' (which requires command_topic).
  // Publishing an empty retained payload removes them from HA's registry.
  {
    char stale[72];
    snprintf(stale, sizeof(stale), "%s/number/%s_door_remaining/config",
             DISCOVERY_PREFIX, DEVICE_ID);
    mqtt.publish(stale, "", true);

    snprintf(stale, sizeof(stale), "%s/number/%s_light_remaining/config",
             DISCOVERY_PREFIX, DEVICE_ID);
    mqtt.publish(stale, "", true);
  }

  // Shared payload buffer - reused for every entity
  char buf[1024];

  // Precompute availability topic (needed in every payload)
  char avail[64];
  strncpy(avail, buildTopic(F("/availability")), sizeof(avail));

  char base[64];
  snprintf(base, sizeof(base), "garage/%s", DEVICE_ID);

  // ========================================================
  // Door Button
  // ========================================================
  {
    StaticJsonDocument<256> doc;

    doc["name"] = "Garage Door";

    char uniq[32];
    snprintf(uniq, sizeof(uniq), "%s_door", DEVICE_ID);
    doc["uniq_id"] = uniq;

    char topic[64];
    makeTopic(topic, sizeof(topic), base, "door/cmd");
    doc["command_topic"] = topic;

    doc["avty_t"] = avail;

    addDevice(doc.createNestedObject("dev"));

    serializeJson(doc, buf, sizeof(buf));
    mqtt.publish(buildDiscoveryTopic(F("button"), F("_door")), buf, true);
    Serial.println(F("Discovery: door button published"));
  }

  // ========================================================
  // Door State Sensor
  // ========================================================
  {
    StaticJsonDocument<256> doc;

    doc["name"] = "Garage Door State";

    char uniq[32];
    snprintf(uniq, sizeof(uniq), "%s_door_state", DEVICE_ID);
    doc["uniq_id"] = uniq;

    char topic[64];
    makeTopic(topic, sizeof(topic), base, "door/state");
    doc["state_topic"] = topic;

    doc["payload_on"] = "open";
    doc["payload_off"] = "closed";

    doc["avty_t"] = avail;

    addDevice(doc.createNestedObject("dev"));

    serializeJson(doc, buf, sizeof(buf));
    mqtt.publish(buildDiscoveryTopic(F("binary_sensor"), F("_door_state")), buf, true);
    Serial.println(F("Discovery: door state published"));
  }

  // ========================================================
  // Door Timeout Number
  // ========================================================
  {
    StaticJsonDocument<256> doc;

    doc["name"] = "Garage Door Timeout";

    char uniq[32];
    snprintf(uniq, sizeof(uniq), "%s_door_timeout", DEVICE_ID);
    doc["uniq_id"] = uniq;

    char topic[64];
    makeTopic(topic, sizeof(topic), base, "door/duration/state");
    doc["state_topic"] = topic;

    makeTopic(topic, sizeof(topic), base, "door/duration/cmd");
    doc["command_topic"] = topic;

    doc["min"] = 1;
    doc["max"] = 120;
    doc["step"] = 1;
    doc["unit_of_measurement"] = "min";

    doc["avty_t"] = avail;

    addDevice(doc.createNestedObject("dev"));

    serializeJson(doc, buf, sizeof(buf));
    mqtt.publish(buildDiscoveryTopic(F("number"), F("_door_timeout")), buf, true);
    Serial.println(F("Discovery: door timeout published"));
  }

  {
    StaticJsonDocument<256> doc;

    doc["name"] = "Garage Door Remaining";

    char uniq[32];
    snprintf(uniq, sizeof(uniq), "%s_door_remaining", DEVICE_ID);
    doc["uniq_id"] = uniq;

    char topic[64];
    makeTopic(topic, sizeof(topic), base, "door/remaining/state");
    doc["state_topic"] = topic;

    doc["min"] = 1;
    doc["max"] = 120;
    doc["step"] = 1;
    doc["unit_of_measurement"] = "min";

    doc["avty_t"] = avail;

    addDevice(doc.createNestedObject("dev"));

    serializeJson(doc, buf, sizeof(buf));
    mqtt.publish(buildDiscoveryTopic(F("sensor"), F("_door_remaining")), buf, true);
    Serial.println(F("Discovery: door remaining published"));
  }

  // ========================================================
  // Light
  // ========================================================
  {
    StaticJsonDocument<256> doc;

    doc["name"] = "Garage Light";

    char uniq[32];
    snprintf(uniq, sizeof(uniq), "%s_light", DEVICE_ID);
    doc["uniq_id"] = uniq;

    char topic[64];

    makeTopic(topic, sizeof(topic), base, "light/state");
    doc["state_topic"] = topic;

    makeTopic(topic, sizeof(topic), base, "light/cmd");
    doc["command_topic"] = topic;

    doc["payload_on"] = "on";
    doc["payload_off"] = "off";

    doc["avty_t"] = avail;
    addDevice(doc.createNestedObject("dev"));

    serializeJson(doc, buf, sizeof(buf));
    mqtt.publish(buildDiscoveryTopic(F("light"), F("_light")), buf, true);
    Serial.println(F("Discovery: light published"));
  }

  // ========================================================
  // Light Timeout Number
  // ========================================================
  {
    StaticJsonDocument<256> doc;

    doc["name"] = "Garage Light Timeout";

    char uniq[32];
    snprintf(uniq, sizeof(uniq), "%s_light_timeout", DEVICE_ID);
    doc["uniq_id"] = uniq;

    char topic[64];
    makeTopic(topic, sizeof(topic), base, "light/duration/state");
    doc["state_topic"] = topic;

    makeTopic(topic, sizeof(topic), base, "light/duration/cmd");
    doc["command_topic"] = topic;

    doc["min"] = 1;
    doc["max"] = 120;
    doc["step"] = 1;
    doc["unit_of_measurement"] = "min";

    doc["avty_t"] = avail;

    addDevice(doc.createNestedObject("dev"));

    serializeJson(doc, buf, sizeof(buf));
    mqtt.publish(buildDiscoveryTopic(F("number"), F("_light_timeout")), buf, true);
    Serial.println(F("Discovery: light timeout published"));
  }

  {
    StaticJsonDocument<256> doc;

    doc["name"] = "Garage Light Remaining";

    char uniq[32];
    snprintf(uniq, sizeof(uniq), "%s_light_remaining", DEVICE_ID);
    doc["uniq_id"] = uniq;

    char topic[64];
    makeTopic(topic, sizeof(topic), base, "light/remaining/state");
    doc["state_topic"] = topic;

    doc["min"] = 1;
    doc["max"] = 120;
    doc["step"] = 1;
    doc["unit_of_measurement"] = "min";

    doc["avty_t"] = avail;

    addDevice(doc.createNestedObject("dev"));

    serializeJson(doc, buf, sizeof(buf));
    mqtt.publish(buildDiscoveryTopic(F("sensor"), F("_light_remaining")), buf, true);
    Serial.println(F("Discovery: Light remaining published"));
  }

  // ========================================================
  // Climate
  // ========================================================
  {
    StaticJsonDocument<1024> doc;

    doc["name"] = "Garage Thermostat";

    char uniq[32];
    snprintf(uniq, sizeof(uniq), "%s_hvac", DEVICE_ID);
    doc["uniq_id"] = uniq;

    JsonArray modes = doc.createNestedArray("modes");
    modes.add("off");
    modes.add("heat");
    modes.add("cool");
    modes.add("heat_cool");

    char hvacBase[64];
    snprintf(hvacBase, sizeof(hvacBase), "%s/hvac", base);

    char topic[64];

    // Reports current HVAC mode.
    makeTopic(topic, sizeof(topic), hvacBase, "mode/state");
    doc["mode_state_topic"] = topic;

    // Sets the HVAC mode.
    makeTopic(topic, sizeof(topic), hvacBase, "mode/cmd");
    doc["mode_command_topic"] = topic;

    // Reports what device is doing.
    makeTopic(topic, sizeof(topic), hvacBase, "action/state");
    doc["action_topic"] = topic;

    // Reports room temperature.
    makeTopic(topic, sizeof(topic), hvacBase, "temp/state");
    doc["current_temperature_topic"] = topic;

    // Reports target temperature.
    makeTopic(topic, sizeof(topic), hvacBase, "heat_set/state");
    doc["temperature_state_topic"] = topic;

    // Sets target temperature.
    makeTopic(topic, sizeof(topic), hvacBase, "heat_set/cmd");
    doc["temperature_command_topic"] = topic;

    // Reports the lower bound.
    makeTopic(topic, sizeof(topic), hvacBase, "heat_set/state");
    doc["temp_low_state_topic"] = topic;

    // Sets the lower bound for Auto.
    makeTopic(topic, sizeof(topic), hvacBase, "heat_set/cmd");
    doc["temp_low_command_topic"] = topic;

    // Reports the upper bound.
    makeTopic(topic, sizeof(topic), hvacBase, "cool_set/state");
    doc["temp_high_state_topic"] = topic;

    // Sets the upper bound for Auto.
    makeTopic(topic, sizeof(topic), hvacBase, "cool_set/cmd");
    doc["temp_high_command_topic"] = topic;

    doc["temperature_unit"] = "F";
    doc["min_temp"] = 32;
    doc["max_temp"] = 90;
    doc["temp_step"] = 1;

    doc["avty_t"] = avail;

    addDevice(doc.createNestedObject("dev"));

    serializeJson(doc, buf, sizeof(buf));
    mqtt.publish(buildDiscoveryTopic(F("climate"), F("_hvac")), buf, true);
    Serial.println(F("Discovery: climate published"));
  }

  // ========================================================
  // Temperature Sensor
  // ========================================================
  {
    StaticJsonDocument<256> doc;

    doc["name"] = "Garage Temperature";

    char uniq[32];
    snprintf(uniq, sizeof(uniq), "%s_temp", DEVICE_ID);
    doc["uniq_id"] = uniq;

    doc["device_class"] = "temperature";

    char topic[64];
    makeTopic(topic, sizeof(topic), base, "hvac/temp/state");
    doc["state_topic"] = topic;

    doc["unit_of_measurement"] = "\xc2\xb0\x46";

    doc["avty_t"] = avail;

    addDevice(doc.createNestedObject("dev"));

    serializeJson(doc, buf, sizeof(buf));
    mqtt.publish(buildDiscoveryTopic(F("sensor"), F("_temp")), buf, true);
    Serial.println(F("Discovery: temp sensor published"));
  }

  // ========================================================
  // Motion Binary Sensor
  // ========================================================
  {
    StaticJsonDocument<256> doc;

    doc["name"] = "Garage Motion";

    char uniq[32];
    snprintf(uniq, sizeof(uniq), "%s_motion", DEVICE_ID);
    doc["uniq_id"] = uniq;

    doc["device_class"] = "motion";

    char topic[64];
    makeTopic(topic, sizeof(topic), base, "motion/state");
    doc["state_topic"] = topic;

    doc["payload_on"] = "on";
    doc["payload_off"] = "off";

    doc["avty_t"] = avail;

    addDevice(doc.createNestedObject("dev"));

    serializeJson(doc, buf, sizeof(buf));
    mqtt.publish(buildDiscoveryTopic(F("binary_sensor"), F("_motion")), buf, true);
    Serial.println(F("Discovery: motion sensor published"));
  }

  // ========================================================
  // HVAC Lockout Binary Sensor
  // ========================================================
  {
    StaticJsonDocument<256> doc;

    doc["name"] = "Garage HVAC Lockout";

    char uniq[32];
    snprintf(uniq, sizeof(uniq), "%s_lockout", DEVICE_ID);
    doc["uniq_id"] = uniq;

    char topic[64];
    makeTopic(topic, sizeof(topic), base, "lockout/state");
    doc["state_topic"] = topic;

    doc["payload_on"] = "Lockout";
    doc["payload_off"] = "OK";

    doc["avty_t"] = avail;

    addDevice(doc.createNestedObject("dev"));

    serializeJson(doc, buf, sizeof(buf));
    mqtt.publish(buildDiscoveryTopic(F("binary_sensor"), F("_lockout")), buf, true);
    Serial.println(F("Discovery: lockout sensor published"));
  }
}

// ============================================================
//  Network status helpers
// ============================================================

MQTTManager::NetStatus MQTTManager::getNetStatus() const
{
  return netStatus;
}

const char *MQTTManager::getNetStatusString() const
{
  switch (netStatus)
  {
  case NetStatus::Connecting:
    return "Connecting";
  case NetStatus::Connected:
    return "Connected";
  case NetStatus::Disabled:
    return "Disabled";
  }
  return "Unknown";
}

bool MQTTManager::isNetworkEnabled() const
{
  return netStatus != NetStatus::Disabled;
}

void MQTTManager::resetNetStatus()
{
  consecutiveFailures = 0;
  if (netStatus == NetStatus::Disabled)
    netStatus = NetStatus::Connecting;
}

void MQTTManager::disableNetwork()
{
  netStatus = NetStatus::Disabled;
  if (mqtt.connected())
    mqtt.disconnect();
  WiFi.disconnect();
}

void MQTTManager::getLocalIP(char *buf, size_t len) const
{
  if (WiFi.status() == WL_CONNECTED)
  {
    IPAddress ip = WiFi.localIP();
    snprintf(buf, len, "%u.%u.%u.%u", ip[0], ip[1], ip[2], ip[3]);
  }
  else
  {
    strncpy(buf, "n/a", len);
    buf[len - 1] = '\0';
  }
}

void MQTTManager::getMqttServerIP(char *buf, size_t len) const
{
  strncpy(buf, MQTT_SERVER, len);
  buf[len - 1] = '\0';
}

#endif // ENABLE_WIFI