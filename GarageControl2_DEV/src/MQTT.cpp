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

#include "Utility.h"
#include "MQTT.h"

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
 */
MQTTManager::MQTTManager()
  : mqtt(wifiClient)
{
  // All literals go to flash; the pointers themselves cost 2 bytes each.
  WIFI_SSID        = "ScubaSpot";
  WIFI_PASSWORD    = "ScubaNet";
  MQTT_SERVER      = "192.168.0.0"; //130
  MQTT_PORT        = 1883;
  MQTT_USER        = "mqtt_user";
  MQTT_PASS        = "mqtt_password";
  DEVICE_ID        = "garage_ctrl_01";
  DEVICE_NAME      = "Garage Controller";
  DISCOVERY_PREFIX = "homeassistant";
}

// ============================================================
//  Topic helpers
// ============================================================

/**
 * Builds "garage/<DEVICE_ID><suffix>" into _topicBuf.
 * suffix is a flash string, e.g. F("/door/state").
 * Returns pointer to _topicBuf.
 */
const char* MQTTManager::buildTopic(const __FlashStringHelper *suffix)
{
  snprintf_P(_topicBuf, sizeof(_topicBuf),
             PSTR("garage/%s%S"), DEVICE_ID, suffix);
  return _topicBuf;
}

/**
 * Builds "<DISCOVERY_PREFIX>/<component>/<DEVICE_ID><entitySuffix>/config"
 * Returns pointer to _topicBuf.
 */
const char* MQTTManager::buildDiscoveryTopic(const __FlashStringHelper *component,
                                             const __FlashStringHelper *entitySuffix)
{
  snprintf_P(_topicBuf, sizeof(_topicBuf),
             PSTR("%s/%S/%s%S/config"),
             DISCOVERY_PREFIX, component, DEVICE_ID, entitySuffix);
  return _topicBuf;
}

/** Public accessor used by GarageController::handleMQTT(). */
const char* MQTTManager::getTopic(const __FlashStringHelper *suffix)
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
  mqtt.setBufferSize(768);
  connectMQTT();
  publishDiscovery();
}

void MQTTManager::loop()
{
  if (netStatus == NetStatus::Disabled)
    return;

  if (WiFi.status() != WL_CONNECTED)
    connectWiFi();

  if (!mqtt.connected()) {
    netStatus = NetStatus::Connecting;
    if (millis() - lastMqttReconnect >= 5000UL) {
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
 * @param durationMins Light timeout in minutes.
 * @param doorState Current door state string.
 * @param tempF Current temperature.
 * @param heatSet Heat setpoint.
 * @param hvacMode HVAC mode string.
 * @param motionActive Motion sensor state.
 * @param lockout HVAC lockout state.
 */
void MQTTManager::publishStateChanges(bool          lightOn,
                                      unsigned long durationMins,
                                      uint8_t       doorCode,
                                      float         tempF,
                                      float         heatSet,
                                      bool          hvacOn,
                                      bool          motionActive,
                                      bool          lockout)
{
  if (netStatus == NetStatus::Disabled) return;
  if (!mqtt.connected()) return;

  // Use a small stack buffer for numeric payloads (replaces heap String temps)
  char val[12];

  if (lightOn != prevLightState) {
    mqtt.publish(buildTopic(F("/light/state")),
                 lightOn ? "ON" : "OFF", true);
    prevLightState = lightOn;
  }

  if (durationMins != prevLightDuration) {
    snprintf(val, sizeof(val), "%lu", durationMins);
    mqtt.publish(buildTopic(F("/light/duration/state")), val, true);
    prevLightDuration = durationMins;
  }

  if (doorCode != prevDoorCode) {
    // Convert code to payload string without heap allocation
    const char *ds;
    switch (doorCode) {
      case 0: ds = "open";     break;
      case 1: ds = "closed";   break;
      case 2: ds = "opening";  break;
      case 3: ds = "error";    break;
      default: ds = "disabled"; break;
    }
    mqtt.publish(buildTopic(F("/door/state")), ds, true);
    prevDoorCode = doorCode;
  }

  if (abs(tempF - prevTemp) >= 0.2f) {
    dtostrf(tempF, 4, 1, val);
    mqtt.publish(buildTopic(F("/hvac/temp/state")), val, true);
    prevTemp = tempF;
  }

  if (abs(heatSet - prevHeatSet) >= 0.5f) {
    dtostrf(heatSet, 4, 1, val);
    mqtt.publish(buildTopic(F("/hvac/heat_set/state")), val, true);
    prevHeatSet = heatSet;
  }

  if (hvacOn != prevHvacOn) {
    mqtt.publish(buildTopic(F("/hvac/mode/state")),
                 hvacOn ? "heat" : "off", true);
    prevHvacOn = hvacOn;
  }

  if (motionActive != prevMotion) {
    mqtt.publish(buildTopic(F("/motion/state")),
                 motionActive ? "ON" : "OFF", true);
    prevMotion = motionActive;
  }

  if (lockout != prevLockout) {
    mqtt.publish(buildTopic(F("/lockout/state")),
                 lockout ? "ON" : "OFF", true);
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
  while (WiFi.status() != WL_CONNECTED && millis() - start < 15000UL) {
    delay(500);
    Serial.print(F("."));
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.print(F("\nWiFi connected, IP: "));
    Serial.println(WiFi.localIP());
    consecutiveFailures = 0;
  } else {
    consecutiveFailures++;
    Serial.println(F("\nWiFi connect failed"));
    if (consecutiveFailures >= MAX_FAILURES) {
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
  if (strlen(MQTT_USER) > 0) {
    ok = mqtt.connect(clientId, MQTT_USER, MQTT_PASS,
                      avail, 1, true, "offline");
  } else {
    ok = mqtt.connect(clientId, nullptr, nullptr,
                      avail, 1, true, "offline");
  }

  if (ok) {
    Serial.println(F("connected"));
    mqtt.publish(avail, "online", true);

    mqtt.subscribe(buildTopic(F("/door/cmd")));
    mqtt.subscribe(buildTopic(F("/light/cmd")));
    mqtt.subscribe(buildTopic(F("/hvac/heat_set/cmd")));
    mqtt.subscribe(buildTopic(F("/hvac/mode/cmd")));
    mqtt.subscribe(buildTopic(F("/light/duration/cmd")));

    publishDiscovery();

    netStatus = NetStatus::Connected;
    consecutiveFailures = 0;
  } else {
    consecutiveFailures++;
    Serial.print(F("MQTT failed rc="));
    Serial.println(mqtt.state());
    if (consecutiveFailures >= MAX_FAILURES) {
      netStatus = NetStatus::Disabled;
      Serial.println(F("Network disabled due to repeated failures"));
      return;
    }
  }
}

/**
 * Publishes all HA auto-discovery configs.
 *
 * A single 512-byte stack buffer is reused for every payload, replacing
 * the heap-concatenated String approach that allocated and freed ~400 bytes
 * six times in a row (causing heap fragmentation).
 */
void MQTTManager::publishDiscovery()
{
  // Shared payload buffer - reused for every entity
  char buf[512];

  // Precompute availability topic (needed in every payload)
  char avail[64];
  strncpy(avail, buildTopic(F("/availability")), sizeof(avail));

  // ── 1. Button (door toggle) ──────────────────────────────────────────
  snprintf_P(buf, sizeof(buf),
    PSTR("{\"name\":\"Garage Door\","
         "\"uniq_id\":\"%s_door\","
         "\"command_topic\":\"garage/%s/door/cmd\","
         "\"avty_t\":\"%s\","
         "\"dev\":{\"ids\":[\"%s\"],\"name\":\"%s\",\"mf\":\"Arduino\",\"mdl\":\"UNO R4 WiFi\"}"
         "}"),
    DEVICE_ID, DEVICE_ID, avail, DEVICE_ID, DEVICE_NAME);
  mqtt.publish(buildDiscoveryTopic(F("button"), F("_door")), buf, true);
  Serial.println(F("Discovery: door button published"));

  // ── 2. Light ─────────────────────────────────────────────────────────
  snprintf_P(buf, sizeof(buf),
    PSTR("{\"name\":\"Garage Light\","
         "\"uniq_id\":\"%s_light\","
         "\"state_topic\":\"garage/%s/light/state\","
         "\"command_topic\":\"garage/%s/light/cmd\","
         "\"payload_on\":\"ON\",\"payload_off\":\"OFF\","
         "\"avty_t\":\"%s\","
         "\"dev\":{\"ids\":[\"%s\"],\"name\":\"%s\",\"mf\":\"Arduino\",\"mdl\":\"UNO R4 WiFi\"}"
         "}"),
    DEVICE_ID, DEVICE_ID, DEVICE_ID, avail, DEVICE_ID, DEVICE_NAME);
  mqtt.publish(buildDiscoveryTopic(F("light"), F("_light")), buf, true);
  Serial.println(F("Discovery: light published"));

  // ── 2a. Number (light timeout) ────────────────────────────────────────
  snprintf_P(buf, sizeof(buf),
    PSTR("{\"name\":\"Garage Light Timeout\","
         "\"uniq_id\":\"%s_light_timeout\","
         "\"state_topic\":\"garage/%s/light/duration/state\","
         "\"command_topic\":\"garage/%s/light/duration/cmd\","
         "\"min\":1,\"max\":120,\"step\":1,"
         "\"unit_of_measurement\":\"min\","
         "\"avty_t\":\"%s\","
         "\"dev\":{\"ids\":[\"%s\"],\"name\":\"%s\",\"mf\":\"Arduino\",\"mdl\":\"UNO R4 WiFi\"}"
         "}"),
    DEVICE_ID, DEVICE_ID, DEVICE_ID, avail, DEVICE_ID, DEVICE_NAME);
  mqtt.publish(buildDiscoveryTopic(F("number"), F("_light_timeout")), buf, true);
  Serial.println(F("Discovery: light timeout published"));

  // ── 3. Climate ────────────────────────────────────────────────────────
  snprintf_P(buf, sizeof(buf),
    PSTR("{\"name\":\"Garage Heater\","
         "\"uniq_id\":\"%s_hvac\","
         "\"modes\":[\"off\",\"heat\"],"
         "\"mode_state_topic\":\"garage/%s/hvac/mode/state\","
         "\"mode_command_topic\":\"garage/%s/hvac/mode/cmd\","
         "\"temperature_state_topic\":\"garage/%s/hvac/heat_set/state\","
         "\"temperature_command_topic\":\"garage/%s/hvac/heat_set/cmd\","
         "\"current_temperature_topic\":\"garage/%s/hvac/temp/state\","
         "\"temperature_unit\":\"F\","
         "\"min_temp\":32,\"max_temp\":90,\"temp_step\":1,"
         "\"avty_t\":\"%s\","
         "\"dev\":{\"ids\":[\"%s\"],\"name\":\"%s\",\"mf\":\"Arduino\",\"mdl\":\"UNO R4 WiFi\"}"
         "}"),
    DEVICE_ID,
    DEVICE_ID, DEVICE_ID, DEVICE_ID, DEVICE_ID, DEVICE_ID,
    avail, DEVICE_ID, DEVICE_NAME);
  mqtt.publish(buildDiscoveryTopic(F("climate"), F("_hvac")), buf, true);
  Serial.println(F("Discovery: climate published"));

  // ── 4. Temperature sensor ─────────────────────────────────────────────
  snprintf_P(buf, sizeof(buf),
    PSTR("{\"name\":\"Garage Temperature\","
         "\"uniq_id\":\"%s_temp\","
         "\"device_class\":\"temperature\","
         "\"state_topic\":\"garage/%s/hvac/temp/state\","
         "\"unit_of_measurement\":\"\xc2\xb0\x46\","
         "\"avty_t\":\"%s\","
         "\"dev\":{\"ids\":[\"%s\"],\"name\":\"%s\",\"mf\":\"Arduino\",\"mdl\":\"UNO R4 WiFi\"}"
         "}"),
    DEVICE_ID, DEVICE_ID, avail, DEVICE_ID, DEVICE_NAME);
  mqtt.publish(buildDiscoveryTopic(F("sensor"), F("_temp")), buf, true);
  Serial.println(F("Discovery: temp sensor published"));

  // ── 5. Motion binary sensor ───────────────────────────────────────────
  snprintf_P(buf, sizeof(buf),
    PSTR("{\"name\":\"Garage Motion\","
         "\"uniq_id\":\"%s_motion\","
         "\"device_class\":\"motion\","
         "\"state_topic\":\"garage/%s/motion/state\","
         "\"payload_on\":\"ON\",\"payload_off\":\"OFF\","
         "\"avty_t\":\"%s\","
         "\"dev\":{\"ids\":[\"%s\"],\"name\":\"%s\",\"mf\":\"Arduino\",\"mdl\":\"UNO R4 WiFi\"}"
         "}"),
    DEVICE_ID, DEVICE_ID, avail, DEVICE_ID, DEVICE_NAME);
  mqtt.publish(buildDiscoveryTopic(F("binary_sensor"), F("_motion")), buf, true);
  Serial.println(F("Discovery: motion sensor published"));

  // ── 6. HVAC lockout binary sensor ─────────────────────────────────────
  snprintf_P(buf, sizeof(buf),
    PSTR("{\"name\":\"Garage HVAC Lockout\","
         "\"uniq_id\":\"%s_lockout\","
         "\"device_class\":\"problem\","
         "\"state_topic\":\"garage/%s/lockout/state\","
         "\"payload_on\":\"ON\",\"payload_off\":\"OFF\","
         "\"avty_t\":\"%s\","
         "\"dev\":{\"ids\":[\"%s\"],\"name\":\"%s\",\"mf\":\"Arduino\",\"mdl\":\"UNO R4 WiFi\"}"
         "}"),
    DEVICE_ID, DEVICE_ID, avail, DEVICE_ID, DEVICE_NAME);
  mqtt.publish(buildDiscoveryTopic(F("binary_sensor"), F("_lockout")), buf, true);
  Serial.println(F("Discovery: lockout sensor published"));
}

// ============================================================
//  Network status helpers
// ============================================================

MQTTManager::NetStatus MQTTManager::getNetStatus() const
{
  return netStatus;
}

const char* MQTTManager::getNetStatusString() const
{
  switch (netStatus) {
    case NetStatus::Connecting: return "Connecting";
    case NetStatus::Connected:  return "Connected";
    case NetStatus::Disabled:   return "Disabled";
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
  if (WiFi.status() == WL_CONNECTED) {
    IPAddress ip = WiFi.localIP();
    snprintf(buf, len, "%u.%u.%u.%u", ip[0], ip[1], ip[2], ip[3]);
  } else {
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
