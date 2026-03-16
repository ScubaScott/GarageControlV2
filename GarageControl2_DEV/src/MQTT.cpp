/**
 * @file MQTT.cpp
 * @brief MQTTManager implementation - compiled only when ENABLE_WIFI is 1.
 *
 * The entire translation unit is wrapped in #if ENABLE_WIFI so the compiler
 * skips it completely in DEV builds, avoiding any dependency on WiFiS3 or
 * PubSubClient.
 */

#include "Utility.h"
#include "MQTT.h"

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
  WIFI_SSID = "ScubaSpot";
  WIFI_PASSWORD = "ScubaNet";
  MQTT_SERVER = "192.168.0.130";
  MQTT_PORT = 1883;
  MQTT_USER = "mqtt_user";
  MQTT_PASS = "mqtt_password";
  DEVICE_ID = "garage_ctrl_01";
  DEVICE_NAME = "Garage Controller";
  DISCOVERY_PREFIX = "homeassistant";
}

/**
 * @brief Initializes WiFi and MQTT connections.
 * @param ctrl Pointer to the GarageController instance.
* @param callback C-style MQTT callback function pointer.
 */
// FIX: Signature updated to accept callback pointer — matches updated header.
void MQTTManager::init(GarageController *ctrl, void (*callback)(char *, byte *, unsigned int)) {
  controller = ctrl;
  buildTopics();
  connectWiFi();
  mqtt.setServer(MQTT_SERVER, MQTT_PORT);
  // FIX: Use the passed-in callback pointer instead of the removed free
  // mqttCallback() function, breaking the circular dependency.
  mqtt.setCallback(callback);
  mqtt.setBufferSize(768);
  connectMQTT();
  publishDiscovery();
}

/**
 * @brief Main loop for maintaining connections and handling MQTT.
 */
void MQTTManager::loop() {
  // Keep WiFi alive
  if (WiFi.status() != WL_CONNECTED)
    connectWiFi();

  // Keep MQTT alive
  if (!mqtt.connected()) {
    if (millis() - lastMqttReconnect >= 5000UL) {
      lastMqttReconnect = millis();
      connectMQTT();
    }
  }
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
void MQTTManager::publishStateChanges(bool lightOn, unsigned long durationMins, const String &doorState,
                                      float tempF, float heatSet, const String &hvacMode,
                                      bool motionActive, bool lockout) {
  if (!mqtt.connected())
    return;

  // Light
  if (lightOn != prevLightState) {
    mqtt.publish(LIGHT_STATE_TOPIC.c_str(), lightOn ? "ON" : "OFF", true);
    prevLightState = lightOn;
  }

  // Light timeout
  if (durationMins != prevLightDuration) {
    mqtt.publish(LIGHT_DURATION_STATE_TOPIC.c_str(), String(durationMins).c_str(), true);
    prevLightDuration = durationMins;
  }

  // Door
  if (doorState != prevDoorState) {
    mqtt.publish(DOOR_STATE_TOPIC.c_str(), doorState.c_str(), true);
    prevDoorState = doorState;
  }

  // Temperature
  if (abs(tempF - prevTemp) >= 0.2) {
    mqtt.publish(TEMP_STATE_TOPIC.c_str(), String(tempF, 1).c_str(), true);
    prevTemp = tempF;
  }

  // Heat setpoint
  if (abs(heatSet - prevHeatSet) >= 0.5) {
    mqtt.publish(HEAT_SET_STATE_TOPIC.c_str(), String(heatSet, 1).c_str(), true);
    prevHeatSet = heatSet;
  }

  // HVAC mode
  if (hvacMode != prevHvacMode) {
    mqtt.publish(MODE_STATE_TOPIC.c_str(), hvacMode.c_str(), true);
    prevHvacMode = hvacMode;
  }

  // Motion
  if (motionActive != prevMotion) {
    mqtt.publish(MOTION_STATE_TOPIC.c_str(), motionActive ? "ON" : "OFF", true);
    prevMotion = motionActive;
  }

  // Lockout
  if (lockout != prevLockout) {
    mqtt.publish(LOCKOUT_STATE_TOPIC.c_str(), lockout ? "ON" : "OFF", true);
    prevLockout = lockout;
  }
}

/**
 * @brief Builds MQTT topic strings from DEVICE_ID.
 */
void MQTTManager::buildTopics() {
  topicBase = String("garage/") + DEVICE_ID;

  DOOR_STATE_TOPIC = topicBase + "/door/state";
  DOOR_CMD_TOPIC = topicBase + "/door/cmd";

  LIGHT_STATE_TOPIC = topicBase + "/light/state";
  LIGHT_CMD_TOPIC = topicBase + "/light/cmd";
  LIGHT_DURATION_STATE_TOPIC = topicBase + "/light/duration/state";
  LIGHT_DURATION_CMD_TOPIC = topicBase + "/light/duration/cmd";

  HEAT_SET_STATE_TOPIC = topicBase + "/hvac/heat_set/state";
  HEAT_SET_CMD_TOPIC = topicBase + "/hvac/heat_set/cmd";
  TEMP_STATE_TOPIC = topicBase + "/hvac/temp/state";
  MODE_STATE_TOPIC = topicBase + "/hvac/mode/state";
  MODE_CMD_TOPIC = topicBase + "/hvac/mode/cmd";

  MOTION_STATE_TOPIC = topicBase + "/motion/state";
  LOCKOUT_STATE_TOPIC = topicBase + "/lockout/state";

  AVAIL_TOPIC = topicBase + "/availability";
}

/**
 * @brief Connects to WiFi network.
 */
void MQTTManager::connectWiFi() 
{
  Serial.print("Connecting to WiFi: ");
  Serial.println(WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < 15000UL) {
    delay(500);
    Serial.print(".");
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("\nWiFi connected, IP: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("\nWiFi connect failed – will retry");
  }
}

/**
 * @brief Connects to MQTT broker and subscribes to topics.
 */
void MQTTManager::connectMQTT() 
{
  String clientId = String(DEVICE_ID) + "_" + String(random(0xffff), HEX);
  Serial.print("Connecting MQTT... ");
  bool ok;
  if (strlen(MQTT_USER) > 0) {
    ok = mqtt.connect(clientId.c_str(), MQTT_USER, MQTT_PASS,
                      AVAIL_TOPIC.c_str(), 1, true, "offline");
  } else {
    ok = mqtt.connect(clientId.c_str(), nullptr, nullptr,
                      AVAIL_TOPIC.c_str(), 1, true, "offline");
  }

  if (ok) {
    Serial.println("connected");
    mqtt.publish(AVAIL_TOPIC.c_str(), "online", true);

    // Subscribe to command topics
    mqtt.subscribe(DOOR_CMD_TOPIC.c_str());
    mqtt.subscribe(LIGHT_CMD_TOPIC.c_str());
    mqtt.subscribe(HEAT_SET_CMD_TOPIC.c_str());
    mqtt.subscribe(MODE_CMD_TOPIC.c_str());
    mqtt.subscribe(LIGHT_DURATION_CMD_TOPIC.c_str());

    // Re-publish discovery after reconnect so HA picks up entities
    publishDiscovery();
  } else {
    Serial.print("MQTT failed rc=");
    Serial.println(mqtt.state());
  }
}

/**
 * @brief Publishes Home Assistant auto-discovery configurations.
 */
void MQTTManager::publishDiscovery() {
  // Shared device block (JSON fragment)
  String dev = String("\"dev\":{\"ids\":[\"") + DEVICE_ID + "\"],"
                                                            "\"name\":\""
               + DEVICE_NAME + "\","
                               "\"mf\":\"Arduino\",\"mdl\":\"UNO R4 WiFi\"}";

  String avail = String("\"avty_t\":\"") + AVAIL_TOPIC + "\"";

  // ── 1. Button (garage door toggle) ──────────────────
  {
    String topic = String(DISCOVERY_PREFIX) + "/button/" + DEVICE_ID + "_door/config";
    String payload = String("{") + "\"name\":\"Garage Door\"," + "\"uniq_id\":\"" + DEVICE_ID + "_door\"," + "\"command_topic\":\"" + DOOR_CMD_TOPIC + "\"," + avail + "," + dev + "}";
    mqtt.publish(topic.c_str(), payload.c_str(), true);
    Serial.println("Discovery: door button published");
  }

  // ── 2. Light ─────────────────────────────────────
  {
    String topic = String(DISCOVERY_PREFIX) + "/light/" + DEVICE_ID + "_light/config";
    String payload = String("{") + "\"name\":\"Garage Light\"," + "\"uniq_id\":\"" + DEVICE_ID + "_light\"," + "\"state_topic\":\"" + LIGHT_STATE_TOPIC + "\"," + "\"command_topic\":\"" + LIGHT_CMD_TOPIC + "\"," + "\"payload_on\":\"ON\"," + "\"payload_off\":\"OFF\"," + avail + "," + dev + "}";
    mqtt.publish(topic.c_str(), payload.c_str(), true);
    Serial.println("Discovery: light published");
  }

  // ── 2a. Number (light timeout) ─────────────────────
  {
    String topic = String(DISCOVERY_PREFIX) + "/number/" + DEVICE_ID + "_light_timeout/config";
    String payload = String("{") + "\"name\":\"Garage Light Timeout\"," + "\"uniq_id\":\"" + DEVICE_ID + "_light_timeout\"," + "\"state_topic\":\"" + LIGHT_DURATION_STATE_TOPIC + "\"," + "\"command_topic\":\"" + LIGHT_DURATION_CMD_TOPIC + "\"," + "\"min\":1," + "\"max\":120," + "\"step\":1," + "\"unit_of_measurement\":\"min\"," + avail + "," + dev + "}";
    mqtt.publish(topic.c_str(), payload.c_str(), true);
    Serial.println("Discovery: light timeout published");
  }

  // ── 3. Climate (heating thermostat) ──────────────
  {
    String topic = String(DISCOVERY_PREFIX) + "/climate/" + DEVICE_ID + "_hvac/config";
    String payload = String("{") + "\"name\":\"Garage Heater\"," + "\"uniq_id\":\"" + DEVICE_ID + "_hvac\"," + "\"modes\":[\"off\",\"heat\"]," + "\"mode_state_topic\":\"" + MODE_STATE_TOPIC + "\"," + "\"mode_command_topic\":\"" + MODE_CMD_TOPIC + "\"," + "\"temperature_state_topic\":\"" + HEAT_SET_STATE_TOPIC + "\"," + "\"temperature_command_topic\":\"" + HEAT_SET_CMD_TOPIC + "\"," + "\"current_temperature_topic\":\"" + TEMP_STATE_TOPIC + "\"," + "\"temperature_unit\":\"F\"," + "\"min_temp\":32," + "\"max_temp\":90," + "\"temp_step\":1," + avail + "," + dev + "}";
    mqtt.publish(topic.c_str(), payload.c_str(), true);
    Serial.println("Discovery: climate published");
  }

  // ── 4. Temperature sensor ─────────────────────────
  {
    String topic = String(DISCOVERY_PREFIX) + "/sensor/" + DEVICE_ID + "_temp/config";
    String payload = String("{") + "\"name\":\"Garage Temperature\"," + "\"uniq_id\":\"" + DEVICE_ID + "_temp\"," + "\"device_class\":\"temperature\"," + "\"state_topic\":\"" + TEMP_STATE_TOPIC + "\"," + "\"unit_of_measurement\":\"°F\"," + avail + "," + dev + "}";
    mqtt.publish(topic.c_str(), payload.c_str(), true);
    Serial.println("Discovery: temp sensor published");
  }

  // ── 5. Motion binary sensor ───────────────────────
  {
    String topic = String(DISCOVERY_PREFIX) + "/binary_sensor/" + DEVICE_ID + "_motion/config";
    String payload = String("{") + "\"name\":\"Garage Motion\"," + "\"uniq_id\":\"" + DEVICE_ID + "_motion\"," + "\"device_class\":\"motion\"," + "\"state_topic\":\"" + MOTION_STATE_TOPIC + "\"," + "\"payload_on\":\"ON\"," + "\"payload_off\":\"OFF\"," + avail + "," + dev + "}";
    mqtt.publish(topic.c_str(), payload.c_str(), true);
    Serial.println("Discovery: motion sensor published");
  }

  // ── 6. HVAC lockout binary sensor ─────────────────
  {
    String topic = String(DISCOVERY_PREFIX) + "/binary_sensor/" + DEVICE_ID + "_lockout/config";
    String payload = String("{") + "\"name\":\"Garage HVAC Lockout\"," + "\"uniq_id\":\"" + DEVICE_ID + "_lockout\"," + "\"device_class\":\"problem\"," + "\"state_topic\":\"" + LOCKOUT_STATE_TOPIC + "\"," + "\"payload_on\":\"ON\"," + "\"payload_off\":\"OFF\"," + avail + "," + dev + "}";
    mqtt.publish(topic.c_str(), payload.c_str(), true);
    Serial.println("Discovery: lockout sensor published");
  }
}

#endif // ENABLE_WIFI
