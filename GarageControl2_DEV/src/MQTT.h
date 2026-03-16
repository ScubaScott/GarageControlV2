#ifndef MQTT_MANAGER_H
#define MQTT_MANAGER_H

#include <Arduino.h>
#include "Utility.h"

// ============================================================
//  MQTTManager is compiled ONLY when WiFi is enabled.
//  In DEV mode (ENABLE_WIFI 0) this entire class is absent,
//  so no WiFi/MQTT libraries need to be present on the board.
// ============================================================

#if ENABLE_WIFI
// #include <WiFiS3.h>
#include <PubSubClient.h>

// Forward declaration - avoids pulling in the full .ino header
class GarageController;

/**
 * @class MQTTManager
 * @brief Manages WiFi and MQTT connectivity for Home Assistant integration.
 *
 * This class handles connecting to WiFi, establishing MQTT connection,
 * publishing device states, subscribing to commands, and handling
 * Home Assistant auto-discovery.
 */
class MQTTManager
{
private:
  WiFiClient   wifiClient;
  PubSubClient mqtt;

  // MQTT topic strings
  String topicBase;

  // Configuration
  const char *WIFI_SSID;
  const char *WIFI_PASSWORD;
  const char *MQTT_SERVER;
  int MQTT_PORT;
  const char *MQTT_USER;
  const char *MQTT_PASS;
  const char *DEVICE_ID;
  const char *DEVICE_NAME;
  const char *DISCOVERY_PREFIX;

  GarageController *controller;

  unsigned long lastMqttReconnect = 0;

  // Previous states for change detection
  bool prevLightState = false;
  unsigned long prevLightDuration = 0;
  String prevDoorState = "";
  float prevTemp = -999;
  float prevHeatSet = -999;
  bool prevMotion = false;
  bool prevLockout = false;
  String prevHvacMode = "";

  void buildTopics();
  void connectWiFi();
  void connectMQTT();
  void publishDiscovery();
 
public:
  // MQTT topic strings (public for controller access)
  String DOOR_STATE_TOPIC;
  String DOOR_CMD_TOPIC;
  String LIGHT_STATE_TOPIC;
  String LIGHT_CMD_TOPIC;
  String LIGHT_DURATION_STATE_TOPIC;
  String LIGHT_DURATION_CMD_TOPIC;
  String HEAT_SET_STATE_TOPIC;
  String HEAT_SET_CMD_TOPIC;
  String TEMP_STATE_TOPIC;
  String MODE_STATE_TOPIC;
  String MODE_CMD_TOPIC;
  String MOTION_STATE_TOPIC;
  String LOCKOUT_STATE_TOPIC;
  String AVAIL_TOPIC;


  /**
   * @brief Constructor for MQTTManager.
   */
  MQTTManager();

  /**
   * @brief Initializes WiFi and MQTT connections.
   * @param ctrl Pointer to the GarageController instance.
   * @param callback C-style MQTT callback function pointer (e.g. GarageController::mqttCallback).
   */
  // FIX: Accept the callback as a parameter instead of referencing the free
  // mqttCallback() from MQTT.cpp. This breaks the circular dependency:
  // MQTT.cpp no longer needs the full GarageController definition to call
  // handleMQTT(), and the .ino passes GarageController::mqttCallback (static)
  // directly, keeping all knowledge of GarageController inside the .ino.
  void init(GarageController *ctrl, void (*callback)(char *, byte *, unsigned int));

  /**
   * @brief Main loop for maintaining connections and handling MQTT.
   */
  void loop();

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
  void publishStateChanges(bool lightOn, unsigned long durationMins, const String &doorState,
                           float tempF, float heatSet, const String &hvacMode,
                           bool motionActive, bool lockout);
};

#endif // ENABLE_WIFI
#endif // MQTT_MANAGER_H
