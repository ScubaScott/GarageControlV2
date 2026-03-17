#ifndef MQTT_MANAGER_H
#define MQTT_MANAGER_H

#include <Arduino.h>
#include "Utility.h"

// ============================================================
//  MQTTManager is compiled ONLY when WiFi is enabled.
//  In DEV mode (ENABLE_WIFI 0) this entire class is absent.
// ============================================================

#if ENABLE_WIFI

#include <WiFiS3.h>   // Built into "Arduino UNO R4 Boards" package
#include <PubSubClient.h>

class GarageController;

/**
 * @class MQTTManager
 * @brief Manages WiFi and MQTT connectivity for Home Assistant integration.
 *
 * Compiled only when ENABLE_WIFI is 1. Set ENABLE_WIFI to 0 in Utility.h
 * for a DEV build that omits all network code.
 *
 * Memory notes
 * ─────────────
 * • All string literals are stored in flash via F() / PROGMEM - not SRAM.
 * • MQTT topics are NOT stored as persistent String members. They are built
 *   on-demand from the DEVICE_ID root each time they are needed, using a
 *   small shared char buffer.  This removes ~400 bytes of heap strings.
 * • prevDoorState and prevHvacMode are stored as single bytes (enum/flag)
 *   instead of heap-allocated String objects.
 */
class MQTTManager
{
public:
  /**
   * Status of the network stack (WiFi + MQTT).
   */
  enum class NetStatus
  {
    Connecting,
    Connected,
    Disabled
  };

private:
  WiFiClient   wifiClient;
  PubSubClient mqtt;

  // Network status reporting / failure tracking
  NetStatus netStatus = NetStatus::Connecting;
  uint8_t  consecutiveFailures = 0;
  static constexpr uint8_t MAX_FAILURES = 5;

  // ── Configuration (all point to flash literals - zero SRAM cost) ──────
  const char *WIFI_SSID;
  const char *WIFI_PASSWORD;
  const char *MQTT_SERVER;
  int         MQTT_PORT;
  const char *MQTT_USER;
  const char *MQTT_PASS;
  const char *DEVICE_ID;       // e.g. "garage_ctrl_01"
  const char *DEVICE_NAME;
  const char *DISCOVERY_PREFIX;

  GarageController *controller;
  unsigned long lastMqttReconnect = 0;

  // ── Previous-state cache (change-detection for publishStateChanges) ────
  // Use compact types instead of String objects to save heap.
  bool          prevLightState    = false;
  unsigned long prevLightDuration = 0;
  uint8_t       prevDoorCode      = 0xFF;  // 0xFF = "not yet sent"
  float         prevTemp          = -999;
  float         prevHeatSet       = -999;
  bool          prevMotion        = false;
  bool          prevLockout       = false;
  bool          prevHvacOn        = false; // false=off true=heat

  // ── Shared topic-building buffer ───────────────────────────────────────
  // Longest topic: "garage/garage_ctrl_01/light/duration/state" = 44 chars
  // Longest discovery topic: "homeassistant/binary_sensor/garage_ctrl_01_lockout/config" = 57 chars
  // 64 bytes covers both with room to spare.
  char _topicBuf[64];

  // Build a runtime topic into _topicBuf and return a pointer to it.
  // suffix must be a flash string (F() macro), e.g. buildTopic(F("/door/state"))
  const char* buildTopic(const __FlashStringHelper *suffix);

  // Build a discovery topic, e.g. buildDiscoveryTopic(F("button"), F("_door"))
  const char* buildDiscoveryTopic(const __FlashStringHelper *component,
                                  const __FlashStringHelper *entitySuffix);

  void connectWiFi();
  void connectMQTT();
  void publishDiscovery();

  // Publish a single discovery entry. topic/payload are built in _topicBuf
  // and a second local buffer; both are reused across calls.
  void publishDiscoveryEntry(const __FlashStringHelper *component,
                             const __FlashStringHelper *entitySuffix,
                             const __FlashStringHelper *payloadBody);

public:
  MQTTManager();

  void init(GarageController *ctrl,
            void (*callback)(char *, byte *, unsigned int));

  void loop();

  // Network status helpers
  NetStatus getNetStatus() const;
  const char* getNetStatusString() const;
  bool isNetworkEnabled() const;
  void resetNetStatus();      // Reset failure counter and retry connections
  void disableNetwork();      // Force disable network/MQTT activity

  // Returns the current local IP address (or "n/a" if not connected).
  void getLocalIP(char *buf, size_t len) const;

  // Returns the configured MQTT server/IP string.
  void getMqttServerIP(char *buf, size_t len) const;

  // Returns a pointer to _topicBuf containing the requested topic.
  // Valid only until the next buildTopic() call - use immediately.
  const char* getTopic(const __FlashStringHelper *suffix);

  void publishStateChanges(bool          lightOn,
                           unsigned long durationMins,
                           uint8_t       doorCode,
                           float         tempF,
                           float         heatSet,
                           bool          hvacOn,
                           bool          motionActive,
                           bool          lockout);
};

extern MQTTManager *g_mqttManager;

#endif // ENABLE_WIFI
#endif // MQTT_MANAGER_H
