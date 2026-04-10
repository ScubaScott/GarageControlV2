#ifndef MQTT_MANAGER_H
#define MQTT_MANAGER_H

#include <Arduino.h>
#include <ArduinoJson.h>
#include "Utility.h"

// ============================================================
//  MQTTManager is compiled ONLY when WiFi is enabled.
//  In DEV mode (ENABLE_WIFI 0) this entire class is absent.
// ============================================================

#if ENABLE_WIFI

#include <WiFiS3.h>       // Built into "Arduino UNO R4 Boards" package
#include <PubSubClient.h>

class GarageController;

/**
 * @class MQTTManager
 * @brief Manages WiFi and MQTT connectivity for Home Assistant integration.
 *
 * Compiled only when ENABLE_WIFI is 1.  Set ENABLE_WIFI to 0 in Utility.h
 * for a DEV build that omits all network code.
 *
 * @section Memory
 * - All string literals are stored in flash via F() / PROGMEM.
 * - MQTT topics are built on-demand into a single 64-byte char buffer
 *   (_topicBuf) rather than persistent String members (~400 bytes of heap).
 * - prevDoorState and prevHvacMode are stored as uint8_t / bool instead of
 *   heap-allocated Strings (~60 bytes saved).
 * - dtostrf() replaces String(float,n) for float→char conversion.
 *
 * @section NVvsCurrent
 * publishStateChanges() receives BOTH current values (live subsystem state)
 * AND NV values (non-volatile EEPROM copies) for door timeout and light
 * timeout.  These are published to separate MQTT topics so that Home
 * Assistant can independently display and control each:
 *   - /door/duration/state   – current door auto-close timeout
 *   - /nv/door_timeout/state – NV (persisted) door auto-close timeout
 *   - /light/duration/state  – current light auto-off timeout
 *   - /nv/light_timeout/state – NV (persisted) light auto-off timeout
 */
class MQTTManager
{
public:
  /**
   * @enum NetStatus
   * @brief Aggregated status of the WiFi + MQTT network stack.
   */
  enum class NetStatus
  {
    Connecting, ///< Attempting to connect (initial state or after failure)
    Connected,  ///< WiFi up and MQTT broker connected
    Disabled    ///< Permanently disabled after MAX_FAILURES consecutive failures
  };

private:
  WiFiClient    wifiClient;
  PubSubClient  mqtt;

  // ── Failure tracking ─────────────────────────────────────────────────────
  NetStatus netStatus          = NetStatus::Connecting;
  uint8_t   consecutiveFailures = 0;
  static constexpr uint8_t MAX_FAILURES = 5;

  // ── Configuration (pointers into flash – zero SRAM cost) ────────────────
  const char *WIFI_SSID;
  const char *WIFI_PASSWORD;
  const char *MQTT_SERVER;
  int         MQTT_PORT;
  const char *MQTT_USER;
  const char *MQTT_PASS;
  const char *DEVICE_ID;     ///< e.g. "garage_ctrl_01"
  const char *DEVICE_NAME;
  const char *DISCOVERY_PREFIX;

  GarageController *controller;
  unsigned long lastMqttReconnect = 0;
  unsigned long lastHourlyRetry   = 0;  ///< Rate-limits reconnect when Disabled
  bool pendingFullPublish         = false; ///< Forces full re-publish after (re)connect

  // ── Previous-state cache (change detection for publishStateChanges) ──────
  // Compact types (no heap Strings) to minimise SRAM footprint.
  bool          prevLightState      = false;
  unsigned long prevLightDuration   = 0;
  unsigned long prevLightRemaining  = 0;
  uint8_t       prevDoorCode        = 0xFF; ///< 0xFF = sentinel "not yet sent"
  unsigned long prevDoorDuration    = 0;
  unsigned long prevDoorRemaining   = 0;
  float         prevTemp            = -999.0f;
  float         prevHeatSet         = -999.0f;
  float         prevCoolSet         = -999.0f;
  float         prevNvHeatSet       = -999.0f;
  float         prevNvCoolSet       = -999.0f;
  unsigned long prevHvacSwing       = 0;
  unsigned long prevNvDoorTimeout   = 0; ///< Tracks last-published NV door timeout
  unsigned long prevNvLightTimeout  = 0; ///< Tracks last-published NV light timeout
  bool          prevMotion          = false;
  bool          prevLockout         = false;
  uint8_t       prevMode            = 0xFF; ///< 0xFF = sentinel "not yet sent"
  uint8_t       prevHvacState       = 0xFF; ///< 0xFF = sentinel "not yet sent"
  bool          hasValidTemp        = false; ///< Gated until first valid DS18B20 reading

  // ── Shared topic-building buffer ─────────────────────────────────────────
  // Longest runtime topic:    "garage/garage_ctrl_01/light/duration/state" = 44 chars
  // Longest discovery topic:  "homeassistant/binary_sensor/garage_ctrl_01_lockout/config" = 57 chars
  // 64 bytes covers both with room to spare.
  char _topicBuf[64];

  /**
   * @brief Builds "garage/<DEVICE_ID><suffix>" into _topicBuf.
   *
   * @param suffix Flash string suffix, e.g. F("/door/state").
   * @return Pointer to _topicBuf (valid until next buildTopic call).
   */
  const char *buildTopic(const __FlashStringHelper *suffix);

  /**
   * @brief Builds "<DISCOVERY_PREFIX>/<component>/<DEVICE_ID><entitySuffix>/config".
   *
   * @param component    HA platform, e.g. F("button").
   * @param entitySuffix Unique suffix, e.g. F("_door").
   * @return Pointer to _topicBuf (valid until next buildDiscoveryTopic call).
   */
  const char *buildDiscoveryTopic(const __FlashStringHelper *component,
                                  const __FlashStringHelper *entitySuffix);

  void connectWiFi();
  void connectMQTT();
  void publishDiscovery();

  /**
   * @brief Publishes a single HA discovery payload.
   *
   * Topic and payload are assembled in _topicBuf and a local stack buffer
   * that are reused across all entity publications to avoid heap churn.
   *
   * @param component    HA platform type string (flash).
   * @param entitySuffix Unique entity suffix (flash).
   * @param payloadBody  JSON payload body (flash).
   */
  void publishDiscoveryEntry(const __FlashStringHelper *component,
                             const __FlashStringHelper *entitySuffix,
                             const __FlashStringHelper *payloadBody);

public:
  /**
   * @brief Constructs the MQTTManager and loads compile-time configuration.
   *
   * All credential strings are read from Config.h macros.
   */
  MQTTManager();

  /**
   * @brief Initialises WiFi, MQTT, and publishes HA discovery configs.
   *
   * @param ctrl      Pointer to the owning GarageController instance.
   * @param callback  PubSubClient message callback for inbound commands.
   */
  void init(GarageController *ctrl,
            void (*callback)(char *, byte *, unsigned int));

  /**
   * @brief Main loop handler – services MQTT and reconnects as needed.
   *
   * Must be called frequently from the main loop.  When WiFi is
   * disconnected, connectWiFi() may block up to ~15 s.  For this reason
   * the main loop calls this AFTER the time-critical motion/light code.
   */
  void loop();

  // ── Network status helpers ───────────────────────────────────────────────
  NetStatus   getNetStatus()       const;
  const char *getNetStatusString() const;
  bool        isNetworkEnabled()   const;
  void        resetNetStatus();   ///< Reset failure counter and retry connections
  void        disableNetwork();   ///< Force-disable all network activity

  /**
   * @brief Writes the local IP address into buf.
   * @param buf Output buffer.
   * @param len Buffer length in bytes.
   */
  void getLocalIP(char *buf, size_t len) const;

  /**
   * @brief Writes the configured MQTT server address into buf.
   * @param buf Output buffer.
   * @param len Buffer length in bytes.
   */
  void getMqttServerIP(char *buf, size_t len) const;

  /**
   * @brief Returns _topicBuf containing the requested runtime topic.
   *
   * Valid only until the next buildTopic() call – use immediately.
   *
   * @param suffix Flash suffix string, e.g. F("/door/cmd").
   */
  const char *getTopic(const __FlashStringHelper *suffix);

  /**
   * @brief Publishes changed sensor and state values to MQTT.
   *
   * Change-detected: only values that differ from the previous publish are
   * transmitted.  On (re)connect, all prev-state sentinels are invalidated
   * via pendingFullPublish so the next call publishes everything.
   *
   * Temperature publication is additionally gated until a valid DS18B20
   * reading is received (hasValidTemp) to avoid sending DEVICE_DISCONNECTED
   * sentinel values to Home Assistant on startup.
   *
   * @param lightOn             Current light relay state (true = on).
   * @param lightDurationMins   Current live light auto-off timeout (minutes).
   * @param lightRemainingMins  Minutes remaining before light auto-off.
   * @param doorCode            Door state: 0=open, 1=closed, 2=moving, 3=error, 4=disabled.
   * @param doorDurationMins    Current live door auto-close timeout (minutes).
   * @param doorRemainingMins   Minutes remaining before door auto-close.
   * @param tempF               Current temperature reading (°F).
   * @param heatSet             Current live HVAC heat setpoint (°F).
   * @param coolSet             Current live HVAC cool setpoint (°F).
   * @param nvHeatSet           NV (persisted) HVAC heat setpoint (°F).
   * @param nvCoolSet           NV (persisted) HVAC cool setpoint (°F).
   * @param nvDoorTimeoutMins   NV (persisted) door auto-close timeout (minutes).
   *                            Independent of doorDurationMins – may differ when
   *                            the user has changed the current setting without
   *                            saving it back to NV.
   * @param nvLightTimeoutMins  NV (persisted) light auto-off timeout (minutes).
   *                            Independent of lightDurationMins – may differ when
   *                            the user has changed the current setting without
   *                            saving it back to NV.
   * @param mode                HVAC mode: 0=Off, 1=Heat, 2=Heat_Cool, 3=Cool.
   * @param hvacState           HVAC runtime state: 0=Waiting, 1=Heating, 2=Cooling, 3=Pending.
   * @param motionActive        True if PIR sensor is currently reading HIGH.
   * @param lockout             True when HVAC is locked out (door not closed).
   */
  void publishStateChanges(bool          lightOn,
                           unsigned long lightDurationMins,
                           unsigned long lightRemainingMins,
                           uint8_t       doorCode,
                           unsigned long doorDurationMins,
                           unsigned long doorRemainingMins,
                           float         tempF,
                           float         heatSet,
                           float         coolSet,
                           float         nvHeatSet,
                           float         nvCoolSet,
                           unsigned long hvacSwing,
                           unsigned long nvDoorTimeoutMins,
                           unsigned long nvLightTimeoutMins,
                           uint8_t       mode,
                           uint8_t       hvacState,
                           bool          motionActive,
                           bool          lockout);

  /**
   * @brief Populates a JSON "dev" object with device identity fields.
   * @param dev ArduinoJson JsonObject to populate.
   */
  void addDevice(JsonObject dev);

  /**
   * @brief Builds a topic string by concatenating base and suffix.
   * @param out   Output buffer.
   * @param len   Buffer length.
   * @param base  Base path string.
   * @param suffix Suffix to append after '/'.
   */
  void makeTopic(char *out, size_t len, const char *base, const char *suffix);
};

extern MQTTManager *g_mqttManager;

#endif // ENABLE_WIFI
#endif // MQTT_MANAGER_H
