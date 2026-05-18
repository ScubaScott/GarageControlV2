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
 * @section Architecture — Dual independent state machines (v2.21.0)
 *
 * WiFi and MQTT are now fully independent state machines with separate failure
 * counters, retry timers, and backoff logic.
 *
 * **WiFi state machine** (serviceWiFi):
 *   Idle → Connecting → Connected → Backoff → [Disabled after MAX_WIFI_FAILURES]
 *   - 15 s connect window; 15 s between retries
 *   - After 4 consecutive failures: Disabled for 15 minutes, then auto-retry
 *
 * **MQTT state machine** (serviceMQTT):
 *   Idle → (blocking connect attempt) → Connected → Backoff
 *   - Exponential backoff: 5 s → 10 s → 20 s → 40 s → 60 s (cap)
 *   - Only runs when WiFi is in Connected phase
 *   - MQTT failures NEVER affect WiFi state or failure counter
 *   - Backoff resets to minimum on successful connect
 *
 * **Why mqtt.connect() still blocks (and why that's acceptable):**
 *   PubSubClient::connect() is synchronous — it performs the TCP handshake
 *   and MQTT CONNACK exchange in a single blocking call that may take several
 *   seconds if the broker is unreachable.  The ISR-driven light activation
 *   (pirISR → lights.activateFromISR) is unaffected by any main-loop blocking,
 *   which is the highest-priority real-time path.  Exponential backoff ensures
 *   that blocking attempts are rare (at most once per backoff period).
 *
 * @section Priority order
 *   1. Motion → light (ISR, never blocked by network)
 *   2. UI responsiveness (menu buttons; blocked only during mqtt.connect())
 *   3. MQTT publish / receive
 *   4. HVAC control
 *
 * @section Memory
 * - All string literals stored in flash via F() / PROGMEM.
 * - MQTT topics built on-demand into a single 64-byte char buffer (_topicBuf).
 * - Numeric state cache uses compact types (uint8_t, bool) — no heap Strings.
 * - dtostrf() replaces String(float,n) for float→char conversion.
 *
 * @version 2.21.0
 */
class MQTTManager
{
public:
  // ── Public status enum (used by LCD and menu) ─────────────────────────────
  /**
   * @enum NetStatus
   * @brief Aggregated network status for UI display.
   *
   * Derived from the internal WiFiPhase and MQTTPhase state machines.
   * Use getNetStatusString() for a more detailed description.
   */
  enum class NetStatus
  {
    Connecting, ///< Either WiFi or MQTT is not yet established
    Connected,  ///< WiFi up and MQTT broker connected
    Disabled    ///< WiFi disabled after MAX_WIFI_FAILURES; retries after 15 min
  };

private:
  WiFiClient    wifiClient;
  PubSubClient  mqtt;

  // ── WiFi state machine ─────────────────────────────────────────────────────
  /**
   * @enum WiFiPhase
   * @brief Internal WiFi connection phase, independent of MQTT state.
   *
   * Transitions:
   *   Idle → Connecting (WiFi.begin() issued)
   *   Connecting → Connected (WL_CONNECTED confirmed) or Backoff (timeout)
   *   Connected → Backoff (WiFi.status() drops)
   *   Backoff → Idle (retry timer expires)
   *   Backoff → Disabled (wifiFailures >= MAX_WIFI_FAILURES)
   *   Disabled → Idle (disable period expires; wifiFailures reset)
   */
  enum class WiFiPhase
  {
    Idle,       ///< Not started; next serviceWiFi() call will begin connection
    Connecting, ///< WiFi.begin() issued; polling WL_CONNECTED each loop
    Connected,  ///< WL_CONNECTED confirmed; MQTT state machine may run
    Backoff,    ///< Timed out or dropped; waiting before retry
    Disabled    ///< Max consecutive failures; long cooldown before re-enable
  };
  WiFiPhase     wifiPhase        = WiFiPhase::Idle; ///< Current WiFi phase
  uint8_t       wifiFailures     = 0;               ///< Consecutive WiFi timeouts/drops
  unsigned long wifiConnectStart = 0;               ///< millis() when WiFi.begin() last called
  unsigned long wifiRetryAt      = 0;               ///< millis() timestamp: earliest next attempt

  static constexpr uint8_t         MAX_WIFI_FAILURES          = 4;
  static constexpr unsigned long    WIFI_TIMEOUT_MS            = 15000UL; ///< Per-attempt limit
  static constexpr unsigned long    WIFI_RETRY_MS              = 15000UL; ///< Between retries
  static constexpr unsigned long    DISABLED_RETRY_INTERVAL_MS = 900000UL;///< 15-min disable period

  // ── MQTT state machine ─────────────────────────────────────────────────────
  /**
   * @enum MQTTPhase
   * @brief Internal MQTT connection phase, fully independent of WiFi failure logic.
   *
   * Note: The Idle→Connected/Backoff transition is synchronous because
   * PubSubClient::connect() is a blocking call.  Exponential backoff
   * minimises how often that blocking occurs.
   *
   * Transitions:
   *   Idle → Connected (mqtt.connect() succeeds) or Backoff (fails)
   *   Connected → Backoff (mqtt.connected() drops between loops)
   *   Backoff → Idle (mqttRetryAt expires)
   *   [Any] → Idle (WiFi drops; MQTT resets to Idle with cleared timer)
   */
  enum class MQTTPhase
  {
    Idle,       ///< Ready to attempt connection (WiFi must be Connected)
    Connected,  ///< mqtt.connected() confirmed; mqtt.loop() running each iteration
    Backoff     ///< Failed or lost; waiting mqttCurrentBackoff ms before retry
  };
  MQTTPhase     mqttPhase          = MQTTPhase::Idle; ///< Current MQTT phase
  uint8_t       mqttFailures       = 0;               ///< Consecutive MQTT connect failures
  unsigned long mqttRetryAt        = 0;               ///< millis() timestamp: earliest next attempt
  unsigned long mqttCurrentBackoff = MQTT_BACKOFF_MIN;///< Current exponential backoff interval (ms)

  static constexpr unsigned long    MQTT_BACKOFF_MIN  = 5000UL;  ///< 5 s initial MQTT backoff
  static constexpr unsigned long    MQTT_BACKOFF_MAX  = 60000UL; ///< 60 s maximum MQTT backoff

  // ── Aggregated public status ───────────────────────────────────────────────
  NetStatus netStatus = NetStatus::Connecting; ///< Updated by state machines; used by UI

  // ── Configuration (pointers into flash/ROM – zero SRAM cost) ─────────────
  const char *WIFI_SSID;
  const char *WIFI_PASSWORD;
  const char *MQTT_SERVER;
  int         MQTT_PORT;
  const char *MQTT_USER;
  const char *MQTT_PASS;
  const char *DEVICE_ID;       ///< e.g. "garage_ctrl_01"
  const char *DEVICE_NAME;
  const char *DISCOVERY_PREFIX;

  GarageController *controller;
  bool         pendingFullPublish = false; ///< Forces full state re-publish after (re)connect

  // ── Previous-state cache (change detection for publishStateChanges) ────────
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
  unsigned long prevNvDoorTimeout   = 0;  ///< Tracks last-published NV door timeout
  unsigned long prevNvLightTimeout  = 0;  ///< Tracks last-published NV light timeout
  bool          prevMotion          = false;
  bool          prevLockout         = false;
  uint8_t       prevMode            = 0xFF; ///< 0xFF = sentinel "not yet sent"
  uint8_t       prevHvacState       = 0xFF; ///< 0xFF = sentinel "not yet sent"
  bool          hasValidTemp        = false; ///< Gated until first valid DS18B20 reading

  // ── Shared topic-building buffer ───────────────────────────────────────────
  // Longest runtime topic:    "garage/garage_ctrl_01/light/duration/state" = 44 chars
  // Longest discovery topic:  "homeassistant/binary_sensor/garage_ctrl_01_door_state/config" = 61 chars
  // 64 bytes covers both with room to spare.
  char _topicBuf[64];

  // ── Private topic helpers ──────────────────────────────────────────────────

  /**
   * @brief Builds "garage/<DEVICE_ID><suffix>" into _topicBuf.
   * @param suffix Flash-string suffix, e.g. F("/door/state").
   * @return Pointer to _topicBuf (valid until next buildTopic call).
   */
  const char *buildTopic(const __FlashStringHelper *suffix);

  /**
   * @brief Builds "<DISCOVERY_PREFIX>/<component>/<DEVICE_ID><entitySuffix>/config".
   * @param component    HA platform, e.g. F("button").
   * @param entitySuffix Unique suffix, e.g. F("_door").
   * @return Pointer to _topicBuf (valid until next buildDiscoveryTopic call).
   */
  const char *buildDiscoveryTopic(const __FlashStringHelper *component,
                                  const __FlashStringHelper *entitySuffix);

  // ── Private state machine methods ──────────────────────────────────────────

  /**
   * @brief Advances the WiFi state machine by one step.
   *
   * Non-blocking: each call performs at most one WiFi action (begin or status
   * check) and returns immediately.  Called once per loop() iteration.
   *
   * WiFi failures are tracked independently; after MAX_WIFI_FAILURES the
   * network is disabled for DISABLED_RETRY_INTERVAL_MS before retrying.
   *
   * @param nowMs Current millis() snapshot passed in from loop().
   */
  void serviceWiFi(unsigned long nowMs);

  /**
   * @brief Advances the MQTT state machine by one step.
   *
   * Called only when wifiPhase == Connected.  Runs mqtt.loop() when
   * connected, or calls attemptMQTTConnect() when in Idle phase.
   *
   * MQTT failures NEVER affect WiFi state.  Failed connections use
   * exponential backoff (MQTT_BACKOFF_MIN → MQTT_BACKOFF_MAX).
   *
   * @param nowMs Current millis() snapshot passed in from loop().
   */
  void serviceMQTT(unsigned long nowMs);

  /**
   * @brief Performs the blocking MQTT connect attempt.
   *
   * @warning mqtt.connect() is synchronous.  If the broker is unreachable,
   *          this call blocks for the TCP connection timeout (typically
   *          several seconds).  Exponential backoff in serviceMQTT() limits
   *          how often this is called.
   *
   * On success: subscribes to all command topics, publishes discovery,
   *             sets pendingFullPublish, transitions phase to Connected.
   * On failure: applies exponential backoff, transitions phase to Backoff.
   *
   * @param nowMs Current millis() snapshot passed in from serviceMQTT().
   */
  void attemptMQTTConnect(unsigned long nowMs);

  /**
   * @brief Publishes all Home Assistant MQTT Discovery configuration payloads.
   *
   * Called automatically after each successful MQTT connection.  A single
   * 2048-byte stack buffer is reused for all entity payloads.
   */
  void publishDiscovery();

  /**
   * @brief Publishes a single HA discovery payload.
   * @param component    HA platform type string (flash).
   * @param entitySuffix Unique entity suffix (flash).
   * @param payloadBody  JSON payload body (flash).
   */
  void publishDiscoveryEntry(const __FlashStringHelper *component,
                             const __FlashStringHelper *entitySuffix,
                             const __FlashStringHelper *payloadBody);

public:
  /**
   * @brief Constructs the MQTTManager and loads compile-time credentials from Config.h.
   *
   * All credential strings are read from Config.h macros.  The WiFi and MQTT
   * state machines start in their Idle phases; actual connection is initiated
   * on the first loop() call.
   */
  MQTTManager();

  /**
   * @brief Initialises MQTT client parameters (does NOT connect yet).
   *
   * Configures PubSubClient with server address, callback, and buffer size.
   * The WiFi and MQTT state machines are reset to Idle; the first loop()
   * call will begin the WiFi connection sequence.
   *
   * @param ctrl      Pointer to the owning GarageController instance.
   * @param callback  PubSubClient message callback for inbound commands.
   */
  void init(GarageController *ctrl,
            void (*callback)(char *, byte *, unsigned int));

  /**
   * @brief Main network loop — services both WiFi and MQTT state machines.
   *
   * Must be called from the Arduino main loop.  The two state machines are
   * completely independent:
   *
   *   serviceWiFi():  Always called; advances WiFi connection phase.
   *   serviceMQTT():  Called only when wifiPhase == Connected.
   *
   * If WiFi drops while MQTT is connected, MQTT is immediately disconnected
   * and reset to Idle (with backoff cleared so it reconnects quickly once
   * WiFi recovers).  WiFi failures disable the stack after MAX_WIFI_FAILURES;
   * MQTT failures only increase the retry backoff — never disable WiFi.
   *
   * @note Keeping this at the END of the main loop (after motion, UI, and
   *       HVAC) ensures the potentially-blocking mqtt.connect() call only
   *       occurs after all time-critical operations have completed.
   */
  void loop();

  // ── Network status helpers ─────────────────────────────────────────────────

  /** @brief Returns aggregated NetStatus for broad status checks. */
  NetStatus   getNetStatus()       const;

  /**
   * @brief Returns a human-readable status string for LCD display.
   *
   * More granular than NetStatus — distinguishes WiFi vs MQTT phases:
   *   "WiFi..."   – connecting WiFi
   *   "WiFi Wait" – WiFi in backoff between retries
   *   "MQTT..."   – WiFi up, connecting MQTT
   *   "MQTT Wait" – MQTT in exponential backoff
   *   "Connected" – both WiFi and MQTT connected
   *   "Disabled"  – WiFi disabled after repeated failures
   */
  const char *getNetStatusString() const;

  /** @brief Returns true if network stack is not in Disabled state. */
  bool        isNetworkEnabled()   const;

  /**
   * @brief Resets both state machines to Idle and clears all failure counters.
   *
   * Forces a clean reconnect attempt on the next loop() call.  Called from
   * the MQTT menu "Connect" action (+/UP).
   */
  void        resetNetStatus();

  /**
   * @brief Immediately disconnects everything and sets Disabled state.
   *
   * Network will not be retried automatically.  Call resetNetStatus() to
   * re-enable.  Called from the MQTT menu "Disable" action (-/DOWN).
   */
  void        disableNetwork();

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
   * Valid only until the next buildTopic() call — use immediately.
   *
   * @param suffix Flash suffix string, e.g. F("/door/cmd").
   */
  const char *getTopic(const __FlashStringHelper *suffix);

  /**
   * @brief Populates a JSON "dev" object with device identity fields.
   * @param dev ArduinoJson JsonObject to populate.
   */
  void addDevice(JsonObject dev);

  /**
   * @brief Builds a topic string by concatenating base and suffix.
   * @param out    Output buffer.
   * @param len    Buffer length.
   * @param base   Base path string.
   * @param suffix Suffix to append after '/'.
   */
  void makeTopic(char *out, size_t len, const char *base, const char *suffix);

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
   * @param hvacSwing           Current live HVAC hysteresis swing (°F).
   * @param nvDoorTimeoutMins   NV door auto-close timeout (minutes).
   * @param nvLightTimeoutMins  NV light auto-off timeout (minutes).
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
};

extern MQTTManager *g_mqttManager;

#endif // ENABLE_WIFI
#endif // MQTT_MANAGER_H
