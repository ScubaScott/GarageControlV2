/**
 * @file MQTT.cpp
 * @brief MQTTManager implementation – compiled only when ENABLE_WIFI is 1.
 *
 * @section Architecture (v2.21.0) — Two independent state machines
 *
 * **WiFi** and **MQTT** are now fully decoupled.  Each has its own phase
 * enum, failure counter, and retry timer.  The overall NetStatus seen by
 * the UI is derived from both phases but neither can affect the other's
 * failure accounting.
 *
 * serviceWiFi() is called unconditionally every loop().
 * serviceMQTT() is called only when wifiPhase == Connected.
 *
 * When WiFi drops while MQTT is active, MQTT is immediately disconnected
 * and reset to Idle (backoff cleared) so it reconnects quickly once WiFi
 * recovers.  WiFi failure accounting is unaffected by MQTT outcomes.
 *
 * @section Blocking behaviour of mqtt.connect()
 *
 * PubSubClient::connect() performs a synchronous TCP handshake + MQTT
 * CONNACK exchange.  If the broker is unreachable this blocks for the
 * TCP connection timeout (potentially several seconds).
 *
 * Mitigation strategy:
 *   - Exponential backoff: 5 s → 10 s → 20 s → 40 s → 60 s (cap).
 *     After the first failure, connect attempts are infrequent.
 *   - ISR-driven light activation (pirISR) is interrupt-based and
 *     completely unaffected by main-loop blocking.
 *   - mqttManager.loop() is positioned LAST in the main control loop,
 *     after motion detection, UI, and HVAC have all been serviced.
 *
 * @section Memory strategy
 * 1. F() macro  – all string literals placed in flash (PROGMEM).
 * 2. No persistent topic Strings – topics built on-demand into _topicBuf.
 * 3. Compact state cache – uint8_t / bool instead of heap Strings.
 * 4. Discovery payloads – assembled in a single reused 2048-byte stack buffer.
 * 5. dtostrf() replaces String(float,n) for float→char conversion.
 *
 * @section NV vs current value separation
 * publishStateChanges() accepts BOTH the live timeout values AND the NV
 * (persisted) values as distinct parameters.  They are compared against
 * separate prev-state members and published to separate MQTT topics.
 *
 * @version 2.21.0
 */

#include <ArduinoJson.h>
#include "Utility.h"
#include "MQTT.h"
#include "../config/Config.h"

/** @brief Global pointer used by the LCD and menu to query network status. */
MQTTManager *g_mqttManager = nullptr;

#if ENABLE_WIFI

// ============================================================
//  Constructor
// ============================================================

/**
 * @brief Constructs MQTTManager and loads compile-time credentials from Config.h.
 *
 * All credential macros resolve to string literals that live in flash; only
 * the 2-byte pointers themselves occupy SRAM.  State machines start in Idle;
 * actual connection is initiated by the first loop() call.
 */
MQTTManager::MQTTManager()
    : mqtt(wifiClient)
{
  WIFI_SSID        = CONFIG_WIFI_SSID;
  WIFI_PASSWORD    = CONFIG_WIFI_PASSWORD;
  MQTT_SERVER      = CONFIG_MQTT_SERVER;
  MQTT_PORT        = CONFIG_MQTT_PORT;
  MQTT_USER        = CONFIG_MQTT_USER;
  MQTT_PASS        = CONFIG_MQTT_PASSWORD;
  DEVICE_ID        = CONFIG_DEVICE_ID;
  DEVICE_NAME      = CONFIG_DEVICE_NAME;
  DISCOVERY_PREFIX = CONFIG_DISCOVERY_PREFIX;
}

// ============================================================
//  Topic helpers
// ============================================================

/**
 * @brief Appends device identity info to a JsonObject for HA discovery.
 * @param dev ArduinoJson JsonObject to populate.
 */
void MQTTManager::addDevice(JsonObject dev)
{
  JsonArray ids = dev.createNestedArray("ids");
  ids.add(DEVICE_ID);
  dev["name"] = DEVICE_NAME;
  dev["mf"]   = "Arduino";
  dev["mdl"]  = "UNO R4 WiFi";
}

/**
 * @brief Concatenates base + "/" + suffix into out.
 * @param out    Output buffer.
 * @param len    Buffer length in bytes.
 * @param base   Base path.
 * @param suffix Suffix string.
 */
void MQTTManager::makeTopic(char *out, size_t len,
                             const char *base, const char *suffix)
{
  snprintf(out, len, "%s/%s", base, suffix);
}

/**
 * @brief Builds "garage/<DEVICE_ID><suffix>" into _topicBuf.
 *
 * Uses strncat_P for the flash-string suffix to avoid unreliable %S
 * behaviour on some toolchains.
 *
 * @param suffix Flash string suffix, e.g. F("/door/state").
 * @return Pointer to _topicBuf (valid until next call).
 */
const char *MQTTManager::buildTopic(const __FlashStringHelper *suffix)
{
  strcpy(_topicBuf, "garage/");
  strncat(_topicBuf, DEVICE_ID,
          sizeof(_topicBuf) - strlen(_topicBuf) - 1);
  strncat_P(_topicBuf, (PGM_P)suffix,
            sizeof(_topicBuf) - strlen(_topicBuf) - 1);
  return _topicBuf;
}

/**
 * @brief Builds the HA discovery topic for one entity into _topicBuf.
 *
 * Format: "<DISCOVERY_PREFIX>/<component>/<DEVICE_ID><entitySuffix>/config"
 *
 * @param component    HA platform type, e.g. F("button").
 * @param entitySuffix Entity-unique suffix, e.g. F("_door").
 * @return Pointer to _topicBuf (valid until next call).
 */
const char *MQTTManager::buildDiscoveryTopic(const __FlashStringHelper *component,
                                             const __FlashStringHelper *entitySuffix)
{
  strcpy(_topicBuf, DISCOVERY_PREFIX);
  strncat(_topicBuf, "/",            sizeof(_topicBuf) - strlen(_topicBuf) - 1);
  strncat_P(_topicBuf, (PGM_P)component,
            sizeof(_topicBuf) - strlen(_topicBuf) - 1);
  strncat(_topicBuf, "/",            sizeof(_topicBuf) - strlen(_topicBuf) - 1);
  strncat(_topicBuf, DEVICE_ID,      sizeof(_topicBuf) - strlen(_topicBuf) - 1);
  strncat_P(_topicBuf, (PGM_P)entitySuffix,
            sizeof(_topicBuf) - strlen(_topicBuf) - 1);
  strncat(_topicBuf, "/config",      sizeof(_topicBuf) - strlen(_topicBuf) - 1);
  return _topicBuf;
}

/** @brief Public alias for buildTopic() used by GarageController::handleMQTT(). */
const char *MQTTManager::getTopic(const __FlashStringHelper *suffix)
{
  return buildTopic(suffix);
}

// ============================================================
//  Public API — init()
// ============================================================

/**
 * @brief Configures the MQTT client and registers global pointer.
 *
 * Does NOT initiate any WiFi or MQTT connection.  The state machines
 * start in Idle; the first loop() call begins the WiFi connection sequence.
 *
 * @param ctrl     Owning GarageController instance.
 * @param callback PubSubClient inbound-message callback.
 */
void MQTTManager::init(GarageController *ctrl,
                       void (*callback)(char *, byte *, unsigned int))
{
  controller    = ctrl;
  g_mqttManager = this;

  // Configure PubSubClient (no connection attempt yet).
  mqtt.setServer(MQTT_SERVER, MQTT_PORT);
  mqtt.setCallback(callback);
  // Buffer size 2048 accommodates the large climate discovery payload.
  mqtt.setBufferSize(2048);

  // Reset both state machines to Idle.
  wifiPhase          = WiFiPhase::Idle;
  wifiFailures       = 0;
  wifiConnectStart   = 0;
  wifiRetryAt        = 0;

  mqttPhase          = MQTTPhase::Idle;
  mqttFailures       = 0;
  mqttRetryAt        = 0;
  mqttCurrentBackoff = MQTT_BACKOFF_MIN;

  netStatus          = NetStatus::Connecting;

  Serial.println(F("MQTT:Manager initialised — state machines in Idle"));
}

// ============================================================
//  Main loop dispatcher
// ============================================================

/**
 * @brief Services both WiFi and MQTT state machines.
 *
 * Called once per Arduino loop() iteration, positioned LAST so that
 * the potentially-blocking mqtt.connect() in attemptMQTTConnect() only
 * fires after motion detection, UI buttons, and HVAC have been serviced.
 *
 * Flow:
 *   1. serviceWiFi() — always called; may transition between Idle,
 *      Connecting, Connected, Backoff, or Disabled.
 *   2. serviceMQTT() — called only when wifiPhase == Connected.
 *      If WiFi is not up, MQTT is disconnected and reset.
 */
void MQTTManager::loop()
{
  unsigned long nowMs = millis();

  // ── Step 1: Advance WiFi state machine ─────────────────────────────────
  serviceWiFi(nowMs);

  // ── Step 2: Advance MQTT state machine (WiFi must be up) ───────────────
  if (wifiPhase == WiFiPhase::Connected)
  {
    serviceMQTT(nowMs);
  }
  else
  {
    // WiFi is not connected — ensure MQTT is torn down cleanly.
    if (mqtt.connected())
    {
      Serial.println(F("MQTT:WiFi down — disconnecting MQTT"));
      mqtt.disconnect();
    }
    // If MQTT thought it was connected, reset to Idle so it reconnects
    // quickly once WiFi recovers.  Preserve backoff if it was already
    // in Backoff (might indicate a broker issue, not just WiFi).
    if (mqttPhase == MQTTPhase::Connected)
    {
      mqttPhase          = MQTTPhase::Idle;
      mqttCurrentBackoff = MQTT_BACKOFF_MIN; // Fresh start when WiFi returns
      mqttRetryAt        = 0;
    }
  }
}

// ============================================================
//  WiFi state machine
// ============================================================

/**
 * @brief Advances the WiFi state machine by one step.
 *
 * Non-blocking: returns immediately after each phase check.  A WiFi
 * connection attempt is started (Idle→Connecting) or its status is
 * polled (Connecting) without any blocking delay.
 *
 * Failure accounting:
 *   - Each timeout increments wifiFailures.
 *   - Drops from Connected reset wifiFailures (WiFi was working).
 *   - After MAX_WIFI_FAILURES timeouts, WiFi enters Disabled state for
 *     DISABLED_RETRY_INTERVAL_MS before attempting again.
 *
 * @param nowMs Current millis() snapshot.
 */
void MQTTManager::serviceWiFi(unsigned long nowMs)
{
  switch (wifiPhase)
  {
    // ── Idle: begin a new connection attempt ────────────────────────────
    case WiFiPhase::Idle:
      Serial.print(F("WiFi:Connecting to "));
      Serial.println(WIFI_SSID);
      WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
      wifiConnectStart = nowMs;
      wifiPhase        = WiFiPhase::Connecting;
      netStatus        = NetStatus::Connecting;
      break;

    // ── Connecting: poll for WL_CONNECTED or timeout ────────────────────
    case WiFiPhase::Connecting:
      if (WiFi.status() == WL_CONNECTED)
      {
        // Success: reset failure counter and allow MQTT to start.
        wifiPhase    = WiFiPhase::Connected;
        wifiFailures = 0;
        // netStatus stays Connecting until MQTT is also up.
        Serial.print(F("WiFi:Connected — IP: "));
        Serial.println(WiFi.localIP());
      }
      else if (nowMs - wifiConnectStart >= WIFI_TIMEOUT_MS)
      {
        // Timed out: back off before retrying.
        wifiFailures++;
        Serial.print(F("WiFi:Connect timeout (failure "));
        Serial.print(wifiFailures);
        Serial.println(F(")"));

        if (wifiFailures >= MAX_WIFI_FAILURES)
        {
          // Disable the network stack for a long cooldown period.
          wifiPhase   = WiFiPhase::Disabled;
          wifiRetryAt = nowMs + DISABLED_RETRY_INTERVAL_MS;
          netStatus   = NetStatus::Disabled;
          Serial.println(F("WiFi:Disabled — max failures reached; retry in 15 min"));
        }
        else
        {
          wifiPhase   = WiFiPhase::Backoff;
          wifiRetryAt = nowMs + WIFI_RETRY_MS;
          netStatus   = NetStatus::Connecting;
          Serial.println(F("WiFi:Backing off before retry"));
        }
      }
      // else: still within the 15-second window — return and check next loop
      break;

    // ── Connected: watch for unexpected drops ───────────────────────────
    case WiFiPhase::Connected:
      if (WiFi.status() != WL_CONNECTED)
      {
        Serial.println(F("WiFi:Connection lost"));
        // Tear down MQTT immediately; reset it to Idle with cleared backoff
        // so it reconnects quickly once WiFi recovers.
        if (mqtt.connected()) mqtt.disconnect();
        mqttPhase          = MQTTPhase::Idle;
        mqttCurrentBackoff = MQTT_BACKOFF_MIN;
        mqttRetryAt        = 0;

        // Count this as a failure only if we've never successfully connected
        // at length.  Drops after a working session don't count toward the
        // initial-connect failure limit.
        wifiPhase   = WiFiPhase::Backoff;
        wifiRetryAt = nowMs + WIFI_RETRY_MS;
        netStatus   = NetStatus::Connecting;
      }
      break;

    // ── Backoff: wait before next attempt ──────────────────────────────
    case WiFiPhase::Backoff:
      if (nowMs >= wifiRetryAt)
      {
        Serial.println(F("WiFi:Backoff expired — retrying"));
        wifiPhase = WiFiPhase::Idle;
      }
      break;

    // ── Disabled: wait for long cooldown, then restart ──────────────────
    case WiFiPhase::Disabled:
      if (nowMs >= wifiRetryAt)
      {
        Serial.println(F("WiFi:Disable period expired — re-enabling"));
        wifiPhase    = WiFiPhase::Idle;
        wifiFailures = 0;
        netStatus    = NetStatus::Connecting;
      }
      break;
  }
}

// ============================================================
//  MQTT state machine
// ============================================================

/**
 * @brief Advances the MQTT state machine by one step.
 *
 * Only called when wifiPhase == Connected.
 *
 * - If mqtt.connected(): calls mqtt.loop() (non-blocking receive pump).
 * - If phase was Connected but mqtt.connected() dropped: enters Backoff
 *   and doubles the backoff interval (exponential).
 * - If in Backoff: returns until mqttRetryAt expires.
 * - If Idle: calls the blocking attemptMQTTConnect().
 *
 * MQTT failures never affect WiFi failure accounting.
 *
 * @param nowMs Current millis() snapshot.
 */
void MQTTManager::serviceMQTT(unsigned long nowMs)
{
  // ── Already connected: run the MQTT receive pump ──────────────────────
  if (mqtt.connected())
  {
    mqttPhase = MQTTPhase::Connected;
    netStatus = NetStatus::Connected;
    mqtt.loop();
    return;
  }

  // ── Detect unexpected disconnect ──────────────────────────────────────
  if (mqttPhase == MQTTPhase::Connected)
  {
    // Was confirmed connected; now mqtt.connected() returned false.
    mqttFailures++;
    Serial.print(F("MQTT:Connection lost (failure "));
    Serial.print(mqttFailures);
    Serial.print(F(") — backoff "));
    Serial.print(mqttCurrentBackoff / 1000UL);
    Serial.println(F(" s"));

    mqttPhase        = MQTTPhase::Backoff;
    mqttRetryAt      = nowMs + mqttCurrentBackoff;
    // Double backoff for next failure, capped at maximum.
    mqttCurrentBackoff = min(mqttCurrentBackoff * 2UL, MQTT_BACKOFF_MAX);
    netStatus        = NetStatus::Connecting;
    return;
  }

  // ── In backoff: wait until retry window opens ─────────────────────────
  if (mqttPhase == MQTTPhase::Backoff)
  {
    if (nowMs >= mqttRetryAt)
    {
      Serial.println(F("MQTT:Backoff expired — retrying"));
      mqttPhase = MQTTPhase::Idle;
    }
    // else: still waiting; return without blocking
    return;
  }

  // ── Idle: attempt connection (BLOCKING call inside) ───────────────────
  // attemptMQTTConnect() will transition phase to Connected or Backoff.
  attemptMQTTConnect(nowMs);
}

// ============================================================
//  MQTT connect attempt
// ============================================================

/**
 * @brief Performs one blocking MQTT connection attempt.
 *
 * Called from serviceMQTT() when mqttPhase == Idle.
 *
 * @warning mqtt.connect() is synchronous.  TCP timeout latency is
 *          determined by the underlying WiFiS3 WiFiClient implementation
 *          and is not directly configurable in all library versions.
 *          Exponential backoff in serviceMQTT() limits how often this
 *          blocking call occurs.
 *
 * On success:
 *   - Publishes "online" to availability topic.
 *   - Subscribes to all live /cmd and /nv/ command topics.
 *   - Calls publishDiscovery() to (re)register all HA entities.
 *   - Sets pendingFullPublish so all state is re-pushed to HA.
 *   - Resets mqttCurrentBackoff to minimum.
 *   - Transitions mqttPhase to Connected.
 *
 * On failure:
 *   - Increments mqttFailures counter.
 *   - Applies current exponential backoff.
 *   - Doubles backoff for next failure.
 *   - Transitions mqttPhase to Backoff.
 *
 * @param nowMs Current millis() snapshot.
 */
void MQTTManager::attemptMQTTConnect(unsigned long nowMs)
{
  // Build a unique client ID to prevent broker-side stale-session collisions.
  char clientId[32];
  snprintf(clientId, sizeof(clientId), "%s_%04x",
           DEVICE_ID, (unsigned)random(0xffff));

  // Copy availability topic before any further buildTopic() calls that
  // would overwrite _topicBuf.
  char avail[64];
  strncpy(avail, buildTopic(F("/availability")), sizeof(avail));
  avail[sizeof(avail) - 1] = '\0';

  Serial.print(F("MQTT:Connecting (backoff="));
  Serial.print(mqttCurrentBackoff / 1000UL);
  Serial.println(F(" s)..."));

  // ── Blocking connect ──────────────────────────────────────────────────
  // Will publish a retained "offline" LWT if connection drops unexpectedly.
  bool ok;
  if (strlen(MQTT_USER) > 0)
    ok = mqtt.connect(clientId, MQTT_USER, MQTT_PASS, avail, 1, true, "offline");
  else
    ok = mqtt.connect(clientId, nullptr, nullptr,     avail, 1, true, "offline");

  if (!ok)
  {
    // ── Connection failed ─────────────────────────────────────────────
    mqttFailures++;
    Serial.print(F("MQTT:Connect failed rc="));
    Serial.print(mqtt.state());
    Serial.print(F(" (failure "));
    Serial.print(mqttFailures);
    Serial.print(F(") — next retry in "));
    Serial.print(mqttCurrentBackoff / 1000UL);
    Serial.println(F(" s"));

    mqttPhase        = MQTTPhase::Backoff;
    mqttRetryAt      = nowMs + mqttCurrentBackoff;
    // Double backoff for next failure, capped at maximum.
    mqttCurrentBackoff = min(mqttCurrentBackoff * 2UL, MQTT_BACKOFF_MAX);
    netStatus        = NetStatus::Connecting;
    return;
  }

  // ── Connection succeeded ──────────────────────────────────────────────
  Serial.println(F("MQTT:Connected"));
  mqtt.publish(avail, "online", true);

  // ── Subscribe to live /cmd topics (Rule 2 — live values only) ─────────
  mqtt.subscribe(buildTopic(F("/door/cmd")));
  mqtt.subscribe(buildTopic(F("/door/duration/cmd")));
  mqtt.subscribe(buildTopic(F("/light/cmd")));
  mqtt.subscribe(buildTopic(F("/light/duration/cmd")));
  mqtt.subscribe(buildTopic(F("/hvac/heat_set/cmd")));
  mqtt.subscribe(buildTopic(F("/hvac/cool_set/cmd")));
  mqtt.subscribe(buildTopic(F("/hvac/mode/cmd")));
  mqtt.subscribe(buildTopic(F("/hvac/swing/cmd")));  // live HVAC swing

  // ── Subscribe to NV /nv/ topics (Rule 3 — RAM only, no auto-save) ─────
  mqtt.subscribe(buildTopic(F("/nv/hvac/heat_set/cmd")));
  mqtt.subscribe(buildTopic(F("/nv/hvac/cool_set/cmd")));
  mqtt.subscribe(buildTopic(F("/nv/door_timeout/cmd")));
  mqtt.subscribe(buildTopic(F("/nv/light_timeout/cmd")));

  // ── Subscribe to NV control topics ────────────────────────────────────
  mqtt.subscribe(buildTopic(F("/nv/save/cmd")));    // persist NV → EEPROM
  mqtt.subscribe(buildTopic(F("/nv/reload/cmd")));  // NV → live subsystems

  // ── Publish HA discovery for all entities ─────────────────────────────
  // Called on every successful connect so HA re-registers entities even
  // if HA was restarted while we were offline.
  publishDiscovery();

  // ── Force full state re-publish ───────────────────────────────────────
  // Invalidates all prev-state sentinels so the next publishStateChanges()
  // call overwrites any stale retained values in HA.
  pendingFullPublish = true;

  // ── Reset MQTT tracking ───────────────────────────────────────────────
  mqttPhase          = MQTTPhase::Connected;
  mqttCurrentBackoff = MQTT_BACKOFF_MIN; // Reset to minimum on success
  mqttFailures       = 0;               // Clear failure streak
  netStatus          = NetStatus::Connected;
}

// ============================================================
//  publishStateChanges
// ============================================================

/**
 * @brief Publishes changed sensor and state values to MQTT.
 *
 * Only values that differ from the previous publish are transmitted.
 * On (re)connect, pendingFullPublish invalidates all prev-state sentinels
 * so the next call immediately overwrites stale retained HA values.
 *
 * Temperature is additionally gated until the first valid DS18B20 reading
 * (hasValidTemp) to prevent DEVICE_DISCONNECTED sentinels reaching HA.
 */
void MQTTManager::publishStateChanges(bool          lightOn,
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
                                      bool          lockout)
{
  // Guard: only publish when the MQTT connection is active.
  if (netStatus == NetStatus::Disabled) return;
  if (!mqtt.connected())               return;

  // ── Full publish after (re)connect ────────────────────────────────────
  // Invalidates all prev-state sentinels so every value is (re)published
  // on this call, overwriting any stale retained values in HA.
  // Temperature is excluded — it remains gated separately by hasValidTemp.
  if (pendingFullPublish)
  {
    prevLightState     = !lightOn;
    prevLightDuration  = lightDurationMins  + 1;
    prevLightRemaining = lightRemainingMins + 1;
    prevDoorCode       = 0xFF;
    prevDoorDuration   = doorDurationMins   + 1;
    prevDoorRemaining  = doorRemainingMins  + 1;
    prevHeatSet        = heatSet    + 10.0f;
    prevCoolSet        = coolSet    + 10.0f;
    prevNvHeatSet      = nvHeatSet  + 10.0f;
    prevNvCoolSet      = nvCoolSet  + 10.0f;
    prevHvacSwing      = hvacSwing  + 1;
    prevNvDoorTimeout  = nvDoorTimeoutMins  + 1;
    prevNvLightTimeout = nvLightTimeoutMins + 1;
    prevMode           = 0xFF;
    prevHvacState      = 0xFF;
    prevMotion         = !motionActive;
    prevLockout        = !lockout;
    // prevTemp intentionally NOT reset — gated separately by hasValidTemp.
    pendingFullPublish = false;
  }

  // Small stack buffer for numeric payloads — avoids heap String temporaries.
  char val[12];

  // ── Light state ────────────────────────────────────────────────────────
  if (lightOn != prevLightState)
  {
    mqtt.publish(buildTopic(F("/light/state")), lightOn ? "on" : "off", true);
    prevLightState = lightOn;
  }

  // ── Current light timeout ──────────────────────────────────────────────
  if (lightDurationMins != prevLightDuration)
  {
    snprintf(val, sizeof(val), "%lu", lightDurationMins);
    mqtt.publish(buildTopic(F("/light/duration/state")), val, true);
    prevLightDuration = lightDurationMins;
  }

  // ── Light remaining ────────────────────────────────────────────────────
  if (lightRemainingMins != prevLightRemaining)
  {
    snprintf(val, sizeof(val), "%lu", lightRemainingMins);
    mqtt.publish(buildTopic(F("/light/remaining/state")), val, true);
    prevLightRemaining = lightRemainingMins;
  }

  // ── Door state ─────────────────────────────────────────────────────────
  if (doorCode != prevDoorCode)
  {
    const char *ds;
    switch (doorCode)
    {
    case 0:  ds = "open";     break;
    case 1:  ds = "closed";   break;
    case 2:  ds = "opening";  break;
    case 3:  ds = "error";    break;
    default: ds = "disabled"; break;
    }
    mqtt.publish(buildTopic(F("/door/state")), ds, true);
    prevDoorCode = doorCode;
  }

  // ── Current door timeout ───────────────────────────────────────────────
  if (doorDurationMins != prevDoorDuration)
  {
    snprintf(val, sizeof(val), "%lu", doorDurationMins);
    mqtt.publish(buildTopic(F("/door/duration/state")), val, true);
    prevDoorDuration = doorDurationMins;
  }

  // ── Door remaining ─────────────────────────────────────────────────────
  if (doorRemainingMins != prevDoorRemaining)
  {
    snprintf(val, sizeof(val), "%lu", doorRemainingMins);
    mqtt.publish(buildTopic(F("/door/remaining/state")), val, true);
    prevDoorRemaining = doorRemainingMins;
  }

  // ── Temperature (gated until first valid reading) ──────────────────────
  if (!hasValidTemp)
  {
    // Accept the reading once it is within DS18B20's valid operating range.
    if (!isnan(tempF) && tempF > -40.0f && tempF < 150.0f)
      hasValidTemp = true;
  }
  if (hasValidTemp && abs(tempF - prevTemp) >= 0.2f)
  {
    dtostrf(tempF, 4, 1, val);
    mqtt.publish(buildTopic(F("/hvac/temp/state")), val, true);
    prevTemp = tempF;
  }

  // ── Live HVAC heat setpoint ────────────────────────────────────────────
  if (abs(heatSet - prevHeatSet) >= 0.5f)
  {
    dtostrf(heatSet, 4, 1, val);
    mqtt.publish(buildTopic(F("/hvac/heat_set/state")), val, true);
    prevHeatSet = heatSet;
  }

  // ── Live HVAC cool setpoint ────────────────────────────────────────────
  if (abs(coolSet - prevCoolSet) >= 0.5f)
  {
    dtostrf(coolSet, 4, 1, val);
    mqtt.publish(buildTopic(F("/hvac/cool_set/state")), val, true);
    prevCoolSet = coolSet;
  }

  // ── Live HVAC swing ────────────────────────────────────────────────────
  if (hvacSwing != prevHvacSwing)
  {
    snprintf(val, sizeof(val), "%lu", hvacSwing);
    mqtt.publish(buildTopic(F("/hvac/swing/state")), val, true);
    prevHvacSwing = hvacSwing;
  }

  // ── NV HVAC heat setpoint ──────────────────────────────────────────────
  if (abs(nvHeatSet - prevNvHeatSet) >= 0.5f)
  {
    dtostrf(nvHeatSet, 4, 1, val);
    mqtt.publish(buildTopic(F("/nv/hvac/heat_set/state")), val, true);
    prevNvHeatSet = nvHeatSet;
  }

  // ── NV HVAC cool setpoint ──────────────────────────────────────────────
  if (abs(nvCoolSet - prevNvCoolSet) >= 0.5f)
  {
    dtostrf(nvCoolSet, 4, 1, val);
    mqtt.publish(buildTopic(F("/nv/hvac/cool_set/state")), val, true);
    prevNvCoolSet = nvCoolSet;
  }

  // ── NV door timeout (independent of live doorDurationMins) ────────────
  if (nvDoorTimeoutMins != prevNvDoorTimeout)
  {
    snprintf(val, sizeof(val), "%lu", nvDoorTimeoutMins);
    mqtt.publish(buildTopic(F("/nv/door_timeout/state")), val, true);
    prevNvDoorTimeout = nvDoorTimeoutMins;
  }

  // ── NV light timeout (independent of live lightDurationMins) ──────────
  if (nvLightTimeoutMins != prevNvLightTimeout)
  {
    snprintf(val, sizeof(val), "%lu", nvLightTimeoutMins);
    mqtt.publish(buildTopic(F("/nv/light_timeout/state")), val, true);
    prevNvLightTimeout = nvLightTimeoutMins;
  }

  // ── HVAC mode ──────────────────────────────────────────────────────────
  if (mode != prevMode)
  {
    const char *modeStr;
    switch (mode)
    {
    case 0:  modeStr = "off";       break; // GarageHVAC::Off
    case 1:  modeStr = "heat";      break; // GarageHVAC::Heat
    case 2:  modeStr = "heat_cool"; break; // GarageHVAC::Heat_Cool
    case 3:  modeStr = "cool";      break; // GarageHVAC::Cool
    default: modeStr = "off";       break;
    }
    mqtt.publish(buildTopic(F("/hvac/mode/state")), modeStr, true);
    prevMode = mode;
  }

  // ── HVAC runtime action ────────────────────────────────────────────────
  if (hvacState != prevHvacState)
  {
    const char *action;
    switch (hvacState)
    {
    case 0:  action = "idle";     break; // GarageHVAC::Waiting
    case 1:  action = "heating";  break; // GarageHVAC::Heating
    case 2:  action = "cooling";  break; // GarageHVAC::Cooling
    case 3:  action = "pending";  break; // GarageHVAC::Pending
    default: action = "unknown";  break;
    }
    mqtt.publish(buildTopic(F("/hvac/action/state")), action, true);
    prevHvacState = hvacState;
  }

  // ── Motion sensor ──────────────────────────────────────────────────────
  if (motionActive != prevMotion)
  {
    mqtt.publish(buildTopic(F("/motion/state")),
                 motionActive ? "on" : "off", true);
    prevMotion = motionActive;
  }

  // ── HVAC lockout ───────────────────────────────────────────────────────
  if (lockout != prevLockout)
  {
    mqtt.publish(buildTopic(F("/lockout/state")),
                 lockout ? "Lockout" : "OK", true);
    prevLockout = lockout;
  }
}

// ============================================================
//  HA Auto-Discovery
// ============================================================

/**
 * @brief Publishes all Home Assistant MQTT Discovery configuration payloads.
 *
 * A single 2048-byte stack buffer is reused for every entity payload.
 * Called automatically after each successful MQTT (re)connect.
 *
 * Tombstones stale 'number' discovery entries for 'remaining' sensors that
 * were previously published with a command_topic (causing HA import errors).
 */
void MQTTManager::publishDiscovery()
{
  // ── Remove stale retained discovery entries ────────────────────────────
  {
    char stale[72];
    snprintf(stale, sizeof(stale), "%s/number/%s_door_remaining/config",
             DISCOVERY_PREFIX, DEVICE_ID);
    mqtt.publish(stale, "", true);

    snprintf(stale, sizeof(stale), "%s/number/%s_light_remaining/config",
             DISCOVERY_PREFIX, DEVICE_ID);
    mqtt.publish(stale, "", true);
  }

  // ── Shared stack buffers ───────────────────────────────────────────────
  char buf[2048]; // Reused for every payload; increased to prevent truncation.

  char avail[64];
  strncpy(avail, buildTopic(F("/availability")), sizeof(avail));
  avail[sizeof(avail) - 1] = '\0';

  char base[64];
  snprintf(base, sizeof(base), "garage/%s", DEVICE_ID);

  // ── Door button ────────────────────────────────────────────────────────
  {
    StaticJsonDocument<256> doc;
    char uniq[32]; snprintf(uniq, sizeof(uniq), "%s_door", DEVICE_ID);
    char topic[64]; makeTopic(topic, sizeof(topic), base, "door/cmd");
    doc["name"]          = "Garage Door";
    doc["uniq_id"]       = uniq;
    doc["command_topic"] = topic;
    doc["avty_t"]        = avail;
    addDevice(doc.createNestedObject("dev"));
    serializeJson(doc, buf, sizeof(buf));
    mqtt.publish(buildDiscoveryTopic(F("button"), F("_door")), buf, true);
    Serial.println(F("Discovery: door button published"));
  }

  // ── Door state binary sensor ───────────────────────────────────────────
  {
    StaticJsonDocument<256> doc;
    char uniq[32]; snprintf(uniq, sizeof(uniq), "%s_door_state", DEVICE_ID);
    char topic[64]; makeTopic(topic, sizeof(topic), base, "door/state");
    doc["name"]        = "Garage Door State";
    doc["uniq_id"]     = uniq;
    doc["state_topic"] = topic;
    doc["payload_on"]  = "open";
    doc["payload_off"] = "closed";
    doc["avty_t"]      = avail;
    addDevice(doc.createNestedObject("dev"));
    serializeJson(doc, buf, sizeof(buf));
    mqtt.publish(buildDiscoveryTopic(F("binary_sensor"), F("_door_state")), buf, true);
    Serial.println(F("Discovery: door state published"));
  }

  // ── Current door timeout number ────────────────────────────────────────
  {
    StaticJsonDocument<256> doc;
    char uniq[32]; snprintf(uniq, sizeof(uniq), "%s_door_timeout", DEVICE_ID);
    char stopic[64]; makeTopic(stopic, sizeof(stopic), base, "door/duration/state");
    char ctopic[64]; makeTopic(ctopic, sizeof(ctopic), base, "door/duration/cmd");
    doc["name"]              = "Garage Door Timeout";
    doc["uniq_id"]           = uniq;
    doc["state_topic"]       = stopic;
    doc["command_topic"]     = ctopic;
    doc["min"]               = 1;
    doc["max"]               = 120;
    doc["step"]              = 1;
    doc["unit_of_measurement"] = "min";
    doc["avty_t"]            = avail;
    addDevice(doc.createNestedObject("dev"));
    serializeJson(doc, buf, sizeof(buf));
    mqtt.publish(buildDiscoveryTopic(F("number"), F("_door_timeout")), buf, true);
    Serial.println(F("Discovery: door timeout published"));
  }

  // ── Door remaining sensor (read-only) ─────────────────────────────────
  {
    StaticJsonDocument<256> doc;
    char uniq[32]; snprintf(uniq, sizeof(uniq), "%s_door_remaining", DEVICE_ID);
    char topic[64]; makeTopic(topic, sizeof(topic), base, "door/remaining/state");
    doc["name"]              = "Garage Door Remaining";
    doc["uniq_id"]           = uniq;
    doc["state_topic"]       = topic;
    doc["unit_of_measurement"] = "min";
    doc["avty_t"]            = avail;
    addDevice(doc.createNestedObject("dev"));
    serializeJson(doc, buf, sizeof(buf));
    mqtt.publish(buildDiscoveryTopic(F("sensor"), F("_door_remaining")), buf, true);
    Serial.println(F("Discovery: door remaining published"));
  }

  // ── NV heat setpoint ──────────────────────────────────────────────────
  {
    StaticJsonDocument<256> doc;
    char uniq[32]; snprintf(uniq, sizeof(uniq), "%s_nv_heat_set", DEVICE_ID);
    char stopic[64]; makeTopic(stopic, sizeof(stopic), base, "nv/hvac/heat_set/state");
    char ctopic[64]; makeTopic(ctopic, sizeof(ctopic), base, "nv/hvac/heat_set/cmd");
    doc["name"]              = "NV Heat Setpoint";
    doc["uniq_id"]           = uniq;
    doc["state_topic"]       = stopic;
    doc["command_topic"]     = ctopic;
    doc["min"]               = 30;
    doc["max"]               = 100;
    doc["step"]              = 0.5;
    doc["unit_of_measurement"] = "°F";
    doc["avty_t"]            = avail;
    addDevice(doc.createNestedObject("dev"));
    serializeJson(doc, buf, sizeof(buf));
    mqtt.publish(buildDiscoveryTopic(F("number"), F("_nv_heat_set")), buf, true);
    Serial.println(F("Discovery: nv heat setpoint published"));
  }

  // ── NV cool setpoint ──────────────────────────────────────────────────
  {
    StaticJsonDocument<256> doc;
    char uniq[32]; snprintf(uniq, sizeof(uniq), "%s_nv_cool_set", DEVICE_ID);
    char stopic[64]; makeTopic(stopic, sizeof(stopic), base, "nv/hvac/cool_set/state");
    char ctopic[64]; makeTopic(ctopic, sizeof(ctopic), base, "nv/hvac/cool_set/cmd");
    doc["name"]              = "NV Cool Setpoint";
    doc["uniq_id"]           = uniq;
    doc["state_topic"]       = stopic;
    doc["command_topic"]     = ctopic;
    doc["min"]               = 30;
    doc["max"]               = 100;
    doc["step"]              = 0.5;
    doc["unit_of_measurement"] = "°F";
    doc["avty_t"]            = avail;
    addDevice(doc.createNestedObject("dev"));
    serializeJson(doc, buf, sizeof(buf));
    mqtt.publish(buildDiscoveryTopic(F("number"), F("_nv_cool_set")), buf, true);
    Serial.println(F("Discovery: nv cool setpoint published"));
  }

  // ── HVAC swing (live) ─────────────────────────────────────────────────
  {
    StaticJsonDocument<256> doc;
    char uniq[32]; snprintf(uniq, sizeof(uniq), "%s_hvac_swing", DEVICE_ID);
    char stopic[64]; makeTopic(stopic, sizeof(stopic), base, "hvac/swing/state");
    char ctopic[64]; makeTopic(ctopic, sizeof(ctopic), base, "hvac/swing/cmd");
    doc["name"]              = "HVAC Swing";
    doc["uniq_id"]           = uniq;
    doc["state_topic"]       = stopic;
    doc["command_topic"]     = ctopic;
    doc["min"]               = 0;
    doc["max"]               = 20;
    doc["step"]              = 1;
    doc["unit_of_measurement"] = "°F";
    doc["avty_t"]            = avail;
    addDevice(doc.createNestedObject("dev"));
    serializeJson(doc, buf, sizeof(buf));
    mqtt.publish(buildDiscoveryTopic(F("number"), F("_hvac_swing")), buf, true);
    Serial.println(F("Discovery: HVAC swing published"));
  }

  // ── NV door timeout ───────────────────────────────────────────────────
  {
    StaticJsonDocument<256> doc;
    char uniq[32]; snprintf(uniq, sizeof(uniq), "%s_nv_door_timeout", DEVICE_ID);
    char stopic[64]; makeTopic(stopic, sizeof(stopic), base, "nv/door_timeout/state");
    char ctopic[64]; makeTopic(ctopic, sizeof(ctopic), base, "nv/door_timeout/cmd");
    doc["name"]              = "NV Door Timeout";
    doc["uniq_id"]           = uniq;
    doc["state_topic"]       = stopic;
    doc["command_topic"]     = ctopic;
    doc["min"]               = 1;
    doc["max"]               = 120;
    doc["step"]              = 1;
    doc["unit_of_measurement"] = "min";
    doc["avty_t"]            = avail;
    addDevice(doc.createNestedObject("dev"));
    serializeJson(doc, buf, sizeof(buf));
    mqtt.publish(buildDiscoveryTopic(F("number"), F("_nv_door_timeout")), buf, true);
    Serial.println(F("Discovery: nv door timeout published"));
  }

  // ── NV light timeout ──────────────────────────────────────────────────
  {
    StaticJsonDocument<256> doc;
    char uniq[32]; snprintf(uniq, sizeof(uniq), "%s_nv_light_timeout", DEVICE_ID);
    char stopic[64]; makeTopic(stopic, sizeof(stopic), base, "nv/light_timeout/state");
    char ctopic[64]; makeTopic(ctopic, sizeof(ctopic), base, "nv/light_timeout/cmd");
    doc["name"]              = "NV Light Timeout";
    doc["uniq_id"]           = uniq;
    doc["state_topic"]       = stopic;
    doc["command_topic"]     = ctopic;
    doc["min"]               = 1;
    doc["max"]               = 120;
    doc["step"]              = 1;
    doc["unit_of_measurement"] = "min";
    doc["avty_t"]            = avail;
    addDevice(doc.createNestedObject("dev"));
    serializeJson(doc, buf, sizeof(buf));
    mqtt.publish(buildDiscoveryTopic(F("number"), F("_nv_light_timeout")), buf, true);
    Serial.println(F("Discovery: nv light timeout published"));
  }

  // ── NV Save button ────────────────────────────────────────────────────
  {
    StaticJsonDocument<256> doc;
    char uniq[32]; snprintf(uniq, sizeof(uniq), "%s_nv_save", DEVICE_ID);
    char topic[64]; makeTopic(topic, sizeof(topic), base, "nv/save/cmd");
    doc["name"]          = "Save NV Settings";
    doc["uniq_id"]       = uniq;
    doc["command_topic"] = topic;
    doc["avty_t"]        = avail;
    addDevice(doc.createNestedObject("dev"));
    serializeJson(doc, buf, sizeof(buf));
    mqtt.publish(buildDiscoveryTopic(F("button"), F("_nv_save")), buf, true);
    Serial.println(F("Discovery: nv save button published"));
  }

  // ── Garage light switch ───────────────────────────────────────────────
  {
    StaticJsonDocument<256> doc;
    char uniq[32]; snprintf(uniq, sizeof(uniq), "%s_light", DEVICE_ID);
    char stopic[64]; makeTopic(stopic, sizeof(stopic), base, "light/state");
    char ctopic[64]; makeTopic(ctopic, sizeof(ctopic), base, "light/cmd");
    doc["name"]          = "Garage Light";
    doc["uniq_id"]       = uniq;
    doc["state_topic"]   = stopic;
    doc["command_topic"] = ctopic;
    doc["payload_on"]    = "on";
    doc["payload_off"]   = "off";
    doc["avty_t"]        = avail;
    addDevice(doc.createNestedObject("dev"));
    serializeJson(doc, buf, sizeof(buf));
    mqtt.publish(buildDiscoveryTopic(F("light"), F("_light")), buf, true);
    Serial.println(F("Discovery: light published"));
  }

  // ── Current light timeout number ──────────────────────────────────────
  {
    StaticJsonDocument<256> doc;
    char uniq[32]; snprintf(uniq, sizeof(uniq), "%s_light_timeout", DEVICE_ID);
    char stopic[64]; makeTopic(stopic, sizeof(stopic), base, "light/duration/state");
    char ctopic[64]; makeTopic(ctopic, sizeof(ctopic), base, "light/duration/cmd");
    doc["name"]              = "Garage Light Timeout";
    doc["uniq_id"]           = uniq;
    doc["state_topic"]       = stopic;
    doc["command_topic"]     = ctopic;
    doc["min"]               = 1;
    doc["max"]               = 120;
    doc["step"]              = 1;
    doc["unit_of_measurement"] = "min";
    doc["avty_t"]            = avail;
    addDevice(doc.createNestedObject("dev"));
    serializeJson(doc, buf, sizeof(buf));
    mqtt.publish(buildDiscoveryTopic(F("number"), F("_light_timeout")), buf, true);
    Serial.println(F("Discovery: light timeout published"));
  }

  // ── Light remaining sensor (read-only) ────────────────────────────────
  {
    StaticJsonDocument<256> doc;
    char uniq[32]; snprintf(uniq, sizeof(uniq), "%s_light_remaining", DEVICE_ID);
    char topic[64]; makeTopic(topic, sizeof(topic), base, "light/remaining/state");
    doc["name"]              = "Garage Light Remaining";
    doc["uniq_id"]           = uniq;
    doc["state_topic"]       = topic;
    doc["unit_of_measurement"] = "min";
    doc["avty_t"]            = avail;
    addDevice(doc.createNestedObject("dev"));
    serializeJson(doc, buf, sizeof(buf));
    mqtt.publish(buildDiscoveryTopic(F("sensor"), F("_light_remaining")), buf, true);
    Serial.println(F("Discovery: light remaining published"));
  }

  // ── Climate (HVAC thermostat) ─────────────────────────────────────────
  // Uses a larger StaticJsonDocument (2048 bytes) — the climate entity
  // has many topic fields each containing the DEVICE_ID.
  {
    StaticJsonDocument<2048> doc;
    char uniq[32]; snprintf(uniq, sizeof(uniq), "%s_hvac", DEVICE_ID);
    doc["name"]    = "Garage Thermostat";
    doc["uniq_id"] = uniq;

    JsonArray modes = doc.createNestedArray("modes");
    modes.add("off"); modes.add("heat");
    modes.add("cool"); modes.add("heat_cool");

    char hvacBase[64];
    snprintf(hvacBase, sizeof(hvacBase), "%s/hvac", base);

    char topic[64];
    makeTopic(topic, sizeof(topic), hvacBase, "mode/state");        doc["mode_state_topic"]          = topic;
    makeTopic(topic, sizeof(topic), hvacBase, "mode/cmd");          doc["mode_command_topic"]         = topic;
    makeTopic(topic, sizeof(topic), hvacBase, "action/state");      doc["action_topic"]               = topic;
    makeTopic(topic, sizeof(topic), hvacBase, "temp/state");        doc["current_temperature_topic"]  = topic;
    makeTopic(topic, sizeof(topic), hvacBase, "heat_set/state");    doc["temperature_state_topic"]    = topic;
    makeTopic(topic, sizeof(topic), hvacBase, "heat_set/cmd");      doc["temperature_command_topic"]  = topic;
    makeTopic(topic, sizeof(topic), hvacBase, "heat_set/state");    doc["temp_low_state_topic"]       = topic;
    makeTopic(topic, sizeof(topic), hvacBase, "heat_set/cmd");      doc["temp_low_command_topic"]     = topic;
    makeTopic(topic, sizeof(topic), hvacBase, "cool_set/state");    doc["temp_high_state_topic"]      = topic;
    makeTopic(topic, sizeof(topic), hvacBase, "cool_set/cmd");      doc["temp_high_command_topic"]    = topic;

    doc["temperature_unit"] = "F";
    doc["min_temp"]         = 32;
    doc["max_temp"]         = 90;
    doc["temp_step"]        = 1;
    doc["avty_t"]           = avail;
    addDevice(doc.createNestedObject("dev"));

    serializeJson(doc, buf, sizeof(buf));
    mqtt.publish(buildDiscoveryTopic(F("climate"), F("_hvac")), buf, true);
    Serial.println(F("Discovery: climate published"));
  }

  // ── Temperature sensor ────────────────────────────────────────────────
  {
    StaticJsonDocument<256> doc;
    char uniq[32]; snprintf(uniq, sizeof(uniq), "%s_temp", DEVICE_ID);
    char topic[64]; makeTopic(topic, sizeof(topic), base, "hvac/temp/state");
    doc["name"]               = "Garage Temperature";
    doc["uniq_id"]            = uniq;
    doc["device_class"]       = "temperature";
    doc["state_topic"]        = topic;
    doc["unit_of_measurement"] = "\xc2\xb0\x46"; // °F in UTF-8
    doc["avty_t"]             = avail;
    addDevice(doc.createNestedObject("dev"));
    serializeJson(doc, buf, sizeof(buf));
    mqtt.publish(buildDiscoveryTopic(F("sensor"), F("_temp")), buf, true);
    Serial.println(F("Discovery: temp sensor published"));
  }

  // ── Motion binary sensor ──────────────────────────────────────────────
  {
    StaticJsonDocument<256> doc;
    char uniq[32]; snprintf(uniq, sizeof(uniq), "%s_motion", DEVICE_ID);
    char topic[64]; makeTopic(topic, sizeof(topic), base, "motion/state");
    doc["name"]         = "Garage Motion";
    doc["uniq_id"]      = uniq;
    doc["device_class"] = "motion";
    doc["state_topic"]  = topic;
    doc["payload_on"]   = "on";
    doc["payload_off"]  = "off";
    doc["avty_t"]       = avail;
    addDevice(doc.createNestedObject("dev"));
    serializeJson(doc, buf, sizeof(buf));
    mqtt.publish(buildDiscoveryTopic(F("binary_sensor"), F("_motion")), buf, true);
    Serial.println(F("Discovery: motion sensor published"));
  }

  // ── HVAC lockout binary sensor ────────────────────────────────────────
  {
    StaticJsonDocument<256> doc;
    char uniq[32]; snprintf(uniq, sizeof(uniq), "%s_lockout", DEVICE_ID);
    char topic[64]; makeTopic(topic, sizeof(topic), base, "lockout/state");
    doc["name"]        = "Garage HVAC Lockout";
    doc["uniq_id"]     = uniq;
    doc["state_topic"] = topic;
    doc["payload_on"]  = "Lockout";
    doc["payload_off"] = "OK";
    doc["avty_t"]      = avail;
    addDevice(doc.createNestedObject("dev"));
    serializeJson(doc, buf, sizeof(buf));
    mqtt.publish(buildDiscoveryTopic(F("binary_sensor"), F("_lockout")), buf, true);
    Serial.println(F("Discovery: lockout sensor published"));
  }
}

// ============================================================
//  Network status helpers
// ============================================================

/** @brief Returns the aggregated public NetStatus. */
MQTTManager::NetStatus MQTTManager::getNetStatus() const { return netStatus; }

/**
 * @brief Returns a granular human-readable status string for LCD display.
 *
 * Reflects the individual WiFi and MQTT phase states, giving the user
 * more specific information than the three-value NetStatus enum alone.
 */
const char *MQTTManager::getNetStatusString() const
{
  // Disabled state takes priority over everything.
  if (wifiPhase == WiFiPhase::Disabled)   return "Disabled";

  // WiFi not yet connected.
  if (wifiPhase == WiFiPhase::Backoff)    return "WiFi Wait";
  if (wifiPhase != WiFiPhase::Connected)  return "WiFi...";

  // WiFi is up — report MQTT phase.
  if (mqttPhase == MQTTPhase::Backoff)    return "MQTT Wait";
  if (mqttPhase == MQTTPhase::Connected)  return "Connected";
  return "MQTT..."; // Idle = about to attempt connection
}

/** @brief Returns true unless the network is in Disabled state. */
bool MQTTManager::isNetworkEnabled() const
{
  return wifiPhase != WiFiPhase::Disabled;
}

/**
 * @brief Resets both state machines to Idle and clears all failure counters.
 *
 * Forces a clean reconnect cycle from scratch on the next loop() call.
 * Called from the MQTT menu "Connect" action.
 */
void MQTTManager::resetNetStatus()
{
  Serial.println(F("MQTT:Network reset requested"));

  // Tear down current connections cleanly.
  if (mqtt.connected()) mqtt.disconnect();
  WiFi.disconnect();

  // Reset WiFi state machine.
  wifiPhase    = WiFiPhase::Idle;
  wifiFailures = 0;
  wifiConnectStart = 0;
  wifiRetryAt  = 0;

  // Reset MQTT state machine.
  mqttPhase          = MQTTPhase::Idle;
  mqttFailures       = 0;
  mqttRetryAt        = 0;
  mqttCurrentBackoff = MQTT_BACKOFF_MIN;

  netStatus = NetStatus::Connecting;
}

/**
 * @brief Immediately disconnects everything and forces Disabled state.
 *
 * No automatic retry will occur.  Call resetNetStatus() to re-enable.
 * Called from the MQTT menu "Disable" action.
 */
void MQTTManager::disableNetwork()
{
  Serial.println(F("MQTT:Network disabled by user"));

  if (mqtt.connected()) mqtt.disconnect();
  WiFi.disconnect();

  wifiPhase          = WiFiPhase::Disabled;
  wifiRetryAt        = millis() + DISABLED_RETRY_INTERVAL_MS;
  mqttPhase          = MQTTPhase::Idle;
  mqttCurrentBackoff = MQTT_BACKOFF_MIN;
  netStatus          = NetStatus::Disabled;
}

/** @brief Writes the local IP address string into buf (or "n/a" if not connected). */
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

/** @brief Writes the configured MQTT server address into buf. */
void MQTTManager::getMqttServerIP(char *buf, size_t len) const
{
  strncpy(buf, MQTT_SERVER, len);
  buf[len - 1] = '\0';
}

#endif // ENABLE_WIFI
