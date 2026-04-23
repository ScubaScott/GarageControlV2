/**
 * @file MQTT.cpp
 * @brief MQTTManager implementation – compiled only when ENABLE_WIFI is 1.
 *
 * @section MemoryStrategy Memory strategy
 * 1. F() macro  – all string literals placed in flash (PROGMEM); never copied to SRAM.
 * 2. No persistent topic Strings – topics built on-demand into a single shared
 *    64-byte _topicBuf.  Previous design held 14 String members (~400 bytes heap).
 * 3. Compact state cache – prevDoorState (was String) → uint8_t; prevHvacMode
 *    (was String) → uint8_t.  Saves ~60 bytes of heap.
 * 4. Discovery payloads – assembled in a single reused 1024-byte stack buffer
 *    instead of heap-concatenated Strings (removed ~6 × 300 byte heap churn).
 * 5. dtostrf() replaces String(float,n) for float→char conversion in
 *    publishStateChanges().
 *
 * @section NVSeparation NV vs current value separation
 * publishStateChanges() accepts BOTH the current live timeout values AND the
 * NV (persisted) timeout values as distinct parameters.  They are compared
 * against separate prev-state members (prevDoorDuration vs prevNvDoorTimeout,
 * prevLightDuration vs prevNvLightTimeout) and published to separate MQTT
 * topics.  This means Home Assistant will reflect the correct NV value even
 * when the user has temporarily changed the current setting without saving.
 *
 * @section NVSaveModel NV save model
 * /nv/ command topics update the NV member variables in RAM only (no auto-save).
 * Changes accumulate in RAM until the client sends /nv/save/cmd, which triggers
 * saveNV() to persist all NV members to EEPROM in a single write pass.
 * This eliminates per-step EEPROM writes when adjusting values incrementally
 * (e.g., stepping a setpoint from 65→75 via HA) and greatly reduces write-cycle
 * wear on the EEPROM cells.
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
 * the 2-byte pointers themselves occupy SRAM.
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
//  Public API
// ============================================================

/**
 * @brief Initialises WiFi, MQTT, and publishes HA discovery payloads.
 *
 * @param ctrl     Owning GarageController instance.
 * @param callback PubSubClient inbound-message callback.
 */
void MQTTManager::init(GarageController *ctrl,
                       void (*callback)(char *, byte *, unsigned int))
{
  controller = ctrl;
  g_mqttManager = this;

  netStatus          = NetStatus::Connecting;
  consecutiveFailures = 0;

  connectWiFi();
  mqtt.setServer(MQTT_SERVER, MQTT_PORT);
  mqtt.setCallback(callback);
  // Buffer size increased to 2048 to accommodate the large climate discovery payload.
  mqtt.setBufferSize(2048);
  connectMQTT();
  publishDiscovery();
}

/**
 * @brief Services the MQTT connection and reconnects as needed.
 *
 * When Disabled, attempts a full reconnect once per hour.  When WiFi drops,
 * connectWiFi() may block up to 15 s – this is why the main loop calls
 * mqttManager.loop() AFTER the time-critical motion/light code.
 */
void MQTTManager::loop()
{
  // ── Disabled: hourly reconnect attempt only ───────────────────────────
  if (netStatus == NetStatus::Disabled)
  {
    if (millis() - lastHourlyRetry >= 900000UL) // 15 minutes
    {
      lastHourlyRetry     = millis();
      consecutiveFailures = 0;
      netStatus           = NetStatus::Connecting;
      Serial.println(F("MQTT:Hourly reconnect attempt"));
      connectWiFi();
      if (WiFi.status() == WL_CONNECTED)
        connectMQTT();
    }
    return;
  }

  // ── WiFi dropped: reconnect (may block) ──────────────────────────────
  if (WiFi.status() != WL_CONNECTED)
    connectWiFi();

  // ── MQTT disconnected: throttled reconnect ────────────────────────────
  if (!mqtt.connected())
  {
    netStatus = NetStatus::Connecting;
    if (millis() - lastMqttReconnect >= 5000UL)
    {
      lastMqttReconnect = millis();
      connectMQTT();
    }
  }

  // ── Normal operation: service the MQTT client ─────────────────────────
  if (netStatus != NetStatus::Disabled && mqtt.connected())
    mqtt.loop();
}

// ============================================================
//  publishStateChanges
// ============================================================

/**
 * @brief Publishes changed sensor and state values to MQTT.
 *
 * Only values that differ from the previous publish are transmitted (change
 * detection).  On (re)connect, pendingFullPublish invalidates all prev-state
 * sentinels so the next call immediately overwrites any stale retained values
 * in Home Assistant.
 *
 * Temperature publication is additionally gated until the first valid DS18B20
 * reading arrives (hasValidTemp), preventing DEVICE_DISCONNECTED sentinels
 * (–196.6 °F) from reaching HA during startup.
 *
 * NV door and light timeouts use SEPARATE prev-state members from the current
 * live timeouts so that both HA entities track their correct independent values.
 *
 * @param lightOn             Current light relay state.
 * @param lightDurationMins   Current live light auto-off timeout (min).
 * @param lightRemainingMins  Minutes remaining until light auto-off.
 * @param doorCode            Door state code (0–4).
 * @param doorDurationMins    Current live door auto-close timeout (min).
 * @param doorRemainingMins   Minutes remaining until door auto-close.
 * @param tempF               Current temperature (°F).
 * @param heatSet             Current live HVAC heat setpoint (°F).
 * @param coolSet             Current live HVAC cool setpoint (°F).
 * @param nvHeatSet           NV HVAC heat setpoint (°F).
 * @param nvCoolSet           NV HVAC cool setpoint (°F).
 * @param hvacSwing           Current live HVAC swing (°F).
 * @param nvDoorTimeoutMins   NV door auto-close timeout (min) – independent of doorDurationMins.
 * @param nvLightTimeoutMins  NV light auto-off timeout (min) – independent of lightDurationMins.
 * @param mode                HVAC mode code (0–3).
 * @param hvacState           HVAC runtime state code (0–3).
 * @param motionActive        True if PIR sensor reads HIGH.
 * @param lockout             True when HVAC is locked out.
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
  if (netStatus == NetStatus::Disabled) return;
  if (!mqtt.connected())               return;

  // ── Full publish after (re)connect ───────────────────────────────────────
  // Invalidate all prev-state sentinels so every value is (re)published on
  // the next call, overwriting any stale retained values in HA.
  // Temperature is excluded here – it remains gated by hasValidTemp.
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
    prevHvacSwing      = hvacSwing + 1;
    prevNvDoorTimeout  = nvDoorTimeoutMins  + 1;
    prevNvLightTimeout = nvLightTimeoutMins + 1;
    prevMode           = 0xFF;
    prevHvacState      = 0xFF;
    prevMotion         = !motionActive;
    prevLockout        = !lockout;
    // prevTemp intentionally NOT reset – gated separately by hasValidTemp.
    pendingFullPublish = false;
  }

  // Small stack buffer for numeric payloads – avoids heap String temporaries.
  char val[12];

  // ── Light state ───────────────────────────────────────────────────────────
  if (lightOn != prevLightState)
  {
    mqtt.publish(buildTopic(F("/light/state")), lightOn ? "on" : "off", true);
    prevLightState = lightOn;
  }

  // ── Current light timeout ─────────────────────────────────────────────────
  if (lightDurationMins != prevLightDuration)
  {
    snprintf(val, sizeof(val), "%lu", lightDurationMins);
    mqtt.publish(buildTopic(F("/light/duration/state")), val, true);
    prevLightDuration = lightDurationMins;
  }

  // ── Light remaining ───────────────────────────────────────────────────────
  if (lightRemainingMins != prevLightRemaining)
  {
    snprintf(val, sizeof(val), "%lu", lightRemainingMins);
    mqtt.publish(buildTopic(F("/light/remaining/state")), val, true);
    prevLightRemaining = lightRemainingMins;
  }

  // ── Door state ────────────────────────────────────────────────────────────
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

  // ── Current door timeout ──────────────────────────────────────────────────
  if (doorDurationMins != prevDoorDuration)
  {
    snprintf(val, sizeof(val), "%lu", doorDurationMins);
    mqtt.publish(buildTopic(F("/door/duration/state")), val, true);
    prevDoorDuration = doorDurationMins;
  }

  // ── Door remaining ────────────────────────────────────────────────────────
  if (doorRemainingMins != prevDoorRemaining)
  {
    snprintf(val, sizeof(val), "%lu", doorRemainingMins);
    mqtt.publish(buildTopic(F("/door/remaining/state")), val, true);
    prevDoorRemaining = doorRemainingMins;
  }

  // ── Temperature (gated until first valid reading) ─────────────────────────
  if (!hasValidTemp)
  {
    // Accept the reading once it's within the DS18B20's valid range.
    if (!isnan(tempF) && tempF > -40.0f && tempF < 150.0f)
      hasValidTemp = true;
  }
  if (hasValidTemp && abs(tempF - prevTemp) >= 0.2f)
  {
    dtostrf(tempF, 4, 1, val);
    mqtt.publish(buildTopic(F("/hvac/temp/state")), val, true);
    prevTemp = tempF;
  }

  // ── Live HVAC setpoints ───────────────────────────────────────────────────
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

  if (abs(hvacSwing - prevHvacSwing) >= 0.5f)
  {
    dtostrf(hvacSwing, 4, 1, val);
    mqtt.publish(buildTopic(F("/hvac/swing/state")), val, true);
    prevHvacSwing = hvacSwing;
  }

  // ── NV HVAC setpoints ─────────────────────────────────────────────────────
  if (abs(nvHeatSet - prevNvHeatSet) >= 0.5f)
  {
    dtostrf(nvHeatSet, 4, 1, val);
    mqtt.publish(buildTopic(F("/nv/hvac/heat_set/state")), val, true);
    prevNvHeatSet = nvHeatSet;
  }

  if (abs(nvCoolSet - prevNvCoolSet) >= 0.5f)
  {
    dtostrf(nvCoolSet, 4, 1, val);
    mqtt.publish(buildTopic(F("/nv/hvac/cool_set/state")), val, true);
    prevNvCoolSet = nvCoolSet;
  }

  // ── NV door timeout ───────────────────────────────────────────────────────
  // Tracked against prevNvDoorTimeout, which is INDEPENDENT of prevDoorDuration.
  if (nvDoorTimeoutMins != prevNvDoorTimeout)
  {
    snprintf(val, sizeof(val), "%lu", nvDoorTimeoutMins);
    mqtt.publish(buildTopic(F("/nv/door_timeout/state")), val, true);
    prevNvDoorTimeout = nvDoorTimeoutMins;
  }

  // ── NV light timeout ──────────────────────────────────────────────────────
  // Tracked against prevNvLightTimeout, INDEPENDENT of prevLightDuration.
  if (nvLightTimeoutMins != prevNvLightTimeout)
  {
    snprintf(val, sizeof(val), "%lu", nvLightTimeoutMins);
    mqtt.publish(buildTopic(F("/nv/light_timeout/state")), val, true);
    prevNvLightTimeout = nvLightTimeoutMins;
  }

  // ── HVAC mode ─────────────────────────────────────────────────────────────
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

  // ── HVAC runtime action ───────────────────────────────────────────────────
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

  // ── Motion sensor ─────────────────────────────────────────────────────────
  if (motionActive != prevMotion)
  {
    mqtt.publish(buildTopic(F("/motion/state")),
                 motionActive ? "on" : "off", true);
    prevMotion = motionActive;
  }

  // ── HVAC lockout ──────────────────────────────────────────────────────────
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

/**
 * @brief Attempts to connect to the configured WiFi network.
 *
 * Blocks for up to 15 s.  On success, resets consecutiveFailures.
 * On failure, increments the counter and disables the network stack
 * after MAX_FAILURES consecutive failures.
 */
void MQTTManager::connectWiFi()
{
  if (netStatus == NetStatus::Disabled) return;

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

/**
 * @brief Connects to the MQTT broker, subscribes to command topics,
 *        and schedules a full state re-publish.
 *
 * Subscribes to all live /cmd topics and NV /nv/ topics.
 * The /nv/save/cmd topic is subscribed here; its handler in
 * GarageController::handleMQTT() persists current NV members to EEPROM
 * without touching live subsystem values (Rule 4 MQTT path).
 *
 * Uses a stack-allocated clientId (no heap String).  The availability
 * topic is copied into a local buffer before the connect() call because
 * buildTopic() reuses _topicBuf.
 */
void MQTTManager::connectMQTT()
{
  if (netStatus == NetStatus::Disabled) return;

  // Client ID on the stack – avoids a heap String allocation.
  char clientId[32];
  snprintf(clientId, sizeof(clientId), "%s_%04x",
           DEVICE_ID, (unsigned)random(0xffff));

  // Copy availability topic before calling buildTopic() again.
  char avail[64];
  strncpy(avail, buildTopic(F("/availability")), sizeof(avail));
  avail[sizeof(avail) - 1] = '\0';

  Serial.print(F("Connecting MQTT... "));

  bool ok;
  if (strlen(MQTT_USER) > 0)
    ok = mqtt.connect(clientId, MQTT_USER, MQTT_PASS, avail, 1, true, "offline");
  else
    ok = mqtt.connect(clientId, nullptr, nullptr,     avail, 1, true, "offline");

  if (ok)
  {
    Serial.println(F("connected"));
    mqtt.publish(avail, "online", true);

    // ── Subscribe to live /cmd topics ─────────────────────────────────────
    mqtt.subscribe(buildTopic(F("/door/cmd")));
    mqtt.subscribe(buildTopic(F("/door/duration/cmd")));
    mqtt.subscribe(buildTopic(F("/light/cmd")));
    mqtt.subscribe(buildTopic(F("/light/duration/cmd")));
    mqtt.subscribe(buildTopic(F("/hvac/heat_set/cmd")));
    mqtt.subscribe(buildTopic(F("/hvac/cool_set/cmd")));
    mqtt.subscribe(buildTopic(F("/hvac/mode/cmd")));
    mqtt.subscribe(buildTopic(F("/hvac/swing/cmd")));

    // ── Subscribe to NV /nv/ topics ───────────────────────────────────────
    // These update NV members in RAM only; no auto-save occurs.
    mqtt.subscribe(buildTopic(F("/nv/hvac/heat_set/cmd")));
    mqtt.subscribe(buildTopic(F("/nv/hvac/cool_set/cmd")));
    mqtt.subscribe(buildTopic(F("/nv/door_timeout/cmd")));
    mqtt.subscribe(buildTopic(F("/nv/light_timeout/cmd")));

    // ── Subscribe to NV control topics ────────────────────────────────────
    mqtt.subscribe(buildTopic(F("/nv/save/cmd")));    // persist NV members → EEPROM
    mqtt.subscribe(buildTopic(F("/nv/reload/cmd")));  // copy NV members → live subsystems

    publishDiscovery();

    // Force a complete state re-publish on the next publishStateChanges() call
    // so any stale retained values in HA are immediately corrected.
    pendingFullPublish = true;

    netStatus           = NetStatus::Connected;
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
    }
  }
}

// ============================================================
//  HA Auto-Discovery
// ============================================================

/**
 * @brief Publishes all Home Assistant MQTT Discovery configuration payloads.
 *
 * A single 1024-byte stack buffer is reused for every entity payload,
 * replacing the heap-concatenated String approach that caused repeated
 * ~400-byte heap allocations and fragmentation at startup and on reconnect.
 *
 * Includes a "Save NV Settings" button entity that triggers /nv/save/cmd,
 * allowing HA users to explicitly commit pending NV RAM changes to EEPROM
 * after making NV adjustments.
 *
 * Also removes stale 'number' discovery entries for read-only 'remaining'
 * sensors that were previously published with a command_topic, which caused
 * HA import errors.
 */
void MQTTManager::publishDiscovery()
{
  // ── Remove stale retained discovery entries ───────────────────────────────
  // These were previously published as 'number' (which requires command_topic).
  // Publishing an empty retained payload removes them from HA's entity registry.
  {
    char stale[72];
    snprintf(stale, sizeof(stale), "%s/number/%s_door_remaining/config",
             DISCOVERY_PREFIX, DEVICE_ID);
    mqtt.publish(stale, "", true);

    snprintf(stale, sizeof(stale), "%s/number/%s_light_remaining/config",
             DISCOVERY_PREFIX, DEVICE_ID);
    mqtt.publish(stale, "", true);
  }

  // ── Shared buffers ────────────────────────────────────────────────────────
  char buf[2048];  // Reused for every entity payload (increased from 1024 to prevent truncation).

  char avail[64];
  strncpy(avail, buildTopic(F("/availability")), sizeof(avail));
  avail[sizeof(avail) - 1] = '\0';

  char base[64];
  snprintf(base, sizeof(base), "garage/%s", DEVICE_ID);

  // ── Door button ───────────────────────────────────────────────────────────
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

  // ── Door state binary sensor ──────────────────────────────────────────────
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

  // ── Current door timeout number ───────────────────────────────────────────
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

  // ── Door remaining sensor ─────────────────────────────────────────────────
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

  // ── NV heat setpoint ──────────────────────────────────────────────────────
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

  // ── NV cool setpoint ──────────────────────────────────────────────────────
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
  // ── HVAC swing ──────────────────────────────────────────────────────
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
    doc["max"]               = 5;
    doc["step"]              = 0.5;
    doc["unit_of_measurement"] = "°F";
    doc["avty_t"]            = avail;
    addDevice(doc.createNestedObject("dev"));
    serializeJson(doc, buf, sizeof(buf));
    mqtt.publish(buildDiscoveryTopic(F("number"), F("_hvac_swing_set")), buf, true);
    Serial.println(F("Discovery: HVAC Swing published"));
  }
  // ── NV door timeout ───────────────────────────────────────────────────────
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

  // ── NV light timeout ──────────────────────────────────────────────────────
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

  // ── NV Save button ────────────────────────────────────────────────────────
  // A momentary button in HA that publishes to /nv/save/cmd, triggering the
  // controller to write current NV member variables to EEPROM.
  // Use this after adjusting any /nv/ number entity to make the changes permanent.
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

  // ── Garage light switch ───────────────────────────────────────────────────
  {
    StaticJsonDocument<256> doc;
    char uniq[32]; snprintf(uniq, sizeof(uniq), "%s_light", DEVICE_ID);
    char stopic[64]; makeTopic(stopic, sizeof(stopic), base, "light/state");
    char ctopic[64]; makeTopic(ctopic, sizeof(ctopic), base, "light/cmd");
    doc["name"]            = "Garage Light";
    doc["uniq_id"]         = uniq;
    doc["state_topic"]     = stopic;
    doc["command_topic"]   = ctopic;
    doc["payload_on"]      = "on";
    doc["payload_off"]     = "off";
    doc["avty_t"]          = avail;
    addDevice(doc.createNestedObject("dev"));
    serializeJson(doc, buf, sizeof(buf));
    mqtt.publish(buildDiscoveryTopic(F("light"), F("_light")), buf, true);
    Serial.println(F("Discovery: light published"));
  }

  // ── Current light timeout number ──────────────────────────────────────────
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

  // ── Light remaining sensor ────────────────────────────────────────────────
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

  // ── Climate (HVAC thermostat) ─────────────────────────────────────────────
  // Uses a larger StaticJsonDocument (2048 bytes) because the climate entity
  // contains many topic fields, all of which include the DEVICE_ID.
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
    makeTopic(topic, sizeof(topic), hvacBase, "hvac_swing/state");  doc["hvac_swing_topic"]           = topic;
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

  // ── Temperature sensor ────────────────────────────────────────────────────
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

  // ── Motion binary sensor ──────────────────────────────────────────────────
  {
    StaticJsonDocument<256> doc;
    char uniq[32]; snprintf(uniq, sizeof(uniq), "%s_motion", DEVICE_ID);
    char topic[64]; makeTopic(topic, sizeof(topic), base, "motion/state");
    doc["name"]        = "Garage Motion";
    doc["uniq_id"]     = uniq;
    doc["device_class"] = "motion";
    doc["state_topic"] = topic;
    doc["payload_on"]  = "on";
    doc["payload_off"] = "off";
    doc["avty_t"]      = avail;
    addDevice(doc.createNestedObject("dev"));
    serializeJson(doc, buf, sizeof(buf));
    mqtt.publish(buildDiscoveryTopic(F("binary_sensor"), F("_motion")), buf, true);
    Serial.println(F("Discovery: motion sensor published"));
  }

  // ── HVAC lockout binary sensor ────────────────────────────────────────────
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

MQTTManager::NetStatus MQTTManager::getNetStatus() const { return netStatus; }

const char *MQTTManager::getNetStatusString() const
{
  switch (netStatus)
  {
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

/** @brief Resets the failure counter and re-enters Connecting state. */
void MQTTManager::resetNetStatus()
{
  consecutiveFailures = 0;
  if (netStatus == NetStatus::Disabled)
    netStatus = NetStatus::Connecting;
}

/** @brief Immediately disconnects WiFi and MQTT and sets Disabled state. */
void MQTTManager::disableNetwork()
{
  netStatus = NetStatus::Disabled;
  if (mqtt.connected()) mqtt.disconnect();
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
