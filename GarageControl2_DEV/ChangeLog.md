 * ── Build modes ──────────────────────────────────────────────────────────────
 *   DEV  (ENABLE_WIFI 0)  All WiFi / MQTT / PubSubClient code excluded.
 *                         Safe to flash on boards without WiFi hardware.
 *
 *   PROD (ENABLE_WIFI 1)  Full WiFi + MQTT + HA auto-discovery enabled.
 *                         Requires "Arduino UNO R4 WiFi" board and PubSubClient.
 *
 *   Change the flag in src/Utility.h to switch modes.
 * ─────────────────────────────────────────────────────────────────────────────
 *
 * Memory optimisations applied throughout this project
 * ──────────────────────────────────────────────────────
 * 1. F() macro on every string literal passed to Serial.print/println.
 *    String literals without F() are copied to SRAM at boot; F() keeps
 *    them in flash.  ~1.3 KB of debug strings moved to flash.
 *
 * 2. No heap-allocated String objects for MQTT topics.
 *    The previous design stored 14 persistent String members in MQTTManager
 *    (~400 bytes of heap).  Topics are now built on-demand into a single
 *    64-byte char buffer and used immediately.
 *
 * 3. prevDoorState / prevHvacMode changed from String to uint8_t / bool.
 *    Removes two heap strings plus fragmentation overhead (~60 bytes).
 *
 * 4. doorStateString() replaced by doorStateCode() returning uint8_t.
 *    Eliminates a heap String created every loop iteration.
 *
 * 5. Discovery payloads built with snprintf into one reused 512-byte
 *    stack buffer instead of heap-concatenated Strings (removed 6 × ~300
 *    byte heap allocations that ran at startup and on every reconnect).
 *
 * 6. LcdController::printLCDText() changed from const String& to
 *    const char* - callers use stack snprintf buffers or string literals.
 *    getHvacStateString/getDoorStateString return const char* literals
 *    instead of heap Strings.
 *
 * 7. dtostrf() used instead of String(float, n) for float → char
 *    conversion in publishStateChanges().