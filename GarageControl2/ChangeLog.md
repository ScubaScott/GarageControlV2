# GarageControl2 – Change Log

## [2.17.0] – 2026-04-20

### Added
- **SetNV Menu System (v2.17.0)**: New sub-menu under Config → NetworkInfo to edit non-volatile (NV) settings
  - Independent NV value editing without affecting live subsystem values (Rule 3)
  - Screens for: heat setpoint, cool setpoint, HVAC swing, min run time, min rest time, door timeout, light timeout
  - Changes persist in RAM; EEPROM update via SaveNV or `/nv/save/cmd`

### Fixed
- **[CRITICAL] NV Interface Compilation Errors**: Added missing virtual method declarations to `IMenuHost` base class
  - Added 7 getter methods: `getNvHeatSet()`, `getNvCoolSet()`, `getNvSwing()`, `getNvMinRunTime()`, `getNvMinRestTime()`, `getNvDoorTimeout()`, `getNvLightTimeout()`
  - Added 8 setter methods: `adjNvHeatSet()`, `adjNvCoolSet()`, `adjNvSwing()`, `adjNvMinRunTime()`, `adjNvMinRestTime()`, `adjNvDoorTimeout()`, `adjNvLightTimeout()`
  - Updated `LcdController` with `host` pointer and `setHost()` method for NV value display
  - Resolved "marked 'override', but does not override" compilation errors

### Architecture
- Strict NV model with five rules:
  - **Rule 1 (Boot)**: EEPROM → NV members → live values (synchronized at startup)
  - **Rule 2 (Live Changes)**: Direct subsystem updates only (no NV/EEPROM impact)
  - **Rule 3 (NV Changes)**: RAM-only NV member updates (no live/EEPROM impact)
  - **Rule 4 (Save)**: Live → NV → EEPROM or NV → EEPROM
  - **Rule 5 (Reload)**: NV → live values (EEPROM unchanged)

---

## Version History Overview

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