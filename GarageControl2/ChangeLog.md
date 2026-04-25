# GarageControl2 – Change Log

## [2.19.1] – 2026-04-23

### Added
- **24-Hour Auto-Revert for Live Values**: Temporary modifications to HVAC setpoints and timeouts now automatically revert to NV values after 24 hours without additional changes
  - When hvac.heatSet, hvac.coolSet, door.autoCloseDuration, or lights.duration are modified via menu or MQTT, a 24-hour timer starts
  - Any change to these values resets the timer (single unified timer for all 4 values)
  - If 24 hours pass without another change, all 4 values automatically revert to their NV (stored) counterparts
  - Allows temporary adjustments during garage work without manual SaveNV; auto-reverts if forgotten
  - Auto-revert check runs in main loop with no notifications (silently reverts in background)

### Architecture
- **Auto-Revert Implementation**:
  - `lastLiveChangeTime` tracks when a live value was last modified
  - `notifyLiveValueChanged()` (IMenuHost interface) called by menu/MQTT handlers to record timestamp
  - `checkAndRevertAutoValues()` runs each loop iteration to detect 24h timeout expiry
  - Reverts all 4 values atomically when timer expires; clears timer after revert
  - NV values serve as the "commit point" – if user wants to keep temp changes, must SaveNV before 24h window

---

## [2.19.0] – 2026-04-23

### Refactored
- **Motion Polling Architecture (v2.19.0)**: Eliminated redundant debouncing in main loop
  - Removed 10-sample debouncing loop from `motion.poll()` – hardware interrupt RISING edge already debounces
  - Added `motion.recordMotion()` method for `pirISR()` to update timestamp immediately
  - Reduced poll() overhead: single `digitalRead()` instead of 10 samples per loop cycle
  - Maintains compatibility with all motion-dependent features (door timeout extension, light timeout)

### Fixed
- **[PERFORMANCE] Main Loop Efficiency**: Eliminated 10 redundant GPIO reads per polling cycle
  - `motion.poll()` now does single pin state check instead of sampling loop
  - Hardware interrupt provides edge detection; no need for software debouncing
  - Motion timestamp updated by ISR at interrupt-time for immediate occupancy tracking
  - Door auto-close and light timeout extension remain fully functional

### Architecture
- **ISR Flow**: `pirISR()` → `motion.recordMotion()` (timestamp) → `lights.turnOn()` (sub-ms response)
- **Poll Flow**: `motion.poll()` checks pin state once, returns true once per motion event
- **Motion Timestamp**: Updated by ISR for real-time occupancy; poll() handles acknowledgment logic
- Maintains ForcedAck mechanism for relay spike suppression (no timestamp update in forceAck now)

---

## [2.18.0] – 2026-04-20

### Added
- **PIR Hardware Interrupt (v2.18.0)**: Forces lights on immediately via RISING edge trigger
  - Lights activate instantly on motion detection, even during MQTT reconnection
  - Interrupt routing respects 15-second cooldown period to prevent unwanted reactivation

- **Light Cooldown Timer (v2.18.0)**: 15-second motion-blocking period after manual turn-off
  - Prevents lights from re-triggering when occupants exit the room
  - Applies to all manual turn-off methods: button press, MQTT command, timeout
  - Works seamlessly with hardware interrupt handler

### Architecture
- Hardware interrupt handler (`pirISR()`) attached to PIR pin 4 (RISING edge)
- Cooldown state tracked in `GarageLight` class with `cooldownUntil` timestamp
- Main loop respects cooldown: motion-triggered lights blocked during 15-second window
- Hardware interrupt bypasses main loop delays but respects cooldown safety mechanism

---

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