/**
 * @file Utility.h
 * @brief Utility functions for timing, version information, and build configuration.
 *
 * This header provides essential utility functions for time-based event scheduling
 * and timeout handling, version tracking for diagnostics and MQTT publication,
 * and a build-time configuration flag for WiFi/MQTT feature compilation.
 *
 * All functions are inline or macro-based for minimal overhead on the constrained
 * Arduino platform, with predictable performance suitable for real-time control loops.
 */

#ifndef Utility_H
#define Utility_H

#include <Arduino.h>

/**
 * @brief Firmware version string.
 *
 * Updated for each release. Printed to Serial on boot and published
 * to MQTT discovery for remote diagnostics. Format: MAJOR.MINOR.PATCH
 *
 * Update this constant in GarageControl2_DEV.ino when releasing new versions.
 *
 * @example
 *   const char *GC_VERSION = "2.8.2";
 */
extern const char* GC_VERSION;

/**
 * @brief Gets the current system time in milliseconds.
 *
 * Wrapper around Arduino's millis() function. Used as a reference point
 * for timing comparisons with the expired() function. Auto-wraps at
 * ~49.7 days due to millis() overflow, but this is handled correctly
 * by the expired() comparison logic using unsigned arithmetic.
 *
 * @return Current time in milliseconds since Arduino boot
 *
 * @see expired() for how to use this with timeout comparisons
 *
 * @example
 *   unsigned long startTime = now();
 *   // do something...
 *   if (expired(startTime, 5000)) {
 *     // 5 seconds have passed
 *   }
 */
unsigned long now();

/**
 * @brief Checks if a time interval has expired since a given timestamp.
 *
 * Non-blocking timeout check suitable for main-loop polling patterns.
 * Correctly handles millis() wraparound at ~49.7 days using unsigned
 * arithmetic properties (wrapping subtraction works correctly for interval
 * values less than 2^31 milliseconds).
 *
 * @param last Timestamp from a previous now() call marking the interval start
 * @param interval Desired timeout duration in milliseconds
 * @return True if (now() - last) >= interval, false otherwise
 *
 * Typical usage pattern:
 * @code
 *   unsigned long lastCheck = now();
 *   
 *   void loop() {
 *     if (expired(lastCheck, 1000)) {  // Every 1000ms (1 second)
 *       doPeriodicTask();
 *       lastCheck = now();  // Reset for next interval
 *     }
 *   }
 * @endcode
 *
 * @note This function is side-effect free and does not modify lastCheck.
 *       You must explicitly reset lastCheck = now() to restart the interval.
 *
 * @see now() to get the current timestamp
 */
bool expired(unsigned long last, unsigned long interval);

/**
 * @brief Enable or disable WiFi and MQTT features at compile time.
 *
 * Set to 1 to compile with full WiFi + MQTT + Home Assistant auto-discovery support.
 * Set to 0 for a lightweight DEV mode that omits all network code, suitable for:
 * - Testing on hardware without WiFi (Arduino UNO R4 Minima)
 * - Reducing final binary size
 * - Eliminating network-related power consumption
 * - Faster boot and loop execution for local-only testing
 *
 * When set to 0:
 * - MQTTManager class is entirely absent from compilation
 * - No WiFi or MQTT library dependencies
 * - Local LCD menu still fully functional
 * - Network status screens in menu will show "Disabled"
 * - Main loop runs faster with no network polling overhead
 *
 * When set to 1:
 * - Full WiFi connectivity and MQTT publish/subscribe
 * - Home Assistant MQTT Discovery protocol
 * - Configuration loaded from config/Config.h
 * - Network status monitoring and auto-reconnect logic
 * - Remote sensor publishing and command reception
 *
 * @note Change this setting and recompile. No runtime flag exists.
 *       Config files should only be included when ENABLE_WIFI is 1.
 *
 * @see config/Config.h for network configuration (only included when ENABLE_WIFI=1)
 */
#define ENABLE_WIFI 1

#endif