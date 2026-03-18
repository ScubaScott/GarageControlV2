/**
 * @file Utility.h
 * @brief Header file for utility functions.
 *
 * This file provides utility functions for time management and timing checks
 * used throughout the garage control system.
 */

#ifndef Utility_H
#define Utility_H

#include <Arduino.h>

/**
 * @brief The current code version
 */

extern const char* GC_VERSION;

/**
 * @brief Gets the current time in milliseconds.
 * @return Current time in milliseconds since program start.
 */
unsigned long now();

/**
 * @brief Checks if a time interval has expired.
 * @param last Timestamp of the last event.
 * @param interval Interval in milliseconds.
 * @return True if the interval has expired since last, false otherwise.
 */
bool expired(unsigned long last, unsigned long interval);

/**
 * @brief Enable or disable WiFi/MQTT features.
 *
 * Set to 1 to enable WiFi + MQTT + Home Assistant auto-discovery.
 * Set to 0 to build in DEV mode with all network code excluded.
 *
 * Note: DEV mode is useful when running on hardware without WiFi.
 */
#define ENABLE_WIFI 1
#endif