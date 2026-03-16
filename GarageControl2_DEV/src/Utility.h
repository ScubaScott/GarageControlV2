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
 * @brief * To disable WiFi/MQTT for debugging, change 1 to 0 here.
 */
#define ENABLE_WIFI 0
#endif