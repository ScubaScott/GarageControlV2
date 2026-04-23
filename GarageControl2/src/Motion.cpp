/**
 * @file Motion.cpp
 * @brief Implementation of motion sensor controller with hardware interrupt integration.
 *
 * This file implements the MotionSensor class methods for PIR motion detection,
 * forced acknowledgment functionality, and ISR integration via recordMotion().
 */

#include <Arduino.h>
#include "Motion.h"
#include "Utility.h"

// ============================================================
//  Motion Sensor
// ============================================================

/**
 * @brief Constructor for MotionSensor.
 * @param p PIR sensor pin number.
 */
MotionSensor::MotionSensor(byte p) : pin(p) { pinMode(pin, INPUT); }

/**
 * @brief Polls the motion sensor for acknowledged activity.
 *
 * Returns true once when motion is detected and not suppressed by forced-ack.
 * Called from main loop; hardware interrupt debouncing is handled by RISING edge
 * detection in pirISR(). This method provides acknowledgment logic for subsystems
 * (e.g., garage door timeout extension) that need occupancy-based state changes.
 *
 * @return True if new motion detected and not in forced-ack window, false otherwise.
 */
bool MotionSensor::poll()
{
  bool motion = (digitalRead(pin) == HIGH);

  if (motion && !acked)
  {
    if (ForcedAck)
    {
      acked = true;
      ForcedAck = false;
      Serial.println(F("Motion: Reset ForcedAck"));
    }
    else
    {
      acked = true;
      Serial.println(F("Motion:Poll confirmed"));
      return true;
    }
  }
  if (!motion)
    acked = false;
  return false;
}

/**
 * @brief Records motion detection and updates lastMotion timestamp.
 *
 * Called from pirISR() when hardware interrupt detects RISING edge.
 * Updates lastMotion for subsystems needing occupancy-based timeout extension.
 */
void MotionSensor::recordMotion()
{
  lastMotion = now();
  Serial.println(F("Motion:ISR recorded"));
}

/**
 * @brief Checks if motion was recently detected within timeout.
 * @param timeout Timeout period in milliseconds.
 * @return True if motion detected within timeout, false otherwise.
 */
bool MotionSensor::recentlyActive(unsigned long timeout)
{
  return (now() - lastMotion) < timeout;
}

/**
 * @brief Forces acknowledgment to suppress false triggers from relay spikes.
 *
 * Prevents the pin from being interpreted as new motion on the next poll() call.
 * Does NOT update lastMotion; that is handled by recordMotion() from pirISR().
 */
void MotionSensor::forceAck()
{
  ForcedAck = true;
  Serial.println(F("Motion:ForceAck"));
}

/**
 * @brief Gets the current active state of the sensor.
 * @return True if sensor pin is HIGH, false otherwise.
 */
bool MotionSensor::isActive() { return digitalRead(pin) == HIGH; }
