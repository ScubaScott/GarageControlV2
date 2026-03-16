/**
 * @file Motion.cpp
 * @brief Implementation of motion sensor controller.
 *
 * This file implements the MotionSensor class methods for PIR motion detection,
 * debouncing, and forced acknowledgment functionality.
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
 * @brief Polls the motion sensor for activity with debouncing.
 * @return True if new motion detected and acknowledged, false otherwise.
 */
bool MotionSensor::poll()
{
  int sum = 0;
  for (int i = 0; i < 10; i++)
    sum += digitalRead(pin);
  bool motion = (sum >= 5);

  if (motion && !acked)
  {
    if (ForcedAck)
    {
      acked = true;
      ForcedAck = false;
      Serial.println("Motion: Reset ForcedAck");
    }
    else
    {
      acked = true;
      lastMotion = now();
      Serial.println("Motion:Motion Detected, and Ackd");
      return true;
    }
  }
  if (!motion)
    acked = false;
  return false;
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
 */
void MotionSensor::forceAck()
{
  ForcedAck = true;
  lastMotion = now();
  Serial.println("Motion: Forced ACK (relay spike suppressed)");
}

/**
 * @brief Gets the current active state of the sensor.
 * @return True if sensor pin is HIGH, false otherwise.
 */
bool MotionSensor::isActive() { return digitalRead(pin) == HIGH; }
