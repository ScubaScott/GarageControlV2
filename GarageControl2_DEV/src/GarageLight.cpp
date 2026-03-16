/**
 * @file GarageLight.cpp
 * @brief Implementation of garage light controller.
 *
 * This file implements the GarageLight class methods for light control,
 * automatic timeout, and motion sensor integration.
 */

#include <Arduino.h>
#include "GarageLight.h"
#include "Motion.h"
#include "Utility.h"

// ============================================================
//  Garage Lights
// ============================================================

/**
 * @brief Constructor for GarageLight.
 * @param p Relay pin number.
 * @param m Reference to MotionSensor instance.
 */
GarageLight::GarageLight(byte p, MotionSensor &m) : pin(p), motion(m)
{
  pinMode(pin, OUTPUT);
  digitalWrite(pin, !onState);
}

/**
 * @brief Turns on the garage light and resets the auto-off timer.
 */
void GarageLight::turnOn()
{
  lastOn = now();
  digitalWrite(pin, onState);
  Serial.println("Lights: On");
}

/**
 * @brief Turns off the garage light and forces an acknowledgment to suppress false triggers.
 */
void GarageLight::turnOff()
{
  motion.forceAck();
  digitalWrite(pin, !onState);
  Serial.println("Lights: Off (command)");
}

/**
 * @brief Checks if the light has been on longer than the set duration, and if so, turns it off.
 */
void GarageLight::poll()
{
  if (digitalRead(pin) == onState && expired(lastOn, duration))
  {
    motion.forceAck();
    digitalWrite(pin, !onState);
    Serial.println("Lights: Off (timeout)");
  }
}

/**
 * @brief Returns true if the light is currently on.
 * @return True if light is on, false otherwise.
 */
bool GarageLight::isOn() { return digitalRead(pin) == onState; }