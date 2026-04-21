/**
 * @file GarageLight.cpp
 * @brief Implementation of garage light controller.
 *
 * This file implements the GarageLight class methods for light control,
 * automatic timeout, and motion sensor integration with cooldown periods.
 */

#include <Arduino.h>
#include "GarageLight.h"
#include "Motion.h"
#include "Utility.h"

// ============================================================
//  Garage Lights
// ============================================================

/** @brief 15-second cooldown period to suppress motion after manual turn-off (ms). */
static constexpr unsigned long LIGHT_COOLDOWN_MS = 15000UL;

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
  Serial.println(F("Lights:On"));
}

/**
 * @brief Turns off the garage light and forces an acknowledgment to suppress false triggers.
 *
 * For manual turn-offs (button, MQTT), starts a 15-second cooldown period to prevent
 * motion from re-triggering the lights, allowing occupants to exit without reactivation.
 * For timeout-based turn-offs, no cooldown is applied, allowing motion to immediately
 * re-trigger if the occupant is still present and moves.
 *
 * @param isManual If true (default), applies cooldown. If false, no cooldown (timeout).
 */
void GarageLight::turnOff(bool isManual)
{
  motion.forceAck();
  digitalWrite(pin, !onState);
  
  if (isManual)
  {
    cooldownUntil = now() + LIGHT_COOLDOWN_MS;  // Start 15-second cooldown for manual turn-off
    Serial.println(F("Lights:Off(cmd)+cooldown"));
  }
  else
  {
    cooldownUntil = 0;  // No cooldown for timeout turn-off
    Serial.println(F("Lights:Off(timeout)"));
  }
}

/**
 * @brief Checks if the light has been on longer than the set duration, and if so, turns it off.
 *
 * When timeout expires, turnOff(false) is called to disable cooldown blocking,
 * allowing motion to immediately re-trigger lights if occupant is still present.
 */
void GarageLight::poll()
{
  if (digitalRead(pin) == onState && expired(lastOn, duration))
  {
    turnOff(false);  // Timeout turn-off: no cooldown, motion can re-trigger immediately
  }
}

/**
 * @brief Returns true if the light is currently on.
 * @return True if light is on, false otherwise.
 */
bool GarageLight::isOn() { return digitalRead(pin) == onState; }

/**
 * @brief Gets the current light remaining time.
 * @return number of milliseconds remaining before the light is turned off.
 */
unsigned long GarageLight::lightRemaining()
{
  if (isOn())
  {
    unsigned long elapsed = now() - lastOn;
    if (elapsed < duration) {
      return duration - elapsed;
    } else {
      return 0;
    }
  }
  return 0;
}

/**
 * @brief Checks if the light is currently in cooldown period after manual turn-off.
 * @return True if currently in cooldown period, false otherwise.
 */
bool GarageLight::isInCooldown() const
{
  return now() < cooldownUntil;
}
