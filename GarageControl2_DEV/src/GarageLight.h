/**
 * @file GarageLight.h
 * @brief Header file for garage light controller.
 *
 * This file defines the GarageLight class which manages garage lighting,
 * automatic timeout, and integration with motion sensors.
 */

#ifndef GARAGE_LIGHT_H
#define GARAGE_LIGHT_H

#include <Arduino.h>

class MotionSensor;

/**
 * @class GarageLight
 * @brief Controls garage light relay with auto-off functionality.
 *
 * This class manages the garage light relay, handles automatic turn-off after
 * a configurable timeout, and integrates with motion sensors to suppress
 * false triggers from relay spikes.
 */
class GarageLight // Controls the garage light relay, handles auto-off after motion timeout, and integrates with the motion sensor for relay spike suppression
{
  byte pin; /**< Light relay pin */
  bool onState = true; /**< Current light state */
  unsigned long lastOn = 0; /**< Timestamp when light was turned on */
  MotionSensor &motion; /**< Reference to motion sensor */

public:
  unsigned long duration = 20UL * 60000UL; /**< Default timeout duration (20 minutes) */

  /**
   * @brief Constructor for GarageLight.
   * @param p Relay pin number.
   * @param m Reference to MotionSensor instance.
   */
  GarageLight(byte p, MotionSensor &m);

  /**
   * @brief Turns the light on and resets the timeout.
   */
  void turnOn();

  /**
   * @brief Turns the light off immediately.
   */
  void turnOff();

  /**
   * @brief Checks if the light should be turned off due to timeout.
   */
  void poll();

  /**
   * @brief Gets the current light state.
   * @return True if light is on, false otherwise.
   */
  bool isOn();
  
  /**
   * @brief Gets the current light remaing time.
   * @return number of minutes remaining before the light is turned off.
   */
  unsigned long lightRemaining();
};

#endif