/**
 * @file GarageLight.h
 * @brief Garage light control with automatic timeout and motion-based extension.
 *
 * This file defines the GarageLight class which manages a relay-controlled light
 * with automatic turn-off after a configurable timeout period. Motion detection
 * extends the timeout to avoid turning off lights during active occupancy.
 *
 * The motion integration also suppresses false motion triggers caused by
 * electromagnetic relay switching spikes, improving sensor reliability.
 */

#ifndef GARAGE_LIGHT_H
#define GARAGE_LIGHT_H

#include <Arduino.h>

class MotionSensor;

/**
 * @class GarageLight
 * @brief Controls garage light relay with configurable auto-off timeout.
 *
 * This class provides:
 * - **Manual control** via manual turn on/off methods
 * - **Automatic timeout** that turns off light after configured duration
 * - **Motion extension** resets timeout when motion is detected
 * - **Relay spike suppression** via motion sensor integration
 *
 * The light state is controlled by an electromagnetic relay on a GPIO pin.
 * Automatic turn-off is suitable for situations where occupants may forget to
 * manually switch lights off (typical for garage environments).
 *
 * @note Timeout is configurable via menu (default: 20 minutes).
 *       Motion detection extends each timeout without requiring additional button presses.
 */
class GarageLight
{
  // Hardware control
  byte pin;           /**< Output pin for light relay (active HIGH) */
  bool onState = true;/**< Current light state (true=on, false=off) */
  
  // State tracking
  unsigned long lastOn = 0; /**< Timestamp when light was last turned on */
  
  // External reference for motion-based timeout extension
  MotionSensor &motion; /**< Detects occupancy to extend auto-off timeout */

public:
  /**
   * @brief Auto-off timeout duration.
   *
   * When light has been on for this duration without motion detection,
   * it automatically turns off. Configurable via menu.
   * Default: 20 minutes (20 * 60000 milliseconds).
   */
  unsigned long duration = 20UL * 60000UL;

  /**
   * @brief Constructor for GarageLight.
   *
   * Initializes hardware pin binding and sets initial state to off.
   * The pin is configured as OUTPUT by the constructor.
   *
   * @param p Output pin for light relay control (active HIGH)
   * @param m Reference to MotionSensor for timeout extension on occupancy
   */
  GarageLight(byte p, MotionSensor &m);

  /**
   * @brief Turns the light on and resets the timeout.
   *
   * Activates the relay and records current timestamp to restart the
   * auto-off countdown. Called by manual control (menu) or MQTT commands.
   */
  void turnOn();

  /**
   * @brief Turns the light off immediately.
   *
   * Deactivates the relay. This is called either by manual control or
   * automatically when the timeout expires.
   */
  void turnOff();

  /**
   * @brief Checks timeout and automatically turns off light if expired.
   *
   * Should be called once per main loop iteration. Implements motion-based
   * timeout extension: if motion has been detected since last call, the
   * timeout counter resets, effectively extending the on duration.
   *
   * @note Motion detection suppresses false triggers from relay EMI spikes,
   *       making this a dual-purpose sensor integration.
   */
  void poll();

  /**
   * @brief Gets the current light on/off state.
   * @return True if light is on, false if light is off
   */
  bool isOn();
  
  /**
   * @brief Gets time remaining before auto-off timeout expires.
   *
   * Useful for displaying remaining time on the LCD menu.
   *
   * @return Number of minutes remaining before the light turns off automatically
   *         (0 if timeout has already expired but not yet processed by poll())
   */
  unsigned long lightRemaining();
};

#endif