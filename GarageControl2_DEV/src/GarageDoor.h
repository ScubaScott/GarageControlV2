/**
 * @file GarageDoor.h
 * @brief Garage door controller with state detection and automatic closing.
 *
 * This file defines the GarageDoor class which manages professional garage door operation
 * including electromagnetic relay activation, dual-limit sensor state detection, automatic
 * closing with configurable timeout, retry logic for failed operations, and error detection.
 *
 * The class maintains state awareness through two independent limit switches (open and closed
 * sensor inputs) and detects error conditions when both sensors are triggered simultaneously,
 * which is physically impossible during normal operation.
 */

#ifndef GARAGE_DOOR_H
#define GARAGE_DOOR_H

#include <Arduino.h>

class MotionSensor;

/**
 * @class GarageDoor
 * @brief Manages garage door operation, state tracking, and automatic closing.
 *
 * This class provides:
 * - **State detection** via two limit switches (open and closed positions)
 * - **Button activation** with configurable press duration (typically 500ms)
 * - **Automatic closing** after a configurable timeout (default: 30 minutes)
 * - **Retry logic** with configurable maximum attempts before disabling
 * - **Error detection** when hardware state becomes impossible
 * - **Motion integration** to extend timeout while motion is detected
 *
 * @note All timing values are configurable via setter methods to support
 *       different door mechanism speeds and user preferences via the menu system.
 *
 * @section State Diagram
 * - **Open**: Door fully open (monitored for auto-close timeout)
 * - **Closed**: Door fully closed (stable state)
 * - **Moving**: Door between positions (after button press)
 * - **Error**: Both sensors active simultaneously (hardware error)
 * - **Disabled**: Auto-close retries exhausted (manual operation only)
 */
class GarageDoor
{
public:
  /**
   * @enum State
   * @brief Door position and operational state.
   */
  enum State
  {
    Open,    /**< Door is fully open (open sensor active)         */
    Closed,  /**< Door is fully closed (closed sensor active)     */
    Moving,  /**< Door between positions (transitional state)     */
    Error,   /**< Both sensors active - hardware malfunction       */
    Disabled /**< Auto-close retries exhausted - blocked from menu */
  };

private:
  // Hardware pin assignments (configured at construction)
  byte buttonPin, openPin, closedPin;
  bool buttonActiveHigh = true;

  // Current state tracking
  State state = Error;
  unsigned long lastOpen = 0;
  unsigned long buttonStart = 0;
  unsigned long errorStart = 0;
  bool buttonPressed = false;

  // Configuration (settable via menu)
  unsigned long doorTravelTime = 10000UL; /**< Door transits from open to closed */
  int maxAttempts = 3;                    /**< Max retries before auto-close disabled */
  int attempts = 0;                       /**< Current attempt count */

  // Timing constants
  const unsigned long buttonPressTime = 500; /**< Relay activation duration (milliseconds) */
  
  // External reference for motion detection
  MotionSensor &motion;

public:
  /**
   * @brief Auto-close timeout duration.
   *
   * When door has been open for this duration, an auto-close attempt is triggered.
   * Resets on motion detection. Configurable via menu. Default: 30 minutes.
   */
  unsigned long autoCloseDuration = 30UL * 60000UL;

  /**
   * @brief Constructor for GarageDoor.
   *
   * Initializes hardware pin bindings and default configuration. Pins are set to
   * appropriate INPUT/OUTPUT modes by the constructor.
   *
   * @param btn     Output pin for door relay (active HIGH)
   * @param open    Input pin for open sensor (HIGH when door fully open)
   * @param closed  Input pin for closed sensor (HIGH when door fully closed)
   * @param m       Reference to MotionSensor for timeout extension on motion
   */
  GarageDoor(byte btn, byte open, byte closed, MotionSensor &m);

  /**
   * @brief Polls door sensors and updates internal state.
   *
   * Should be called once per main loop iteration (typically 100-200ms intervals).
   * Detects state transitions, monitors auto-close timeout with motion extension,
   * and retries auto-close on failure.
   *
   * @param motionDetected True if motion sensor detected activity in this polling cycle
   * @return Current State enum value after polling
   *
   * @note Motion detection extends the auto-close timeout, useful for occupants
   *       performing multiple activities in the garage.
   */
  State poll(bool motionDetected);

  /**
   * @brief Manually activates the door relay button.
   *
   * Sends a 500ms pulse to the relay, triggering the door opener mechanism.
   * This is called by auto-close logic; also accessible via menu for manual control.
   */
  void manualActivate();

  /**
   * @brief Gets the configured auto-close duration.
   * @return Auto-close duration in milliseconds
   *
   * @see setAutoClose(), autoCloseDuration member variable
   */
  unsigned long getAutoClose() const;

  /**
   * @brief Sets the auto-close duration.
   *
   * Updated via the menu Config->Door->Timeout screen. Changes take effect
   * on the next door opening.
   *
   * @param ms Auto-close duration in milliseconds (e.g., 20 * 60000 for 20 minutes)
   *
   * @see getAutoClose()
   */
  void setAutoClose(unsigned long ms);

  /**
   * @brief Gets the configured door travel time.
   * @return Door travel time in milliseconds
   *
   * @note This is the expected time for door to fully transition from open to closed
   *       (or vice versa). Detects error state if door hasn't reached end position
   *       within this time.
   */
  unsigned long getDoorTravelTime() const;

  /**
   * @brief Gets remaining time until auto-close timeout.
   *
   * Only meaningful when door is in Open state. Returns time remaining in current
   * auto-close interval.
   *
   * @return Time remaining in milliseconds before auto-close triggers
   */
  unsigned long getDoorRemainingTime();

  /**
   * @brief Sets the expected door travel time.
   *
   * If door hasn't fully opened/closed within this duration after button press,
   * the system enters error state. Configurable via menu to support different
   * opener mechanisms.
   *
   * @param ms Door travel time in milliseconds (typically 10000-15000)
   *
   * @see getDoorTravelTime()
   */
  void setDoorTravelTime(unsigned long ms);

  /**
   * @brief Gets the maximum auto-close retry attempts.
   * @return Maximum retry count before auto-close is permanently disabled
   *
   * @see setMaxAttempts()
   */
  int getMaxAttempts() const;

  /**
   * @brief Sets the maximum auto-close retry attempts.
   *
   * If auto-close fails (door doesn't fully close) this many times, the system
   * disables further auto-close attempts from the menu to avoid relay cycling.
   * Manual override is still possible. Restored on system reboot.
   *
   * @param v Maximum retry count (typically 2-5)
   *
   * @see getMaxAttempts()
   */
  void setMaxAttempts(int v);

  /**
   * @brief Gets the current door position/state.
   * @return Current State enum value (Open, Closed, Moving, Error, or Disabled)
   *
   * Called by MQTT publish and display update routines to determine what to
   * show the user.
   */
  State getState() const;

private:
  /**
   * @brief Internal: Activates the door relay for 500ms.
   *
   * Sets button pin HIGH, records timestamp, then clears flag on next poll()
   * when timeout expires.
   */
  void pressButton();
};

#endif