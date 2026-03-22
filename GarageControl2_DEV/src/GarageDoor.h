/**
 * @file GarageDoor.h
 * @brief Header file for garage door controller.
 *
 * This file defines the GarageDoor class which manages garage door operation,
 * state detection, automatic closing, and error handling.
 */

#ifndef GARAGE_DOOR_H
#define GARAGE_DOOR_H

#include <Arduino.h>

class MotionSensor;

/**
 * @class GarageDoor
 * @brief Controls garage door operation and monitoring.
 *
 * This class handles garage door button activation, state detection via sensors,
 * automatic closing after timeout, and retry logic for failed operations.
 */
class GarageDoor
{
public:
  /**
   * @enum State
   * @brief Enumeration of possible garage door states.
   */
  enum State
  {
    Open,    /**< Door is fully open */
    Closed,  /**< Door is fully closed */
    Moving,  /**< Door is in motion */
    Error,   /**< Door operation error */
    Disabled /**< Door control disabled */
  };

private:
  byte buttonPin, openPin, closedPin; /**< Pin assignments for door control */
  bool buttonActiveHigh = true;       /**< Button activation polarity */

  State state = Error;           /**< Current door state */
  unsigned long lastOpen = 0;    /**< Timestamp when door was last opened */
  unsigned long buttonStart = 0; /**< Timestamp when button was pressed */
  unsigned long errorStart = 0;  /**< Timestamp when error started */
  bool buttonPressed = false;    /**< Flag for button press state */

  unsigned long doorTravelTime = 10000UL; /**< Door travel time in milliseconds */
  int maxAttempts = 3;                    /**< Maximum retry attempts */
  int attempts = 0;                       /**< Current attempt count */

  const unsigned long buttonPressTime = 500; /**< Button press duration */
  MotionSensor &motion;                      /**< Reference to motion sensor */

public:
  unsigned long autoCloseDuration = 30UL * 60000UL; /**< Auto-close timeout (30 minutes) */
  /**
   * @brief Constructor for GarageDoor.
   * @param btn Button pin number.
   * @param open Open sensor pin number.
   * @param closed Closed sensor pin number.
   * @param m Reference to MotionSensor instance.
   */
  GarageDoor(byte btn, byte open, byte closed, MotionSensor &m);

  /**
   * @brief Polls door sensors and updates state.
   * @param motionDetected True if motion is detected.
   * @return Current door state.
   */
  State poll(bool motionDetected);

  /**
   * @brief Manually activates the door button.
   */
  void manualActivate();

  /**
   * @brief Gets the auto-close duration.
   * @return Auto-close duration in milliseconds.
   */
  unsigned long getAutoClose() const;

  /**
   * @brief Sets the auto-close duration.
   * @param ms Auto-close duration in milliseconds.
   */
  void setAutoClose(unsigned long ms);

  /**
   * @brief Gets the door travel time.
   * @return Door travel time in milliseconds.
   */
  unsigned long getDoorTravelTime() const;

  /**
   * @brief Gets the door open time remaining.
   * @return door open time remaining in milliseconds.
   */
  unsigned long getDoorRemainingTime();

  /**
   * @brief Sets the door travel time.
   * @param ms Door travel time in milliseconds.
   */
  void setDoorTravelTime(unsigned long ms);

  /**
   * @brief Gets the maximum retry attempts.
   * @return Maximum retry attempts.
   */
  int getMaxAttempts() const;

  /**
   * @brief Sets the maximum retry attempts.
   * @param v Maximum retry attempts.
   */
  void setMaxAttempts(int v);

  /**
   * @brief Gets the current door state.
   * @return Current State enum value.
   */
  State getState() const;

private:
  void pressButton();
};

#endif