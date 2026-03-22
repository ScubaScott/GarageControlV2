/**
 * @file GarageDoor.cpp
 * @brief Implementation of garage door controller.
 *
 * This file implements the GarageDoor class methods for door operation,
 * state monitoring, automatic closing, and error handling.
 */

#include <Arduino.h>
#include "GarageDoor.h"
#include "Motion.h"
#include "Utility.h"

// ============================================================
//  Garage Door
// ============================================================

/**
 * @brief Constructor for GarageDoor.
 * @param btn Button pin number.
 * @param open Open sensor pin number.
 * @param closed Closed sensor pin number.
 * @param m Reference to MotionSensor instance.
 */
GarageDoor::GarageDoor(byte btn, byte open, byte closed, MotionSensor &m)
    : buttonPin(btn), openPin(open), closedPin(closed), motion(m)
{
  pinMode(buttonPin, OUTPUT);
  pinMode(openPin, INPUT);
  pinMode(closedPin, INPUT);
  digitalWrite(buttonPin, !buttonActiveHigh);
}

/**
 * @brief Polls door sensors and updates state.
 * @param motionDetected True if motion is detected.
 * @return Current door state.
 */
GarageDoor::State GarageDoor::poll(bool motionDetected)
{
  bool open = digitalRead(openPin);
  bool closed = digitalRead(closedPin);

  if (open && closed)
  {
    if (errorStart == 0)
      errorStart = now();
    if (expired(errorStart, doorTravelTime))
    {
      if (state != Error)
        Serial.println(F("Door:Error"));
      state = Error;
    }
  }
  else
  {
    errorStart = 0;
    if (open)
    {
      if (state != Open)
      {
        Serial.println(F("Door:Open"));
        state = Open;
        lastOpen = now();
        attempts = 0;
      }
      else if (motionDetected)
      {
        lastOpen = now();
      }
    }
    else if (closed)
    {
      if (state != Closed)
        Serial.println(F("Door:Closed"));
      state = Closed;
    }
    else
    {
      if (state != Moving)
        Serial.println(F("Door:Moving"));
      state = Moving;
    }
  }

  if (state == Open && attempts < maxAttempts)
  {
    if (expired(lastOpen, autoCloseDuration))
    {
      pressButton();
      attempts++;
    }
  }

  if (attempts >= maxAttempts)
  {
    if (state != Disabled)
      Serial.println(F("Door:Disabled"));
    state = Disabled;
  }

  if (buttonPressed && expired(buttonStart, buttonPressTime))
  {
    digitalWrite(buttonPin, !buttonActiveHigh);
    buttonPressed = false;
  }

  return state;
}

/**
 * @brief Manually activates the door button.
 */
void GarageDoor::manualActivate()
{
  Serial.println(F("Door:Manual activate"));
  pressButton();
  attempts = 0;
}

/**
 * @brief Gets the auto-close duration.
 * @return Auto-close duration in milliseconds.
 */
unsigned long GarageDoor::getAutoClose() const { return autoCloseDuration; }

/**
 * @brief Sets the auto-close duration.
 * @param ms Auto-close duration in milliseconds (minimum 1 minute).
 */
void GarageDoor::setAutoClose(unsigned long ms)
{
  if (ms < 60000UL)
    ms = 60000UL;
  autoCloseDuration = ms;
}

/**
 * @brief Gets the door travel time.
 * @return Door travel time in milliseconds.
 */
unsigned long GarageDoor::getDoorTravelTime() const { return doorTravelTime; }

/**
 * @brief Sets the door travel time.
 * @param ms Door travel time in milliseconds (minimum 1 second).
 */
void GarageDoor::setDoorTravelTime(unsigned long ms)
{
  if (ms < 1000UL)
    ms = 1000UL;
  doorTravelTime = ms;
}

/**
 * @brief Gets the maximum retry attempts.
 * @return Maximum retry attempts.
 */
int GarageDoor::getMaxAttempts() const { return maxAttempts; }

/**
 * @brief Sets the maximum retry attempts.
 * @param v Maximum retry attempts.
 */
void GarageDoor::setMaxAttempts(int v)
{
  if (v < 1)
    v = 1;
  if (v > 10)
    v = 10;
  maxAttempts = v;
}

/**
 * @brief Gets the current door state.
 * @return Current State enum value.
 */
GarageDoor::State GarageDoor::getState() const { return state; }

/**
 * @brief Gets the door open time remaining.
 * @return door open time remaining in milliseconds.
 */
unsigned long GarageDoor::getDoorRemainingTime(){
  if (getState() == Open){
    unsigned long elapsed = now() - lastOpen;
    if (elapsed < autoCloseDuration) {
      return autoCloseDuration - elapsed;
    } else {
      return 0;
    }
  }
  return 0;
}

/**
 * @brief Presses the door button for activation.
 */
void GarageDoor::pressButton()
{
  motion.forceAck();
  digitalWrite(buttonPin, buttonActiveHigh);
  buttonStart = now();
  buttonPressed = true;
  Serial.println(F("Door:Button pressed"));
}
