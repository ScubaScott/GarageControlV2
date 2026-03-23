/**
 * @file GarageDoor.cpp
 * @brief Implementation of garage door control and state monitoring.
 *
 * This file implements the GarageDoor class methods including:
 * - Constructor with hardware pin initialization
 * - State machine polling with sensor debouncing via timeout logic
 * - Automatic close timeout with motion-based extension
 * - Retry logic preventing excessive relay cycles
 * - Configuration getter/setter methods with bounds checking
 *
 * The state machine monitors two limit switches and detects impossible states
 * (both switches active) to signal hardware malfunction. Motion detection prevents
 * false auto-close timeouts during occupancy. Failed auto-close attempts disable
 * the feature after maxAttempts to avoid relay wear.
 */

#include <Arduino.h>
#include "GarageDoor.h"
#include "Motion.h"
#include "Utility.h"

// ============================================================
//  Garage Door - State Machine and Control
// ============================================================

/**
 * @brief Constructor for GarageDoor.
 *
 * Initializes hardware pin bindings and sets pins to appropriate IO modes.
 * Button pin is set LOW (inactive) since buttonActiveHigh=true.
 *
 * @param btn   Output pin for door relay (active HIGH)
 * @param open  Input pin for open sensor (HIGH when fully open)
 * @param closed Input pin for closed sensor (HIGH when fully closed)
 * @param m     Reference to MotionSensor for timeout extension
 */
GarageDoor::GarageDoor(byte btn, byte open, byte closed, MotionSensor &m)
    : buttonPin(btn), openPin(open), closedPin(closed), motion(m)
{
  // Configure hardware pins for door control
  pinMode(buttonPin, OUTPUT);
  pinMode(openPin, INPUT);
  pinMode(closedPin, INPUT);
  
  // Initialize button to inactive state (LOW since activeHigh=true)
  digitalWrite(buttonPin, !buttonActiveHigh);
}

/**
 * @brief Polls door sensors and updates state machine.
 *
 * Implements a state machine monitoring door position via two limit switches
 * and managing automatic close timeout with motion extension. Called once per
 * main loop iteration to maintain accurate state tracking.
 *
 * State transitions:
 * - **Error detection**: Both sensors active → Error state (impossible condition)
 * - **Position detection**: Sensors determine Open/Closed/Moving
 * - **Motion extension**: Door open + motion detected → extends auto-close timeout
 * - **Auto-close trigger**: Open timeout expires → activates relay, increments attempts
 * - **Disabled**: Max attempts reached → disables further auto-close
 *
 * @param motionDetected True if motion sensor detected activity this polling cycle
 * @return Current state enum (Open, Closed, Moving, Error, or Disabled)
 */
GarageDoor::State GarageDoor::poll(bool motionDetected)
{
  // Read current sensor states (HIGH = sensor triggered, LOW = inactive)
  bool open = digitalRead(openPin);
  bool closed = digitalRead(closedPin);

  // ──────────────────────────────────────────────────────────
  // ERROR DETECTION: Both sensors active simultaneously is impossible
  // ──────────────────────────────────────────────────────────
  if (open && closed)
  {
    // First time entering error condition: start error timer
    if (errorStart == 0)
      errorStart = now();
    
    // Error state confirmed if both sensors remain active for doorTravelTime
    if (expired(errorStart, doorTravelTime))
    {
      if (state != Error)
        Serial.println(F("Door:Error"));
      state = Error;
    }
  }
  else
  {
    // Exit error condition: at least one sensor released
    errorStart = 0;
    
    // ──────────────────────────────────────────────────────────
    // POSITION STATE: Determine door position from sensors
    // ──────────────────────────────────────────────────────────
    if (open)
    {
      // Door fully open (open sensor active)
      if (state != Open)
      {
        Serial.println(F("Door:Open"));
        state = Open;
        lastOpen = now();
        attempts = 0;  // Reset attempts counter on new opening
      }
      else if (motionDetected)
      {
        // Motion detected while door is open: extend auto-close timeout
        // User is actively using the garage - don't auto-close yet
        lastOpen = now();
      }
    }
    else if (closed)
    {
      // Door fully closed (closed sensor active)
      if (state != Closed)
        Serial.println(F("Door:Closed"));
      state = Closed;
    }
    else
    {
      // Door between positions (neither sensor active)
      if (state != Moving)
        Serial.println(F("Door:Moving"));
      state = Moving;
    }
  }

  // ──────────────────────────────────────────────────────────
  // AUTO-CLOSE LOGIC: Trigger relay activation on timeout
  // ──────────────────────────────────────────────────────────
  if (state == Open && attempts < maxAttempts)
  {
    // Check if auto-close timeout has expired since door opened
    if (expired(lastOpen, autoCloseDuration))
    {
      // Timeout expired: activate door button and track this attempt
      pressButton();
      attempts++;
    }
  }

  // ──────────────────────────────────────────────────────────
  // DISABLE AUTO-CLOSE: Stop retrying after max failed attempts
  // ──────────────────────────────────────────────────────────
  if (attempts >= maxAttempts)
  {
    if (state != Disabled)
      Serial.println(F("Door:Disabled"));
    state = Disabled;
  }

  // ──────────────────────────────────────────────────────────
  // RELAY PULSE: Deactivate button after 500ms pulse duration
  // ──────────────────────────────────────────────────────────
  if (buttonPressed && expired(buttonStart, buttonPressTime))
  {
    // Release button pin (return to inactive state)
    digitalWrite(buttonPin, !buttonActiveHigh);
    buttonPressed = false;
  }

  return state;
}

/**
 * @brief Manually activates the door relay button.
 *
 * Initiates a button press via pressButton(). Resets the retry attempt counter
 * to 0, allowing the auto-close feature to resume if manual activation succeeds
 * in closing the door.
 *
 * Called by menu button press or MQTT command to open/toggle the door manually.
 */
void GarageDoor::manualActivate()
{
  Serial.println(F("Door:Manual activate"));
  pressButton();
  attempts = 0;  // Reset attempts so auto-close can try again after manual
}

/**
 * @brief Gets the configured auto-close duration.
 * @return Auto-close timeout in milliseconds
 */
unsigned long GarageDoor::getAutoClose() const { return autoCloseDuration; }

/**
 * @brief Sets the auto-close duration.
 *
 * Enforces a minimum of 1 minute (60000 ms) to prevent excessive cycling.
 *
 * @param ms Desired auto-close timeout in milliseconds (enforced minimum: 60000)
 */
void GarageDoor::setAutoClose(unsigned long ms)
{
  // Enforce minimum 1-minute timeout to prevent excessive relay cycling
  if (ms < 60000UL)
    ms = 60000UL;
  autoCloseDuration = ms;
}

/**
 * @brief Gets the expected door travel time.
 * @return Door travel time in milliseconds
 */
unsigned long GarageDoor::getDoorTravelTime() const { return doorTravelTime; }

/**
 * @brief Sets the expected door travel time.
 *
 * If door fails to reach end position within this duration after button press,
 * the system enters error state. Enforces a minimum of 1 second.
 *
 * @param ms Desired door travel time in milliseconds (enforced minimum: 1000)
 */
void GarageDoor::setDoorTravelTime(unsigned long ms)
{
  // Enforce minimum 1-second to prevent false error triggers
  if (ms < 1000UL)
    ms = 1000UL;
  doorTravelTime = ms;
}

/**
 * @brief Gets the maximum auto-close retry attempts.
 * @return Current max retry count
 */
int GarageDoor::getMaxAttempts() const { return maxAttempts; }

/**
 * @brief Sets the maximum auto-close retry attempts.
 *
 * Enforces bounds: minimum 1, maximum 10 attempts. After this many failed
 * auto-close attempts, the feature is disabled to prevent relay wear.
 *
 * @param v Desired max attempts (enforced range: 1-10)
 */
void GarageDoor::setMaxAttempts(int v)
{
  // Enforce bounds to prevent unreasonable values
  if (v < 1)
    v = 1;
  if (v > 10)
    v = 10;
  maxAttempts = v;
}

/**
 * @brief Gets the current door position/state.
 * @return Current State enum (Open, Closed, Moving, Error, or Disabled)
 */
GarageDoor::State GarageDoor::getState() const { return state; }

/**
 * @brief Gets time remaining until auto-close timeout expires.
 *
 * Calculates elapsed time since door opened and subtracts from the
 * auto-close duration. Only valid when door is in Open state.
 *
 * @return Milliseconds remaining before auto-close triggers (0 if expired or not Open)
 */
unsigned long GarageDoor::getDoorRemainingTime(){
  if (getState() == Open){
    unsigned long elapsed = now() - lastOpen;
    if (elapsed < autoCloseDuration) {
      // Return time remaining
      return autoCloseDuration - elapsed;
    } else {
      // Timeout already expired
      return 0;
    }
  }
  // Door not open - no auto-close active
  return 0;
}

/**
 * @brief Internal: Activates the door relay button for 500ms.
 *
 * Sets button pin HIGH to activate the relay, records the activation timestamp,
 * and sets a flag to clear the pin after 500ms. The relay pulse is stretched
 * by the poll() method when it detects the timeout has expired.
 *
 * Calls forceAck() on the motion sensor to suppress false motion triggers
 * from the electromagnetic relay pulse.
 *
 * @note This method does NOT immediately deactivate the button. The poll()
 *       method handles the deactivation after the pulse duration expires.
 */
void GarageDoor::pressButton()
{
  // Suppress false motion triggers from relay EMI spike
  motion.forceAck();
  
  // Activate button relay (set pin HIGH since activeHigh=true)
  digitalWrite(buttonPin, buttonActiveHigh);
  
  // Record activation time for pulse duration tracking
  buttonStart = now();
  buttonPressed = true;
  
  Serial.println(F("Door:Button pressed"));
}
