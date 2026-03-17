/**
 * @file HVAC.cpp
 * @brief Implementation of HVAC controller.
 *
 * This file implements the GarageHVAC class methods for temperature-based
 * heating control with motion sensor integration and lockout protection.
 */

#include <Arduino.h>
#include "HVAC.h"
#include "Motion.h"
#include "Utility.h"

// ============================================================
//  HVAC Controller
// ============================================================

/**
 * @brief Constructor for GarageHVAC.
 * @param heat Heating relay pin number.
 * @param cool Cooling relay pin number.
 * @param m Reference to MotionSensor instance.
 */
GarageHVAC::GarageHVAC(byte heat, byte cool, MotionSensor &m) : heatPin(heat), coolPin(cool), motion(m)
{
  pinMode(heatPin, OUTPUT);
  digitalWrite(heatPin, !heatActiveHigh);
  pinMode(coolPin, OUTPUT);
  digitalWrite(coolPin, !coolActiveHigh);
}

/**
 * @brief Polls temperature and controls HVAC operation based on mode.
 * @param tempF Current temperature in Fahrenheit.
 * @return Current State enum value.
 */
GarageHVAC::State GarageHVAC::poll(float tempF)
{
  if (mode == Off)
  {
    if (state != Waiting)
    {
      motion.forceAck();
      digitalWrite(heatPin, !heatActiveHigh);
      digitalWrite(coolPin, !coolActiveHigh);
      state = Waiting;
      Serial.println(F("HVAC:Off"));
    }
    return state;
  }

  bool useLockout = (mode == Auto);

  // Heating logic
  if (tempF < heatSet)
  {
    if (!useLockout || !lockout)
    {
      if (state != Heating)
      {
        motion.forceAck();
        digitalWrite(heatPin, heatActiveHigh);
        digitalWrite(coolPin, !coolActiveHigh);
        state = Heating;
        Serial.println(F("HVAC:Heating"));
      }
    }
    else
    {
      motion.forceAck();
      digitalWrite(heatPin, !heatActiveHigh);
      state = Pending;
      Serial.println(F("HVAC:Pending"));
    }
  }
  else if (tempF > heatSet + HVACSwing && state == Heating)
  {
    motion.forceAck();
    digitalWrite(heatPin, !heatActiveHigh);
    state = Waiting;
    Serial.println(F("HVAC:Waiting"));
  }

  // Cooling logic
  if (tempF > coolSet)
  {
    if (!useLockout || !lockout)
    {
      if (state != Cooling)
      {
        motion.forceAck();
        digitalWrite(coolPin, coolActiveHigh);
        digitalWrite(heatPin, !heatActiveHigh);
        state = Cooling;
        Serial.println(F("HVAC:Cooling"));
      }
    }
    else
    {
      motion.forceAck();
      digitalWrite(coolPin, !coolActiveHigh);
      if (state == Cooling) state = Waiting;
      Serial.println(F("HVAC:Pending"));
    }
  }
  else if (tempF < coolSet - HVACSwing && state == Cooling)
  {
    motion.forceAck();
    digitalWrite(coolPin, !coolActiveHigh);
    state = Waiting;
    Serial.println(F("HVAC:Waiting"));
  }

  return state;
}
