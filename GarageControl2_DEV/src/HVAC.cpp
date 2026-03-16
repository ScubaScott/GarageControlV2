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
 * @param m Reference to MotionSensor instance.
 */
GarageHVAC::GarageHVAC(byte heat, MotionSensor &m) : heatPin(heat), motion(m)
{
  pinMode(heatPin, OUTPUT);
  digitalWrite(heatPin, !heatActiveHigh);
}

/**
 * @brief Polls temperature and controls HVAC operation.
 * @param tempF Current temperature in Fahrenheit.
 * @return Current State enum value.
 */
GarageHVAC::State GarageHVAC::poll(float tempF)
{
  if (!enabled)
  {
    if (state != Waiting)
    {
      motion.forceAck();
      digitalWrite(heatPin, !heatActiveHigh);
      state = Waiting;
      Serial.println("HVAC: Disabled by HA");
    }
    return state;
  }

  if (tempF < heatSet)
  {
    if (!lockout)
    {
      if (state != Heating)
      {
        motion.forceAck();
        digitalWrite(heatPin, heatActiveHigh);
        state = Heating;
        Serial.println("HVAC: Heating");
      }
    }
    else
    {
      motion.forceAck();
      digitalWrite(heatPin, !heatActiveHigh);
      state = Pending;
      Serial.println("HVAC: Pending");
    }
  }

  if (tempF > heatSet + HVACSwing && state == Heating)
  {
    motion.forceAck();
    digitalWrite(heatPin, !heatActiveHigh);
    state = Waiting;
    Serial.println("HVAC: Waiting");
  }

  if (tempF > coolSet)
  {
    if (state != Cooling)
    {
      state = Cooling;
      Serial.println("HVAC: Cooling");
    }
  }
  else
  {
    if (state == Cooling)
    {
      state = Waiting;
      Serial.println("HVAC: Waiting");
    }
  }

  return state;
}