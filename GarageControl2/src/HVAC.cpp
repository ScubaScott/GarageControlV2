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
  unsigned long nowMs = now();

  if (mode == Off)
  {
    if (state != Waiting)
    {
      motion.forceAck();
      digitalWrite(heatPin, !heatActiveHigh);
      digitalWrite(coolPin, !coolActiveHigh);
      state = Waiting;
      lastRunEndTime = nowMs;
      Serial.println(F("HVAC:Off"));
    }
    return state;
  }

  // Heat_Cool is the only mode that respects the door-open lockout.
  // Heat-only and Cool-only run unconditionally (manual intent).
  bool useLockout = (mode == Heat_Cool);
  bool canHeat    = (mode == Heat || mode == Heat_Cool);
  bool canCool    = (mode == Cool || mode == Heat_Cool);

  // ── Turn off relays that the current mode no longer uses ─────────────
  // Handles mid-run mode changes (e.g. Heat_Cool → Cool leaves heater on).
  if (!canHeat && state == Heating)
  {
    motion.forceAck();
    digitalWrite(heatPin, !heatActiveHigh);
    state = Waiting;
    Serial.println(F("HVAC:Waiting(mode chg)"));
  }
  if (!canCool && state == Cooling)
  {
    motion.forceAck();
    digitalWrite(coolPin, !coolActiveHigh);
    state = Waiting;
    lastRunEndTime = nowMs;
    Serial.println(F("HVAC:Waiting(mode chg)"));
  }

  bool minRunElapsed = false;
  if (state == Heating || state == Cooling)
  {
    minRunElapsed = (nowMs - lastRunStartTime) >= (unsigned long)minRunTimeMins * 60000UL;
  }

  bool inRest = false;
  if (state != Heating && state != Cooling && lastRunEndTime != 0)
  {
    inRest = (nowMs - lastRunEndTime) < (unsigned long)minRestTimeMins * 60000UL;
  }

  // ── Heating continuation / stop logic ──────────────────────────────────
  if (state == Heating)
  {
    if (useLockout && lockout)
    {
      motion.forceAck();
      digitalWrite(heatPin, !heatActiveHigh);
      state = Pending;
      lastRunEndTime = nowMs;
      Serial.println(F("HVAC:Pending"));
      return state;
    }

    if (tempF > heatSet + HVACSwing && minRunElapsed)
    {
      motion.forceAck();
      digitalWrite(heatPin, !heatActiveHigh);
      state = Waiting;
      lastRunEndTime = nowMs;
      Serial.println(F("HVAC:Waiting"));
      return state;
    }

    return state;
  }

  // ── Heating activation logic ───────────────────────────────────────────
  if (canHeat && tempF < heatSet - HVACSwing)
  {
    if (!useLockout || !lockout)
    {
      if (!inRest)
      {
        motion.forceAck();
        digitalWrite(heatPin, heatActiveHigh);
        digitalWrite(coolPin, !coolActiveHigh);
        state = Heating;
        lastRunStartTime = nowMs;
        lastRunEndTime = 0;
        Serial.println(F("HVAC:Heating"));
        return state;
      }
    }
    else
    {
      motion.forceAck();
      digitalWrite(heatPin, !heatActiveHigh);
      state = Pending;
      Serial.println(F("HVAC:Pending"));
      return state;
    }
  }

  // ── Cooling continuation / stop logic ──────────────────────────────────
  if (state == Cooling)
  {
    if (useLockout && lockout)
    {
      motion.forceAck();
      digitalWrite(coolPin, !coolActiveHigh);
      state = Pending;
      lastRunEndTime = nowMs;
      Serial.println(F("HVAC:Pending"));
      return state;
    }

    if (tempF < coolSet - HVACSwing && minRunElapsed)
    {
      motion.forceAck();
      digitalWrite(coolPin, !coolActiveHigh);
      state = Waiting;
      lastRunEndTime = nowMs;
      Serial.println(F("HVAC:Waiting"));
      return state;
    }

    return state;
  }

  // ── Cooling activation logic ───────────────────────────────────────────
  if (canCool && tempF > coolSet + HVACSwing)
  {
    if (!useLockout || !lockout)
    {
      if (!inRest)
      {
        motion.forceAck();
        digitalWrite(coolPin, coolActiveHigh);
        digitalWrite(heatPin, !heatActiveHigh);
        state = Cooling;
        lastRunStartTime = nowMs;
        lastRunEndTime = 0;
        Serial.println(F("HVAC:Cooling"));
        return state;
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

  return state;
}