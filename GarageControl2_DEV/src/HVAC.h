/**
 * @file HVAC.h
 * @brief Header file for HVAC controller.
 *
 * This file defines the GarageHVAC class which manages heating and cooling
 * control based on temperature thresholds and motion sensor integration.
 */

#ifndef HVAC_H
#define HVAC_H

#include <Arduino.h>
#include "Motion.h"

class GarageHVAC
{
    byte heatPin; /**< Heating relay pin */
    bool heatActiveHigh = true; /**< Heating relay activation polarity */
    byte coolPin; /**< Cooling relay pin */
    bool coolActiveHigh = true; /**< Cooling relay activation polarity */
    MotionSensor &motion; /**< Reference to motion sensor */

public:
    float heatSet = 65; /**< Heating setpoint temperature (°F) */
    float coolSet = 85; /**< Cooling setpoint temperature (°F) */
    int HVACSwing = 2; /**< Temperature swing/hysteresis (°F) */
    bool lockout = false; /**< HVAC lockout flag (when door is open) */

    /**
     * @enum Mode
     * @brief Enumeration of HVAC operation modes.
     */
    enum Mode
    {
        Off, /**< HVAC system off */
        Auto, /**< Automatic operation with lockout */
        On /**< Manual operation without lockout */
    };
    Mode mode = Auto; /**< Current HVAC mode */

    /**
     * @enum State
     * @brief Enumeration of HVAC system states.
     */
    enum State
    {
        Waiting, /**< System idle */
        Heating, /**< Heating active */
        Cooling, /**< Cooling active */
        Pending /**< Waiting due to lockout */
    };
    State state = Waiting; /**< Current HVAC state */

    /**
     * @brief Constructor for GarageHVAC.
     * @param heat Heating relay pin number.
     * @param cool Cooling relay pin number.
     * @param m Reference to MotionSensor instance.
     */
    GarageHVAC(byte heat, byte cool, MotionSensor &m);

    /**
     * @brief Polls temperature and controls HVAC operation based on mode.
     * @param tempF Current temperature in Fahrenheit.
     * @return Current State enum value.
     */
    State poll(float tempF);
};

#endif