/**
 * @file HVAC.h
 * @brief HVAC heating and cooling system control with temperature-based switching.
 *
 * This file defines the GarageHVAC class which manages independent heating and cooling
 * relays with configurable setpoint temperatures, hysteresis (swing), and operational modes.
 * Integrates with motion sensor for lockout detection (disables when door is open).
 *
 * The system provides four operational modes:
 * - **Off**: No heating/cooling
 * - **Heat**: Heating only, enables anytime door is open
 * - **Cool**: Cooling only, enables anytime door is open  
 * - **Heat_Cool**: Both enabled, but locked out when door is open (energy saving)
 *
 * Temperature control uses hysteresis to prevent relay chatter near setpoints.
 */

#ifndef HVAC_H
#define HVAC_H

#include <Arduino.h>
#include "Motion.h"

/**
 * @class GarageHVAC
 * @brief Controls heating and cooling relays based on temperature setpoints.
 *
 * This class manages:
 * - **Temperature control** with separate heat and cool setpoints
 * - **Hysteresis** (swing) to prevent relay oscillation near setpoints
 * - **Mode selection** (Off, Heat, Cool, Heat_Cool)
 * - **Door-open lockout** to avoid heating/cooling when garage is open
 * - **Energy efficiency** via mode-specific lockout logic
 *
 * Heat/Cool setpoints and swing are configurable via the menu system.
 * Default values: heatSet=65°F, coolSet=85°F, swing=2°F
 *
 * @note All setpoint values are in Fahrenheit. Temperature sensor polling is
 *       handled by the main controller; this class only performs switching logic.
 */
class GarageHVAC
{
    // Hardware configuration
    byte heatPin;           /**< Output pin for heater relay */
    bool heatActiveHigh = true;
    byte coolPin;           /**< Output pin for cooler relay */
    bool coolActiveHigh = true;
    
    // External sensor reference
    MotionSensor &motion;   /**< Detects door open for lockout logic */

public:
    // Configuration variables (settable via menu)
    float heatSet = 65;     /**< Temperature setpoint for heating activation (°F) */
    float coolSet = 85;     /**< Temperature setpoint for cooling activation (°F) */
    int HVACSwing = 1;      /**< Hysteresis range around setpoints to prevent relay chatter (°F) */
    bool lockout = false;   /**< Set to true when door is open to prevent conditioning */

    /**
     * @enum Mode
     * @brief Operational modes for heating and cooling.
     */
    enum Mode
    {
        Off,        /**< No heating or cooling - compressor disabled       */
        Heat,       /**< Heat only - allows heating even when door open   */
        Heat_Cool,  /**< Both heat and cool - locked out when door open   */
        Cool        /**< Cool only - allows cooling even when door open   */
    };
    Mode mode = Heat_Cool;  /**< Current operational mode */

    /**
     * @enum State
     * @brief Current operational state of the HVAC system.
     */
    enum State
    {
        Waiting,    /**< Idle - temperature within setpoint band           */
        Heating,    /**< Heating active - temperature below heatSet-swing  */
        Cooling,    /**< Cooling active - temperature above coolSet+swing  */
        Pending     /**< Cannot activate due to lockout (door open)        */
    };
    State state = Waiting;  /**< Current operational state */

    /**
     * @brief Constructor for GarageHVAC.
     *
     * Initializes hardware pin bindings and default configuration. Pins are set to
     * OUTPUT mode by the constructor.
     *
     * @param heat    Output pin for heater relay (active HIGH)
     * @param cool    Output pin for cooler relay (active HIGH)
     * @param m       Reference to MotionSensor to detect door-open lockout condition
     */
    GarageHVAC(byte heat, byte cool, MotionSensor &m);

    /**
     * @brief Polls temperature and controls heating/cooling relays.
     *
     * Should be called once per main loop with current temperature reading.
     * Implements hysteresis logic to prevent relay cycling near setpoints:
     * - Delays heating until temp < (heatSet - swing)
     * - Delays cooling until temp > (coolSet + swing)
     *
     * Respects operational mode and door-open lockout to avoid conditioning
     * when door is open (in modes that require lockout).
     *
     * @param tempF Current temperature reading in Fahrenheit
     * @return Current State enum value after switching logic
     *
     * @note The lockout flag should be set by the main controller before calling poll()
     *       whenever the garage door is detected as open.
     */
    State poll(float tempF);
};

#endif