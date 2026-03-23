/**
 * @file Motion.h
 * @brief PIR motion sensor with debouncing and electromagnetic interference rejection.
 *
 * This file defines the MotionSensor class which manages passive infrared (PIR)
 * motion detection with digital debouncing and a forced-acknowledge mechanism to
 * suppress false motion triggers caused by electromagnetic interference from
 * relay switching.
 *
 * The forced acknowledge feature is critical for reliable operation, as the
 * electromagnetic relay pulses can trigger false positive motion events if not
 * suppressed at the sensor level.
 */

#ifndef MOTION_H
#define MOTION_H

#include <Arduino.h>

/**
 * @class MotionSensor
 * @brief PIR motion sensor with debouncing and relay spike rejection.
 *
 * This class provides:
 * - **Digital debouncing** to filter electrical noise and prevent fluttering
 * - **Motion detection** via HIGH signal from PIR sensor module
 * - **Forced acknowledgment** to suppress false triggers from EMI/relay spikes
 * - **Timeout-based activity tracking** for configurable-duration occupancy detection
 *
 * The forced acknowledgment mechanism is called by the GarageLight controller
 * immediately after relay activation, preventing the electromagnetic pulse from
 * being misinterpreted as human motion detection.
 *
 * @note Debouncing is simple HIGH-to-LOW state verification. For more sophisticated
 *       filtering, examine the implementation for duration-based requirements.
 */
class MotionSensor
{
    byte pin;             /**< GPIO pin connected to PIR sensor output (active HIGH) */
    bool ForcedAck = false; /**< Flag indicating forced acknowledgment is active */
    bool acked = false;   /**< Acknowledgment state tracker */
    unsigned long lastMotion = 0; /**< Timestamp of last motion event */

public:
    /**
     * @brief Constructor for MotionSensor.
     *
     * Initializes hardware pin binding. The pin is configured as INPUT by the
     * constructor (may be INPUT_PULLUP depending on sensor module design).
     *
     * @param p GPIO pin connected to PIR sensor output (active HIGH)
     */
    MotionSensor(byte p);

    /**
     * @brief Polls the motion sensor for new motion activity.
     *
     * Should be called once per main loop iteration (typically 100-200ms).
     * Returns true only once per motion event, even if PIR signal remains high.
     * Motion detection is suppressed during forced acknowledgment window.
     *
     * @return True if NEW motion detected and not in forced-ack window,
     *         false otherwise (including during suppression period)
     *
     * @note This method implements debouncing by verifying the pin state
     *       has stabilized HIGH. See poll() implementation for timing details.
     */
    bool poll();

    /**
     * @brief Checks if motion was detected within a specified timeout window.
     *
     * Useful for extending auto-off timers or other features that should
     * respond to recent occupancy without requiring continuous motion.
     *
     * @param timeout Timeout period in milliseconds (e.g., 5000 for 5 seconds)
     * @return True if motion was detected within the last `timeout` milliseconds,
     *         false if longer ago or never detected
     *
     * @note This can be called multiple times without resetting the motion
     *       timestamp, making it suitable for multiple independent consumers.
     */
    bool recentlyActive(unsigned long timeout);

    /**
     * @brief Forces acknowledgment to suppress false motion triggers.
     *
     * Call immediately after relay activation to prevent the electromagnetic
     * pulse from being detected as motion on the next poll() cycle.
     *
     * Motion detection is suppressed for a short duration (see implementation).
     * This is called by GarageLight::turnOn() and must be called before
     * electromagnetic relay actuation to be effective.
     *
     * @note The forced acknowledgment window duration is hardware-dependent.
     *       Pin configuration and debounce timing affects effectiveness.
     */
    void forceAck();

    /**
     * @brief Gets the current real-time motion sensor pin state.
     *
     * Returns the instantaneous GPIO pin reading without debouncing.
     * This is NOT the same as motion detection; it's the raw sensor output.
     *
     * @return True if PIR sensor is currently reading HIGH (motion detected now),
     *         false if reading LOW (no motion at this instant)
     *
     * @note This should be used sparingly. For time-windowed queries,
     *       use recentlyActive() instead.
     */
    bool isActive();
};
#endif