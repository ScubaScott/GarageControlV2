/**
 * @file Motion.h
 * @brief PIR motion sensor with hardware interrupt integration and EMI rejection.
 *
 * This file defines the MotionSensor class which manages passive infrared (PIR)
 * motion detection with hardware interrupt (RISING edge) and a forced-acknowledge
 * mechanism to suppress false motion triggers caused by electromagnetic interference
 * from relay switching.
 *
 * The hardware interrupt provides immediate light activation when motion is detected,
 * while the forced acknowledge feature suppresses false triggers from electromagnetic
 * relay pulses that would otherwise be misinterpreted as human motion.
 *
 * @section ISR Integration
 * The recordMotion() method should be called from pirISR() when the hardware
 * interrupt triggers, recording the motion timestamp for subsystems that need
 * occupancy-based timeout extension (e.g., garage door auto-close).
 */

#ifndef MOTION_H
#define MOTION_H

#include <Arduino.h>

/**
 * @class MotionSensor
 * @brief PIR motion sensor with hardware interrupt and relay spike rejection.
 *
 * This class provides:
 * - **Hardware interrupt** (RISING edge) for immediate motion detection
 * - **ISR integration** via recordMotion() called from pirISR()
 * - **Poll-based acknowledgment** for subsystems requiring occupancy timeout extension
 * - **Forced acknowledgment** to suppress false triggers from EMI/relay spikes
 * - **Timeout-based activity tracking** for configurable-duration occupancy detection
 *
 * The hardware interrupt (pirISR) provides immediate light activation. The recordMotion()
 * method records the timestamp for other subsystems (e.g., garage door) that need to
 * extend timeouts based on recent occupancy. The forced acknowledgment mechanism
 * suppresses false motion triggers from electromagnetic relay pulses.
 *
 * @section Flow
 * 1. Hardware detects RISING edge on PIR pin
 * 2. pirISR() calls recordMotion() to update lastMotion timestamp
 * 3. pirISR() immediately activates lights (no poll delay)
 * 4. poll() is called in main loop, returns true once per motion event for other subsystems
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
     * @brief Polls the motion sensor for acknowledged motion activity.
     *
     * Called once per main loop iteration (typically 100-200ms). Returns true only
     * once per motion event, even if PIR signal remains high. Motion detection is
     * suppressed during forced acknowledgment window.
     *
     * Hardware interrupt (recordMotion) handles debouncing via RISING edge detection.
     * This poll method simply checks acknowledgment state and suppression windows
     * for subsystems (e.g., door timeout extension) that depend on occupancy.
     *
     * @return True if NEW motion detected and not in forced-ack window,
     *         false otherwise (including during suppression period)
     *
     * @note Called from main loop. recordMotion() is called from pirISR().
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
     * @brief Records a motion detection event and updates the timestamp.
     *
     * Called from pirISR() when hardware interrupt (RISING edge) detects motion.
     * Updates the lastMotion timestamp for subsystems needing occupancy-based
     * timeout extension (e.g., garage door auto-close, light timeout).
     *
     * @note This is called at interrupt level from pirISR().
     * @see pirISR() in GarageControl2.ino
     */
    void recordMotion();

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