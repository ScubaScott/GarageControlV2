/**
 * @file Motion.h
 * @brief Header file for motion sensor controller.
 *
 * This file defines the MotionSensor class which handles PIR motion detection
 * with debouncing and forced acknowledgment for relay spike suppression.
 */

#ifndef MOTION_H
#define MOTION_H

#include <Arduino.h>

/**
 * @class MotionSensor
 * @brief Manages PIR motion sensor with debouncing and acknowledgment.
 *
 * This class handles motion detection from PIR sensors, provides debouncing,
 * and includes forced acknowledgment functionality to suppress false triggers
 * from relay switching spikes.
 */
class MotionSensor
{
    byte pin; /**< PIR sensor pin */
    bool ForcedAck = false; /**< Flag for forced acknowledgment */
    bool acked = false; /**< Acknowledgment state */
    unsigned long lastMotion = 0; /**< Timestamp of last motion detection */

public:
    /**
     * @brief Constructor for MotionSensor.
     * @param p PIR sensor pin number.
     */
    MotionSensor(byte p);

    /**
     * @brief Polls the motion sensor for activity.
     * @return True if new motion detected, false otherwise.
     */
    bool poll();

    /**
     * @brief Checks if motion was recently detected within timeout.
     * @param timeout Timeout period in milliseconds.
     * @return True if motion detected within timeout, false otherwise.
     */
    bool recentlyActive(unsigned long timeout);

    /**
     * @brief Forces acknowledgment to suppress false triggers.
     */
    void forceAck();

    /**
     * @brief Gets the current active state of the sensor.
     * @return True if sensor is currently active, false otherwise.
     */
    bool isActive();
};

#endif