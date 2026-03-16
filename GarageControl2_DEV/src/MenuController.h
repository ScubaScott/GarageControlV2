/**
 * @file MenuController.h
 * @brief Header file for menu navigation controller.
 *
 * This file defines the MenuController class which handles LCD menu navigation,
 * button input processing, and timeout management for the garage control system.
 */

#ifndef MENU_CONTROLLER_H
#define MENU_CONTROLLER_H

#include <Arduino.h>

class GarageHVAC;
class GarageLight;
class GarageDoor;

/**
 * @class MenuController
 * @brief Manages LCD menu navigation and button input processing.
 *
 * This class handles menu screen transitions, button debouncing, timeout management,
 * and editing modes for configuring garage door, lighting, and HVAC settings.
 */
class MenuController
{
public:
  /**
   * @enum Screen
   * @brief Enumeration of available menu screens.
   */
  enum class Screen
  {
    Main, /**< Main status screen */
    HVACMenu, /**< HVAC settings menu */
    SetHeat, /**< Heat setpoint adjustment */
    SetCool, /**< Cool setpoint adjustment */
    SetSwing, /**< Temperature swing adjustment */
    HVACBack, /**< Return to HVAC menu */
    LightMenu, /**< Light settings menu */
    SetLightTimeout, /**< Light timeout adjustment */
    LightBack, /**< Return to light menu */
    DoorMenu, /**< Door settings menu */
    SetDoorTimeout, /**< Door timeout adjustment */
    SetDoorAttempts, /**< Door retry attempts adjustment */
    DoorBack, /**< Return to door menu */
    MenuExit, /**< Exit menu */
    Count /**< Number of screens */
  };

  bool EditMode = false; /**< Flag indicating if in edit mode */

private:
  Screen current = Screen::Main; /**< Current menu screen */

  unsigned long lastButton = 0; /**< Timestamp of last button press */
  const unsigned long timeout = 15000UL; /**< Menu timeout in milliseconds */

  byte pinUp, pinDown, pinSet; /**< Button pin assignments */

public:
  /**
   * @brief Constructor for MenuController.
   * @param up Pin number for up button.
   * @param down Pin number for down button.
   * @param set Pin number for set button.
   */
  MenuController(byte up, byte down, byte set);

  /**
   * @brief Initializes the menu controller.
   */
  void begin();

  /**
   * @brief Gets the current menu screen.
   * @return Current Screen enum value.
   */
  Screen get() const;

  /**
   * @brief Polls button inputs and updates menu state.
   * @param hvac Reference to GarageHVAC for settings.
   * @param lights Reference to GarageLight for settings.
   * @param door Reference to GarageDoor for settings.
   * @return True if any button press or screen change occurred.
   */
  bool poll(GarageHVAC &hvac, GarageLight &lights, GarageDoor &door);

  /**
   * @brief Resets the menu timeout due to external activity.
   */
  void noteActivity();

private:
  bool pressed(byte pin);
  void handleSet();
  void handleUp(GarageHVAC &hvac, GarageLight &lights, GarageDoor &door);
  void handleDown(GarageHVAC &hvac, GarageLight &lights, GarageDoor &door);
};

#endif