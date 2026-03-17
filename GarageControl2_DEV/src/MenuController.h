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
    Main,
    HVACMenu,
    SetHeat,
    SetCool,
    SetSwing,
    HVACBack,
    LightMenu,
    SetLightTimeout,
    LightBack,
    DoorMenu,
    SetDoorTimeout,
    SetDoorAttempts,
    DoorBack,
    ConfigMenu,
    NetworkInfo,
    NetworkReset,
    ConfigBack,
    MenuExit,
    Count
  };

  bool EditMode = false;

private:
  Screen current = Screen::Main;

  unsigned long lastActivity = 0;          // drives menu timeout
  const unsigned long MENU_TIMEOUT = 20000UL;
  const unsigned long DEBOUNCE_MS  = 50UL; // reduced from 300 ms

  byte pinUp, pinDown, pinSet;

  // Per-button debounce state - each button tracked independently
  enum { BTN_UP = 0, BTN_DOWN = 1, BTN_SET = 2, BTN_COUNT = 3 };
  unsigned long lastPressTime[BTN_COUNT] = {0, 0, 0};
  bool          lastPinState[BTN_COUNT]  = {true, true, true}; // HIGH = unpressed (INPUT_PULLUP)

  bool pressed(byte btnIndex);
  void handleSet();
  void handleUp(GarageHVAC &hvac, GarageLight &lights, GarageDoor &door);
  void handleDown(GarageHVAC &hvac, GarageLight &lights, GarageDoor &door);

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
};

#endif
