/**
 * @file MenuController.cpp
 * @brief Implementation of menu navigation controller.
 *
 * This file implements the MenuController class methods for handling button inputs,
 * menu screen transitions, and configuration editing for the garage control system.
 */

#include <Arduino.h>
#include <LiquidCrystal_I2C.h>
#include "MenuController.h"
#include "HVAC.h"
#include "GarageLight.h"
#include "GarageDoor.h"
#include "Utility.h"

// ============================================================
//  Menu Controller
// ============================================================

/**
 * @brief Constructor for MenuController.
 * @param up Pin number for up button.
 * @param down Pin number for down button.
 * @param set Pin number for set button.
 */
MenuController::MenuController(byte up, byte down, byte set)
    : pinUp(up), pinDown(down), pinSet(set) {}

/**
 * @brief Initializes the menu controller pins.
 */
void MenuController::begin()
{
  pinMode(pinUp, INPUT_PULLUP);
  pinMode(pinDown, INPUT_PULLUP);
  pinMode(pinSet, INPUT_PULLUP);
}

/**
 * @brief Gets the current menu screen.
 * @return Current Screen enum value.
 */
MenuController::Screen MenuController::get() const { return current; }

/**
 * @brief Polls button inputs and updates menu state.
 * @param hvac Reference to GarageHVAC for settings.
 * @param lights Reference to GarageLight for settings.
 * @param door Reference to GarageDoor for settings.
 * @return True if any button press or screen change occurred.
 */
bool MenuController::poll(GarageHVAC &hvac, GarageLight &lights, GarageDoor &door)
{
  bool activity = false;

  if (expired(lastButton, timeout))
  {
    if (current != Screen::Main)
    {
      Serial.println("Menu:Menu timeout → Main");
      activity = true;
    }
    current = Screen::Main;
  }

  unsigned long t = now();

  if (current == Screen::Main)
  {
    // Main Screen only - Special functions.
    // Single press of UP → toggle door
    if (pressed(pinUp))
    {
      lastButton = t;
      door.manualActivate();
      Serial.println("Menu:Main: UP pressed → door toggle");
      activity = true;
    }
    // Single press of DOWN → toggle light
    if (pressed(pinDown))
    {
      lastButton = t;
      if (lights.isOn())
        lights.turnOff();
      else
        lights.turnOn();
      Serial.println("Menu:Main: DOWN pressed → light toggle");
      activity = true;
    }
    // SET enters the menu
    if (pressed(pinSet))
    {
      lastButton = t;
      handleSet();
      activity = true;
    }
  }
  else
  {
    // All other screens: UP/DOWN/SET navigate/adjust as before
    if (pressed(pinUp))
    {
      lastButton = t;
      Serial.println("Menu:Menu: UP pressed");
      handleUp(hvac, lights, door);
      activity = true;
    }
    if (pressed(pinDown))
    {
      lastButton = t;
      Serial.println("Menu:Menu: DOWN pressed");
      handleDown(hvac, lights, door);
      activity = true;
    }
    if (pressed(pinSet))
    {
      lastButton = t;
      Serial.println("Menu:Menu: SET pressed");
      handleSet();
      activity = true;
    }
  }

  return activity;
}

/**
 * @brief Resets the menu timeout due to external activity.
 */
void MenuController::noteActivity()
{
  lastButton = now();
}

/**
 * @brief Checks if a button is pressed with debouncing.
 * @param pin Pin number to check.
 * @return True if button is pressed and debounced.
 */
bool MenuController::pressed(byte pin)
{
  static unsigned long debounce = 300;
  static unsigned long last = 0;
  if (digitalRead(pin) == LOW && expired(last, debounce))
  {
    last = now();
    return true;
  }
  return false;
}

void MenuController::handleSet()
{
  switch (current)
  {
  case Screen::Main:
    current = Screen::HVACMenu;
    EditMode = false; // ensure edit mode dosen't persist.
    break;
  case Screen::HVACMenu:
    current = Screen::SetHeat;
    break;
  case Screen::SetHeat:
    EditMode = !EditMode;
    // current = Screen::SetCool;
    break;
  case Screen::SetCool:
    EditMode = !EditMode;
    // current = Screen::SetSwing;
    break;
  case Screen::SetSwing:
    EditMode = !EditMode;
    // current = Screen::LightMenu;
    break;
  case Screen::HVACBack:
    current = Screen::HVACMenu;
    break;
  case Screen::LightMenu:
    current = Screen::SetLightTimeout;
    break;
  case Screen::SetLightTimeout:
    EditMode = !EditMode;
    // current = Screen::DoorMenu;
    break;
  case Screen::LightBack:
    current = Screen::LightMenu;
    break;
  case Screen::DoorMenu:
    current = Screen::SetDoorTimeout;
    break;
  case Screen::SetDoorTimeout:
    EditMode = !EditMode;
    // current = Screen::SetDoorAttempts;
    break;
  case Screen::SetDoorAttempts:
    EditMode = !EditMode;
    // current = Screen::Main;
    break;
  case Screen::DoorBack:
    current = Screen::DoorMenu;
    break;
  case Screen::MenuExit:
    current = Screen::Main;
    break;
  default:
    current = Screen::Main;
    break;
  }
}

void MenuController::handleUp(GarageHVAC &hvac, GarageLight &lights, GarageDoor &door)
{
  switch (current)
  {
  case Screen::LightMenu:
    current = Screen::HVACMenu;
    break;
  case Screen::DoorMenu:
    current = Screen::LightMenu;
    break;
  case Screen::MenuExit:
    current = Screen::DoorMenu;
    break;
  case Screen::SetHeat:
    if (EditMode)
      hvac.heatSet++;
    break;
  case Screen::SetCool:
    if (EditMode)
    {
      hvac.coolSet++;
    }
    else
    {
      current = Screen::SetHeat;
    }
    break;
  case Screen::SetSwing:
    if (EditMode)
    {
      hvac.HVACSwing++;
    }
    else
    {
      current = Screen::SetCool;
    }
    break;
  case Screen::HVACBack:
    current = Screen::SetSwing;
    break;
  case Screen::SetLightTimeout:
    if (EditMode)
      lights.duration += 60000UL;
    break;
  case Screen::LightBack:
    current = Screen::SetLightTimeout;
    break;
  case Screen::SetDoorTimeout:
    if (EditMode)
      door.setAutoClose(door.getAutoClose() + 60000UL);
    break;
  case Screen::SetDoorAttempts:
    if (EditMode)
    {
      door.setMaxAttempts(door.getMaxAttempts() + 1);
    }
    else
    {
      current = Screen::SetDoorTimeout;
    }
    break;
  case Screen::DoorBack:
    current = Screen::SetDoorAttempts;
    break;
  default:
    break;
  }
}

void MenuController::handleDown(GarageHVAC &hvac, GarageLight &lights, GarageDoor &door)
{
  switch (current)
  {
  case Screen::HVACMenu:
    current = Screen::LightMenu;
    break;
  case Screen::LightMenu:
    current = Screen::DoorMenu;
    break;
  case Screen::DoorMenu:
    current = Screen::MenuExit;
    break;
  case Screen::SetHeat:
    if (EditMode)
    {
      hvac.heatSet--;
    }
    else
    {
      current = Screen::SetCool;
    }
    break;
  case Screen::SetCool:
    if (EditMode)
    {
      hvac.coolSet--;
    }
    else
    {
      current = Screen::SetSwing;
    }
    break;
  case Screen::SetSwing:
    if (EditMode)
    {
      hvac.HVACSwing--;
    }
    else
    {
      current = Screen::HVACBack;
    }
    break;
  case Screen::SetLightTimeout:
    if (EditMode)
    {
      lights.duration -= 60000UL;
    }
    else
    {
      current = Screen::LightBack;
    }
    break;
  case Screen::SetDoorTimeout:
    if (EditMode)
    {
      door.setAutoClose(door.getAutoClose() - 60000UL);
    }
    else
    {
      current = Screen::SetDoorAttempts;
    }
    break;
  case Screen::SetDoorAttempts:
    if (EditMode)
    {
      door.setMaxAttempts(door.getMaxAttempts() - 1);
    }
    else
    {
      current = Screen::DoorBack;
    }
    break;
  default:
    break;
  }
}
