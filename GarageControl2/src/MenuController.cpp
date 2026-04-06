/**
 * @file MenuController.cpp
 *
 * Button debounce approach
 * ────────────────────────
 * Each button has its own lastPressTime and lastPinState so they are
 * completely independent. The old code used a single shared `static`
 * timestamp, meaning pressing any one button would lock out all three
 * for the debounce period — requiring long holds to register reliably.
 *
 * Debounce logic: a press is registered on the falling edge (HIGH→LOW),
 * provided the pin has been stable for at least DEBOUNCE_MS. This fires
 * exactly once per physical press regardless of how long the button is held.
 */

#include <Arduino.h>
#include <LiquidCrystal_I2C.h>
#include "MenuController.h"
#include "HVAC.h"
#include "GarageLight.h"
#include "GarageDoor.h"
#include "Utility.h"

#if ENABLE_WIFI
#include "MQTT.h"
#endif

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

void MenuController::noteActivity()
{
  lastActivity = now();
}

// ── Debounce ──────────────────────────────────────────────────────────────────
/**
 * Returns true exactly once on the falling edge (button press) for the
 * given button index (BTN_UP / BTN_DOWN / BTN_SET), with debounce.
 *
 * Each button maintains its own lastPressTime and lastPinState so they
 * are completely independent of each other.
 */
bool MenuController::pressed(byte btnIndex)
{
  byte pin;
  switch (btnIndex)
  {
  case BTN_UP:
    pin = pinUp;
    break;
  case BTN_DOWN:
    pin = pinDown;
    break;
  default:
    pin = pinSet;
    break;
  }

  bool currentState = digitalRead(pin); // LOW = pressed (INPUT_PULLUP)

  // Detect falling edge: was HIGH, now LOW
  if (lastPinState[btnIndex] == HIGH && currentState == LOW)
  {
    // Only accept if enough time has passed since the last registered press
    if (expired(lastPressTime[btnIndex], DEBOUNCE_MS))
    {
      lastPressTime[btnIndex] = now();
      lastPinState[btnIndex] = currentState;
      return true;
    }
  }

  lastPinState[btnIndex] = currentState;
  return false;
}

// ── Main poll ─────────────────────────────────────────────────────────────────

bool MenuController::poll(IMenuHost &controller, GarageHVAC &hvac, GarageLight &lights, GarageDoor &door)
{
  bool activity = false;

  // Menu idle timeout → return to Main screen
  if (current != Screen::Main && expired(lastActivity, MENU_TIMEOUT))
  {
    Serial.println(F("Menu:Timeout->Main"));
    current = Screen::Main;
    EditMode = false;
    activity = true;
  }

  if (current == Screen::Main)
  {
    // On the main screen UP/DOWN act as direct shortcuts
    if (pressed(BTN_UP))
    {
      noteActivity();
      door.manualActivate();
      Serial.println(F("Menu:UP->door toggle"));
      activity = true;
    }
    if (pressed(BTN_DOWN))
    {
      noteActivity();
      lights.isOn() ? lights.turnOff() : lights.turnOn();
      Serial.println(F("Menu:DOWN->light toggle"));
      activity = true;
    }
    if (pressed(BTN_SET))
    {
      noteActivity();
      handleSet(controller);
      activity = true;
    }
  }
  else
  {
    if (pressed(BTN_UP))
    {
      noteActivity();
      Serial.println(F("Menu:UP"));
      handleUp(hvac, lights, door);
      activity = true;
    }
    if (pressed(BTN_DOWN))
    {
      noteActivity();
      Serial.println(F("Menu:DOWN"));
      handleDown(hvac, lights, door);
      activity = true;
    }
    if (pressed(BTN_SET))
    {
      noteActivity();
      Serial.println(F("Menu:SET"));
      handleSet(controller);
      activity = true;
    }
  }

  return activity;
}

// ── Button action handlers ────────────────────────────────────────────────────

void MenuController::handleSet(IMenuHost &controller)
{
  switch (current)
  {
  case Screen::Main:
    current = Screen::HVACMenu;
    EditMode = false;
    break;
  case Screen::HVACMenu:
    current = Screen::SetHeat;
    break;
  case Screen::SetHeat:
    EditMode = !EditMode;
    break;
  case Screen::SetCool:
    EditMode = !EditMode;
    break;
  case Screen::SetSwing:
    EditMode = !EditMode;
    break;
  case Screen::SetMinRunTime:
    EditMode = !EditMode;
    break;
  case Screen::SetMinRestTime:
    EditMode = !EditMode;
    break;
  case Screen::SetMode:
    EditMode = !EditMode;
    break;
  case Screen::ReloadNV:
    controller.reloadNV();
    break;
  case Screen::HVACBack:
    current = Screen::HVACMenu;
    break;
  case Screen::LightMenu:
    current = Screen::SetLightTimeout;
    break;
  case Screen::SetLightTimeout:
    EditMode = !EditMode;
    break;
  case Screen::LightBack:
    current = Screen::LightMenu;
    break;
  case Screen::DoorMenu:
    current = Screen::SetDoorTimeout;
    break;
  case Screen::SetDoorTimeout:
    EditMode = !EditMode;
    break;
  case Screen::SetDoorAttempts:
    EditMode = !EditMode;
    break;
  case Screen::DoorBack:
    current = Screen::DoorMenu;
    break;
  case Screen::ConfigMenu:
    current = Screen::NetworkInfo;
    break;
  case Screen::NetworkInfo:
    EditMode = !EditMode;
    break;
  case Screen::ConfigBack:
    current = Screen::ConfigMenu;
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
    current = Screen::ConfigMenu;
    break;
  case Screen::HVACBack:
    current = Screen::ReloadNV;
    break;
  case Screen::LightBack:
    current = Screen::SetLightTimeout;
    break;
  case Screen::DoorBack:
    current = Screen::SetDoorAttempts;
    break;
  case Screen::ConfigMenu:
    current = Screen::DoorMenu;
    break;
  case Screen::ConfigBack:
    current = Screen::NetworkInfo;
    break;

  case Screen::SetHeat:
    if (EditMode)
      hvac.heatSet++;
    break;
  case Screen::SetCool:
    if (EditMode)
      hvac.coolSet++;
    else
      current = Screen::SetHeat;
    break;
  case Screen::SetSwing:
    if (EditMode)
      hvac.HVACSwing++;
    else
      current = Screen::SetMinRunTime;
    break;
  case Screen::SetMinRunTime:
    if (EditMode)
    {
      if (hvac.minRunTimeMins < 120)
        hvac.minRunTimeMins++;
    }
    else
      current = Screen::SetSwing;
    break;
  case Screen::SetMinRestTime:
    if (EditMode)
    {
      if (hvac.minRestTimeMins < 120)
        hvac.minRestTimeMins++;
    }
    else
      current = Screen::SetMode;
    break;
  case Screen::SetMode:
    if (EditMode)
      hvac.mode = (GarageHVAC::Mode)((hvac.mode + 1) % 4);
    else
      current = Screen::ReloadNV;
    break;
  case Screen::ReloadNV:
    current = Screen::SetMode;
    break;
  case Screen::SetLightTimeout:
    if (EditMode)
      lights.duration += 60000UL;
    break;
  case Screen::SetDoorTimeout:
    if (EditMode)
      door.setAutoClose(door.getAutoClose() + 60000UL);
    break;
  case Screen::SetDoorAttempts:
    if (EditMode)
      door.setMaxAttempts(door.getMaxAttempts() + 1);
    else
      current = Screen::SetDoorTimeout;
    break;
  case Screen::NetworkInfo:
    if (EditMode)
    {
#if ENABLE_WIFI
      if (g_mqttManager)
      {
        g_mqttManager->resetNetStatus();
        Serial.println(F("Menu:Network reset (retry connections)"));
      }
#endif
      EditMode = false;
    }
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
    current = Screen::ConfigMenu;
    break;
  case Screen::ConfigMenu:
    current = Screen::MenuExit;
    break;

  case Screen::SetHeat:
    if (EditMode)
      hvac.heatSet--;
    else
      current = Screen::SetCool;
    break;
  case Screen::SetCool:
    if (EditMode)
      hvac.coolSet--;
    else
      current = Screen::SetSwing;
    break;
  case Screen::SetSwing:
    if (EditMode)
      hvac.HVACSwing--;
    else
      current = Screen::SetMinRunTime;
    break;
  case Screen::SetMinRunTime:
    if (EditMode)
    {
      if (hvac.minRunTimeMins > 1)
        hvac.minRunTimeMins--;
    }
    else
      current = Screen::SetSwing;
    break;
  case Screen::SetMinRestTime:
    if (EditMode)
    {
      if (hvac.minRestTimeMins > 0)
        hvac.minRestTimeMins--;
    }
    else
      current = Screen::SetMinRunTime;
    break;
  case Screen::SetMode:
    if (EditMode)
      hvac.mode = (GarageHVAC::Mode)((hvac.mode + 3) % 4);
    else
      current = Screen::SetMinRestTime;
    break;
  case Screen::ReloadNV:
    current = Screen::HVACBack;
    break;
  case Screen::SetLightTimeout:
    if (EditMode)
      lights.duration -= 60000UL;
    else
      current = Screen::LightBack;
    break;
  case Screen::SetDoorTimeout:
    if (EditMode)
      door.setAutoClose(door.getAutoClose() - 60000UL);
    else
      current = Screen::SetDoorAttempts;
    break;
  case Screen::SetDoorAttempts:
    if (EditMode)
      door.setMaxAttempts(door.getMaxAttempts() - 1);
    else
      current = Screen::DoorBack;
    break;
  case Screen::NetworkInfo:
    if (EditMode)
    {
#if ENABLE_WIFI
      if (g_mqttManager)
      {
        g_mqttManager->disableNetwork();
        Serial.println(F("Menu:Network disabled"));
      }
#endif
      EditMode = false;
    }
    else
      current = Screen::ConfigBack;
    break;
  default:
    break;
  }
}