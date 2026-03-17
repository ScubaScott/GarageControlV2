/**
 * @file LcdController.cpp
 * @brief Implementation of LCD display controller for garage control system.
 *
 * This file implements the LcdController class methods for managing a 4x20 I2C LCD display,
 * including initialization, text display, backlight control, and status updates.
 */

#include <Arduino.h>
#include <LiquidCrystal_I2C.h>
#include "LcdController.h"
#include "MenuController.h"
#include "GarageDoor.h"
#include "GarageLight.h"
#include "HVAC.h"
#include "Utility.h"

#if ENABLE_WIFI
#include "MQTT.h"
#endif

// ============================================================
//  LCD Controller Constructor & Init
// ============================================================

/**
 * @brief Constructor for LcdController.
 * @param lcdRef Reference to the LiquidCrystal_I2C instance.
 * @param menuRef Reference to the MenuController instance.
 */
LcdController::LcdController(LiquidCrystal_I2C &lcdRef, MenuController &menuRef)
    : lcd(lcdRef), menu(menuRef) {}

/**
 * @brief Initializes the LCD display and creates custom characters.
 */
void LcdController::begin()
{
  lcd.init();
  byte UpArrow[] = {B00100, B01110, B10101, B00100, B00100, B00100, B00100, B00100};
  byte DownArrow[] = {B00100, B00100, B00100, B00100, B00100, B10101, B01110, B00100};
  byte DoubleArrow[] = {B00100, B01110, B10101, B00100, B00100, B10101, B01110, B00100};
  byte Degree[] = {B01000, B10100, B01000, B00000, B00000, B00000, B00000, B00000};
  lcd.createChar(0, UpArrow);
  lcd.createChar(1, DownArrow);
  lcd.createChar(2, DoubleArrow);
  lcd.createChar(3, Degree);
  clearDisplay();
  printLCDText(1, true, "Garage Control");
  printLCDText(2, true, "version 2.0");
}

// ============================================================
//  Private Helper Methods
// ============================================================

/**
 * @brief Clears the LCD display.
 */
void LcdController::clearDisplay()
{
  lcd.clear();
}

/**
 * @brief Sets the cursor position on the LCD.
 * @param row Row number (1-based).
 * @param col Column number (0-based).
 */
void LcdController::setCursor(int row, int col)
{
  lcd.setCursor(col, row - 1);
}

/**
 * @brief Prints text to the LCD with optional centering.
 * @param row Row to print on (1-based).
 * @param center If true, centers the text on the line.
 * @param text Text to print.
 */
void LcdController::printLCDText(int row, bool center, const char *text)
{
  int len = strlen(text);
  int col = 0;
  if (center)
  {
    col = (20 - len) / 2;
    if (col < 0)
      col = 0;
  }
  lcd.setCursor(col, row - 1);
  lcd.print(text);
}

// ============================================================
//  State String Helpers
// ============================================================

/**
 * @brief Gets the door state as a display string.
 * @param door Reference to the GarageDoor instance.
 * @return String representation of door state.
 */
const char *LcdController::getDoorStateString(const GarageDoor &door)
{
  switch (door.getState())
  {
  case GarageDoor::Open:
    return "Open";
  case GarageDoor::Closed:
    return "Closed";
  case GarageDoor::Moving:
    return "Moving";
  case GarageDoor::Error:
    return "Error";
  case GarageDoor::Disabled:
    return "Dsbld";
  }
  return "Unknown";
}

/**
 * @brief Gets the HVAC state as a display string.
 * @param hvac Reference to the GarageHVAC instance.
 * @return String representation of HVAC state.
 */
const char *LcdController::getHvacStateString(const GarageHVAC &hvac)
{
  switch (hvac.state)
  {
  case GarageHVAC::Heating:
    return "Heat";
  case GarageHVAC::Cooling:
    return "Cool";
  case GarageHVAC::Pending:
    return "Pend";
  default:
    return "Wait";
  }
}

// ── Display update ───────────────────────────────────────────────────────────

// ============================================================
//  Main Display Update
// ============================================================

/**
 * @brief Marks the display as needing an update.
 * @param WakeBackLight If true, turns on the backlight.
 */
void LcdController::SetDirty(bool WakeBackLight)
{
  IsDirty = true;
  if (WakeBackLight)
  {
    Serial.println(F("LCD:Wake backlight"));
    setBacklight(true);
  }
}

/**
 * @brief Updates the LCD display with current system status.
 * @param hvac Reference to the GarageHVAC instance.
 * @param door Reference to the GarageDoor instance.
 * @param lights Reference to the GarageLight instance.
 * @param tempF Current temperature in Fahrenheit.
 */
void LcdController::updateDisplay(GarageHVAC &hvac, GarageDoor &door,
                                  GarageLight &lights, float tempF)
{
  if (!IsDirty)
    return;
  Serial.println(F("LCD:Update"));
  IsDirty = false;

  clearDisplay();

  MenuController::Screen currentScreen = menu.get();
  bool EditMode = menu.EditMode;

  // Single shared stack buffer for all snprintf calls in this function.
  // 21 bytes = 20 LCD columns + null terminator.
  char buf[21];

  switch (currentScreen)
  {
  case MenuController::Screen::Main:
  {

    snprintf(buf, sizeof(buf), "F:%.1f\x03  H:%d/C:%d",
             tempF, (int)hvac.heatSet, (int)hvac.coolSet);
    printLCDText(1, true, buf);

    snprintf(buf, sizeof(buf), "Ht:%s / Dr:%s",
             getHvacStateString(hvac), getDoorStateString(door));
    printLCDText(2, true, buf);

    snprintf(buf, sizeof(buf), "HVAC:%s",
             hvac.lockout ? "LOCKOUT" : "Normal");
    printLCDText(3, true, buf);

#if ENABLE_WIFI
    const char *statusStr;
    if (g_mqttManager)
    {
      statusStr = g_mqttManager->getNetStatusString();
    }
    else
    {
      statusStr = "Disabled";
    }
    snprintf(buf, sizeof(buf), "Network:%s", statusStr);
    printLCDText(4, true, buf);
#endif

    break;
  }
  case MenuController::Screen::HVACMenu:
    printLCDText(1, true, "HVAC Settings");
    printLCDText(3, true, "Heat/Cool/Swing/etc");
    printLCDText(4, false, "\x00");
    break;

  case MenuController::Screen::SetHeat:
    printLCDText(1, true, "Heat Setpoint");
    printLCDText(4, false, "\x01");
    EditMode ? lcd.blink_on() : lcd.blink_off();
    snprintf(buf, sizeof(buf), "%d\x03", (int)hvac.heatSet);
    printLCDText(3, true, buf);
    break;

  case MenuController::Screen::SetCool:
    printLCDText(1, true, "Cool Setpoint");
    printLCDText(4, false, "\x02");
    EditMode ? lcd.blink_on() : lcd.blink_off();
    snprintf(buf, sizeof(buf), "%d\x03", (int)hvac.coolSet);
    printLCDText(3, true, buf);
    break;

  case MenuController::Screen::SetSwing:
    printLCDText(1, true, "HVAC Swing");
    printLCDText(4, false, "\x02");
    EditMode ? lcd.blink_on() : lcd.blink_off();
    snprintf(buf, sizeof(buf), "%d\x03", hvac.HVACSwing);
    printLCDText(3, true, buf);
    break;

    // case MenuController::Screen::HVACBack:
    //   printLCDText(1, true, "Back...");
    //   printLCDText(4, false, "\x00");
    //   break;

  case MenuController::Screen::LightMenu:
    printLCDText(1, true, "Light Settings");
    printLCDText(3, true, "Set Timeout");
    printLCDText(4, false, "\x02");
    break;

  case MenuController::Screen::SetLightTimeout:
    printLCDText(1, true, "Light Timeout");
    printLCDText(4, false, "\x01");
    EditMode ? lcd.blink_on() : lcd.blink_off();
    snprintf(buf, sizeof(buf), "%lu minutes", lights.duration / 60000UL);
    printLCDText(3, true, buf);
    break;

    // case MenuController::Screen::LightBack:
    //   printLCDText(1, true, "Back...");
    //   printLCDText(4, false, "\x00");
    //   break;

  case MenuController::Screen::DoorMenu:
    printLCDText(1, true, "Door Settings");
    printLCDText(3, true, "Timeout/Attempts");
    printLCDText(4, false, "\x02");
    break;

  case MenuController::Screen::SetDoorTimeout:
    printLCDText(1, true, "Door Timeout");
    printLCDText(4, false, "\x01");
    EditMode ? lcd.blink_on() : lcd.blink_off();
    snprintf(buf, sizeof(buf), "%lu min", door.getAutoClose() / 60000UL);
    printLCDText(3, true, buf);
    break;

  case MenuController::Screen::SetDoorAttempts:
    printLCDText(1, true, "Close Attempts");
    printLCDText(4, false, "\x02");
    EditMode ? lcd.blink_on() : lcd.blink_off();
    snprintf(buf, sizeof(buf), "%d", door.getMaxAttempts());
    printLCDText(3, true, buf);
    break;

  case MenuController::Screen::ConfigMenu:
    printLCDText(1, true, "Config Settings");
    printLCDText(3, true, "General Settings");
    printLCDText(4, false, "\x02");
    break;

  case MenuController::Screen::NetworkInfo:
  {
    char ipStr[21];
    char mqttStr[21];
    const char *statusStr = "n/a";

#if ENABLE_WIFI
    if (g_mqttManager)
    {
      g_mqttManager->getLocalIP(ipStr, sizeof(ipStr));
      g_mqttManager->getMqttServerIP(mqttStr, sizeof(mqttStr));
      statusStr = g_mqttManager->getNetStatusString();
    }
    else
    {
      strncpy(ipStr, "n/a", sizeof(ipStr));
      ipStr[sizeof(ipStr) - 1] = '\0';
      strncpy(mqttStr, "n/a", sizeof(mqttStr));
      mqttStr[sizeof(mqttStr) - 1] = '\0';
    }
#else
    strncpy(ipStr, "n/a", sizeof(ipStr));
    ipStr[sizeof(ipStr) - 1] = '\0';
    strncpy(mqttStr, "n/a", sizeof(mqttStr));
    mqttStr[sizeof(mqttStr) - 1] = '\0';
    statusStr = "Disabled";
#endif

    printLCDText(1, true, "Network Info");
    snprintf(buf, sizeof(buf), "IP: %s", ipStr);
    printLCDText(2, true, buf);
    snprintf(buf, sizeof(buf), "MQTT: %s", mqttStr);
    printLCDText(3, true, buf);
    EditMode ? lcd.blink_on() : lcd.blink_off();
    snprintf(buf, sizeof(buf), "\x02 Status: %s", statusStr);
    printLCDText(4, false, buf);
  }
  break;

  case MenuController::Screen::ConfigBack:
  case MenuController::Screen::LightBack:
  case MenuController::Screen::HVACBack:
  case MenuController::Screen::DoorBack:
    printLCDText(1, true, "Back...");
    printLCDText(4, false, "\x00");
    break;

  case MenuController::Screen::MenuExit:
    printLCDText(1, true, "Menu Exit...");
    printLCDText(4, false, "\x00");
    break;

  default:
    printLCDText(1, true, "Garage Control");
    printLCDText(3, true, "Unknown Menu");
    printLCDText(4, false, "?");
    break;
  }
}

// ── Backlight ────────────────────────────────────────────────────────────────

// ============================================================
//  Backlight helper
// ============================================================

/**
 * @brief Controls the LCD backlight.
 * @param on True to turn on backlight, false to turn off.
 */
void LcdController::setBacklight(bool on)
{
  if (on && !backlightOn)
  {
    Serial.println(F("LCD:Backlight on"));
    lcd.backlight();
    backlightOn = true;
  }
  if (!on)
  {
    Serial.println(F("LCD:Backlight off"));
    lcd.noBacklight();
    backlightOn = false;
  }
}
