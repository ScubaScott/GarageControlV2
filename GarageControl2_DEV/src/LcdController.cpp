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

// ============================================================
//  LCD Controller Constructor & Init
// ============================================================

/**
 * @brief Constructor for LcdController.
 * @param lcdRef Reference to the LiquidCrystal_I2C instance.
 * @param menuRef Reference to the MenuController instance.
 */
LcdController::LcdController(LiquidCrystal_I2C &lcdRef, MenuController &menuRef)
  : lcd(lcdRef), menu(menuRef) {
}

/**
 * @brief Initializes the LCD display and creates custom characters.
 */
void LcdController::begin() {
  lcd.init();
  // create then custom Chars.
  byte UpArrow[] = { B00100, B01110, B10101, B00100, B00100, B00100, B00100, B00100 };
  byte DownArrow[] = { B00100, B00100, B00100, B00100, B00100, B10101, B01110, B00100 };
  byte DoubleArrow[] = { B00100, B01110, B10101, B00100, B00100, B10101, B01110, B00100 };
  byte Degree[] = { B01000, B10100, B01000, B00000, B00000, B00000, B00000, B00000 };

  lcd.createChar(0, UpArrow);
  lcd.createChar(1, DownArrow);
  lcd.createChar(2, DoubleArrow);
  lcd.createChar(3, Degree);

  clearDisplay();
  printLCDText(1, true, "Garage Control");
  printLCDText(1, true, "version 2.0");
}

// ============================================================
//  Private Helper Methods
// ============================================================

/**
 * @brief Clears the LCD display.
 */
void LcdController::clearDisplay() {
  lcd.clear();
}

/**
 * @brief Sets the cursor position on the LCD.
 * @param row Row number (1-based).
 * @param col Column number (0-based).
 */
void LcdController::setCursor(int row, int col) {
  // adjust for the zero based row index
  row = row - 1;
  lcd.setCursor(col, row);
}

/**
 * @brief Prints text to the LCD with optional centering.
 * @param row Row to print on (1-based).
 * @param center If true, centers the text on the line.
 * @param text Text to print.
 */
void LcdController::printLCDText(int row, bool center, const String &text) {
  // 4x20 display: 20 characters per line

  // adjust for the zero based row index
  row = row - 1;

  // center text
  if (center) {
    // Calculate needed initial spaces
    int spaces = (20 - text.length()) / 2;
    if (spaces < 0)
      spaces = 0;
    Serial.print("LCD:row:" + String(row) + ":");
    Serial.println(String(text));

    // print spaces to center text
    setCursor(row, 0);
    for (int i = 0; i < spaces; i++)
      lcd.print(" ");
  }

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
String LcdController::getDoorStateString(const GarageDoor &door) {
  switch (door.getState()) {
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
String LcdController::getHvacStateString(const GarageHVAC &hvac) {
  switch (hvac.state) {
    case GarageHVAC::Heating:
      return "Heat";
    case GarageHVAC::Cooling:
      return "Cool";
    case GarageHVAC::Pending:
      return "Pend";
    case GarageHVAC::Waiting:
    default:
      return "Wait";
  }
}

// ============================================================
//  Main Display Update
// ============================================================

/**
 * @brief Marks the display as needing an update.
 * @param WakeBackLight If true, turns on the backlight.
 */
void LcdController::SetDirty(bool WakeBackLight) {
  // mark entire screen dirty; next updateDisplay will redraw all four lines
  LcdController::IsDirty = true;
  if (WakeBackLight){
    Serial.println("LCD:waking backlight");
    LcdController::setBacklight(true);
  }
}

/**
 * @brief Updates the LCD display with current system status.
 * @param hvac Reference to the GarageHVAC instance.
 * @param door Reference to the GarageDoor instance.
 * @param lights Reference to the GarageLight instance.
 * @param tempF Current temperature in Fahrenheit.
 */
void LcdController::updateDisplay(GarageHVAC &hvac, GarageDoor &door, GarageLight &lights, float tempF) {
  
  if (!LcdController::IsDirty) {
    return;
  } else {
    Serial.println("LCD:updating LCD");
    LcdController::IsDirty = false;
  }

  clearDisplay();

  MenuController::Screen currentScreen = menu.get();
  bool EditMode = menu.EditMode;

  switch (currentScreen) {
    case MenuController::Screen::Main:
      {

        char buffer[21];  // 20 chars + 1 for null terminator

        // snprintf ensures we never write more than 21 bytes
        // It also automatically adds the '\0' at the end.

        // print line 1
        snprintf(buffer, sizeof(buffer), "F:%d\x03  H:%d/C:%d", tempF, hvac.heatSet, hvac.coolSet);
        printLCDText(1, true, buffer);

        // print line 2
        snprintf(buffer, sizeof(buffer), "Ht:%s / Dr:%s", getHvacStateString(hvac), getDoorStateString(door));
        printLCDText(1, true, buffer);

        // print line 3

        // snprintf(buffer, sizeof(buffer), "HVAC:%s", (hvac.lockout ? "LOCKOUT" : "Normal"));
        snprintf(buffer, sizeof(buffer), "HVAC:%s", (hvac.lockout ? "LOCKOUT" : "Normal"));
        printLCDText(1, true, buffer);

        /*
    setCursor(1, 0);
    lcd.print("T:");
    lcd.print(tempF, 0);
    lcd.write(3);
    lcd.print("F  H:");
    lcd.print((int)hvac.heatSet);
    lcd.print("/C:");
    lcd.print((int)hvac.coolSet);

    setCursor(2, 0);
    //String hvacStatus = getHvacStateString(hvac);
    //String doorStatus = getDoorStateString(door);
    lcd.print("HV:");
    lcd.print(hvacStatus);
    lcd.print(" DR:");
    lcd.print(doorStatus);

    setCursor(3, 0);
    if (hvac.lockout)
      lcd.print("HVAC: LOCKOUT");
    else
      lcd.print("HVAC: Normal");
      */
      }
      break;

    case MenuController::Screen::HVACMenu:
      printLCDText(1, true, "HVAC Settings");
      printLCDText(3, true, "Set Heat/Cool/Swing");
      printLCDText(4, false, "\x01");
      break;

    case MenuController::Screen::SetHeat:
      {
        printLCDText(1, true, "Heat Setpoint");
        if (EditMode) {
          lcd.blink_on();
        } else {
          lcd.blink_off();
          setCursor(4, 0);
          lcd.write(1);
        }
        setCursor(3, 0);
        for (int i = 0; i < 20; i++)
          lcd.print(" ");
        printLCDText(3, true, String((int)hvac.heatSet));
        // lcd.write(3);
      }
      break;

    case MenuController::Screen::SetCool:
      {
        printLCDText(1, true, "Cool Setpoint");
        if (EditMode) {
          lcd.blink_on();
        } else {
          lcd.blink_off();
          setCursor(4, 0);
          lcd.write(2);
        }
        setCursor(3, 0);
        for (int i = 0; i < 20; i++)
          lcd.print(" ");
        printLCDText(3, true, String(hvac.coolSet, 1));
        lcd.write(3);
      }
      break;

    case MenuController::Screen::SetSwing:
      {
        printLCDText(1, true, "HVAC Swing");
        if (EditMode) {
          lcd.blink_on();
        } else {
          lcd.blink_off();
          setCursor(4, 0);
          lcd.write(2);
        }
        setCursor(3, 0);
        for (int i = 0; i < 20; i++)
          lcd.print(" ");
        printLCDText(3, true, String(hvac.HVACSwing) + "\x03");
      }
      break;
    case MenuController::Screen::HVACBack:
      printLCDText(1, true, "Back...");
      setCursor(4, 0);
      lcd.write(0);

      break;
    case MenuController::Screen::LightMenu:
      printLCDText(1, true, "Light Settings");
      printLCDText(3, true, "Set Timeout");
      break;

    case MenuController::Screen::SetLightTimeout:
      {
        printLCDText(1, true, "Light Timeout");
        EditMode ? lcd.blink_on() : lcd.blink_off();
        setCursor(3, 0);
        for (int i = 0; i < 20; i++)
          lcd.print(" ");
        unsigned long mins = lights.duration / 60000UL;
        printLCDText(3, true, String(mins) + " minutes");
      }
      break;
    case MenuController::Screen::LightBack:

      printLCDText(1, true, "Back...");
      break;
    case MenuController::Screen::DoorMenu:
      printLCDText(1, true, "Door Settings");
      printLCDText(3, true, "Timeout/Attempts");
      break;

    case MenuController::Screen::SetDoorTimeout:
      {
        printLCDText(1, true, "Door Timeout");
        EditMode ? lcd.blink_on() : lcd.blink_off();
        setCursor(3, 0);
        for (int i = 0; i < 20; i++)
          lcd.print(" ");
        printLCDText(3, true, String(door.getAutoClose() / 60000UL) + " min");
      }
      break;

    case MenuController::Screen::SetDoorAttempts:
      {
        printLCDText(1, true, "Close Attempts");
        EditMode ? lcd.blink_on() : lcd.blink_off();
        setCursor(3, 0);
        for (int i = 0; i < 20; i++)
          lcd.print(" ");
        printLCDText(3, true, String(door.getMaxAttempts()));
      }
      break;
    case MenuController::Screen::DoorBack:

      printLCDText(1, true, "Back...");
      break;
    case MenuController::Screen::MenuExit:

      printLCDText(1, true, "Menu Exit...");
      break;
    default:
      printLCDText(1, true, "Garage Control");
      printLCDText(3, true, "Unknown Menu");
      break;
  }
}

// ============================================================
//  Backlight helper
// ============================================================

/**
 * @brief Controls the LCD backlight.
 * @param on True to turn on backlight, false to turn off.
 */
void LcdController::setBacklight(bool on) {
  Serial.println("LCD:updating backlight:"+String(on));
  if (on && !backlightOn) {
    Serial.println("LCD: backlight on");
    lcd.backlight();
    backlightOn = true;
  } 
  if (!on) {
    Serial.println("LCD: backlight off");
    lcd.noBacklight();
    backlightOn = false;
  }
}
