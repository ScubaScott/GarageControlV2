/**
 * @file LcdController.h
 * @brief Header file for LCD display controller class.
 *
 * This file defines the LcdController class which manages a 4x20 I2C LCD display
 * for the garage control system, handling menu navigation, status display,
 * and backlight control.
 */

#ifndef LCD_CONTROLLER_H
#define LCD_CONTROLLER_H

#include <Arduino.h>
#include <LiquidCrystal_I2C.h>

class MenuController;
class GarageHVAC;
class GarageDoor;
class GarageLight;

/**
 * @class LcdController
 * @brief Manages a 4x20 I2C LCD for the garage control system.
 *
 * Memory notes
 * ────────────
 * printLCDText() accepts const char* instead of const String& to avoid
 * heap allocation at every call site.  Callers use snprintf into a small
 * stack buffer (already present in updateDisplay) or pass string literals
 * directly.
 *
 * getHvacStateString / getDoorStateString return const char* (flash or
 * static data) instead of heap-allocated String objects.
 */
class LcdController
{
private:
  LiquidCrystal_I2C &lcd;
  MenuController    &menu;

  bool IsDirty    = false;
  bool backlightOn = false;

public:
  LcdController(LiquidCrystal_I2C &lcdRef, MenuController &menuRef);

  void begin();
  void SetDirty(bool WakeBackLight);
  void updateDisplay(GarageHVAC &hvac, GarageDoor &door,
                     GarageLight &lights, float tempF);
  void setBacklight(bool on);
  bool isBacklightOn() const { return backlightOn; }

private:
  void clearDisplay();
  void setCursor(int row, int col);

  // Accepts const char* - no heap String allocation
  void printLCDText(int row, bool center, const char *text);

  // Returns a flash/static string literal - no heap allocation
  const char* getHvacStateString(const GarageHVAC &hvac);
  const char* getDoorStateString(const GarageDoor &door);
};

#endif
