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
  /**
   * @brief Constructs an LCD controller.
   * @param lcdRef Reference to an initialized LiquidCrystal_I2C instance.
   * @param menuRef Reference to the MenuController for navigation state.
   */
  LcdController(LiquidCrystal_I2C &lcdRef, MenuController &menuRef);

  /**
   * @brief Initializes the LCD and global display state.
   */
  void begin();

  /**
   * @brief Marks the display as dirty to force a refresh.
   * @param WakeBackLight If true, also wake the backlight on next update.
   */
  void SetDirty(bool WakeBackLight);

  /**
   * @brief Updates the LCD display with current system state.
   * @param hvac HVAC controller state.
   * @param door Garage door controller state.
   * @param lights Garage light controller state.
   * @param tempF Current temperature reading.
   */
  void updateDisplay(GarageHVAC &hvac, GarageDoor &door,
                     GarageLight &lights, float tempF);

  /**
   * @brief Enables or disables the LCD backlight.
   * @param on True to turn backlight on, false to turn it off.
   */
  void setBacklight(bool on);

  /**
   * @brief Gets the current backlight state.
   * @return True if backlight is on.
   */
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
