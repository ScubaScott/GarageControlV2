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

// Forward declarations
class MenuController;
class GarageHVAC;
class GarageDoor;
class GarageLight;

/**
 * @class LcdController
 * @brief Manages LCD display operations for the garage control system.
 *
 * This class handles displaying system status, menu navigation, and backlight
 * control for a 4x20 I2C LCD display. It integrates with the menu system and
 * updates display content based on garage door, lighting, and HVAC states.
 */
class LcdController
{
private:
  LiquidCrystal_I2C &lcd; /**< Reference to the LiquidCrystal_I2C instance */
  MenuController &menu; /**< Reference to the MenuController instance */

  //unsigned long lastUpdate = 0;
  //const unsigned long UPDATE_INTERVAL = 500UL;
  bool IsDirty = false; /**< Flag indicating if display needs refresh */
  bool backlightOn = false; /**< Current backlight state */

public:
  /**
   * @brief Constructor for LcdController.
   * @param lcdRef Reference to the LiquidCrystal_I2C instance.
   * @param menuRef Reference to the MenuController instance.
   */
  LcdController(LiquidCrystal_I2C &lcdRef, MenuController &menuRef);

  /**
   * @brief Initializes the LCD display.
   */
  void begin();

  /**
   * @brief Marks the display as needing an update.
   * @param WakeBackLight If true, turns on the backlight.
   */
  void SetDirty(bool WakeBackLight);

  /**
   * @brief Updates the LCD display with current system status.
   * @param hvac Reference to the GarageHVAC instance.
   * @param door Reference to the GarageDoor instance.
   * @param lights Reference to the GarageLight instance.
   * @param tempF Current temperature in Fahrenheit.
   */
  void updateDisplay(GarageHVAC &hvac, GarageDoor &door, GarageLight &lights, float tempF);

  /**
   * @brief Controls the LCD backlight.
   * @param on True to turn on backlight, false to turn off.
   */
  void setBacklight(bool on);

  /**
   * @brief Gets the current backlight state.
   * @return True if backlight is on, false otherwise.
   */
  bool isBacklightOn() const { return backlightOn; }

private:
  /**
   * @brief Clears the LCD display.
   */
  void clearDisplay();

  /**
   * @brief Sets the cursor position on the LCD.
   * @param row Row number (0-based).
   * @param col Column number (0-based).
   */
  void setCursor(int row, int col);

  /**
   * @brief Prints text to the LCD with optional centering.
   * @param row Row to print on.
   * @param center If true, centers the text.
   * @param text Text to print.
   */
  void printLCDText(int row, bool center, const String &text);

  /**
   * @brief Gets the HVAC state as a display string.
   * @param hvac Reference to the GarageHVAC instance.
   * @return String representation of HVAC state.
   */
  String getHvacStateString(const GarageHVAC &hvac);

  /**
   * @brief Gets the door state as a display string.
   * @param door Reference to the GarageDoor instance.
   * @return String representation of door state.
   */
  String getDoorStateString(const GarageDoor &door);
};

#endif