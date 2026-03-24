/**
 * @file LcdController.h
 * @brief LCD display manager for 4x20 I2C backlit display.
 *
 * This file defines the LcdController class which manages a 4-row by 20-column
 * I2C-connected liquid crystal display (LCD), rendering the main status screen
 * and hierarchical menu system. Includes backlight management with timeout-based
 * auto-dimming to reduce power consumption and extend display lifespan.
 *
 * Memory design focuses on minimizing heap allocations - all text handling uses
 * const char* pointers (flash or stack buffers) rather than heap-allocated String
 * objects, improving RAM utilization on memory-constrained Arduino platforms.
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
 * @brief Manages a 4x20 I2C LCD display for status and menu rendering.
 *
 * This class provides:
 * - **Status display** shows door state, temperature, motion, light state
 * - **Menu rendering** displays hierarchical configuration menus from MenuController
 * - **Edit mode** highlight for numeric parameters being adjusted
 * - **Backlight management** with configurable on/off state and timeout-based dimming
 * - **Dirty flag tracking** to avoid unnecessary screen updates during polling
 *
 * The LCD displays updates based on MenuController's current screen selection,
 * coordinating with HVAC, door, and light subsystems to reflect real-time state.
 *
 * Memory strategy:
 * - All text rendering uses const char* (from flash or stack buffers)
 * - No persistent String members (saves ~200 bytes heap vs string-based approach)
 * - State string functions return flash-stored constants, not heap allocations
 * - Reusable stack-based formatting buffer in updateDisplay()
 *
 * Hardware Requirements:
 * - 4x20 character LCD module
 * - PCF8574 I2C backpack (address 0x27) for pin control
 * - SDA/SCL connections to Arduino I2C pins
 */
class LcdController
{
private:
  // Hardware references
  LiquidCrystal_I2C &lcd;   /**< Reference to initialized LCD controller */
  MenuController    &menu;  /**< Reference to menu navigation state */

  // Display update tracking
  bool IsDirty    = false;  /**< Flag to force redraw on next updateDisplay() */
  bool backlightOn = false; /**< Current backlight state */

public:
  /**
   * @brief Constructor for LcdController.
   *
   * Stores references to LCD and menu controllers. Both references must be
   * valid throughout the LcdController's lifetime. The referenced objects
   * must be initialized (LCD via begin(), menu via MenuController::begin())
   * before calling LcdController methods.
   *
   * @param lcdRef   Reference to initialized LiquidCrystal_I2C instance
   * @param menuRef  Reference to initialized MenuController instance
   *
   * @see LiquidCrystal_I2C initialization in main sketch
   * @see MenuController::begin()
   */
  LcdController(LiquidCrystal_I2C &lcdRef, MenuController &menuRef);

  /**
   * @brief Initializes the LCD display.
   *
   * Must be called once during setup. Clears the display and performs any
   * initial LCD configuration. Typically called after LiquidCrystal_I2C
   * has been initialized with begin().
   */
  void begin();

  /**
   * @brief Marks the display as dirty to force a complete refresh.
   *
   * Used when the system state changes significantly and the display should
   * be redrawn completely on the next updateDisplay() call. Normal polling
   * updates are optimized to only refresh changed portions; this forces
   * full-screen redraw.
   *
   * @param WakeBackLight If true, also turns on the backlight and resets
   *                      the backlight timeout. Use when user activity occurs.
   *
   * @example
   *   // After door state change from external MQTT command:
   *   lcd.SetDirty(true);  // Force redraw and wake the backlight
   */
  void SetDirty(bool WakeBackLight);

  /**
   * @brief Updates the LCD display with current system state.
   *
   * Should be called once per main loop iteration. Renders the appropriate
   * screen (main status or menu screen) based on MenuController's current
   * selection. Main status shows door state, temperature, lights, and motion.
   * Menu screens show configuration options.
   *
   * This method implements display-update optimization:
   * - Skips expensive updates if IsDirty flag is not set
   * - Uses const char* for all text (no heap allocations)
   * - Reuses stack-based formatting buffer for temperature/time values
   *
   * @param hvac    Current HVAC controller state (for temperature, mode, state display)
   * @param door    Current door controller state (for door position/state display)
   * @param lights  Current light controller state (on/off, timeout remaining)
   * @param tempF   Current temperature reading in Fahrenheit (for display)
   *
   * @note Call this every main loop iteration for responsive display updates.
   *       The dirty flag prevents unnecessary LCD writes when nothing changed.
   */
  void updateDisplay(GarageHVAC &hvac, GarageDoor &door,
                     GarageLight &lights, float tempF);

  /**
   * @brief Controls the LCD backlight on/off state.
   *
   * Sets the backlight GPIO pin to the specified state. Turning off the
   * backlight reduces power consumption. Typically called by idle timeout
   * logic or low-power management code.
   *
   * @param on True to turn backlight on (GPIO HIGH), false to turn off (GPIO LOW)
   *
   * @see isBacklightOn() to query current state
   */
  void setBacklight(bool on);

  /**
   * @brief Gets the current backlight on/off state.
   *
   * @return True if backlight is currently on, false if off
   *
   * @see setBacklight()
   */
  bool isBacklightOn() const { return backlightOn; }

private:
  // Internal display rendering methods
  
  /**
   * @brief Clears the LCD screen buffer and resets cursor position.
   *
   * Called before rendering a new screen to ensure clean state.
   * Does not call updateDisplay() but prepares for subsequent writes.
   */
  void clearDisplay();

  /**
   * @brief Sets the cursor position for the next text output.
   *
   * @param row Row number (0-3 for a 4-row display)
   * @param col Column number (0-19 for a 20-column display)
   */
  void setCursor(int row, int col);

  /**
   * @brief Prints text to the LCD at the current cursor position.
   *
   * Uses const char* to avoid heap allocation on every call. All strings
   * are expected to be flash-resident or be small temporary buffers.
   *
   * @param row    Row number (0-3) - cursor is moved to this row
   * @param center If true, the text is centered in the 20-char row
   * @param text   Pointer to const flash string or stack buffer (no heap strings)
   *
   * @note Text longer than 20 characters will be truncated.
   *       Caller is responsible for string length validity.
   */
  void printLCDText(int row, bool center, const char *text);

  /**
   * @brief Gets the display string for current HVAC state.
   *
   * Returns a human-readable string representation of HVAC state
   * (Waiting, Heating, Cooling, Pending) without heap allocation.
   *
   * @param hvac Reference to GarageHVAC instance
   * @return Pointer to flash-resident string (do NOT free)
   *
   * @note Returns constant string pointers - safe for all applications
   */
  const char* getHvacStateString(const GarageHVAC &hvac);

  /**
   * @brief Gets the display string for current door position.
   *
   * Returns a human-readable string representation of door state
   * (Open, Closed, Moving, Error, Disabled) without heap allocation.
   *
   * @param door Reference to GarageDoor instance
   * @return Pointer to flash-resident string (do NOT free)
   *
   * @note Returns constant string pointers - safe for all applications
   */
  const char* getDoorStateString(const GarageDoor &door);
};

#endif
