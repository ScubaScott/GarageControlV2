/**
 * @file MenuController.h
 * @brief Menu navigation controller with debounced button input.
 *
 * This file defines the MenuController class which manages the multi-level LCD menu system,
 * handles debounced button input from the three navigation buttons (up/down/set),
 * provides timeout-based auto-return to status display, and coordinates with
 * the LCD display and controllable subsystems (door, lights, HVAC).
 *
 * The menu system is hierarchical with a main status screen, configuration submenus
 * for each subsystem, and editing modes for numeric values (timeouts, temperatures).
 */

#ifndef MENU_CONTROLLER_H
#define MENU_CONTROLLER_H

#include <Arduino.h>

class IMenuHost
{
public:
  virtual void reloadNV() = 0;
  virtual ~IMenuHost() = default;
};

class GarageHVAC;
class GarageLight;
class GarageDoor;

/**
 * @class MenuController
 * @brief Manages hierarchical LCD menu navigation and button input processing.
 *
 * This class provides:
 * - **Screen hierarchy** organizing status display and subsystem configuration
 * - **Button debouncing** with independent state tracking for each of 3 buttons
 * - **Menu timeout** (20 seconds) to auto-return to main status display
 * - **Edit mode** for numeric values (temperatures, timeouts, retry counts)
 * - **Activity tracking** to extend timeout when buttons are pressed or MQTT commands arrive
 *
 * Navigation Flow:
 * - **Main** - Status screen showing current system state
 * - Press **Set** → Enter configuration menu hierarchy
 * - **HVAC/Light/Door/Config** - Submenu categories
 * - **Set*** screens - Edit individual parameters
 * - **Back** options - Return to parent menus
 * - 20-second inactivity → Auto-return to Main
 *
 * Button assignments (INPUT_PULLUP, so unpressed=HIGH):
 * - **Up**: Navigate menus up, decrease numeric values
 * - **Down**: Navigate menus down, increase numeric values
 * - **Set**: Enter submenu/edit mode, confirm changes
 *
 * @note EditMode flag indicates when a numeric parameter is being edited.
 *       The LCD controller uses this to highlight the editable field.
 */
class MenuController
{
public:
  /**
   * @enum Screen
   * @brief Enumeration of all available menu screens in the hierarchy.
   *
   * The menu hierarchy is organized by subsystem function:
   * - **Main**: Status summary screen (primary display)
   * - **HVAC section**: Heating/cooling configuration
   * - **Light section**: Lighting timeout configuration
   * - **Door section**: Garage door auto-close configuration
   * - **Config section**: Network status and diagnostics
   * - **Navigation**: Back buttons and exit paths
   *
   * Each configuration section follows the pattern:
   * - [Subsystem]Menu → lists configurable options
   * - Set[Parameter] → edit individual parameters
   * - [Subsystem]Back → return to Main
   */
  enum class Screen
  {
    // ── Main Status Screen ──────────────────────────────────
    Main,               ///< Main status screen (door/light/HVAC/motion/temperature)
    
    // ── HVAC Configuration Submenu ──────────────────────────
    HVACMenu,           ///< HVAC menu: select what to configure (mode, setpoints, etc)
    SetHeat,            ///< Edit heat setpoint temperature (°F)
    SetCool,            ///< Edit cool setpoint temperature (°F)
    SetSwing,           ///< Edit temperature hysteresis/swing (°F)
    SetMinRunTime,      ///< Edit HVAC minimum runtime (minutes)
    SetMinRestTime,     ///< Edit HVAC minimum rest time (minutes)
    SetMode,            ///< Select HVAC mode (Off/Heat/Heat_Cool/Cool)
    ReloadNV,           ///< Reload settings from NV storage
    HVACBack,           ///< Return from HVAC menu to Main
    
    // ── Light Configuration Submenu ─────────────────────────
    LightMenu,          ///< Light menu: select what to configure (timeout)
    SetLightTimeout,    ///< Edit auto-off timeout duration (minutes)
    LightBack,          ///< Return from Light menu to Main
    
    // ── Door Configuration Submenu ──────────────────────────
    DoorMenu,           ///< Door menu: select what to configure (timeout, retries)
    SetDoorTimeout,     ///< Edit auto-close timeout duration (minutes)
    SetDoorAttempts,    ///< Edit max retry attempts for auto-close
    DoorBack,           ///< Return from Door menu to Main
    
    // ── Network/Config Submenu ─────────────────────────────
    ConfigMenu,         ///< Config menu: view status or trigger actions
    NetworkInfo,        ///< Display network status (WiFi/MQTT connection state)
    NetworkReset,       ///< Trigger network reconnection attempt
    ConfigBack,         ///< Return from Config menu to Main
    
    // ── Menu Control ────────────────────────────────────────
    MenuExit,           ///< Exit menu system and return to Main
    Count               ///< Internal: total number of screens (for bounds checking)
  };

  /**
   * @brief Edit mode flag for numeric parameter editing.
   *
   * Set to true in the LCD controller when displaying an editable parameter.
   * Used to highlight the field being edited on the LCD display.
   */
  bool EditMode = false;

private:
  // Menu navigation state
  Screen current = Screen::Main;

  // Timeout and debouncing configuration
  unsigned long lastActivity = 0;           // Timestamp of last user activity
  const unsigned long MENU_TIMEOUT = 20000UL; // Return to Main after 20 seconds of inactivity
  const unsigned long DEBOUNCE_MS  = 50UL;  // Button debounce threshold (milliseconds)

  // Hardware button pin assignments
  byte pinUp, pinDown, pinSet;

  // Per-button debounce state tracking (accessed by btnIndex)
  enum { BTN_UP = 0, BTN_DOWN = 1, BTN_SET = 2, BTN_COUNT = 3 };
  unsigned long lastPressTime[BTN_COUNT] = {0, 0, 0}; // Debounce timer per button
  bool          lastPinState[BTN_COUNT]  = {true, true, true}; // HIGH = unpressed (INPUT_PULLUP)

  // Internal button and menu handlers (called by poll())
  bool pressed(byte btnIndex);
  void handleSet(IMenuHost &controller);
  void handleUp(GarageHVAC &hvac, GarageLight &lights, GarageDoor &door);
  void handleDown(GarageHVAC &hvac, GarageLight &lights, GarageDoor &door);

public:
  /**
   * @brief Constructor for MenuController.
   *
   * Initializes button pin assignments. Pins must be configured as INPUT_PULLUP
   * externally before calling begin().
   *
   * @param up   GPIO pin number for "up" navigation button (active LOW, INPUT_PULLUP)
   * @param down GPIO pin number for "down" navigation button (active LOW, INPUT_PULLUP)
   * @param set  GPIO pin number for "set/select" button (active LOW, INPUT_PULLUP)
   */
  MenuController(byte up, byte down, byte set);

  /**
   * @brief Initializes the menu controller.
   *
   * Must be called once during setup after the MenuController is constructed.
   * Configures button pins as INPUT_PULLUP.
   */
  void begin();

  /**
   * @brief Gets the current menu screen.
   *
   * Used by the LCD display controller to determine what content to render.
   *
   * @return Current Screen enum value
   *
   * @see Screen enum for screen hierarchy and organization
   */
  Screen get() const;

  /**
   * @brief Polls button inputs and updates menu state.
   *
   * Should be called once per main loop iteration (typically 100-200ms).
   * Debounces button presses, detects transitions, calls appropriate handlers,
   * manages menu screen navigation, and auto-timeout to Main screen.
   *
   * @param controller Reference to GarageController for reloadNV functionality
   * @param hvac   Reference to GarageHVAC instance for reading/updating HVAC settings
   * @param lights Reference to GarageLight instance for reading/updating light timeout
   * @param door   Reference to GarageDoor instance for reading/updating door settings
   * @return True if any button press or screen change occurred, false if no activity
   *         (useful for avoiding unnecessary LCD updates)
   *
   * @note This method modifies HVAC mode, setpoints, light timeout, and door
   *       timeout/retry values directly. Called subsystems should not modify
   *       these values externally during menu operation.
   */
  bool poll(IMenuHost &controller, GarageHVAC &hvac, GarageLight &lights, GarageDoor &door);

  /**
   * @brief Resets the menu timeout due to external activity.
   *
   * Called by external systems when significant activity occurs (e.g., MQTT
   * commands received, door button pressed manually). This extends the time
   * before auto-returning to Main display, keeping the menu active during
   * active system use.
   *
   * @note The menu timeout is automatically extended by button presses.
   *       This method is for non-button activity notifications.
   */
  void noteActivity();
};

#endif
