/**
 * @file LcdController.cpp
 * @brief LCD display controller implementation for garage control system.
 *
 * Implements LcdController class methods for managing a 4x20 I2C LCD display:
 * initialization, splash screen, custom characters, status rendering, menu
 * screen rendering, backlight control, and dirty-flag-gated updates.
 *
 * @section SetNVRendering SetNV screen rendering (v2.17.0)
 * Eight new cases were added to the updateDisplay() switch block to render
 * the SetNV Values sub-menu:
 *
 *   SetNVMenu        – entry / overview screen
 *   SetNVHeatSet     – NV heat setpoint (°F)
 *   SetNVCoolSet     – NV cool setpoint (°F)
 *   SetNVSwing       – NV HVAC hysteresis swing (°F)
 *   SetNVMinRunTime  – NV minimum HVAC run time (minutes)
 *   SetNVMinRestTime – NV minimum HVAC rest time (minutes)
 *   SetNVDoorTimeout – NV door auto-close timeout (minutes)
 *   SetNVLightTimeout– NV light auto-off timeout (minutes)
 *   SetNVBack        – back navigation screen
 *
 * Each edit screen reads the current NV value via host->getNv*() and
 * enables the LCD cursor blink when EditMode is active.  If host is null
 * (should not occur in normal operation) the value field shows "---".
 *
 * @version 2.17.0
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
//  Constructor & initialization
// ============================================================

/**
 * @brief Constructor — stores references to LCD and menu controller.
 */
LcdController::LcdController(LiquidCrystal_I2C &lcdRef, MenuController &menuRef)
    : lcd(lcdRef), menu(menuRef), host(nullptr) {}

/**
 * @brief Initializes LCD hardware, loads custom characters, shows splash.
 *
 * Custom characters (stored in LCD CGRAM slots 0–3):
 *   0 – Up arrow    (used on "first" screens where only DOWN is available)
 *   1 – Down arrow  (used on "last" screens where only UP is available)
 *   2 – Double arrow (used on screens where both UP and DOWN navigate)
 *   3 – Degree symbol (used in temperature / setpoint displays)
 */
void LcdController::begin()
{
  lcd.init();

  // Define custom bitmaps for navigation arrows and degree symbol
  byte UpArrow[]    = {B00100, B01110, B10101, B00100, B00100, B00100, B00100, B00100};
  byte DownArrow[]  = {B00100, B00100, B00100, B00100, B00100, B10101, B01110, B00100};
  byte DoubleArrow[]= {B00100, B01110, B10101, B00100, B00100, B10101, B01110, B00100};
  byte Degree[]     = {B01000, B10100, B01000, B00000, B00000, B00000, B00000, B00000};

  lcd.createChar(0, UpArrow);
  lcd.createChar(1, DownArrow);
  lcd.createChar(2, DoubleArrow);
  lcd.createChar(3, Degree);

  // Show splash screen with firmware version
  clearDisplay();
  printLCDText(1, true, "Garage Control");
  char buf[21];
  snprintf(buf, sizeof(buf), "Version: %s", GC_VERSION);
  printLCDText(2, true, buf);
}

// ============================================================
//  Private helper methods
// ============================================================

/** @brief Clears all characters from the LCD. */
void LcdController::clearDisplay()
{
  lcd.clear();
}

/**
 * @brief Moves the LCD cursor to a given position.
 * @param row 1-based row (1–4).
 * @param col 0-based column (0–19).
 */
void LcdController::setCursor(int row, int col)
{
  lcd.setCursor(col, row - 1);
}

/**
 * @brief Prints text to the LCD, optionally centered.
 * @param row    1-based row number.
 * @param center Center the text in the 20-character row if true.
 * @param text   Null-terminated string (const char* — no heap allocation).
 */
void LcdController::printLCDText(int row, bool center, const char *text)
{
  int len = strlen(text);
  int col = 0;
  if (center)
  {
    col = (20 - len) / 2;
    if (col < 0) col = 0;
  }
  lcd.setCursor(col, row - 1);
  lcd.print(text);
}

// ============================================================
//  State string helpers  (return flash-resident constants)
// ============================================================

/**
 * @brief Returns a short display string for the current door state.
 * @param door GarageDoor reference.
 * @return Pointer to a constant string literal.
 */
const char *LcdController::getDoorStateString(const GarageDoor &door)
{
  switch (door.getState())
  {
  case GarageDoor::Open:     return "Open";
  case GarageDoor::Closed:   return "Closed";
  case GarageDoor::Moving:   return "Moving";
  case GarageDoor::Error:    return "Error";
  case GarageDoor::Disabled: return "Dsbld";
  }
  return "Unknown";
}

/**
 * @brief Returns a short display string for the current HVAC state.
 * @param hvac GarageHVAC reference.
 * @return Pointer to a constant string literal.
 */
const char *LcdController::getHvacStateString(const GarageHVAC &hvac)
{
  switch (hvac.state)
  {
  case GarageHVAC::Heating: return "Heat";
  case GarageHVAC::Cooling: return "Cool";
  case GarageHVAC::Pending: return "Pend";
  default:                  return "Wait";
  }
}

// ============================================================
//  Dirty flag / backlight helpers
// ============================================================

/**
 * @brief Marks the display as needing a full redraw.
 * @param WakeBackLight If true, also turns on the backlight.
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
 * @brief Controls the LCD backlight.
 * @param on True to turn on; false to turn off.
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

// ============================================================
//  updateDisplay  –  main rendering dispatcher
// ============================================================

/**
 * @brief Redraws the LCD if the dirty flag is set; otherwise returns early.
 *
 * Dispatches to the appropriate rendering block based on the current
 * MenuController screen.  All string formatting uses a single 21-byte
 * stack buffer (buf) to avoid heap allocation.
 *
 * SetNV screens query NV values via the @c host pointer (IMenuHost).  If
 * host is null a safe fallback "---" is shown; this should not occur in
 * normal operation as setHost() is called in GarageController::begin().
 *
 * @param hvac   GarageHVAC subsystem (live values and state).
 * @param door   GarageDoor subsystem (state, timeout).
 * @param lights GarageLight subsystem (on/off, duration).
 * @param tempF  Current temperature in Fahrenheit.
 */
void LcdController::updateDisplay(GarageHVAC &hvac, GarageDoor &door,
                                  GarageLight &lights, float tempF)
{
  if (!IsDirty) return;

  Serial.println(F("LCD:Update"));
  IsDirty = false;

  clearDisplay();

  MenuController::Screen currentScreen = menu.get();
  bool EditMode = menu.EditMode;

  // Single reused stack buffer: 21 bytes = 20 LCD columns + null terminator.
  char buf[21];

  switch (currentScreen)
  {
  // ══════════════════════════════════════════════════════════════════════════
  //  Main status screen
  // ══════════════════════════════════════════════════════════════════════════
  case MenuController::Screen::Main:
  {
    // Row 1: temperature + heat/cool setpoints
    snprintf(buf, sizeof(buf), "%.1f\x03  H:%d/C:%d",
             tempF, (int)hvac.heatSet, (int)hvac.coolSet);
    printLCDText(1, true, buf);

    // Row 2: HVAC action + door position
    snprintf(buf, sizeof(buf), "HV:%s / Dr:%s",
             getHvacStateString(hvac), getDoorStateString(door));
    printLCDText(2, true, buf);

    // Row 3: HVAC mode
    const char *modeStr;
    switch (hvac.mode)
    {
    case GarageHVAC::Off:       modeStr = "Off";  break;
    case GarageHVAC::Heat:      modeStr = "Heat"; break;
    case GarageHVAC::Heat_Cool: modeStr = "H+C";  break;
    case GarageHVAC::Cool:      modeStr = "Cool"; break;
    default:                    modeStr = "???";  break;
    }
    snprintf(buf, sizeof(buf), "Mode:%s", modeStr);
    printLCDText(3, true, buf);

    // Row 4: network status
#if ENABLE_WIFI
    const char *statusStr = g_mqttManager ? g_mqttManager->getNetStatusString() : "Disabled";
    snprintf(buf, sizeof(buf), "Network:%s", statusStr);
    printLCDText(4, true, buf);
#endif
    break;
  }

  // ══════════════════════════════════════════════════════════════════════════
  //  HVAC sub-menu screens
  // ══════════════════════════════════════════════════════════════════════════
  case MenuController::Screen::HVACMenu:
    printLCDText(1, true, "HVAC Settings");
    printLCDText(3, true, "Ht/Cl/Swing/Time");
    printLCDText(4, false, "\x01"); // down arrow — more options below
    break;

  case MenuController::Screen::SetHeat:
    printLCDText(1, true, "Heat Setpoint");
    EditMode ? lcd.blink_on() : lcd.blink_off();
    snprintf(buf, sizeof(buf), "%d\x03", (int)hvac.heatSet);
    printLCDText(3, true, buf);
    printLCDText(4, false, "\x01");
    break;

  case MenuController::Screen::SetCool:
    printLCDText(1, true, "Cool Setpoint");
    EditMode ? lcd.blink_on() : lcd.blink_off();
    snprintf(buf, sizeof(buf), "%d\x03", (int)hvac.coolSet);
    printLCDText(3, true, buf);
    printLCDText(4, false, "\x02");
    break;

  case MenuController::Screen::SetSwing:
    printLCDText(1, true, "HVAC Swing");
    EditMode ? lcd.blink_on() : lcd.blink_off();
    snprintf(buf, sizeof(buf), "%d\x03", hvac.HVACSwing);
    printLCDText(3, true, buf);
    printLCDText(4, false, "\x02");
    break;

  case MenuController::Screen::SetMinRunTime:
    printLCDText(1, true, "HVAC Min Run");
    EditMode ? lcd.blink_on() : lcd.blink_off();
    snprintf(buf, sizeof(buf), "%u minutes", hvac.minRunTimeMins);
    printLCDText(3, true, buf);
    printLCDText(4, false, "\x02");
    break;

  case MenuController::Screen::SetMinRestTime:
    printLCDText(1, true, "HVAC Min Rest");
    EditMode ? lcd.blink_on() : lcd.blink_off();
    snprintf(buf, sizeof(buf), "%u minutes", hvac.minRestTimeMins);
    printLCDText(3, true, buf);
    printLCDText(4, false, "\x02");
    break;

  case MenuController::Screen::SetMode:
  {
    printLCDText(1, true, "HVAC Mode");
    EditMode ? lcd.blink_on() : lcd.blink_off();
    const char *modeStr2;
    switch (hvac.mode)
    {
    case GarageHVAC::Off:       modeStr2 = "Off";  break;
    case GarageHVAC::Heat:      modeStr2 = "Heat"; break;
    case GarageHVAC::Heat_Cool: modeStr2 = "H+C";  break;
    case GarageHVAC::Cool:      modeStr2 = "Cool"; break;
    default:                    modeStr2 = "???";  break;
    }
    printLCDText(3, true, modeStr2);
    printLCDText(4, false, "\x02");
    break;
  }

  case MenuController::Screen::LoadNV:
    printLCDText(1, true, "Load from NV");
    printLCDText(3, true, "Press SET to Load");
    printLCDText(4, false, "\x02");
    break;

  case MenuController::Screen::SaveNV:
    printLCDText(1, true, "Save to NV");
    printLCDText(3, true, "Press SET to Save");
    printLCDText(4, false, "\x02");
    break;

  case MenuController::Screen::HVACBack:
    printLCDText(1, true, "Back...");
    printLCDText(4, false, "\x00"); // up arrow — navigate back up
    break;

  // ══════════════════════════════════════════════════════════════════════════
  //  Light sub-menu screens
  // ══════════════════════════════════════════════════════════════════════════
  case MenuController::Screen::LightMenu:
    printLCDText(1, true, "Light Settings");
    printLCDText(3, true, "Set Timeout");
    printLCDText(4, false, "\x02");
    break;

  case MenuController::Screen::SetLightTimeout:
    printLCDText(1, true, "Light Timeout");
    EditMode ? lcd.blink_on() : lcd.blink_off();
    snprintf(buf, sizeof(buf), "%lu minutes", lights.duration / 60000UL);
    printLCDText(3, true, buf);
    printLCDText(4, false, "\x01");
    break;

  case MenuController::Screen::LightBack:
    printLCDText(1, true, "Back...");
    printLCDText(4, false, "\x00");
    break;

  // ══════════════════════════════════════════════════════════════════════════
  //  Door sub-menu screens
  // ══════════════════════════════════════════════════════════════════════════
  case MenuController::Screen::DoorMenu:
    printLCDText(1, true, "Door Settings");
    printLCDText(3, true, "Timeout/Attempts");
    printLCDText(4, false, "\x02");
    break;

  case MenuController::Screen::SetDoorTimeout:
    printLCDText(1, true, "Door Timeout");
    EditMode ? lcd.blink_on() : lcd.blink_off();
    snprintf(buf, sizeof(buf), "%lu min", door.getAutoClose() / 60000UL);
    printLCDText(3, true, buf);
    printLCDText(4, false, "\x01");
    break;

  case MenuController::Screen::SetDoorAttempts:
    printLCDText(1, true, "Close Attempts");
    EditMode ? lcd.blink_on() : lcd.blink_off();
    snprintf(buf, sizeof(buf), "%d", door.getMaxAttempts());
    printLCDText(3, true, buf);
    printLCDText(4, false, "\x02");
    break;

  case MenuController::Screen::DoorBack:
    printLCDText(1, true, "Back...");
    printLCDText(4, false, "\x00");
    break;

  // ══════════════════════════════════════════════════════════════════════════
  //  Config sub-menu screens
  // ══════════════════════════════════════════════════════════════════════════
  case MenuController::Screen::ConfigMenu:
    printLCDText(1, true, "Config Settings");
    snprintf(buf, sizeof(buf), "Version: %s", GC_VERSION);
    printLCDText(2, true, buf);
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
      strncpy(ipStr,   "n/a", sizeof(ipStr));   ipStr[sizeof(ipStr)-1]   = '\0';
      strncpy(mqttStr, "n/a", sizeof(mqttStr)); mqttStr[sizeof(mqttStr)-1]= '\0';
    }
#else
    strncpy(ipStr,   "n/a", sizeof(ipStr));   ipStr[sizeof(ipStr)-1]   = '\0';
    strncpy(mqttStr, "n/a", sizeof(mqttStr)); mqttStr[sizeof(mqttStr)-1]= '\0';
    statusStr = "Disabled";
#endif

    printLCDText(1, true, "Network Info");
    snprintf(buf, sizeof(buf), "IP: %s", ipStr);
    printLCDText(2, true, buf);
    snprintf(buf, sizeof(buf), "MQTT: %s", mqttStr);
    printLCDText(3, true, buf);
    EditMode ? lcd.blink_on() : lcd.blink_off();
    snprintf(buf, sizeof(buf), "\x01 Status: %s", statusStr);
    printLCDText(4, false, buf);
    break;
  }

  // ══════════════════════════════════════════════════════════════════════════
  //  SetNV Values sub-menu screens (v2.17.0)
  //
  //  Each edit screen:
  //    Row 1 – screen title
  //    Row 3 – current NV value (read via host->getNv*(); "---" if host null)
  //    Row 4 – navigation arrow indicator
  //  EditMode enables LCD cursor blink to indicate an editable field.
  //
  //  Changes made here update the in-RAM NV members only; EEPROM is not
  //  written until the user navigates to SaveNV or sends /nv/save/cmd.
  // ══════════════════════════════════════════════════════════════════════════

  case MenuController::Screen::SetNVMenu:
    // Overview / entry screen for the NV editing sub-menu.
    // Row 2 gives a hint of what can be edited here.
    printLCDText(1, true, "Set NV Values");
    printLCDText(2, true, "Ht/Cl/Sw/Run/Rest");
    printLCDText(3, true, "Dr/Lt Timeouts");
    printLCDText(4, false, "\x02"); // double arrow — UP goes back to NetworkInfo, DOWN goes to LoadNV
    break;

  case MenuController::Screen::SetNVHeatSet:
    // Edit NV heat setpoint independently of the live hvac.heatSet value.
    printLCDText(1, true, "NV Heat Setpoint");
    EditMode ? lcd.blink_on() : lcd.blink_off();
    if (host)
      snprintf(buf, sizeof(buf), "%d\x03", (int)host->getNvHeatSet());
    else
      strncpy(buf, "---", sizeof(buf));
    printLCDText(3, true, buf);
    printLCDText(4, false, "\x01"); // down arrow — first screen in sub-menu, no UP nav
    break;

  case MenuController::Screen::SetNVCoolSet:
    // Edit NV cool setpoint independently of the live hvac.coolSet value.
    printLCDText(1, true, "NV Cool Setpoint");
    EditMode ? lcd.blink_on() : lcd.blink_off();
    if (host)
      snprintf(buf, sizeof(buf), "%d\x03", (int)host->getNvCoolSet());
    else
      strncpy(buf, "---", sizeof(buf));
    printLCDText(3, true, buf);
    printLCDText(4, false, "\x02");
    break;

  case MenuController::Screen::SetNVSwing:
    // Edit NV HVAC hysteresis swing independently of the live hvac.HVACSwing.
    printLCDText(1, true, "NV HVAC Swing");
    EditMode ? lcd.blink_on() : lcd.blink_off();
    if (host)
      snprintf(buf, sizeof(buf), "%d\x03", host->getNvSwing());
    else
      strncpy(buf, "---", sizeof(buf));
    printLCDText(3, true, buf);
    printLCDText(4, false, "\x02");
    break;

  case MenuController::Screen::SetNVMinRunTime:
    // Edit NV minimum HVAC run time independently of the live value.
    printLCDText(1, true, "NV HVAC Min Run");
    EditMode ? lcd.blink_on() : lcd.blink_off();
    if (host)
      snprintf(buf, sizeof(buf), "%u minutes", host->getNvMinRunTime());
    else
      strncpy(buf, "---", sizeof(buf));
    printLCDText(3, true, buf);
    printLCDText(4, false, "\x02");
    break;

  case MenuController::Screen::SetNVMinRestTime:
    // Edit NV minimum HVAC rest time independently of the live value.
    printLCDText(1, true, "NV HVAC Min Rest");
    EditMode ? lcd.blink_on() : lcd.blink_off();
    if (host)
      snprintf(buf, sizeof(buf), "%u minutes", host->getNvMinRestTime());
    else
      strncpy(buf, "---", sizeof(buf));
    printLCDText(3, true, buf);
    printLCDText(4, false, "\x02");
    break;

  case MenuController::Screen::SetNVDoorTimeout:
    // Edit NV door auto-close timeout independently of door.autoCloseDuration.
    printLCDText(1, true, "NV Door Timeout");
    EditMode ? lcd.blink_on() : lcd.blink_off();
    if (host)
      snprintf(buf, sizeof(buf), "%lu min", host->getNvDoorTimeout() / 60000UL);
    else
      strncpy(buf, "---", sizeof(buf));
    printLCDText(3, true, buf);
    printLCDText(4, false, "\x02");
    break;

  case MenuController::Screen::SetNVLightTimeout:
    // Edit NV light auto-off timeout independently of lights.duration.
    printLCDText(1, true, "NV Light Timeout");
    EditMode ? lcd.blink_on() : lcd.blink_off();
    if (host)
      snprintf(buf, sizeof(buf), "%lu min", host->getNvLightTimeout() / 60000UL);
    else
      strncpy(buf, "---", sizeof(buf));
    printLCDText(3, true, buf);
    printLCDText(4, false, "\x02");
    break;

  case MenuController::Screen::SetNVBack:
    // Navigation screen at the bottom of the SetNV sub-menu.
    // UP returns to SetNVLightTimeout; SET returns to SetNVMenu.
    printLCDText(1, true, "NV Values Back");
    printLCDText(3, true, "SET->NV Menu");
    printLCDText(4, false, "\x00"); // up arrow — navigate back up through sub-menu
    break;

  // ══════════════════════════════════════════════════════════════════════════
  //  Menu-level navigation screens
  // ══════════════════════════════════════════════════════════════════════════
  case MenuController::Screen::ConfigBack:
    printLCDText(1, true, "Back...");
    printLCDText(4, false, "\x00");
    break;

  case MenuController::Screen::MenuExit:
    printLCDText(1, true, "Menu Exit...");
    printLCDText(4, false, "\x00");
    break;

  // ══════════════════════════════════════════════════════════════════════════
  //  Fallback
  // ══════════════════════════════════════════════════════════════════════════
  default:
    printLCDText(1, true, "Garage Control");
    printLCDText(3, true, "Unknown Menu");
    printLCDText(4, false, "?");
    break;
  }
}
