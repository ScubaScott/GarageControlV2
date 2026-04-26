# Garage Control Menu Structure

## Menu Navigation Flowchart

```mermaid
graph TD
    Main["🏠 Main Screen<br/>Temp/Setpoints/HVAC/Door/Network<br/><br/>UP: Toggle Door<br/>DOWN: Toggle Light<br/>SET: Enter Menu"]
    
    Main -->|SET| HVACMenu["⚙️ HVAC Settings<br/>Ht/Cl/Swing/Time"]
    
    HVACMenu -->|SET| SetHeat["🌡️ Heat Setpoint<br/>Edit Mode: UP/DOWN adjust<br/>Normal Mode: DOWN next"]
    SetHeat -->|UP| SetHeat
    SetHeat -->|DOWN| SetCool
    SetHeat -->|SET| SetHeat
    
    SetCool["🌡️ Cool Setpoint<br/>Edit Mode: UP/DOWN adjust<br/>Normal Mode: UP/DOWN nav"]
    SetHeat -->|DOWN| SetCool
    SetCool -->|UP| SetHeat
    SetCool -->|DOWN| SetSwing
    SetCool -->|SET| SetCool
    
    SetSwing["🔄 HVAC Swing<br/>Edit Mode: UP/DOWN adjust<br/>Normal Mode: UP/DOWN nav"]
    SetCool -->|DOWN| SetSwing
    SetSwing -->|UP| SetCool
    SetSwing -->|DOWN| SetMinRunTime
    SetSwing -->|SET| SetSwing
    
    SetMinRunTime["⏱️ HVAC Min Run Time<br/>Edit Mode: UP/DOWN adjust<br/>Normal Mode: UP/DOWN nav"]
    SetSwing -->|DOWN| SetMinRunTime
    SetMinRunTime -->|UP| SetSwing
    SetMinRunTime -->|DOWN| SetMinRestTime
    SetMinRunTime -->|SET| SetMinRunTime
    
    SetMinRestTime["⏱️ HVAC Min Rest Time<br/>Edit Mode: UP/DOWN adjust<br/>Normal Mode: UP/DOWN nav"]
    SetMinRunTime -->|DOWN| SetMinRestTime
    SetMinRestTime -->|UP| SetMinRunTime
    SetMinRestTime -->|DOWN| SetMode
    SetMinRestTime -->|SET| SetMinRestTime
    
    SetMode["📋 HVAC Mode<br/>Off/Heat/H+C/Cool<br/>Edit Mode: UP/DOWN cycle<br/>Normal Mode: UP/DOWN nav"]
    SetMinRestTime -->|DOWN| SetMode
    SetMode -->|UP| SetMinRestTime
    SetMode -->|DOWN| HVACBack
    SetMode -->|SET| SetMode
    
    HVACBack["↩️ Back to HVAC Menu"]
    SetMode -->|DOWN| HVACBack
    HVACBack -->|UP| SetMode
    HVACBack -->|SET| HVACMenu
    
    HVACMenu -->|UP| HVACMenu
    HVACMenu -->|DOWN| LightMenu
    HVACMenu -->|SET| SetHeat
    
    LightMenu["💡 Light Settings<br/>Set Timeout"]
    HVACMenu -->|DOWN| LightMenu
    
    SetLightTimeout["⏰ Light Timeout<br/>Edit Mode: UP/DOWN adjust<br/>Normal Mode: DOWN next"]
    LightMenu -->|SET| SetLightTimeout
    SetLightTimeout -->|UP| SetLightTimeout
    SetLightTimeout -->|DOWN| LightBack
    SetLightTimeout -->|SET| SetLightTimeout
    
    LightBack["↩️ Back to Light Menu"]
    SetLightTimeout -->|DOWN| LightBack
    LightBack -->|UP| SetLightTimeout
    LightBack -->|SET| LightMenu
    
    LightMenu -->|DOWN| DoorMenu
    
    DoorMenu["🚪 Door Settings<br/>Timeout/Attempts"]
    
    SetDoorTimeout["⏰ Door Timeout<br/>Edit Mode: UP/DOWN adjust<br/>Normal Mode: DOWN next"]
    DoorMenu -->|SET| SetDoorTimeout
    SetDoorTimeout -->|UP| SetDoorTimeout
    SetDoorTimeout -->|DOWN| SetDoorAttempts
    SetDoorTimeout -->|SET| SetDoorTimeout
    
    SetDoorAttempts["🔢 Close Attempts<br/>Edit Mode: UP/DOWN adjust<br/>Normal Mode: UP/DOWN nav"]
    SetDoorTimeout -->|DOWN| SetDoorAttempts
    SetDoorAttempts -->|UP| SetDoorTimeout
    SetDoorAttempts -->|DOWN| DoorBack
    SetDoorAttempts -->|SET| SetDoorAttempts
    
    DoorBack["↩️ Back to Door Menu"]
    SetDoorAttempts -->|DOWN| DoorBack
    DoorBack -->|UP| SetDoorAttempts
    DoorBack -->|SET| DoorMenu
    
    DoorMenu -->|DOWN| ConfigMenu
    
    ConfigMenu["⚙️ Config Settings<br/>Version Info"]
    
    NetworkInfo["🌐 Network Info<br/>IP/MQTT/Status<br/>Edit: Reset/Disable"]
    ConfigMenu -->|SET| NetworkInfo
    NetworkInfo -->|SET| SetNVMenu
    NetworkInfo -->|DOWN| SetNVMenu
    
    SetNVMenu["📝 Set NV Values<br/>Non-Volatile EEPROM Settings<br/>Ht/Cl/Sw/Run/Rest/Dr/Lt"]
    
    SetNVHeatSet["🌡️ NV Heat Setpoint<br/>Edit Mode: UP/DOWN adjust"]
    SetNVMenu -->|SET| SetNVHeatSet
    SetNVHeatSet -->|DOWN| SetNVCoolSet
    SetNVHeatSet -->|SET| SetNVHeatSet
    
    SetNVCoolSet["🌡️ NV Cool Setpoint<br/>Edit Mode: UP/DOWN adjust"]
    SetNVHeatSet -->|DOWN| SetNVCoolSet
    SetNVCoolSet -->|UP| SetNVHeatSet
    SetNVCoolSet -->|DOWN| SetNVSwing
    SetNVCoolSet -->|SET| SetNVCoolSet
    
    SetNVSwing["🔄 NV HVAC Swing<br/>Edit Mode: UP/DOWN adjust"]
    SetNVCoolSet -->|DOWN| SetNVSwing
    SetNVSwing -->|UP| SetNVCoolSet
    SetNVSwing -->|DOWN| SetNVMinRunTime
    SetNVSwing -->|SET| SetNVSwing
    
    SetNVMinRunTime["⏱️ NV HVAC Min Run<br/>Edit Mode: UP/DOWN adjust"]
    SetNVSwing -->|DOWN| SetNVMinRunTime
    SetNVMinRunTime -->|UP| SetNVSwing
    SetNVMinRunTime -->|DOWN| SetNVMinRestTime
    SetNVMinRunTime -->|SET| SetNVMinRunTime
    
    SetNVMinRestTime["⏱️ NV HVAC Min Rest<br/>Edit Mode: UP/DOWN adjust"]
    SetNVMinRunTime -->|DOWN| SetNVMinRestTime
    SetNVMinRestTime -->|UP| SetNVMinRunTime
    SetNVMinRestTime -->|DOWN| SetNVDoorTimeout
    SetNVMinRestTime -->|SET| SetNVMinRestTime
    
    SetNVDoorTimeout["⏰ NV Door Timeout<br/>Edit Mode: UP/DOWN adjust"]
    SetNVMinRestTime -->|DOWN| SetNVDoorTimeout
    SetNVDoorTimeout -->|UP| SetNVMinRestTime
    SetNVDoorTimeout -->|DOWN| SetNVLightTimeout
    SetNVDoorTimeout -->|SET| SetNVDoorTimeout
    
    SetNVLightTimeout["⏰ NV Light Timeout<br/>Edit Mode: UP/DOWN adjust"]
    SetNVDoorTimeout -->|DOWN| SetNVLightTimeout
    SetNVLightTimeout -->|UP| SetNVDoorTimeout
    SetNVLightTimeout -->|DOWN| SetNVBack
    SetNVLightTimeout -->|SET| SetNVLightTimeout
    
    SetNVBack["↩️ NV Values Back<br/>SET returns to NV Menu"]
    SetNVLightTimeout -->|DOWN| SetNVBack
    SetNVBack -->|UP| SetNVLightTimeout
    SetNVBack -->|SET| SetNVMenu
    
    SetNVMenu -->|UP| NetworkInfo
    
    LoadNV["📥 Load from NV<br/>Restores EEPROM values"]
    SaveNV["💾 Save to NV<br/>Writes to EEPROM"]
    
    SetNVMenu -->|DOWN| LoadNV
    LoadNV -->|DOWN| SaveNV
    SaveNV -->|DOWN| ConfigBack
    
    ConfigBack["↩️ Back to Config Menu"]
    ConfigBack -->|UP| SaveNV
    ConfigBack -->|SET| ConfigMenu
    
    MenuExit["🚪 Menu Exit<br/>Return to Main"]
    ConfigMenu -->|DOWN| MenuExit
    MenuExit -->|UP| ConfigMenu
    MenuExit -->|SET| Main
    
    style Main fill:#90EE90
    style MenuExit fill:#FFB6C6
    style SetHeat fill:#87CEEB
    style SetCool fill:#87CEEB
    style SetSwing fill:#87CEEB
    style SetMinRunTime fill:#87CEEB
    style SetMinRestTime fill:#87CEEB
    style SetMode fill:#87CEEB
    style SetLightTimeout fill:#FFD700
    style SetDoorTimeout fill:#FFA500
    style SetDoorAttempts fill:#FFA500
    style SetNVHeatSet fill:#DDA0DD
    style SetNVCoolSet fill:#DDA0DD
    style SetNVSwing fill:#DDA0DD
    style SetNVMinRunTime fill:#DDA0DD
    style SetNVMinRestTime fill:#DDA0DD
    style SetNVDoorTimeout fill:#DDA0DD
    style SetNVLightTimeout fill:#DDA0DD
```

## Key Features

### Button Behavior

- **UP Button**: Navigate up or adjust values in edit mode
- **DOWN Button**: Navigate down or adjust values in edit mode  
- **SET Button**: Enter edit mode, confirm values, or navigate into submenus

### Main Menu Hierarchy

1. **HVAC Settings** - Temperature and system control
2. **Light Settings** - Garage light auto-off timeout
3. **Door Settings** - Door auto-close timeout and attempt limits
4. **Config Settings** - Network info and NV value editing
5. **Menu Exit** - Return to main status screen

### Edit Mode Behavior

When `EditMode` is enabled (toggled by SET button):
- **UP/DOWN** buttons adjust the displayed value
- Values increment/decrement by appropriate steps (1°F for temps, 1 min for timeouts, etc.)

When `EditMode` is disabled:
- **UP/DOWN** buttons navigate between menu items
- The current value is displayed but cannot be changed

### Non-Volatile (NV) Settings

The SetNV submenu allows editing EEPROM-backed settings independently from live values:
- Changes only affect in-RAM NV members
- Changes persist to EEPROM only when **SaveNV** is executed
- **LoadNV** restores all NV values from EEPROM

### Auto-Timeout

The menu automatically returns to the **Main** screen after `MENU_TIMEOUT` milliseconds of inactivity.

