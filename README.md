# GarageControl2

GarageControl2 is an Arduino-based garage automation system with:

- Door auto-close and retry logic
- Motion-aware lighting with timeout and cooldown
- HVAC temperature control with hysteresis and door-open lockout
- WiFi + MQTT integration for Home Assistant discovery and command control
- Non-volatile (NV) configuration storage with separate live and persisted values

## Current release

- Version: `2.20.0`
- Status: Working release, documentation reviewed and updated

## Project structure

- `GarageControl2/GarageControl2.ino` — main sketch and top-level controller
- `GarageControl2/src/` — subsystem source files and class implementations
- `GarageControl2/config/Config.h` — build-time WiFi/MQTT configuration
- `GarageControl2/ChangeLog.md` — release history and change notes

## Notes

- The project supports two build modes via `ENABLE_WIFI` in `GarageControl2/src/Utility.h`
  - `1` for full WiFi/MQTT/Home Assistant support
  - `0` for DEV builds without network code
- Version string is maintained in `GarageControl2/GarageControl2.ino`
- Release documentation was refreshed for `2.20.0` without advancing the version number
