/**
 * @file Config.h
 * @brief Configuration file for Garage Control System.
 *
 * This file contains sensitive configuration information including WiFi SSID,
 * passwords, MQTT server details, and device identity.
 *
 * @warning THIS FILE MUST NOT BE COMMITTED TO VERSION CONTROL
 *          It is listed in .gitignore to prevent accidental exposure of credentials.
 *          Copy Config_example.h and rename/customize it to Config.h for local use.
 */

#ifndef CONFIG_H
#define CONFIG_H

// ============================================================
//  WiFi Configuration
// ============================================================
/**
 * @brief WiFi network SSID (network name)
 */
#define CONFIG_WIFI_SSID "ScubaSpot"

/**
 * @brief WiFi network password
 */
#define CONFIG_WIFI_PASSWORD "ScubaNet"

// ============================================================
//  MQTT Broker Configuration
// ============================================================
/**
 * @brief MQTT broker server address (IP or hostname)
 */
#define CONFIG_MQTT_SERVER "192.168.0.130"

/**
 * @brief MQTT broker port (default: 1883)
 */
#define CONFIG_MQTT_PORT 1883

/**
 * @brief MQTT broker username (if authentication required)
 */
#define CONFIG_MQTT_USER "garagecoltroller"

/**
 * @brief MQTT broker password (if authentication required)
 */
#define CONFIG_MQTT_PASSWORD "ScubaGarage!"

// ============================================================
//  Device Identity Configuration
// ============================================================
/**
 * @brief Unique device identifier used in MQTT topic structure.
 * Must be unique among all devices publishing to the same MQTT broker.
 * Used in topics like: garage/[DEVICE_ID]/door/state
 */
#define CONFIG_DEVICE_ID "garage_ctrl_01"

/**
 * @brief Friendly device name displayed in Home Assistant discovery
 */
#define CONFIG_DEVICE_NAME "Garage Controller"

/**
 * @brief Home Assistant MQTT Discovery prefix.
 * Default is "homeassistant" - change only if your HA instance uses a custom prefix.
 */
#define CONFIG_DISCOVERY_PREFIX "homeassistant"

#endif // CONFIG_H
