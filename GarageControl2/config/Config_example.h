/**
 * @file Config_example.h
 * @brief Example configuration template for Garage Control System.
 *
 * This file demonstrates the required configuration parameters for WiFi and MQTT connectivity.
 * 
 * To use this project:
 * 1. Copy this file to Config.h in the same directory
 * 2. Update the values with your own WiFi SSID, password, MQTT server address, and credentials
 * 3. Adjust device identity as needed
 * 
 * @warning Config.h is in .gitignore and should NEVER be committed to version control.
 *          It contains sensitive information like passwords and IP addresses.
 */

#ifndef CONFIG_H
#define CONFIG_H

// ============================================================
//  WiFi Configuration
// ============================================================
/**
 * @brief WiFi network SSID (network name)
 */
#define CONFIG_WIFI_SSID "YourSSIDHere"

/**
 * @brief WiFi network password
 */
#define CONFIG_WIFI_PASSWORD "YourPasswordHere"

// ============================================================
//  MQTT Broker Configuration
// ============================================================
/**
 * @brief MQTT broker server address (IP or hostname)
 */
#define CONFIG_MQTT_SERVER "192.168.1.100"

/**
 * @brief MQTT broker port (default: 1883)
 */
#define CONFIG_MQTT_PORT 1883

/**
 * @brief MQTT broker username (if authentication required)
 */
#define CONFIG_MQTT_USER "mqtt_username"

/**
 * @brief MQTT broker password (if authentication required)
 */
#define CONFIG_MQTT_PASSWORD "mqtt_password"

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
