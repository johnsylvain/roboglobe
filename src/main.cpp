#include <Arduino.h>
#include "config_store.h"
#include "wifi_manager.h"
#include "servo_driver.h"
#include "globe_mapping.h"
#include "http_api.h"

// Global instances
ConfigStore configStore;
WiFiManager wifiManager(configStore);
ServoDriver servoDriver(configStore);
GlobeMapping globeMapping;
HttpAPI httpAPI(wifiManager, globeMapping, servoDriver, configStore);

void setup() {
    // Initialize Serial
    Serial.begin(115200);
    delay(1000);
    Serial.println("\n\n=== RoboGlobe Firmware ===");
    Serial.println("Initializing...");
    
    // Initialize NVS
    if (!configStore.begin()) {
        Serial.println("ERROR: Failed to initialize NVS");
        while (1) {
            delay(1000);
        }
    }
    Serial.println("NVS initialized");
    
    // Initialize I2C and PCA9685
    if (!servoDriver.begin()) {
        Serial.println("WARNING: Failed to initialize servo driver");
        Serial.println("The device will continue but /point endpoint will return errors.");
        Serial.println("Please check hardware connections and restart after fixing.");
        // Continue anyway - device can still serve API without servos
    } else {
        Serial.println("✓ Servo driver initialized successfully");
    }
    
    // Initialize Wi-Fi (AP or STA mode)
    if (!wifiManager.begin()) {
        Serial.println("ERROR: Failed to initialize Wi-Fi");
        while (1) {
            delay(1000);
        }
    }
    
    // Print Wi-Fi status
    Serial.printf("Wi-Fi Mode: %s\n", wifiManager.getMode() == ROBOGLOBE_WIFI_MODE_AP ? "AP" : "STA");
    Serial.printf("SSID: %s\n", wifiManager.getSSID().c_str());
    Serial.printf("IP Address: %s\n", wifiManager.getIPAddress().toString().c_str());
    
    // Initialize HTTP server
    if (!httpAPI.begin()) {
        Serial.println("ERROR: Failed to initialize HTTP server");
        while (1) {
            delay(1000);
        }
    }
    
    Serial.println("=== Initialization Complete ===");
    Serial.println("HTTP server ready");
    Serial.printf("Access the device at: http://%s\n", wifiManager.getIPAddress().toString().c_str());
    
    if (wifiManager.getMode() == ROBOGLOBE_WIFI_MODE_AP) {
        Serial.println("Provisioning mode active - connect to configure Wi-Fi");
    }
}

void loop() {
    // Handle DNS server for captive portal (in AP mode)
    wifiManager.processDNS();
    
    // Update HTTP API uptime
    httpAPI.updateUptime();
    
    // Update animation (if active)
    httpAPI.updateAnimation();
    
    // Update smooth servo movements
    servoDriver.update();
    
    // Small delay to prevent watchdog issues
    delay(10);
}

