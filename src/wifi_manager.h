#ifndef WIFI_MANAGER_H
#define WIFI_MANAGER_H

#include <Arduino.h>
#include <WiFi.h>
#include <DNSServer.h>
#include <ESPmDNS.h>
#include "config_store.h"

// Wi-Fi connection timeout (milliseconds)
#define WIFI_CONNECT_TIMEOUT 30000
#define WIFI_RETRY_DELAY 500

// AP mode settings
#define AP_PASSWORD ""  // Open network for easier provisioning

enum RoboGlobeWiFiMode {
    ROBOGLOBE_WIFI_MODE_AP,   // Access Point mode (provisioning)
    ROBOGLOBE_WIFI_MODE_STA   // Station mode (normal operation)
};

class WiFiManager {
public:
    WiFiManager(ConfigStore& configStore);
    ~WiFiManager();
    
    // Initialize and start Wi-Fi (AP or STA based on stored credentials)
    bool begin();
    
    // Get current mode
    RoboGlobeWiFiMode getMode() const { return currentMode; }
    
    // Get current IP address
    IPAddress getIPAddress() const;
    
    // Check if connected (for STA mode)
    bool isConnected() const;
    
    // Get SSID (for AP mode) or connected SSID (for STA mode)
    String getSSID() const;
    
    // Force AP mode (for provisioning)
    bool startAPMode();
    
    // Try to connect to stored Wi-Fi credentials
    bool connectSTA();
    
    // Get chip ID as hex string (last 4 digits)
    static String getChipIDString();
    
    // Process DNS requests (call in loop for AP mode)
    void processDNS();
    
    // Start mDNS service for friendly device discovery
    bool startMDNS();

private:
    ConfigStore& configStore;
    RoboGlobeWiFiMode currentMode;
    DNSServer dnsServer;
    bool dnsStarted;
    
    // Generate AP SSID: ROBOGLOBE-<chip_id>
    String generateAPSSID() const;
    
    // Start DNS server for captive portal
    void startDNSServer();
    
    // Stop DNS server
    void stopDNSServer();
};

#endif // WIFI_MANAGER_H

