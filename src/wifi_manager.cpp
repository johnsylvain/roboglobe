#include "wifi_manager.h"

WiFiManager::WiFiManager(ConfigStore& configStore) 
    : configStore(configStore), currentMode(ROBOGLOBE_WIFI_MODE_AP), dnsStarted(false) {
}

WiFiManager::~WiFiManager() {
    stopDNSServer();
}

bool WiFiManager::begin() {
    // Check if we have stored Wi-Fi credentials
    if (configStore.hasWiFiCredentials()) {
        Serial.println("Found stored Wi-Fi credentials, attempting STA mode...");
        if (connectSTA()) {
            currentMode = ROBOGLOBE_WIFI_MODE_STA;
            return true;
        } else {
            Serial.println("Failed to connect to Wi-Fi, falling back to AP mode");
        }
    }
    
    // No credentials or connection failed, start AP mode
    Serial.println("Starting AP mode for provisioning...");
    return startAPMode();
}

bool WiFiManager::startAPMode() {
    currentMode = ROBOGLOBE_WIFI_MODE_AP;
    
    String apSSID = generateAPSSID();
    Serial.printf("Starting AP: %s\n", apSSID.c_str());
    
    // Start access point
    bool success = WiFi.softAP(apSSID.c_str(), AP_PASSWORD);
    if (!success) {
        Serial.println("ERROR: Failed to start AP");
        return false;
    }
    
    IPAddress apIP = WiFi.softAPIP();
    Serial.printf("AP IP address: %s\n", apIP.toString().c_str());
    
    // Start DNS server for captive portal
    startDNSServer();
    
    // Start mDNS for friendly device discovery
    startMDNS();
    
    return true;
}

bool WiFiManager::connectSTA() {
    char ssid[64];
    char password[64];
    
    if (!configStore.getWiFiSSID(ssid, sizeof(ssid))) {
        Serial.println("ERROR: Could not retrieve Wi-Fi SSID");
        return false;
    }
    
    configStore.getWiFiPassword(password, sizeof(password));
    
    Serial.printf("Connecting to Wi-Fi: %s\n", ssid);
    
    WiFi.mode(WIFI_STA);
    
    // Set hostname before connecting
    String hostname = "roboglobe-" + getChipIDString();
    hostname.toLowerCase();
    WiFi.setHostname(hostname.c_str());
    Serial.printf("Hostname set to: %s\n", hostname.c_str());
    
    WiFi.begin(ssid, strlen(password) > 0 ? password : NULL);
    
    // Wait for connection with timeout
    unsigned long startTime = millis();
    while (WiFi.status() != WL_CONNECTED) {
        if (millis() - startTime > WIFI_CONNECT_TIMEOUT) {
            Serial.println("Wi-Fi connection timeout");
            return false;
        }
        delay(WIFI_RETRY_DELAY);
        Serial.print(".");
    }
    
    Serial.println();
    Serial.printf("Connected! IP address: %s\n", WiFi.localIP().toString().c_str());
    
    // Start mDNS with hostname for STA mode
    startMDNS();
    
    // Stop DNS server if it was running
    stopDNSServer();
    
    return true;
}

IPAddress WiFiManager::getIPAddress() const {
    if (currentMode == ROBOGLOBE_WIFI_MODE_AP) {
        return WiFi.softAPIP();
    } else {
        return WiFi.localIP();
    }
}

bool WiFiManager::isConnected() const {
    if (currentMode == ROBOGLOBE_WIFI_MODE_STA) {
        return WiFi.status() == WL_CONNECTED;
    }
    return true; // AP mode is always "connected"
}

String WiFiManager::getSSID() const {
    if (currentMode == ROBOGLOBE_WIFI_MODE_AP) {
        return generateAPSSID();
    } else {
        return WiFi.SSID();
    }
}

String WiFiManager::generateAPSSID() const {
    String chipID = getChipIDString();
    return "ROBOGLOBE-" + chipID;
}

String WiFiManager::getChipIDString() {
    uint64_t chipid = ESP.getEfuseMac();
    // Get last 4 hex digits
    char chipIDStr[5];
    snprintf(chipIDStr, sizeof(chipIDStr), "%04X", (uint16_t)(chipid & 0xFFFF));
    return String(chipIDStr);
}

void WiFiManager::startDNSServer() {
    if (dnsStarted) {
        return;
    }
    
    // Start DNS server for captive portal
    // Redirect all DNS queries to the AP IP
    dnsServer.start(53, "*", WiFi.softAPIP());
    dnsStarted = true;
    Serial.println("DNS server started for captive portal");
}

void WiFiManager::stopDNSServer() {
    if (dnsStarted) {
        dnsServer.stop();
        dnsStarted = false;
        Serial.println("DNS server stopped");
    }
}

void WiFiManager::processDNS() {
    if (dnsStarted && currentMode == ROBOGLOBE_WIFI_MODE_AP) {
        dnsServer.processNextRequest();
    }
}

bool WiFiManager::startMDNS() {
    // Stop any existing mDNS service
    MDNS.end();
    
    // Generate friendly hostname: roboglobe-<chip_id>
    String hostname = "roboglobe-" + getChipIDString();
    hostname.toLowerCase();
    
    // In STA mode, use the hostname that was set via WiFi.setHostname()
    // In AP mode, use the generated hostname
    if (currentMode == ROBOGLOBE_WIFI_MODE_STA) {
        // Get the hostname that was set via WiFi.setHostname()
        const char* setHostname = WiFi.getHostname();
        if (setHostname && strlen(setHostname) > 0) {
            hostname = String(setHostname);
        }
    }
    
    if (!MDNS.begin(hostname.c_str())) {
        Serial.println("WARNING: Failed to start mDNS service");
        return false;
    }
    
    // Add HTTP service
    MDNS.addService("http", "tcp", 80);
    
    Serial.printf("mDNS started: %s.local\n", hostname.c_str());
    return true;
}
