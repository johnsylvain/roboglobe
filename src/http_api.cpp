#include "http_api.h"
#include <SPIFFS.h>
#include <WiFi.h>
#include <math.h>

// Captive portal handler - catches all requests in AP mode and redirects to provisioning page
class CaptiveRequestHandler : public AsyncWebHandler {
private:
    WiFiManager& wifiManager;
    
public:
    CaptiveRequestHandler(WiFiManager& wifiManager) : wifiManager(wifiManager) {}
    
    bool canHandle(AsyncWebServerRequest* request) const override {
        // Only handle in AP mode
        if (wifiManager.getMode() != ROBOGLOBE_WIFI_MODE_AP) {
            return false;
        }
        
        // Don't intercept our own API endpoints
        String url = request->url();
        if (url == "/" || url == "/health" || url == "/point" || url == "/points" ||
            url == "/state" || url == "/calibrate" || url == "/reset-wifi" || 
            url == "/provision" || url.startsWith("/static/")) {
            return false;
        }
        
        // Catch all other requests for captive portal
        return true;
    }
    
    void handleRequest(AsyncWebServerRequest* request) override {
        // Redirect to root provisioning page
        request->redirect("/");
    }
};

HttpAPI::HttpAPI(WiFiManager& wifiManager, GlobeMapping& globeMapping,
                 ServoDriver& servoDriver, ConfigStore& configStore)
    : server(HTTP_PORT), wifiManager(wifiManager), globeMapping(globeMapping),
      servoDriver(servoDriver), configStore(configStore) {
    bootTime = millis();
    state.last_lat = 0.0f;
    state.last_lng = 0.0f;
    state.last_lat_servo_deg = 90.0f;
    state.last_lng_servo_deg = 90.0f;
    
    // Initialize animation state
    currentAnimationType = ANIMATION_NONE;
    animationState.active = false;
    animationState.points = nullptr;
    animationState.pointCount = 0;
    animationState.currentIndex = 0;
    animationState.intervalMs = 1000;
    animationState.repeats = 1;
    animationState.currentRepeat = 0;
    animationState.lastMoveTime = 0;
    animationState.waitingForServo = false;
    
    // Initialize sinusoidal animation state
    sinusoidalState.active = false;
    sinusoidalState.centerLat = 0.0f;
    sinusoidalState.centerLng = 0.0f;
    sinusoidalState.radius = 30.0f;
    sinusoidalState.speed = 10.0f; // degrees per second
    sinusoidalState.startTime = 0;
    sinusoidalState.durationMs = 0; // 0 = infinite
    sinusoidalState.lastUpdateTime = 0;
    sinusoidalState.waitingForServo = false;
    
    // Initialize ISS tracking state
    issState.active = false;
    issState.intervalMs = 5000; // Default 5 seconds
    issState.durationMs = 0; // 0 = infinite
    issState.startTime = 0;
    issState.lastFetchTime = 0;
    issState.waitingForServo = false;
    issState.lastLat = 0.0f;
    issState.lastLng = 0.0f;
}

HttpAPI::~HttpAPI() {
    // Clean up animation state if active
    if (animationState.points != nullptr) {
        delete[] animationState.points;
        animationState.points = nullptr;
    }
}

bool HttpAPI::begin() {
    // Initialize SPIFFS for serving static files
    if (!SPIFFS.begin(true)) {
        Serial.println("WARNING: SPIFFS initialization failed, provisioning page may not be available");
    }
    
    // Add captive portal handler FIRST (only in AP mode)
    // This catches all requests that don't match specific routes and redirects to provisioning
    server.addHandler(new CaptiveRequestHandler(wifiManager)).setFilter(ON_AP_FILTER);
    
    // Root endpoint - serve provisioning page in AP mode, redirect in STA mode
    server.on("/", HTTP_GET, [this](AsyncWebServerRequest* request) {
        this->handleRoot(request);
    });
    
    // Health endpoint
    server.on("/health", HTTP_GET, [this](AsyncWebServerRequest* request) {
        this->handleHealth(request);
    });
    
    // Point endpoint - POST with JSON body
    server.on("/point", HTTP_POST, [](AsyncWebServerRequest* request) {},
              [](AsyncWebServerRequest* request, const String& filename, size_t index, uint8_t* data, size_t len, bool final) {},
              [this](AsyncWebServerRequest* request, uint8_t* data, size_t len, size_t index, size_t total) {
                  this->handlePointPost(request, data, len, index, total);
              });
    
    // Point endpoint - GET with query parameters
    server.on("/point", HTTP_GET, [this](AsyncWebServerRequest* request) {
        this->handlePointGet(request);
    });
    
    // State endpoint
    server.on("/state", HTTP_GET, [this](AsyncWebServerRequest* request) {
        this->handleState(request);
    });
    
    // Calibrate endpoint
    server.on("/calibrate", HTTP_POST, [](AsyncWebServerRequest* request) {},
              [](AsyncWebServerRequest* request, const String& filename, size_t index, uint8_t* data, size_t len, bool final) {},
              [this](AsyncWebServerRequest* request, uint8_t* data, size_t len, size_t index, size_t total) {
                  this->handleCalibrate(request, data, len, index, total);
              });
    
    // Reset Wi-Fi endpoint
    server.on("/reset-wifi", HTTP_POST, [this](AsyncWebServerRequest* request) {
        this->handleResetWifi(request);
    });
    
    // WiFi scan endpoint (for network discovery)
    server.on("/scan", HTTP_GET, [this](AsyncWebServerRequest* request) {
        this->handleWiFiScan(request);
    });
    
    // I2C scan endpoint (for hardware diagnostics)
    server.on("/i2c-scan", HTTP_GET, [this](AsyncWebServerRequest* request) {
        this->handleI2CScan(request);
    });
    
    // Provision endpoint (for Wi-Fi setup)
    server.on("/provision", HTTP_POST, [](AsyncWebServerRequest* request) {},
              [](AsyncWebServerRequest* request, const String& filename, size_t index, uint8_t* data, size_t len, bool final) {},
              [this](AsyncWebServerRequest* request, uint8_t* data, size_t len, size_t index, size_t total) {
                  this->handleProvision(request, data, len, index, total);
              });
    
    // Points endpoint (for animating through multiple points)
    server.on("/points", HTTP_POST, [](AsyncWebServerRequest* request) {},
              [](AsyncWebServerRequest* request, const String& filename, size_t index, uint8_t* data, size_t len, bool final) {},
              [this](AsyncWebServerRequest* request, uint8_t* data, size_t len, size_t index, size_t total) {
                  this->handlePoints(request, data, len, index, total);
              });
    
    // Animate endpoint (for sinusoidal rotation)
    server.on("/animate", HTTP_POST, [](AsyncWebServerRequest* request) {},
              [](AsyncWebServerRequest* request, const String& filename, size_t index, uint8_t* data, size_t len, bool final) {},
              [this](AsyncWebServerRequest* request, uint8_t* data, size_t len, size_t index, size_t total) {
                  this->handleAnimate(request, data, len, index, total);
              });
    
    // ISS endpoint (for tracking ISS position)
    server.on("/iss", HTTP_POST, [](AsyncWebServerRequest* request) {},
              [](AsyncWebServerRequest* request, const String& filename, size_t index, uint8_t* data, size_t len, bool final) {},
              [this](AsyncWebServerRequest* request, uint8_t* data, size_t len, size_t index, size_t total) {
                  this->handleISS(request, data, len, index, total);
              });
    
    // Captive portal detection endpoints - redirect to provisioning page
    // These are queried by different OSes to detect captive portals
    server.on("/generate_204", HTTP_GET, [this](AsyncWebServerRequest* request) {
        // Android captive portal detection
        this->handleRoot(request);
    }).setFilter(ON_AP_FILTER);
    
    server.on("/gen_204", HTTP_GET, [this](AsyncWebServerRequest* request) {
        // Android captive portal detection (alternative)
        this->handleRoot(request);
    }).setFilter(ON_AP_FILTER);
    
    server.on("/hotspot-detect.html", HTTP_GET, [this](AsyncWebServerRequest* request) {
        // iOS/macOS captive portal detection
        this->handleRoot(request);
    }).setFilter(ON_AP_FILTER);
    
    server.on("/success.txt", HTTP_GET, [this](AsyncWebServerRequest* request) {
        // iOS/macOS captive portal detection (alternative)
        this->handleRoot(request);
    }).setFilter(ON_AP_FILTER);
    
    server.on("/ncsi.txt", HTTP_GET, [this](AsyncWebServerRequest* request) {
        // Windows captive portal detection
        this->handleRoot(request);
    }).setFilter(ON_AP_FILTER);
    
    server.on("/connecttest.txt", HTTP_GET, [this](AsyncWebServerRequest* request) {
        // Windows captive portal detection (alternative)
        this->handleRoot(request);
    }).setFilter(ON_AP_FILTER);
    
    server.on("/kindle-wifi/wifiredirect", HTTP_GET, [this](AsyncWebServerRequest* request) {
        // Kindle captive portal detection
        this->handleRoot(request);
    }).setFilter(ON_AP_FILTER);
    
    // 404 handler
    server.onNotFound([this](AsyncWebServerRequest* request) {
        this->handleNotFound(request);
    });
    
    server.begin();
    Serial.printf("HTTP server started on port %d\n", HTTP_PORT);
    
    return true;
}

void HttpAPI::handleDNS() {
    if (wifiManager.getMode() == ROBOGLOBE_WIFI_MODE_AP) {
        // DNS server is handled by WiFiManager
    }
}

void HttpAPI::handleRoot(AsyncWebServerRequest* request) {
    if (wifiManager.getMode() == ROBOGLOBE_WIFI_MODE_AP) {
        // Serve provisioning page from SPIFFS
        if (SPIFFS.exists("/index.html")) {
            request->send(SPIFFS, "/index.html", "text/html");
        } else {
            // Fallback: simple HTML if file not found
            request->send(200, "text/html", 
                "<html><body><h1>RoboGlobe Provisioning</h1>"
                "<p>Please use the provisioning interface.</p></body></html>");
        }
    } else {
        // In STA mode, redirect to /health or show status
        request->redirect("/health");
    }
}

void HttpAPI::handleHealth(AsyncWebServerRequest* request) {
    updateUptime();
    
    StaticJsonDocument<256> doc;
    doc["status"] = "ok";
    doc["mode"] = getModeString();
    doc["ip"] = wifiManager.getIPAddress().toString();
    doc["uptime_ms"] = state.uptime_ms;
    doc["servo_initialized"] = servoDriver.isInitialized();
    
    sendJSON(request, 200, doc);
}

void HttpAPI::handlePointPost(AsyncWebServerRequest* request, uint8_t* data, size_t len,
                                  size_t index, size_t total) {
    // Accumulate data if needed (for large payloads)
    static String body;
    if (index == 0) {
        body = "";
    }
    
    for (size_t i = 0; i < len; i++) {
        body += (char)data[i];
    }
    
    // Process when complete
    if (index + len == total) {
        Serial.printf("POST /point received: %s\n", body.c_str());
        
        StaticJsonDocument<256> doc;
        DeserializationError error = deserializeJson(doc, body);
        
        if (error) {
            Serial.printf("ERROR: JSON parse error: %s\n", error.c_str());
            sendError(request, 400, "Invalid JSON");
            return;
        }
        
        if (!doc.containsKey("lat") || !doc.containsKey("lng")) {
            Serial.println("ERROR: Missing 'lat' or 'lng' field");
            sendError(request, 400, "Missing 'lat' or 'lng' field");
            return;
        }
        
        float lat = doc["lat"];
        float lng = doc["lng"];
        Serial.printf("Processing point: lat=%.4f, lng=%.4f\n", lat, lng);
        
        // Validate coordinates
        if (!GlobeMapping::validateLatitude(lat) || !GlobeMapping::validateLongitude(lng)) {
            Serial.printf("ERROR: Invalid coordinates - lat=%.4f, lng=%.4f\n", lat, lng);
            sendError(request, 400, "Invalid latitude or longitude range");
            return;
        }
        
        // Check if servo driver is initialized
        if (!servoDriver.isInitialized()) {
            Serial.println("ERROR: Servo driver not initialized");
            sendError(request, 503, "Servo driver not initialized. Check hardware connections (PCA9685, I2C wiring) and serial monitor for errors.");
            return;
        }
        
        // Map coordinates to servo angles
        MappingResult mapping = globeMapping.mapCoordinates(lat, lng);
        if (!mapping.valid) {
            Serial.println("ERROR: Failed to map coordinates");
            sendError(request, 500, "Failed to map coordinates");
            return;
        }
        
        Serial.printf("Mapped to servo angles: lat_servo=%.2f°, lng_servo=%.2f°\n", 
                      mapping.lat_servo_deg, mapping.lng_servo_deg);
        
        // Set servo positions smoothly with easing (1500ms duration for smooth movement)
        if (!servoDriver.setServoAngleSmooth(SERVO_CHANNEL_LATITUDE, mapping.lat_servo_deg, 1500)) {
            Serial.printf("ERROR: Failed to set latitude servo to %.2f degrees\n", mapping.lat_servo_deg);
            sendError(request, 500, "Failed to set latitude servo. Check serial monitor for details.");
            return;
        }
        
        Serial.printf("Moving latitude servo smoothly to %.2f degrees\n", mapping.lat_servo_deg);
        
        if (!servoDriver.setServoAngleSmooth(SERVO_CHANNEL_LONGITUDE, mapping.lng_servo_deg, 1500)) {
            Serial.printf("ERROR: Failed to set longitude servo to %.2f degrees\n", mapping.lng_servo_deg);
            sendError(request, 500, "Failed to set longitude servo. Check serial monitor for details.");
            return;
        }
        
        Serial.printf("Moving longitude servo smoothly to %.2f degrees\n", mapping.lng_servo_deg);
        
        // Update state
        state.last_lat = lat;
        state.last_lng = lng;
        state.last_lat_servo_deg = mapping.lat_servo_deg;
        state.last_lng_servo_deg = mapping.lng_servo_deg;
        
        // Return response
        StaticJsonDocument<256> response;
        response["lat"] = lat;
        response["lng"] = lng;
        response["lat_servo_deg"] = mapping.lat_servo_deg;
        response["lng_servo_deg"] = mapping.lng_servo_deg;
        
        Serial.println("POST /point completed successfully");
        sendJSON(request, 200, response);
    }
}

void HttpAPI::handlePointGet(AsyncWebServerRequest* request) {
    if (request->hasParam("lat") && request->hasParam("lng")) {
        float lat = request->getParam("lat")->value().toFloat();
        float lng = request->getParam("lng")->value().toFloat();
        
        // Validate coordinates
        if (!GlobeMapping::validateLatitude(lat) || !GlobeMapping::validateLongitude(lng)) {
            sendError(request, 400, "Invalid latitude or longitude range");
            return;
        }
        
        // Check if servo driver is initialized
        if (!servoDriver.isInitialized()) {
            sendError(request, 503, "Servo driver not initialized. Check hardware connections (PCA9685, I2C wiring) and serial monitor for errors.");
            return;
        }
        
        // Map coordinates to servo angles
        MappingResult mapping = globeMapping.mapCoordinates(lat, lng);
        if (!mapping.valid) {
            sendError(request, 500, "Failed to map coordinates");
            return;
        }
        
        // Set servo positions smoothly with easing (1500ms duration for smooth movement)
        if (!servoDriver.setServoAngleSmooth(SERVO_CHANNEL_LATITUDE, mapping.lat_servo_deg, 1500)) {
            Serial.printf("ERROR: Failed to set latitude servo to %.2f degrees\n", mapping.lat_servo_deg);
            sendError(request, 500, "Failed to set latitude servo. Check serial monitor for details.");
            return;
        }
        
        if (!servoDriver.setServoAngleSmooth(SERVO_CHANNEL_LONGITUDE, mapping.lng_servo_deg, 1500)) {
            Serial.printf("ERROR: Failed to set longitude servo to %.2f degrees\n", mapping.lng_servo_deg);
            sendError(request, 500, "Failed to set longitude servo. Check serial monitor for details.");
            return;
        }
        
        // Update state
        state.last_lat = lat;
        state.last_lng = lng;
        state.last_lat_servo_deg = mapping.lat_servo_deg;
        state.last_lng_servo_deg = mapping.lng_servo_deg;
        
        // Return response
        StaticJsonDocument<256> response;
        response["lat"] = lat;
        response["lng"] = lng;
        response["lat_servo_deg"] = mapping.lat_servo_deg;
        response["lng_servo_deg"] = mapping.lng_servo_deg;
        
        sendJSON(request, 200, response);
    } else {
        sendError(request, 400, "Missing 'lat' or 'lng' query parameter");
    }
}

void HttpAPI::handleState(AsyncWebServerRequest* request) {
    updateUptime();
    
    StaticJsonDocument<512> doc;
    doc["last_lat"] = state.last_lat;
    doc["last_lng"] = state.last_lng;
    doc["last_lat_servo_deg"] = state.last_lat_servo_deg;
    doc["last_lng_servo_deg"] = state.last_lng_servo_deg;
    doc["uptime_ms"] = state.uptime_ms;
    
    // Add calibration info
    JsonObject cal_lat = doc.createNestedObject("calibration_lat");
    JsonObject cal_lng = doc.createNestedObject("calibration_lng");
    
    ServoCalibration cal;
    if (configStore.getServoCalibration(SERVO_CHANNEL_LATITUDE, cal)) {
        cal_lat["pulse_min_us"] = cal.pulse_min_us;
        cal_lat["pulse_max_us"] = cal.pulse_max_us;
        cal_lat["invert"] = cal.invert;
        cal_lat["offset_deg"] = cal.offset_deg;
    }
    
    if (configStore.getServoCalibration(SERVO_CHANNEL_LONGITUDE, cal)) {
        cal_lng["pulse_min_us"] = cal.pulse_min_us;
        cal_lng["pulse_max_us"] = cal.pulse_max_us;
        cal_lng["invert"] = cal.invert;
        cal_lng["offset_deg"] = cal.offset_deg;
    }
    
    sendJSON(request, 200, doc);
}

void HttpAPI::handleCalibrate(AsyncWebServerRequest* request, uint8_t* data, size_t len,
                               size_t index, size_t total) {
    static String body;
    if (index == 0) {
        body = "";
    }
    
    for (size_t i = 0; i < len; i++) {
        body += (char)data[i];
    }
    
    if (index + len == total) {
        StaticJsonDocument<512> doc;
        DeserializationError error = deserializeJson(doc, body);
        
        if (error) {
            sendError(request, 400, "Invalid JSON");
            return;
        }
        
        if (!doc.containsKey("channel")) {
            sendError(request, 400, "Missing 'channel' field");
            return;
        }
        
        uint8_t channel = doc["channel"];
        if (channel > 1) {
            sendError(request, 400, "Invalid channel (must be 0 or 1)");
            return;
        }
        
        ServoCalibration cal;
        configStore.getServoCalibration(channel, cal);
        
        // Update calibration from JSON if provided
        if (doc.containsKey("pulse_min_us")) {
            cal.pulse_min_us = doc["pulse_min_us"];
        }
        if (doc.containsKey("pulse_max_us")) {
            cal.pulse_max_us = doc["pulse_max_us"];
        }
        if (doc.containsKey("invert")) {
            cal.invert = doc["invert"];
        }
        if (doc.containsKey("offset_deg")) {
            cal.offset_deg = doc["offset_deg"];
        }
        
        if (!configStore.setServoCalibration(channel, cal)) {
            sendError(request, 500, "Failed to save calibration");
            return;
        }
        
        StaticJsonDocument<256> response;
        response["channel"] = channel;
        response["pulse_min_us"] = cal.pulse_min_us;
        response["pulse_max_us"] = cal.pulse_max_us;
        response["invert"] = cal.invert;
        response["offset_deg"] = cal.offset_deg;
        
        sendJSON(request, 200, response);
    }
}

void HttpAPI::handleResetWifi(AsyncWebServerRequest* request) {
    if (configStore.clearWiFiCredentials()) {
        StaticJsonDocument<128> doc;
        doc["status"] = "ok";
        doc["message"] = "Wi-Fi credentials cleared, rebooting...";
        sendJSON(request, 200, doc);
        
        delay(1000);
        ESP.restart();
    } else {
        sendError(request, 500, "Failed to clear Wi-Fi credentials");
    }
}

void HttpAPI::handleWiFiScan(AsyncWebServerRequest* request) {
    // Only allow scanning in AP mode
    if (wifiManager.getMode() != ROBOGLOBE_WIFI_MODE_AP) {
        sendError(request, 403, "WiFi scan only available in AP mode");
        return;
    }
    
    // Start scan if not already scanning
    int n = WiFi.scanComplete();
    if (n == -2) {
        // Scan not started, start it
        WiFi.scanNetworks(true, true); // async, show hidden
        StaticJsonDocument<128> doc;
        doc["status"] = "scanning";
        doc["message"] = "Scan started, please retry in a few seconds";
        sendJSON(request, 202, doc);
        return;
    } else if (n == -1) {
        // Scan in progress
        StaticJsonDocument<128> doc;
        doc["status"] = "scanning";
        doc["message"] = "Scan in progress, please retry";
        sendJSON(request, 202, doc);
        return;
    }
    
    // Build JSON array of networks
    StaticJsonDocument<2048> doc;
    JsonArray networks = doc.createNestedArray("networks");
    
    for (int i = 0; i < n; ++i) {
        JsonObject network = networks.createNestedObject();
        network["ssid"] = WiFi.SSID(i);
        network["rssi"] = WiFi.RSSI(i);
        network["channel"] = WiFi.channel(i);
        network["encryption"] = (WiFi.encryptionType(i) != WIFI_AUTH_OPEN);
        // Note: ESP32 doesn't have isHidden() method, hidden networks are detected by empty SSID
        network["hidden"] = (WiFi.SSID(i).length() == 0);
        
        // Determine encryption type string
        const char* encType = "Unknown";
        switch (WiFi.encryptionType(i)) {
            case WIFI_AUTH_OPEN: encType = "None"; break;
            case WIFI_AUTH_WEP: encType = "WEP"; break;
            case WIFI_AUTH_WPA_PSK: encType = "WPA"; break;
            case WIFI_AUTH_WPA2_PSK: encType = "WPA2"; break;
            case WIFI_AUTH_WPA_WPA2_PSK: encType = "WPA/WPA2"; break;
            case WIFI_AUTH_WPA2_ENTERPRISE: encType = "WPA2 Enterprise"; break;
            case WIFI_AUTH_WPA3_PSK: encType = "WPA3"; break;
            case WIFI_AUTH_WPA2_WPA3_PSK: encType = "WPA2/WPA3"; break;
            default: encType = "Unknown"; break;
        }
        network["encryption_type"] = encType;
    }
    
    doc["count"] = n;
    
    // Delete scan results and start new scan for next time
    WiFi.scanDelete();
    if (WiFi.scanComplete() == -2) {
        WiFi.scanNetworks(true, true);
    }
    
    sendJSON(request, 200, doc);
}

void HttpAPI::handleI2CScan(AsyncWebServerRequest* request) {
    // Perform I2C scan and get JSON results
    String scanResults = ServoDriver::scanI2CBusJSON();
    
    // Parse the JSON string to send it properly
    StaticJsonDocument<2048> doc;
    DeserializationError error = deserializeJson(doc, scanResults);
    
    if (error) {
        sendError(request, 500, "Failed to scan I2C bus");
        return;
    }
    
    sendJSON(request, 200, doc);
}

void HttpAPI::handleProvision(AsyncWebServerRequest* request, uint8_t* data, size_t len,
                               size_t index, size_t total) {
    static String body;
    if (index == 0) {
        body = "";
    }
    
    for (size_t i = 0; i < len; i++) {
        body += (char)data[i];
    }
    
    if (index + len == total) {
        StaticJsonDocument<256> doc;
        DeserializationError error = deserializeJson(doc, body);
        
        if (error) {
            sendError(request, 400, "Invalid JSON");
            return;
        }
        
        if (!doc.containsKey("ssid")) {
            sendError(request, 400, "Missing 'ssid' field");
            return;
        }
        
        const char* ssid = doc["ssid"];
        const char* password = doc.containsKey("password") ? doc["password"].as<const char*>() : "";
        
        if (!configStore.setWiFiCredentials(ssid, password)) {
            sendError(request, 500, "Failed to save Wi-Fi credentials");
            return;
        }
        
        StaticJsonDocument<128> response;
        response["status"] = "ok";
        response["message"] = "Wi-Fi credentials saved, connecting...";
        sendJSON(request, 200, response);
        
        // Attempt to connect
        delay(500);
        if (wifiManager.connectSTA()) {
            // Connection successful, will continue in STA mode
            response["connected"] = true;
        } else {
            response["connected"] = false;
            response["message"] = "Credentials saved but connection failed. Please check credentials.";
        }
    }
}

void HttpAPI::handlePoints(AsyncWebServerRequest* request, uint8_t* data, size_t len,
                           size_t index, size_t total) {
    // Accumulate data if needed (for large payloads)
    static String body;
    if (index == 0) {
        body = "";
    }
    
    for (size_t i = 0; i < len; i++) {
        body += (char)data[i];
    }
    
    // Process when complete
    if (index + len == total) {
        Serial.printf("POST /points received: %s\n", body.c_str());
        
        StaticJsonDocument<2048> doc;
        DeserializationError error = deserializeJson(doc, body);
        
        if (error) {
            Serial.printf("ERROR: JSON parse error: %s\n", error.c_str());
            sendError(request, 400, "Invalid JSON");
            return;
        }
        
        // Validate required fields
        if (!doc.containsKey("points") || !doc["points"].is<JsonArray>()) {
            Serial.println("ERROR: Missing or invalid 'points' array");
            sendError(request, 400, "Missing or invalid 'points' array");
            return;
        }
        
        JsonArray pointsArray = doc["points"].as<JsonArray>();
        if (pointsArray.size() == 0) {
            Serial.println("ERROR: Points array is empty");
            sendError(request, 400, "Points array cannot be empty");
            return;
        }
        
        // Get interval (default 1000ms)
        unsigned long intervalMs = 1000;
        if (doc.containsKey("interval_ms")) {
            intervalMs = doc["interval_ms"].as<unsigned long>();
            if (intervalMs < 100) {
                intervalMs = 100; // Minimum 100ms
            }
        }
        
        // Get repeats (default 1)
        unsigned int repeats = 1;
        if (doc.containsKey("repeats")) {
            repeats = doc["repeats"].as<unsigned int>();
            if (repeats == 0) {
                repeats = 1; // Minimum 1 repeat
            }
        }
        
        // Check if servo driver is initialized
        if (!servoDriver.isInitialized()) {
            Serial.println("ERROR: Servo driver not initialized");
            sendError(request, 503, "Servo driver not initialized. Check hardware connections (PCA9685, I2C wiring) and serial monitor for errors.");
            return;
        }
        
        // Validate all points
        for (JsonObject point : pointsArray) {
            if (!point.containsKey("lat") || !point.containsKey("lng")) {
                Serial.println("ERROR: Point missing 'lat' or 'lng' field");
                sendError(request, 400, "All points must have 'lat' and 'lng' fields");
                return;
            }
            
            float lat = point["lat"].as<float>();
            float lng = point["lng"].as<float>();
            
            if (!GlobeMapping::validateLatitude(lat) || !GlobeMapping::validateLongitude(lng)) {
                Serial.printf("ERROR: Invalid coordinates - lat=%.4f, lng=%.4f\n", lat, lng);
                sendError(request, 400, "Invalid latitude or longitude range");
                return;
            }
        }
        
        // Stop any existing animation
        stopAllAnimations();
        
        // Allocate memory for points
        size_t pointCount = pointsArray.size();
        animationState.points = new Point[pointCount];
        
        // Copy points
        size_t i = 0;
        for (JsonObject point : pointsArray) {
            animationState.points[i].lat = point["lat"].as<float>();
            animationState.points[i].lng = point["lng"].as<float>();
            i++;
        }
        
        // Initialize animation state
        currentAnimationType = ANIMATION_POINTS;
        animationState.active = true;
        animationState.pointCount = pointCount;
        animationState.currentIndex = 0;
        animationState.intervalMs = intervalMs;
        animationState.repeats = repeats;
        animationState.currentRepeat = 0;
        animationState.lastMoveTime = millis();
        animationState.waitingForServo = false;
        
        // Move to first point immediately
        Point firstPoint = animationState.points[0];
        MappingResult mapping = globeMapping.mapCoordinates(firstPoint.lat, firstPoint.lng);
        if (mapping.valid) {
            servoDriver.setServoAngleSmooth(SERVO_CHANNEL_LATITUDE, mapping.lat_servo_deg, 1500);
            servoDriver.setServoAngleSmooth(SERVO_CHANNEL_LONGITUDE, mapping.lng_servo_deg, 1500);
            animationState.waitingForServo = true;
            
            // Update state
            state.last_lat = firstPoint.lat;
            state.last_lng = firstPoint.lng;
            state.last_lat_servo_deg = mapping.lat_servo_deg;
            state.last_lng_servo_deg = mapping.lng_servo_deg;
        }
        
        Serial.printf("Animation started: %zu points, %lu ms interval, %u repeats\n",
                      pointCount, intervalMs, repeats);
        
        // Return response
        StaticJsonDocument<256> response;
        response["status"] = "started";
        response["points"] = pointCount;
        response["interval_ms"] = intervalMs;
        response["repeats"] = repeats;
        
        sendJSON(request, 200, response);
    }
}

void HttpAPI::updateAnimation() {
    unsigned long now = millis();
    
    // Handle different animation types
    switch (currentAnimationType) {
        case ANIMATION_POINTS:
            updatePointsAnimation(now);
            break;
        case ANIMATION_SINUSOIDAL:
            updateSinusoidalAnimation(now);
            break;
        case ANIMATION_ISS:
            updateISSAnimation(now);
            break;
        case ANIMATION_NONE:
        default:
            return;
    }
}

void HttpAPI::updatePointsAnimation(unsigned long now) {
    if (!animationState.active || animationState.points == nullptr) {
        return;
    }
    
    // Check if servos are still moving
    if (animationState.waitingForServo) {
        bool latMoving = servoDriver.isMoving(SERVO_CHANNEL_LATITUDE);
        bool lngMoving = servoDriver.isMoving(SERVO_CHANNEL_LONGITUDE);
        
        if (latMoving || lngMoving) {
            // Still moving, wait
            return;
        }
        
        // Movement complete, start waiting for interval
        animationState.waitingForServo = false;
        animationState.lastMoveTime = now;
        return;
    }
    
    // Check if interval has elapsed
    if (now - animationState.lastMoveTime < animationState.intervalMs) {
        return;
    }
    
    // Move to next point
    animationState.currentIndex++;
    
    // Check if we've completed all points in this repeat
    if (animationState.currentIndex >= animationState.pointCount) {
        animationState.currentRepeat++;
        animationState.currentIndex = 0;
        
        // Check if we've completed all repeats
        if (animationState.currentRepeat >= animationState.repeats) {
            // Animation complete
            Serial.println("Points animation completed");
            delete[] animationState.points;
            animationState.points = nullptr;
            animationState.active = false;
            currentAnimationType = ANIMATION_NONE;
            return;
        }
        
        // Start next repeat
        Serial.printf("Starting repeat %u/%u\n", 
                      animationState.currentRepeat + 1, animationState.repeats);
    }
    
    // Move to next point
    Point nextPoint = animationState.points[animationState.currentIndex];
    MappingResult mapping = globeMapping.mapCoordinates(nextPoint.lat, nextPoint.lng);
    
    if (mapping.valid) {
        Serial.printf("Moving to point %zu/%zu: lat=%.4f, lng=%.4f\n",
                      animationState.currentIndex + 1, animationState.pointCount,
                      nextPoint.lat, nextPoint.lng);
        
        servoDriver.setServoAngleSmooth(SERVO_CHANNEL_LATITUDE, mapping.lat_servo_deg, 1500);
        servoDriver.setServoAngleSmooth(SERVO_CHANNEL_LONGITUDE, mapping.lng_servo_deg, 1500);
        animationState.waitingForServo = true;
        
        // Update state
        state.last_lat = nextPoint.lat;
        state.last_lng = nextPoint.lng;
        state.last_lat_servo_deg = mapping.lat_servo_deg;
        state.last_lng_servo_deg = mapping.lng_servo_deg;
    } else {
        Serial.printf("ERROR: Failed to map point %zu\n", animationState.currentIndex);
        // Skip this point and continue
        animationState.lastMoveTime = now;
    }
}

void HttpAPI::updateSinusoidalAnimation(unsigned long now) {
    if (!sinusoidalState.active) {
        return;
    }
    
    // Check duration
    if (sinusoidalState.durationMs > 0) {
        if (now - sinusoidalState.startTime >= sinusoidalState.durationMs) {
            Serial.println("Sinusoidal animation completed");
            sinusoidalState.active = false;
            currentAnimationType = ANIMATION_NONE;
            return;
        }
    }
    
    // Check if servos are still moving - wait for movement to complete before setting new target
    if (sinusoidalState.waitingForServo) {
        bool latMoving = servoDriver.isMoving(SERVO_CHANNEL_LATITUDE);
        bool lngMoving = servoDriver.isMoving(SERVO_CHANNEL_LONGITUDE);
        
        if (latMoving || lngMoving) {
            return;
        }
        // Movement complete, ready for next update
        sinusoidalState.waitingForServo = false;
    }
    
    // Calculate elapsed time in seconds
    float elapsedSeconds = (now - sinusoidalState.startTime) / 1000.0f;
    
    // Calculate angle for sinusoidal rotation
    float angle = elapsedSeconds * sinusoidalState.speed;
    
    // Generate coordinates using sinusoidal pattern
    // Latitude: oscillate around center with radius
    float lat = sinusoidalState.centerLat + sinusoidalState.radius * sin(angle);
    
    // Longitude: rotate around the globe
    float lng = sinusoidalState.centerLng + sinusoidalState.radius * cos(angle);
    
    // Clamp to valid ranges
    lat = GlobeMapping::clampLatitude(lat);
    lng = GlobeMapping::clampLongitude(lng);
    
    // Map to servo angles
    MappingResult mapping = globeMapping.mapCoordinates(lat, lng);
    if (mapping.valid) {
        // Use longer duration (1500ms) for smoother, more continuous motion
        // This creates a seamless easing effect as each movement flows into the next
        servoDriver.setServoAngleSmooth(SERVO_CHANNEL_LATITUDE, mapping.lat_servo_deg, 1500);
        servoDriver.setServoAngleSmooth(SERVO_CHANNEL_LONGITUDE, mapping.lng_servo_deg, 1500);
        sinusoidalState.waitingForServo = true;
        sinusoidalState.lastUpdateTime = now;
        
        // Update state
        state.last_lat = lat;
        state.last_lng = lng;
        state.last_lat_servo_deg = mapping.lat_servo_deg;
        state.last_lng_servo_deg = mapping.lng_servo_deg;
    }
}

void HttpAPI::updateISSAnimation(unsigned long now) {
    if (!issState.active) {
        return;
    }
    
    // Check duration
    if (issState.durationMs > 0) {
        if (now - issState.startTime >= issState.durationMs) {
            Serial.println("ISS tracking completed");
            issState.active = false;
            currentAnimationType = ANIMATION_NONE;
            return;
        }
    }
    
    // Check if servos are still moving
    if (issState.waitingForServo) {
        bool latMoving = servoDriver.isMoving(SERVO_CHANNEL_LATITUDE);
        bool lngMoving = servoDriver.isMoving(SERVO_CHANNEL_LONGITUDE);
        
        if (latMoving || lngMoving) {
            return;
        }
        issState.waitingForServo = false;
    }
    
    // Check if it's time to fetch new ISS position
    if (now - issState.lastFetchTime < issState.intervalMs) {
        return;
    }
    
    // Fetch ISS position
    float lat, lng;
    if (fetchISSPosition(lat, lng)) {
        Serial.printf("ISS position: lat=%.4f, lng=%.4f\n", lat, lng);
        
        // Map to servo angles
        MappingResult mapping = globeMapping.mapCoordinates(lat, lng);
        if (mapping.valid) {
            servoDriver.setServoAngleSmooth(SERVO_CHANNEL_LATITUDE, mapping.lat_servo_deg, 1500);
            servoDriver.setServoAngleSmooth(SERVO_CHANNEL_LONGITUDE, mapping.lng_servo_deg, 1500);
            issState.waitingForServo = true;
            issState.lastFetchTime = now;
            issState.lastLat = lat;
            issState.lastLng = lng;
            
            // Update state
            state.last_lat = lat;
            state.last_lng = lng;
            state.last_lat_servo_deg = mapping.lat_servo_deg;
            state.last_lng_servo_deg = mapping.lng_servo_deg;
        } else {
            Serial.println("ERROR: Failed to map ISS coordinates");
            issState.lastFetchTime = now; // Still update time to avoid rapid retries
        }
    } else {
        Serial.println("ERROR: Failed to fetch ISS position");
        issState.lastFetchTime = now; // Still update time to avoid rapid retries
    }
}

void HttpAPI::handleNotFound(AsyncWebServerRequest* request) {
    sendError(request, 404, "Not found");
}

DeviceState HttpAPI::getState() const {
    return state;
}

void HttpAPI::updateUptime() {
    state.uptime_ms = millis() - bootTime;
}

String HttpAPI::getModeString() const {
    return wifiManager.getMode() == ROBOGLOBE_WIFI_MODE_AP ? "ap" : "sta";
}

void HttpAPI::sendJSON(AsyncWebServerRequest* request, int code, const JsonDocument& doc) {
    String json;
    serializeJson(doc, json);
    request->send(code, "application/json", json);
}

void HttpAPI::sendError(AsyncWebServerRequest* request, int code, const char* message) {
    StaticJsonDocument<128> doc;
    doc["error"] = message;
    doc["code"] = code;
    sendJSON(request, code, doc);
}

void HttpAPI::handleAnimate(AsyncWebServerRequest* request, uint8_t* data, size_t len,
                             size_t index, size_t total) {
    // Accumulate data if needed (for large payloads)
    static String body;
    if (index == 0) {
        body = "";
    }
    
    for (size_t i = 0; i < len; i++) {
        body += (char)data[i];
    }
    
    // Process when complete
    if (index + len == total) {
        Serial.printf("POST /animate received: %s\n", body.c_str());
        
        StaticJsonDocument<512> doc;
        DeserializationError error = deserializeJson(doc, body);
        
        if (error) {
            Serial.printf("ERROR: JSON parse error: %s\n", error.c_str());
            sendError(request, 400, "Invalid JSON");
            return;
        }
        
        // Check if servo driver is initialized
        if (!servoDriver.isInitialized()) {
            Serial.println("ERROR: Servo driver not initialized");
            sendError(request, 503, "Servo driver not initialized. Check hardware connections (PCA9685, I2C wiring) and serial monitor for errors.");
            return;
        }
        
        // Stop any existing animation
        stopAllAnimations();
        
        // Get parameters (all optional with defaults)
        float centerLat = 0.0f;
        float centerLng = 0.0f;
        float radius = 30.0f; // Default 30 degrees radius
        float speed = 10.0f; // Default 10 degrees per second
        unsigned long durationMs = 0; // 0 = infinite
        
        if (doc.containsKey("center_lat")) {
            centerLat = doc["center_lat"].as<float>();
        }
        if (doc.containsKey("center_lng")) {
            centerLng = doc["center_lng"].as<float>();
        }
        if (doc.containsKey("radius")) {
            radius = doc["radius"].as<float>();
            if (radius < 1.0f) radius = 1.0f;
            if (radius > 90.0f) radius = 90.0f;
        }
        if (doc.containsKey("speed")) {
            speed = doc["speed"].as<float>();
            if (speed < 0.1f) speed = 0.1f;
            if (speed > 360.0f) speed = 360.0f;
        }
        if (doc.containsKey("duration_ms")) {
            durationMs = doc["duration_ms"].as<unsigned long>();
        }
        
        // Validate center coordinates
        centerLat = GlobeMapping::clampLatitude(centerLat);
        centerLng = GlobeMapping::clampLongitude(centerLng);
        
        // Initialize sinusoidal animation state
        currentAnimationType = ANIMATION_SINUSOIDAL;
        sinusoidalState.active = true;
        sinusoidalState.centerLat = centerLat;
        sinusoidalState.centerLng = centerLng;
        sinusoidalState.radius = radius;
        sinusoidalState.speed = speed;
        sinusoidalState.startTime = millis();
        sinusoidalState.durationMs = durationMs;
        sinusoidalState.lastUpdateTime = millis();
        sinusoidalState.waitingForServo = false;
        
        Serial.printf("Sinusoidal animation started: center=(%.4f, %.4f), radius=%.2f°, speed=%.2f°/s, duration=%lu ms\n",
                      centerLat, centerLng, radius, speed, durationMs);
        
        // Return response
        StaticJsonDocument<256> response;
        response["status"] = "started";
        response["center_lat"] = centerLat;
        response["center_lng"] = centerLng;
        response["radius"] = radius;
        response["speed"] = speed;
        response["duration_ms"] = durationMs;
        
        sendJSON(request, 200, response);
    }
}

void HttpAPI::handleISS(AsyncWebServerRequest* request, uint8_t* data, size_t len,
                         size_t index, size_t total) {
    // Accumulate data if needed (for large payloads)
    static String body;
    if (index == 0) {
        body = "";
    }
    
    for (size_t i = 0; i < len; i++) {
        body += (char)data[i];
    }
    
    // Process when complete
    if (index + len == total) {
        Serial.printf("POST /iss received: %s\n", body.c_str());
        
        StaticJsonDocument<512> doc;
        DeserializationError error = deserializeJson(doc, body);
        
        if (error) {
            Serial.printf("ERROR: JSON parse error: %s\n", error.c_str());
            sendError(request, 400, "Invalid JSON");
            return;
        }
        
        // Check if servo driver is initialized
        if (!servoDriver.isInitialized()) {
            Serial.println("ERROR: Servo driver not initialized");
            sendError(request, 503, "Servo driver not initialized. Check hardware connections (PCA9685, I2C wiring) and serial monitor for errors.");
            return;
        }
        
        // Check WiFi connection (needed for API calls)
        if (wifiManager.getMode() != ROBOGLOBE_WIFI_MODE_STA || WiFi.status() != WL_CONNECTED) {
            Serial.println("ERROR: WiFi not connected");
            sendError(request, 503, "WiFi not connected. ISS tracking requires internet connection.");
            return;
        }
        
        // Stop any existing animation
        stopAllAnimations();
        
        // Get parameters (all optional with defaults)
        unsigned long intervalMs = 5000; // Default 5 seconds
        unsigned long durationMs = 0; // 0 = infinite
        
        if (doc.containsKey("interval_ms")) {
            intervalMs = doc["interval_ms"].as<unsigned long>();
            if (intervalMs < 1000) {
                intervalMs = 1000; // Minimum 1 second
            }
        }
        if (doc.containsKey("duration_ms")) {
            durationMs = doc["duration_ms"].as<unsigned long>();
        }
        
        // Initialize ISS tracking state
        currentAnimationType = ANIMATION_ISS;
        issState.active = true;
        issState.intervalMs = intervalMs;
        issState.durationMs = durationMs;
        issState.startTime = millis();
        issState.lastFetchTime = 0; // Fetch immediately
        issState.waitingForServo = false;
        issState.lastLat = 0.0f;
        issState.lastLng = 0.0f;
        
        Serial.printf("ISS tracking started: interval=%lu ms, duration=%lu ms\n",
                      intervalMs, durationMs);
        
        // Return response
        StaticJsonDocument<256> response;
        response["status"] = "started";
        response["interval_ms"] = intervalMs;
        response["duration_ms"] = durationMs;
        
        sendJSON(request, 200, response);
    }
}

void HttpAPI::stopAllAnimations() {
    // Stop points animation
    if (animationState.active && animationState.points != nullptr) {
        delete[] animationState.points;
        animationState.points = nullptr;
        animationState.active = false;
    }
    
    // Stop sinusoidal animation
    sinusoidalState.active = false;
    
    // Stop ISS tracking
    issState.active = false;
    
    currentAnimationType = ANIMATION_NONE;
}

bool HttpAPI::fetchISSPosition(float& lat, float& lng) {
    HTTPClient http;
    
    // Begin HTTP request
    http.begin("http://api.open-notify.org/iss-now.json");
    http.setTimeout(5000); // 5 second timeout
    
    // Send GET request
    int httpCode = http.GET();
    
    if (httpCode != HTTP_CODE_OK) {
        Serial.printf("ERROR: HTTP request failed with code %d\n", httpCode);
        http.end();
        return false;
    }
    
    // Read response
    String payload = http.getString();
    http.end();
    
    // Parse JSON response
    StaticJsonDocument<256> doc;
    DeserializationError error = deserializeJson(doc, payload);
    
    if (error) {
        Serial.printf("ERROR: JSON parse error: %s\n", error.c_str());
        return false;
    }
    
    // Check for success message
    if (!doc.containsKey("message") || strcmp(doc["message"], "success") != 0) {
        Serial.println("ERROR: API returned non-success message");
        return false;
    }
    
    // Extract position
    if (!doc.containsKey("iss_position")) {
        Serial.println("ERROR: Missing 'iss_position' in response");
        return false;
    }
    
    JsonObject position = doc["iss_position"];
    if (!position.containsKey("latitude") || !position.containsKey("longitude")) {
        Serial.println("ERROR: Missing 'latitude' or 'longitude' in iss_position");
        return false;
    }
    
    lat = position["latitude"].as<float>();
    lng = position["longitude"].as<float>();
    
    // Validate coordinates
    if (!GlobeMapping::validateLatitude(lat) || !GlobeMapping::validateLongitude(lng)) {
        Serial.printf("ERROR: Invalid ISS coordinates - lat=%.4f, lng=%.4f\n", lat, lng);
        return false;
    }
    
    return true;
}
