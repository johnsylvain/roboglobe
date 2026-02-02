#ifndef HTTP_API_H
#define HTTP_API_H

#include <Arduino.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>
#include <HTTPClient.h>
#include <WiFi.h>
#include "wifi_manager.h"
#include "globe_mapping.h"
#include "servo_driver.h"
#include "config_store.h"

// HTTP server port
#define HTTP_PORT 80

// State structure for tracking last position
struct DeviceState {
    float last_lat;
    float last_lng;
    float last_lat_servo_deg;
    float last_lng_servo_deg;
    unsigned long uptime_ms;
};

// Point structure for animation
struct Point {
    float lat;
    float lng;
};

// Animation type enum
enum AnimationType {
    ANIMATION_NONE = 0,
    ANIMATION_POINTS = 1,
    ANIMATION_SINUSOIDAL = 2,
    ANIMATION_ISS = 3
};

// Animation state structure for point-based animations
struct AnimationState {
    bool active;
    Point* points;
    size_t pointCount;
    size_t currentIndex;
    unsigned long intervalMs;
    unsigned int repeats;
    unsigned int currentRepeat;
    unsigned long lastMoveTime;
    bool waitingForServo;
};

// Sinusoidal rotation animation state
struct SinusoidalAnimationState {
    bool active;
    float centerLat;          // Center latitude for rotation
    float centerLng;          // Center longitude for rotation
    float radius;             // Rotation radius in degrees
    float speed;              // Rotation speed (degrees per second)
    unsigned long startTime;  // When animation started
    unsigned long durationMs; // Duration in ms (0 = infinite)
    unsigned long lastUpdateTime;
    bool waitingForServo;
};

// ISS tracking animation state
struct ISSTrackingState {
    bool active;
    unsigned long intervalMs; // How often to fetch ISS position
    unsigned long durationMs; // Duration in ms (0 = infinite)
    unsigned long startTime;  // When tracking started
    unsigned long lastFetchTime; // Last time we fetched ISS position
    bool waitingForServo;
    float lastLat;
    float lastLng;
};

class HttpAPI {
public:
    HttpAPI(WiFiManager& wifiManager, GlobeMapping& globeMapping, 
            ServoDriver& servoDriver, ConfigStore& configStore);
    ~HttpAPI();
    
    // Initialize and start HTTP server
    bool begin();
    
    // Handle DNS requests (for captive portal)
    void handleDNS();
    
    // Get current device state
    DeviceState getState() const;
    
    // Update uptime
    void updateUptime();
    
    // Update animation (call this in loop)
    void updateAnimation();

private:
    AsyncWebServer server;
    WiFiManager& wifiManager;
    GlobeMapping& globeMapping;
    ServoDriver& servoDriver;
    ConfigStore& configStore;
    DeviceState state;
    unsigned long bootTime;
    AnimationType currentAnimationType;
    AnimationState animationState;
    SinusoidalAnimationState sinusoidalState;
    ISSTrackingState issState;
    
    // Endpoint handlers
    void handleRoot(AsyncWebServerRequest* request);
    void handleHealth(AsyncWebServerRequest* request);
    void handlePointPost(AsyncWebServerRequest* request, uint8_t* data, size_t len, 
                         size_t index, size_t total);
    void handlePointGet(AsyncWebServerRequest* request);
    void handleState(AsyncWebServerRequest* request);
    void handleCalibrate(AsyncWebServerRequest* request, uint8_t* data, size_t len,
                         size_t index, size_t total);
    void handleResetWifi(AsyncWebServerRequest* request);
    void handleWiFiScan(AsyncWebServerRequest* request);
    void handleI2CScan(AsyncWebServerRequest* request);
    void handleProvision(AsyncWebServerRequest* request, uint8_t* data, size_t len,
                         size_t index, size_t total);
    void handlePoints(AsyncWebServerRequest* request, uint8_t* data, size_t len,
                      size_t index, size_t total);
    void handleAnimate(AsyncWebServerRequest* request, uint8_t* data, size_t len,
                       size_t index, size_t total);
    void handleISS(AsyncWebServerRequest* request, uint8_t* data, size_t len,
                   size_t index, size_t total);
    void handleNotFound(AsyncWebServerRequest* request);
    
    // Helper functions
    void stopAllAnimations();
    bool fetchISSPosition(float& lat, float& lng);
    void updatePointsAnimation(unsigned long now);
    void updateSinusoidalAnimation(unsigned long now);
    void updateISSAnimation(unsigned long now);
    String getModeString() const;
    void sendJSON(AsyncWebServerRequest* request, int code, const JsonDocument& doc);
    void sendError(AsyncWebServerRequest* request, int code, const char* message);
};

#endif // HTTP_API_H

