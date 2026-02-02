#ifndef CONFIG_STORE_H
#define CONFIG_STORE_H

#include <Arduino.h>
#include <Preferences.h>

// Servo calibration structure
struct ServoCalibration {
    uint16_t pulse_min_us;  // Minimum pulse width in microseconds
    uint16_t pulse_max_us;  // Maximum pulse width in microseconds
    bool invert;            // Invert servo direction
    float offset_deg;       // Offset in degrees
};

// Default calibration values for SG90 servos
#define DEFAULT_PULSE_MIN_US 500
#define DEFAULT_PULSE_MAX_US 2500
#define DEFAULT_INVERT false
#define DEFAULT_OFFSET_DEG 0.0f

class ConfigStore {
public:
    ConfigStore();
    ~ConfigStore();
    
    // Initialize NVS
    bool begin();
    
    // Wi-Fi credentials
    bool hasWiFiCredentials();
    bool getWiFiSSID(char* ssid, size_t maxLen);
    bool getWiFiPassword(char* password, size_t maxLen);
    bool setWiFiCredentials(const char* ssid, const char* password);
    bool clearWiFiCredentials();
    
    // Servo calibration
    bool getServoCalibration(uint8_t channel, ServoCalibration& cal);
    bool setServoCalibration(uint8_t channel, const ServoCalibration& cal);
    void resetServoCalibration(uint8_t channel);
    
    // Get default calibration
    static ServoCalibration getDefaultCalibration();

private:
    Preferences prefs;
    static const char* NVS_NAMESPACE;
    static const char* KEY_WIFI_SSID;
    static const char* KEY_WIFI_PASSWORD;
    static const char* KEY_SERVO_CAL_PREFIX;
};

#endif // CONFIG_STORE_H

