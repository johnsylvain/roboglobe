#include "config_store.h"

const char* ConfigStore::NVS_NAMESPACE = "roboglobe";
const char* ConfigStore::KEY_WIFI_SSID = "wifi_ssid";
const char* ConfigStore::KEY_WIFI_PASSWORD = "wifi_pass";
const char* ConfigStore::KEY_SERVO_CAL_PREFIX = "servo_cal_";

ConfigStore::ConfigStore() {
}

ConfigStore::~ConfigStore() {
    prefs.end();
}

bool ConfigStore::begin() {
    return prefs.begin(NVS_NAMESPACE, false);
}

bool ConfigStore::hasWiFiCredentials() {
    return prefs.isKey(KEY_WIFI_SSID) && prefs.isKey(KEY_WIFI_PASSWORD);
}

bool ConfigStore::getWiFiSSID(char* ssid, size_t maxLen) {
    if (!prefs.isKey(KEY_WIFI_SSID)) {
        return false;
    }
    String ssidStr = prefs.getString(KEY_WIFI_SSID, "");
    if (ssidStr.length() == 0 || ssidStr.length() >= maxLen) {
        return false;
    }
    strncpy(ssid, ssidStr.c_str(), maxLen - 1);
    ssid[maxLen - 1] = '\0';
    return true;
}

bool ConfigStore::getWiFiPassword(char* password, size_t maxLen) {
    if (!prefs.isKey(KEY_WIFI_PASSWORD)) {
        return false;
    }
    String passStr = prefs.getString(KEY_WIFI_PASSWORD, "");
    if (passStr.length() >= maxLen) {
        return false;
    }
    strncpy(password, passStr.c_str(), maxLen - 1);
    password[maxLen - 1] = '\0';
    return true;
}

bool ConfigStore::setWiFiCredentials(const char* ssid, const char* password) {
    if (!ssid || strlen(ssid) == 0) {
        return false;
    }
    
    bool success = true;
    success = success && prefs.putString(KEY_WIFI_SSID, ssid);
    success = success && prefs.putString(KEY_WIFI_PASSWORD, password ? password : "");
    return success;
}

bool ConfigStore::clearWiFiCredentials() {
    bool success = true;
    success = success && prefs.remove(KEY_WIFI_SSID);
    success = success && prefs.remove(KEY_WIFI_PASSWORD);
    return success;
}

bool ConfigStore::getServoCalibration(uint8_t channel, ServoCalibration& cal) {
    char key[32];
    snprintf(key, sizeof(key), "%s%d", KEY_SERVO_CAL_PREFIX, channel);
    
    // Check if calibration exists
    if (!prefs.isKey(key)) {
        cal = getDefaultCalibration();
        return false; // Not found, but return default
    }
    
    // Read calibration values
    cal.pulse_min_us = prefs.getUShort((String(key) + "_min").c_str(), DEFAULT_PULSE_MIN_US);
    cal.pulse_max_us = prefs.getUShort((String(key) + "_max").c_str(), DEFAULT_PULSE_MAX_US);
    cal.invert = prefs.getBool((String(key) + "_inv").c_str(), DEFAULT_INVERT);
    cal.offset_deg = prefs.getFloat((String(key) + "_off").c_str(), DEFAULT_OFFSET_DEG);
    
    return true;
}

bool ConfigStore::setServoCalibration(uint8_t channel, const ServoCalibration& cal) {
    char key[32];
    snprintf(key, sizeof(key), "%s%d", KEY_SERVO_CAL_PREFIX, channel);
    
    bool success = true;
    success = success && prefs.putUShort((String(key) + "_min").c_str(), cal.pulse_min_us);
    success = success && prefs.putUShort((String(key) + "_max").c_str(), cal.pulse_max_us);
    success = success && prefs.putBool((String(key) + "_inv").c_str(), cal.invert);
    success = success && prefs.putFloat((String(key) + "_off").c_str(), cal.offset_deg);
    
    return success;
}

void ConfigStore::resetServoCalibration(uint8_t channel) {
    ServoCalibration defaultCal = getDefaultCalibration();
    setServoCalibration(channel, defaultCal);
}

ServoCalibration ConfigStore::getDefaultCalibration() {
    ServoCalibration cal;
    cal.pulse_min_us = DEFAULT_PULSE_MIN_US;
    cal.pulse_max_us = DEFAULT_PULSE_MAX_US;
    cal.invert = DEFAULT_INVERT;
    cal.offset_deg = DEFAULT_OFFSET_DEG;
    return cal;
}

