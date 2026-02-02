#include "globe_mapping.h"

GlobeMapping::GlobeMapping() {
}

MappingResult GlobeMapping::mapCoordinates(float lat, float lng) {
    MappingResult result;
    result.valid = false;
    
    // Validate inputs
    if (!validateLatitude(lat) || !validateLongitude(lng)) {
        return result;
    }
    
    // Clamp to valid ranges
    lat = clampLatitude(lat);
    lng = clampLongitude(lng);
    
    // Map to servo angles
    result.lat_servo_deg = mapLatitude(lat);
    result.lng_servo_deg = mapLongitude(lng);
    result.valid = true;
    
    return result;
}

bool GlobeMapping::validateLatitude(float lat) {
    return (lat >= -90.0f && lat <= 90.0f);
}

bool GlobeMapping::validateLongitude(float lng) {
    return (lng >= -180.0f && lng <= 180.0f);
}

float GlobeMapping::clampLatitude(float lat) {
    if (lat < -90.0f) return -90.0f;
    if (lat > 90.0f) return 90.0f;
    return lat;
}

float GlobeMapping::clampLongitude(float lng) {
    if (lng < -180.0f) return -180.0f;
    if (lng > 180.0f) return 180.0f;
    return lng;
}

float GlobeMapping::mapLatitude(float lat) {
    // Servo 1: latitude -90° to 90° maps to servo -90° to 90° (non-reversed)
    // lat -90° → servo 0°, lat 0° → servo 90°, lat 90° → servo 180°
    // Formula: servo_angle = lat + 90°
    float servoAngle = lat + 90.0f;
    
    // Ensure result is in 0-180 range
    if (servoAngle < 0.0f) servoAngle = 0.0f;
    if (servoAngle > 180.0f) servoAngle = 180.0f;
    
    return servoAngle;
}

float GlobeMapping::mapLongitude(float lng) {
    // Work directly with -180° to +180° range (no conversion needed)
    // lng is already clamped to -180° to +180° by clampLongitude()
    
    Serial.printf("[DEBUG] mapLongitude: input lng=%.4f (range -180 to +180)\n", lng);
    
    // Servo 2: longitude -180° to +180° maps to servo 0° to 180° (with 2:1 gear ratio)
    // With 2:1 gear ratio, 360° of globe rotation = 180° of servo rotation
    // lng -180° (Date Line West) → servo 0°
    // lng 0° (Greenwich) → servo 90°
    // lng +180° (Date Line East) → servo 180°
    // Formula: servo_angle = 90° + (lng / 2)
    // This maps -180° to 0°, 0° to 90°, +180° to 180°
    float servoAngle = 90.0f + (lng / 2.0f);
    
    // Fine-tuning offset: Tokyo (139.79°E) overshoots by ~12°, so adjust for eastern hemisphere
    // Apply a proportional correction: subtract ~8.6% of the offset from 90° for positive longitudes
    // This corrects the overshoot while keeping western hemisphere accurate
    if (lng > 0.0f) {
        // For eastern hemisphere, reduce the angle proportionally
        // At 139.79°E, we need to subtract ~12°, which is about 8.6% of 139.79°
        float correction = lng * 0.086f;  // Proportional correction factor
        servoAngle -= correction;
        Serial.printf("[DEBUG] mapLongitude: applied eastern hemisphere correction: -%.4f\n", correction);
    }
    
    Serial.printf("[DEBUG] mapLongitude: calculated servoAngle=%.4f\n", servoAngle);
    
    // Ensure result is in 0-180 range
    if (servoAngle < 0.0f) servoAngle = 0.0f;
    if (servoAngle > 180.0f) servoAngle = 180.0f;
    
    return servoAngle;
}

