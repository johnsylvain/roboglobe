#ifndef GLOBE_MAPPING_H
#define GLOBE_MAPPING_H

#include <Arduino.h>

// Result structure for coordinate mapping
struct MappingResult {
    float lat_servo_deg;  // Computed latitude servo angle (0-180)
    float lng_servo_deg;  // Computed longitude servo angle (0-180)
    bool valid;           // Whether the mapping was successful
};

class GlobeMapping {
public:
    GlobeMapping();
    
    // Map latitude/longitude to servo angles
    // Latitude: -90 to +90 degrees
    // Longitude: -180 to +180 degrees (or 0-360)
    MappingResult mapCoordinates(float lat, float lng);
    
    // Validate input coordinates
    static bool validateLatitude(float lat);
    static bool validateLongitude(float lng);
    
    // Clamp coordinates to valid ranges
    static float clampLatitude(float lat);
    static float clampLongitude(float lng);

private:
    // Map latitude to servo angle (0-180 degrees)
    float mapLatitude(float lat);
    
    // Map longitude to servo angle (0-180 degrees) with 2:1 gear ratio
    float mapLongitude(float lng);
};

#endif // GLOBE_MAPPING_H

