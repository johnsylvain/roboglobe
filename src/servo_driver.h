#ifndef SERVO_DRIVER_H
#define SERVO_DRIVER_H

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include "config_store.h"

// Servo channel definitions
#define SERVO_CHANNEL_LATITUDE  0
#define SERVO_CHANNEL_LONGITUDE 1

// PCA9685 I2C address
#define PCA9685_ADDRESS 0x40

// I2C pins for ESP32
#define I2C_SDA 23
#define I2C_SCL 22

// PWM frequency for servos (50Hz standard)
#define SERVO_PWM_FREQ 50

// Deadband threshold to reduce jitter (degrees)
// Only update servo if angle change exceeds this threshold
#define SERVO_DEADBAND_DEG 0.5f

// Minimum time between PWM updates (milliseconds) to reduce jitter
#define SERVO_MIN_UPDATE_MS 20

class ServoDriver {
public:
    ServoDriver(ConfigStore& configStore);
    ~ServoDriver();
    
    // Initialize I2C and PCA9685
    bool begin();
    
    // Set servo angle (0-180 degrees) with calibration applied
    // force: if true, bypass deadband and update interval checks (use when movement completes)
    bool setServoAngle(uint8_t channel, float angle, bool force = false);
    
    // Set servo angle smoothly with easing (non-blocking, call update() in loop)
    bool setServoAngleSmooth(uint8_t channel, float targetAngle, unsigned long durationMs = 1000);
    
    // Update smooth servo movements (call this in loop())
    void update();
    
    // Check if a servo is currently moving
    bool isMoving(uint8_t channel) const;
    
    // Get current angle (last set value)
    float getServoAngle(uint8_t channel);
    
    // Check if driver is initialized
    bool isInitialized() const { return initialized; }
    
    // Scan I2C bus for devices (static method, can be called without instance)
    static void scanI2CBus();
    
    // Scan I2C bus and return results as JSON string
    static String scanI2CBusJSON();

private:
    Adafruit_PWMServoDriver pwm;
    ConfigStore& configStore;
    bool initialized;
    float currentAngles[2]; // Track current angles for channels 0 and 1
    uint16_t lastPulseTicks[2]; // Track last PWM pulse width sent to reduce jitter
    unsigned long lastUpdateTime[2]; // Track last update time for each channel
    
    // Smooth movement state
    struct SmoothMovement {
        bool active;
        float startAngle;
        float targetAngle;
        unsigned long startTime;
        unsigned long durationMs;
    };
    SmoothMovement smoothMovements[2];
    
    // Easing function (ease-in-out cubic)
    static float easeInOutCubic(float t);
    
    // Convert angle to PWM pulse width with calibration
    uint16_t angleToPulseWidth(uint8_t channel, float angle);
    
    // Clamp angle to valid range
    float clampAngle(float angle);
};

#endif // SERVO_DRIVER_H

