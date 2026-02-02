#include "servo_driver.h"
#include <math.h>

ServoDriver::ServoDriver(ConfigStore& configStore) 
    : pwm(PCA9685_ADDRESS), configStore(configStore), initialized(false) {
    currentAngles[0] = 0.0f; // Will be set to 0° initially, then smoothly move to center
    currentAngles[1] = 0.0f;
    
    // Initialize jitter reduction tracking
    for (int i = 0; i < 2; i++) {
        lastPulseTicks[i] = 0;
        lastUpdateTime[i] = 0;
    }
    
    // Initialize smooth movement state
    for (int i = 0; i < 2; i++) {
        smoothMovements[i].active = false;
        smoothMovements[i].startAngle = 0.0f;
        smoothMovements[i].targetAngle = 90.0f;
        smoothMovements[i].startTime = 0;
        smoothMovements[i].durationMs = 1000;
    }
}

ServoDriver::~ServoDriver() {
}

bool ServoDriver::begin() {
    Serial.println("Initializing servo driver...");
    Serial.printf("I2C pins: SDA=%d, SCL=%d\n", I2C_SDA, I2C_SCL);
    Serial.printf("PCA9685 address: 0x%02X\n", PCA9685_ADDRESS);
    
    // Initialize I2C
    Wire.begin(I2C_SDA, I2C_SCL);
    delay(100); // Give I2C bus time to stabilize
    
    // Check if PCA9685 is present on I2C bus
    Wire.beginTransmission(PCA9685_ADDRESS);
    uint8_t error = Wire.endTransmission();
    if (error != 0) {
        Serial.printf("ERROR: PCA9685 not found at address 0x%02X (I2C error: %d)\n", PCA9685_ADDRESS, error);
        Serial.println("\nScanning I2C bus for devices...");
        scanI2CBus();
        Serial.println("\nTroubleshooting:");
        Serial.println("  1. Check I2C wiring: SDA->GPIO21, SCL->GPIO22");
        Serial.println("  2. Verify PCA9685 is powered (5V)");
        Serial.println("  3. Check I2C address jumpers (default: 0x40)");
        Serial.println("  4. Verify pull-up resistors on SDA/SCL (usually 4.7kΩ)");
        Serial.println("  5. If PCA9685 found at different address, update PCA9685_ADDRESS in servo_driver.h");
        return false;
    }
    
    // Initialize PCA9685 (begin() returns void, so we check I2C above)
    pwm.begin();
    
    // Set PWM frequency to 50Hz for servos
    pwm.setOscillatorFrequency(27000000);
    pwm.setPWMFreq(SERVO_PWM_FREQ);
    
    initialized = true;
    Serial.println("PCA9685 initialized successfully");
    
    // Smoothly move servos to center position (2500ms for gentle initial movement)
    // Start from current position (0°) and smoothly ease to center (90°)
    setServoAngleSmooth(SERVO_CHANNEL_LATITUDE, 90.0f, 2500);
    setServoAngleSmooth(SERVO_CHANNEL_LONGITUDE, 90.0f, 2500);
    
    Serial.println("Servos moving smoothly to center position...");
    
    return true;
}

bool ServoDriver::setServoAngle(uint8_t channel, float angle, bool force) {
    if (!initialized) {
        Serial.println("ERROR: ServoDriver not initialized");
        return false;
    }
    
    if (channel > 1) {
        Serial.printf("ERROR: Invalid servo channel %d\n", channel);
        return false;
    }
    
    // Clamp angle to valid range
    angle = clampAngle(angle);
    
    // Get current time (used for both checks and update)
    unsigned long now = millis();
    
    // Check deadband - only update if angle change is significant (unless forced)
    if (!force) {
        float angleDiff = fabs(angle - currentAngles[channel]);
        if (angleDiff < SERVO_DEADBAND_DEG) {
            // Angle change is too small, skip update to reduce jitter
            return true;
        }
        
        // Check minimum update interval to prevent too frequent updates
        if (now - lastUpdateTime[channel] < SERVO_MIN_UPDATE_MS) {
            // Too soon since last update, skip to reduce jitter
            return true;
        }
    }
    
    // Convert angle to PWM pulse width with calibration
    uint16_t pulseWidth = angleToPulseWidth(channel, angle);
    
    // Set PWM on PCA9685
    // PCA9685 uses 12-bit resolution (0-4095) for pulse width
    // We need to convert microseconds to ticks
    // At 50Hz, period is 20ms = 20000us
    // Resolution: 4096 ticks per period
    // Ticks per microsecond: 4096 / 20000 = 0.2048
    
    // Calculate pulse width in ticks
    uint16_t pulseTicks = (uint16_t)(pulseWidth * 4096.0f / 20000.0f);
    
    // Only update PWM if the value has changed significantly (reduces I2C traffic and jitter)
    // Compare with a small threshold to account for rounding differences
    uint16_t tickDiff = (pulseTicks > lastPulseTicks[channel]) ? 
                        (pulseTicks - lastPulseTicks[channel]) : 
                        (lastPulseTicks[channel] - pulseTicks);
    
    // Only update if pulse width changed by at least 2 ticks (roughly 0.1° servo movement)
    // Or if forced (e.g., when movement completes)
    if (tickDiff >= 2 || lastPulseTicks[channel] == 0 || force) {
        // Set the pulse width (on at 0, off at pulseTicks)
        pwm.setPWM(channel, 0, pulseTicks);
        lastPulseTicks[channel] = pulseTicks;
        lastUpdateTime[channel] = now;
    }
    
    // Store current angle (always update, even if PWM wasn't changed)
    currentAngles[channel] = angle;
    
    return true;
}

float ServoDriver::getServoAngle(uint8_t channel) {
    if (channel > 1) {
        return 0.0f;
    }
    return currentAngles[channel];
}

bool ServoDriver::setServoAngleSmooth(uint8_t channel, float targetAngle, unsigned long durationMs) {
    if (!initialized) {
        Serial.println("ERROR: ServoDriver not initialized");
        return false;
    }
    
    if (channel > 1) {
        Serial.printf("ERROR: Invalid servo channel %d\n", channel);
        return false;
    }
    
    // Clamp target angle
    targetAngle = clampAngle(targetAngle);
    
    // Initialize smooth movement
    smoothMovements[channel].active = true;
    smoothMovements[channel].startAngle = currentAngles[channel];
    smoothMovements[channel].targetAngle = targetAngle;
    smoothMovements[channel].startTime = millis();
    smoothMovements[channel].durationMs = durationMs;
    
    return true;
}

void ServoDriver::update() {
    if (!initialized) {
        return;
    }
    
    unsigned long currentTime = millis();
    
    for (uint8_t channel = 0; channel < 2; channel++) {
        if (!smoothMovements[channel].active) {
            continue;
        }
        
        SmoothMovement& movement = smoothMovements[channel];
        unsigned long elapsed = currentTime - movement.startTime;
        
        if (elapsed >= movement.durationMs) {
            // Movement complete, set to final position (force update to ensure exact target)
            setServoAngle(channel, movement.targetAngle, true);
            movement.active = false;
        } else {
            // Calculate eased progress (0.0 to 1.0)
            float progress = (float)elapsed / (float)movement.durationMs;
            float easedProgress = easeInOutCubic(progress);
            
            // Interpolate between start and target
            float angle = movement.startAngle + 
                         (movement.targetAngle - movement.startAngle) * easedProgress;
            
            // Set intermediate position
            setServoAngle(channel, angle);
        }
    }
}

bool ServoDriver::isMoving(uint8_t channel) const {
    if (channel > 1) {
        return false;
    }
    return smoothMovements[channel].active;
}

float ServoDriver::easeInOutCubic(float t) {
    // Clamp t to [0, 1]
    if (t <= 0.0f) return 0.0f;
    if (t >= 1.0f) return 1.0f;
    
    // Ease-in-out cubic: smooth acceleration and deceleration
    if (t < 0.5f) {
        return 4.0f * t * t * t;
    } else {
        float f = 2.0f * t - 2.0f;
        return 1.0f + f * f * f / 2.0f;
    }
}

uint16_t ServoDriver::angleToPulseWidth(uint8_t channel, float angle) {
    // Get calibration for this channel
    ServoCalibration cal;
    configStore.getServoCalibration(channel, cal);
    
    // Apply offset
    float calibratedAngle = angle + cal.offset_deg;
    
    // Clamp to 0-180 range after offset
    if (calibratedAngle < 0.0f) calibratedAngle = 0.0f;
    if (calibratedAngle > 180.0f) calibratedAngle = 180.0f;
    
    // If inverted, flip the angle
    if (cal.invert) {
        calibratedAngle = 180.0f - calibratedAngle;
    }
    
    // Linear interpolation: angle 0-180 maps to pulse_min_us to pulse_max_us
    float normalized = calibratedAngle / 180.0f;
    uint16_t pulseWidth = (uint16_t)(cal.pulse_min_us + 
                                      (cal.pulse_max_us - cal.pulse_min_us) * normalized);
    
    return pulseWidth;
}

float ServoDriver::clampAngle(float angle) {
    if (angle < 0.0f) return 0.0f;
    if (angle > 180.0f) return 180.0f;
    return angle;
}

void ServoDriver::scanI2CBus() {
    Serial.println("Scanning I2C bus (addresses 0x08-0x77)...");
    Serial.println("Address  Device");
    Serial.println("-------  ------");
    
    uint8_t foundCount = 0;
    for (uint8_t address = 0x08; address < 0x78; address++) {
        Wire.beginTransmission(address);
        uint8_t error = Wire.endTransmission();
        
        if (error == 0) {
            Serial.printf("  0x%02X   ", address);
            
            // Try to identify common devices
            if (address == 0x40) {
                Serial.print("PCA9685 (default)");
            } else if (address >= 0x40 && address <= 0x47) {
                Serial.print("PCA9685 (possible)");
            } else if (address == 0x48 || address == 0x49) {
                Serial.print("ADS1115/ADS1015 (ADC)");
            } else if (address == 0x50 || address == 0x51 || address == 0x52 || address == 0x53) {
                Serial.print("EEPROM");
            } else if (address == 0x68) {
                Serial.print("DS1307/DS3231 (RTC)");
            } else if (address == 0x76 || address == 0x77) {
                Serial.print("BME280/BMP280 (sensor)");
            } else {
                Serial.print("Unknown device");
            }
            
            Serial.println();
            foundCount++;
        } else if (error == 4) {
            Serial.printf("  0x%02X   Unknown error\n", address);
        }
        
        delay(10); // Small delay between scans
    }
    
    if (foundCount == 0) {
        Serial.println("  (none)  No I2C devices found!");
        Serial.println("\nPossible issues:");
        Serial.println("  - I2C wiring incorrect (SDA/SCL swapped or disconnected)");
        Serial.println("  - No devices powered or connected");
        Serial.println("  - Missing pull-up resistors (4.7kΩ on SDA/SCL to VCC)");
    } else {
        Serial.printf("\nFound %d device(s) on I2C bus\n", foundCount);
        Serial.println("\nNote: PCA9685 can be at addresses 0x40-0x47 depending on jumpers:");
        Serial.println("  A0 A1 A2  Address");
        Serial.println("  0  0  0   0x40 (default)");
        Serial.println("  1  0  0   0x41");
        Serial.println("  0  1  0   0x42");
        Serial.println("  1  1  0   0x43");
        Serial.println("  0  0  1   0x44");
        Serial.println("  1  0  1   0x45");
        Serial.println("  0  1  1   0x46");
        Serial.println("  1  1  1   0x47");
    }
}

String ServoDriver::scanI2CBusJSON() {
    // Initialize I2C if not already done (for standalone calls)
    static bool i2cInitialized = false;
    if (!i2cInitialized) {
        Wire.begin(I2C_SDA, I2C_SCL);
        delay(100);
        i2cInitialized = true;
    }
    
    String json = "{";
    json += "\"devices\":[";
    
    uint8_t foundCount = 0;
    bool firstDevice = true;
    
    for (uint8_t address = 0x08; address < 0x78; address++) {
        Wire.beginTransmission(address);
        uint8_t error = Wire.endTransmission();
        
        if (error == 0) {
            if (!firstDevice) json += ",";
            firstDevice = false;
            
            json += "{";
            json += "\"address\":\"0x";
            if (address < 16) json += "0";
            json += String(address, HEX);
            json += "\",";
            json += "\"address_dec\":";
            json += String(address);
            json += ",";
            
            // Identify device type
            String deviceType = "Unknown";
            bool isPCA9685 = false;
            
            if (address == 0x40) {
                deviceType = "PCA9685 (default)";
                isPCA9685 = true;
            } else if (address >= 0x40 && address <= 0x47) {
                deviceType = "PCA9685 (possible)";
                isPCA9685 = true;
            } else if (address == 0x48 || address == 0x49) {
                deviceType = "ADS1115/ADS1015 (ADC)";
            } else if (address == 0x50 || address == 0x51 || address == 0x52 || address == 0x53) {
                deviceType = "EEPROM";
            } else if (address == 0x68) {
                deviceType = "DS1307/DS3231 (RTC)";
            } else if (address == 0x76 || address == 0x77) {
                deviceType = "BME280/BMP280 (sensor)";
            }
            
            json += "\"type\":\"";
            json += deviceType;
            json += "\",";
            json += "\"is_pca9685\":";
            json += isPCA9685 ? "true" : "false";
            json += "}";
            
            foundCount++;
        }
        
        delay(5); // Small delay between scans
    }
    
    json += "],";
    json += "\"count\":";
    json += String(foundCount);
    json += ",";
    json += "\"expected_address\":\"0x";
    if (PCA9685_ADDRESS < 16) json += "0";
    json += String(PCA9685_ADDRESS, HEX);
    json += "\",";
    json += "\"found_pca9685\":";
    json += (foundCount > 0) ? "true" : "false";
    json += "}";
    
    return json;
}
