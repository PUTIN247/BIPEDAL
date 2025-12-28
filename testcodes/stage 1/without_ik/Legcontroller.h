#ifndef LEGCONTROLLER_H
#define LEGCONTROLLER_H

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

class LegController {
  public:
    // Constructor: Pass the servo driver and channel numbers
    LegController(Adafruit_PWMServoDriver *driver, int hipCh, int kneeCh, int ankleCh);

    // Initialize: Calculates homes based on 90-degree neutral + offsets
    void begin();

    // Configuration
    void setOffsets(float hipOff, float kneeOff, float ankleOff);
    void setSpeed(float multiplier); // Adjust speed dynamically

    // Main Action: Runs one full walking cycle (Blocking with delay)
    void stepCycle();

    // Helper to manually move to a pose (useful for testing)
    void moveToPose(float hipDeg, float kneeDeg, float ankleDeg, int duration_ms);

  private:
    Adafruit_PWMServoDriver *_pwm;
    int _hipCh, _kneeCh, _ankleCh;

    // Tuning Parameters
    float _speedMultiplier;
    float _hipOffset, _kneeOffset, _ankleOffset;
    
    // Effective Home Positions (Calculated)
    float _hipHome, _kneeHome, _ankleHome;

    // Servo Pulse Settings
    const int SERVOMIN = 150;
    const int SERVOMAX = 600;

    // Physical Limits (Joint Space)
    const float HIP_MIN   = -45.0;
    const float HIP_MAX   = 45.0;
    const float KNEE_MIN  = 0.0;
    const float KNEE_MAX  = 90.0;
    const float ANKLE_MIN = -30.0;
    const float ANKLE_MAX = 30.0;

    // Smoothness Settings
    const int BASE_STEPS = 80;
    const int MIN_STEPS  = 20;

    // Gait Timing (ms)
    const int LIFT_TIME_MS    = 220;
    const int SWING_TIME_MS   = 240;
    const int PLACE_TIME_MS   = 200;
    const int RECOVER_TIME_MS = 200;
    const int CYCLE_DELAY_MS  = 120;

    // Internal Helpers
    void setServoAngle(int channel, float angleDeg);
    float angleConstrain(float servoAngle, float jointLo, float jointHi);
    void moveLegSmooth(float hFrom, float hTo, float kFrom, float kTo, float aFrom, float aTo, int duration_ms);
};

#endif
