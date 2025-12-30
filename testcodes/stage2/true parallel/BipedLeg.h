#ifndef BIPEDLEG_H
#define BIPEDLEG_H

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <math.h>

class BipedLeg {
public:
    // Constructor
    BipedLeg(Adafruit_PWMServoDriver *driver, int hipCh, int kneeCh, int ankleCh, bool isRightLeg);

    // Calibration
    void setOffsets(float hipOff, float kneeOff, float ankleOff);
    
    // CONTROL: The New Parallel Logic
    // 1. Tell the leg where to go and how long to take (Non-blocking)
    void setTarget(float targetX, float targetY, unsigned long duration_ms);
    
    // 2. Call this inside loop() constantly to move the servos
    // Returns true if the leg has reached its target
    bool update();

    // Instant Teleport (only for setup)
    void setPositionSingle(float x, float y);

    // Getters
    float getX() { return _currX; }
    float getY() { return _currY; }
    bool isMoving() { return _isMoving; }

private:
    Adafruit_PWMServoDriver *_pwm;
    int _hipCh, _kneeCh, _ankleCh;
    bool _isRightLeg; // To handle mirroring if needed later

    // Current State
    float _currX, _currY;
    
    // Motion Trajectory Variables
    float _startX, _startY;
    float _targetX, _targetY;
    unsigned long _moveStartTime;
    unsigned long _moveDuration;
    bool _isMoving;

    // Calibration Offsets
    float _hipOffset, _kneeOffset, _ankleOffset;

    // Dimensions & Limits (From your Context)
    const float L_FEMUR = 9.5;
    const float L_TIBIA = 9.5;
    
    // Constraints (Degrees relative to Neutral 90)
    const float HIP_MIN = -45.0, HIP_MAX = 45.0;
    const float KNEE_MIN = 0.0, KNEE_MAX = 90.0;
    const float ANKLE_MIN = -45.0, ANKLE_MAX = 45.0;

    // Servo Pulse Limits
    const int SERVOMIN = 150;
    const int SERVOMAX = 600;

    // Helper: The "Brain"
    bool calculateIK(float x, float y, float &tHip, float &tKnee, float &tAnkle);
    void setServoRaw(int channel, float angle);
};

#endif
