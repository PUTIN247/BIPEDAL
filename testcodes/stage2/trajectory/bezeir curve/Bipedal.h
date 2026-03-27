#pragma once
#include <Arduino.h>
#include <Adafruit_PWMServoDriver.h>


struct IKResult {
    float hip;      
    float knee;    
    float ankle;    
    bool  valid;    
};

class BipedLeg {
public:
    static constexpr float L1 = 9.5f;   // hip  → knee
    static constexpr float L2 = 9.5f;   // knee → ankle

    // ---- Joint limits (degrees) 
    static constexpr float HIP_MIN    = -90.0f;
    static constexpr float HIP_MAX    =  60.0f;
    static constexpr float KNEE_MIN   = -90.0f;
    static constexpr float KNEE_MAX   =  90.0f;   // MUST NEVER be exceeded
    static constexpr float ANKLE_MIN  = -90.0f;
    static constexpr float ANKLE_MAX  =  70.0f;

    static constexpr uint16_t PULSE_MIN = 150;
    static constexpr uint16_t PULSE_MAX = 600;

    //  Construction 
    //  pwm      : shared Adafruit_PWMServoDriver instance (already begin()'d)
    //  hipCh    : PCA9685 channel assigned to the hip servo
    //  kneeCh   : PCA9685 channel assigned to the knee servo
    //  ankleCh  : PCA9685 channel assigned to the ankle servo
    //  mirrorX  : set true for the right leg to mirror the x-axis
    BipedLeg(Adafruit_PWMServoDriver& pwm,
             uint8_t hipCh, uint8_t kneeCh, uint8_t ankleCh,
             bool mirrorX = false);

    void setOffsets(float hipOff, float kneeOff, float ankleOff);

    IKResult calculateIK(float x, float y) const;

    // ---- Drive leg to (x, y)
    bool moveTo(float x, float y);

    // ---- Raw servo access  
    void setServoRaw(uint8_t ch, float angleDeg);

private:
    Adafruit_PWMServoDriver& _pwm;
    uint8_t  _hipCh, _kneeCh, _ankleCh;
    bool     _mirrorX;
    float    _hipOff, _kneeOff, _ankleOff;

    static float    clamp(float v, float lo, float hi);
    uint16_t        angleToPulse(float angleDeg) const;
    void            driveJoints(float hipDeg, float kneeDeg, float ankleDeg);
};
