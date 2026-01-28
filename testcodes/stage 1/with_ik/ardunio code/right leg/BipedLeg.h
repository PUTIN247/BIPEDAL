/*
  BipedLeg.h - Library for controlling a Zig-Zag leg
  Updated for Right Leg Mirroring
*/

#ifndef BIPEDLEG_H
#define BIPEDLEG_H

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <math.h>

class BipedLeg {
  public:
    // Constructor updated to accept isRightLeg flag
    BipedLeg(Adafruit_PWMServoDriver *driver, int hipCh, int kneeCh, int ankleCh, bool isRightLeg);

    void setServoRaw(int channel, float angle);
    void setOffsets(float hipOff, float kneeOff, float ankleOff);
    
    // Main Motion Function
    void moveToSmooth(float targetX, float targetY, int duration_ms);

    // Getters for position and angles
    float getX();
    float getY();
    float getHipAngle();
    float getKneeAngle();
    float getAnkleAngle();

  private:
    Adafruit_PWMServoDriver *_pwm;
    int _hipCh, _kneeCh, _ankleCh;
    bool _isRightLeg; // Flag to enable mirror logic

    float _currX, _currY;
    float _hipOffset, _kneeOffset, _ankleOffset;
    
    // Storage for logging
    float _lastHip, _lastKnee, _lastAnkle;

    // Physical Geometry (cm)
    const float L_FEMUR = 9.5;
    const float L_TIBIA = 9.5;

    // Servo Pulse Widths
    const int SERVOMIN = 150; 
    const int SERVOMAX = 600; 

    // Safety Constraints
    const float HIP_MIN   = -90.0; const float HIP_MAX   = 60.0;
    const float KNEE_MIN  = -90.0; const float KNEE_MAX  = 90.0; 
    const float ANKLE_MIN = -90.0; const float ANKLE_MAX = 70.0;

    bool calculateIK(float x, float y, float &tHip, float &tKnee, float &tAnkle);
};

#endif
