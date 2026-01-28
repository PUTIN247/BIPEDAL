/*
  BipedLeg.h - Library for controlling a single Zig-Zag leg
  Target: Arduino Uno
*/

#ifndef BIPEDLEG_H
#define BIPEDLEG_H

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <math.h>

class BipedLeg {
  public:
    BipedLeg(Adafruit_PWMServoDriver *driver, int hipCh, int kneeCh, int ankleCh);

    void setServoRaw(int channel, float angle);
    void setOffsets(float hipOff, float kneeOff, float ankleOff);
    void moveToSmooth(float targetX, float targetY, int duration_ms);

    float getX();
    float getY();

    // NEW: Functions to read the current servo angles
    float getHipAngle();
    float getKneeAngle();
    float getAnkleAngle();

  private:
    Adafruit_PWMServoDriver *_pwm;
    int _hipCh, _kneeCh, _ankleCh;

    float _currX, _currY;
    float _hipOffset, _kneeOffset, _ankleOffset;
    
    // NEW: Variables to store the last calculated angles
    float _lastHip, _lastKnee, _lastAnkle;

    const float L_FEMUR = 9.5;
    const float L_TIBIA = 9.5;

    const int SERVOMIN = 150; 
    const int SERVOMAX = 600; 

    const float HIP_MIN   = -90.0; 
    const float HIP_MAX   = 60.0;
    const float KNEE_MIN  = -90.0;
    const float KNEE_MAX  = 90.0; 
    const float ANKLE_MIN = -90.0;
    const float ANKLE_MAX = 70.0;

    bool calculateIK(float x, float y, float &tHip, float &tKnee, float &tAnkle);
};

#endif
