/*
  BipedLeg.h - Library for controlling a single Zig-Zag leg
  Project: Experimental Biped
  Stage: 1 (Single Leg IK)
*/

#ifndef BIPEDLEG_H
#define BIPEDLEG_H

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <math.h>

class BipedLeg {
  public:
    // Constructor: defined by the servo driver and specific channels
    BipedLeg(Adafruit_PWMServoDriver *driver, int hipCh, int kneeCh, int ankleCh);

    void setServoRaw(int channel, float angle);

    // Setup function to initialize offsets
    //offset angles with reference to 0 -- measure the offset angles here 
    void setOffsets(float hipOff, float kneeOff, float ankleOff);

    // Main Motion Function: Move foot to (x, y) over time
    void moveToSmooth(float targetX, float targetY, int duration_ms);

    // Returns current position
    float getX();
    float getY();

  private:
    // Pointers and Hardware
    Adafruit_PWMServoDriver *_pwm;
    int _hipCh, _kneeCh, _ankleCh;

    // State Variables
    float _currX, _currY;
    
    // Calibration Offsets
    float _hipOffset, _kneeOffset, _ankleOffset;

    // Physical Geometry (cm)
    const float L_FEMUR = 9.5;
    const float L_TIBIA = 9.5;

    // Servo limits (Pulse Width)
    const int SERVOMIN = 150;  //0 degrees
    const int SERVOMAX = 600;  //180 degrees

    // Safety Constraints (Degrees relative to Neutral)
    const float HIP_MIN   = -45.0; 
    const float HIP_MAX   = 45.0;
    const float KNEE_MIN  = 0.0;
    const float KNEE_MAX  = 90.0; // Hard mechanical limit
    const float ANKLE_MIN = -40.0;
    const float ANKLE_MAX = 40.0;

    // Private Helpers
    bool calculateIK(float x, float y, float &tHip, float &tKnee, float &tAnkle);
};

#endif
