/*
  BipedLeg.cpp - Implementation with Mirror Logic
*/

#include "BipedLeg.h"

// Constructor
BipedLeg::BipedLeg(Adafruit_PWMServoDriver *driver, int hipCh, int kneeCh, int ankleCh, bool isRightLeg) {
  _pwm = driver;
  _hipCh = hipCh;
  _kneeCh = kneeCh;
  _ankleCh = ankleCh;
  _isRightLeg = isRightLeg; // Store the Right Leg Flag
  
  _currX = 0;
  _currY = -14.0;
  
  _hipOffset = 0; _kneeOffset = 0; _ankleOffset = 0;
  _lastHip = 0; _lastKnee = 0; _lastAnkle = 0;
}

void BipedLeg::setOffsets(float hipOff, float kneeOff, float ankleOff) {
  _hipOffset = hipOff;
  _kneeOffset = kneeOff;
  _ankleOffset = ankleOff;
}

float BipedLeg::getX() { return _currX; }
float BipedLeg::getY() { return _currY; }
float BipedLeg::getHipAngle()   { return _lastHip; }
float BipedLeg::getKneeAngle()  { return _lastKnee; }
float BipedLeg::getAnkleAngle() { return _lastAnkle; }

void BipedLeg::setServoRaw(int channel, float angle) {
  int a = constrain((int)angle, 0, 180);
  int pulse = map(a, 0, 180, SERVOMIN, SERVOMAX);
  _pwm->setPWM(channel, 0, pulse);
}

bool BipedLeg::calculateIK(float x, float y, float &tHip, float &tKnee, float &tAnkle) {
  float dist = sqrt(x*x + y*y);

  if (dist > (L_FEMUR + L_TIBIA - 0.5)) return false; 
  if (dist < 4.0) return false; 

  // Knee
  float cosGamma = (L_FEMUR*L_FEMUR + L_TIBIA*L_TIBIA - dist*dist) / (2 * L_FEMUR * L_TIBIA);
  cosGamma = constrain(cosGamma, -1.0, 1.0);
  float gamma = acos(cosGamma);
  float kneeRad = PI - gamma; 

  // Hip
  float alpha = atan2(x, -y); 
  float cosBeta = (L_FEMUR*L_FEMUR + dist*dist - L_TIBIA*L_TIBIA) / (2 * L_FEMUR * dist);
  cosBeta = constrain(cosBeta, -1.0, 1.0);
  float beta = acos(cosBeta);
  float hipRad = alpha - beta;

  // Ankle
  float ankleRad = -(hipRad + kneeRad);

  tHip   = hipRad * 180.0 / PI;
  tKnee  = kneeRad * 180.0 / PI;
  tAnkle = ankleRad * 180.0 / PI;

  return true;
}

void BipedLeg::moveToSmooth(float targetX, float targetY, int duration_ms) {
  float startX = _currX;
  float startY = _currY;
  
  int steps = duration_ms / 10;
  if (steps < 1) steps = 1;

  for (int i = 1; i <= steps; i++) {
    float t = (float)i / steps;
    float nowX = startX + (targetX - startX) * t;
    float nowY = startY + (targetY - startY) * t;

    float h, k, a;
    
    if (calculateIK(nowX, nowY, h, k, a)) {
      h = constrain(h, HIP_MIN, HIP_MAX);
      k = constrain(k, KNEE_MIN, KNEE_MAX);
      a = constrain(a, ANKLE_MIN, ANKLE_MAX);

      float finalHip, finalKnee, finalAnkle;

      // --- MIRROR LOGIC ---
      if (_isRightLeg) {
        // Right Leg: 90 - Angle (Inverted)
        finalHip   = 90.0 - h;
        finalKnee  = 90.0 - k;
        finalAnkle = 90.0 - a;
      } else {
        // Left Leg: 90 + Angle (Standard)
        finalHip   = 90.0 + h;
        finalKnee  = 90.0 + k;
        finalAnkle = 90.0 + a;
      }

      setServoRaw(_hipCh,   finalHip   + _hipOffset);
      setServoRaw(_kneeCh,  finalKnee  + _kneeOffset);
      setServoRaw(_ankleCh, finalAnkle + _ankleOffset);

      _lastHip = h; _lastKnee = k; _lastAnkle = a;
    }
    delay(10);
  }
  _currX = targetX;
  _currY = targetY;
}
