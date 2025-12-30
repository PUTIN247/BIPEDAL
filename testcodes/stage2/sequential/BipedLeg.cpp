/*
  BipedLeg.cpp - Implementation of Zig-Zag Leg Kinematics
*/

#include "BipedLeg.h"

// Constructor
BipedLeg::BipedLeg(Adafruit_PWMServoDriver *driver, int hipCh, int kneeCh, int ankleCh) {
  _pwm = driver;
  _hipCh = hipCh;
  _kneeCh = kneeCh;
  _ankleCh = ankleCh;
  
  // Default start position (Leg hanging down)
  _currX = 0;
  _currY = -14.0;
  
  _hipOffset = 0;
  _kneeOffset = 0;
  _ankleOffset = 0;
}

void BipedLeg::setOffsets(float hipOff, float kneeOff, float ankleOff) {
  _hipOffset = hipOff;
  _kneeOffset = kneeOff;
  _ankleOffset = ankleOff;
}

float BipedLeg::getX() { return _currX; }
float BipedLeg::getY() { return _currY; }

// --- LOW LEVEL SERVO CONTROL ---
void BipedLeg::setServoRaw(int channel, float angle) {
  // Constrain to physical servo limits (0-180)
  int a = constrain((int)angle, 0, 180);
  int pulse = map(a, 0, 180, SERVOMIN, SERVOMAX);
  _pwm->setPWM(channel, 0, pulse);
}

// --- INVERSE KINEMATICS (IK) ---
// Solves angles for Custom Zig-Zag Geometry
bool BipedLeg::calculateIK(float x, float y, float &tHip, float &tKnee, float &tAnkle) {
  
  float dist = sqrt(x*x + y*y);

  // Safety: Reach Check
  if (dist > (L_FEMUR + L_TIBIA - 0.5)) return false; // Too far
  if (dist < 4.0) return false; // Too close (self-collision)

  // 1. KNEE (Law of Cosines)
  // Internal angle gamma
  float cosGamma = (L_FEMUR*L_FEMUR + L_TIBIA*L_TIBIA - dist*dist) / (2 * L_FEMUR * L_TIBIA);
  cosGamma = constrain(cosGamma, -1.0, 1.0); // Floating point safety
  float gamma = acos(cosGamma);

  // Knee Deviation from Straight (PI - gamma)
  float kneeRad = PI - gamma; 

  // 2. HIP
  // Alpha: Angle of vector Hip->Foot from Vertical
  // using -y because down is negative in our coords
  float alpha = atan2(x, -y); 
  
  // Beta: Internal angle at Hip
  float cosBeta = (L_FEMUR*L_FEMUR + dist*dist - L_TIBIA*L_TIBIA) / (2 * L_FEMUR * dist);
  cosBeta = constrain(cosBeta, -1.0, 1.0);
  float beta = acos(cosBeta);

  // ZIG-ZAG LOGIC: Hip angle is Alpha - Beta
  float hipRad = alpha - beta;

  // 3. ANKLE (Auto-leveling)
  // To keep foot horizontal (0 deg global), Ankle cancels out Hip + Knee
  float ankleRad = -(hipRad + kneeRad);

  // Convert to Degrees
  tHip   = hipRad * 180.0 / PI;
  tKnee  = kneeRad * 180.0 / PI;
  tAnkle = ankleRad * 180.0 / PI;

  return true;
}

// --- SMOOTH MOVEMENT INTERPOLATOR ---
void BipedLeg::moveToSmooth(float targetX, float targetY, int duration_ms) {
  float startX = _currX;
  float startY = _currY;
  
  // Define time steps (10ms resolution)
  int steps = duration_ms / 10;
  if (steps < 1) steps = 1;

  for (int i = 1; i <= steps; i++) {
    float t = (float)i / steps;
    
    // Linear Interpolation
    float nowX = startX + (targetX - startX) * t;
    float nowY = startY + (targetY - startY) * t;

    float h, k, a;
    
    // Solve IK for intermediate point
    if (calculateIK(nowX, nowY, h, k, a)) {
      
      // Apply Safety Constraints (The "Guard" Logic)
      h = constrain(h, HIP_MIN, HIP_MAX);
      k = constrain(k, KNEE_MIN, KNEE_MAX); // Strictly protect 90 deg limit
      a = constrain(a, ANKLE_MIN, ANKLE_MAX);

      // Add 90.0 (Neutral) + Calibration Offsets
      setServoRaw(_hipCh,   90.0 + h + _hipOffset);
      setServoRaw(_kneeCh,  90.0 + k + _kneeOffset);
      setServoRaw(_ankleCh, 90.0 + a + _ankleOffset);
    }
    
    // Blocking delay (For Stage 1 simplicity)
    delay(10);
  }
  
  // Update state
  _currX = targetX;
  _currY = targetY;
}
