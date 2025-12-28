#include "LegController.h"

LegController::LegController(Adafruit_PWMServoDriver *driver, int hipCh, int kneeCh, int ankleCh) {
  _pwm = driver;
  _hipCh = hipCh;
  _kneeCh = kneeCh;
  _ankleCh = ankleCh;
  
  // Default defaults
  _speedMultiplier = 2.5;
  _hipOffset = 0;
  _kneeOffset = 0;
  _ankleOffset = 0;
}

void LegController::setOffsets(float hipOff, float kneeOff, float ankleOff) {
  _hipOffset = hipOff;
  _kneeOffset = kneeOff;
  _ankleOffset = ankleOff;
}

void LegController::setSpeed(float multiplier) {
  _speedMultiplier = multiplier;
}

void LegController::begin() {
  // 1. Calculate Effective Home (90 deg base + offset)
  _hipHome   = 90.0 + _hipOffset;
  _kneeHome  = 90.0 + _kneeOffset;
  _ankleHome = 90.0 + _ankleOffset;

  // 2. Initial Safety Constrain (Relative to 90 deg neutral)
  _hipHome   = constrain(_hipHome, 90.0 + HIP_MIN, 90.0 + HIP_MAX);
  _kneeHome  = constrain(_kneeHome, 90.0 + KNEE_MIN, 90.0 + KNEE_MAX);
  _ankleHome = constrain(_ankleHome, 90.0 + ANKLE_MIN, 90.0 + ANKLE_MAX);

  // 3. Move to Home immediately
  setServoAngle(_hipCh, _hipHome);
  setServoAngle(_kneeCh, _kneeHome);
  setServoAngle(_ankleCh, _ankleHome);
}

// --- LOW LEVEL HELPERS ---

void LegController::setServoAngle(int channel, float angleDeg) {
  float a = constrain(angleDeg, 0.0, 180.0);
  long pulse = map((int)a, 0, 180, SERVOMIN, SERVOMAX);
  _pwm->setPWM(channel, 0, pulse);
}

float LegController::angleConstrain(float servoAngle, float jointLo, float jointHi) {
  // Convert joint-centered limits to servo-centered (90-base) limits
  float lo = 90.0 + jointLo;
  float hi = 90.0 + jointHi;
  return constrain(servoAngle, lo, hi);
}

// --- MOTION LOGIC ---

void LegController::moveLegSmooth(float hFrom, float hTo, float kFrom, float kTo, float aFrom, float aTo, int duration_ms) {
  // Scale duration
  float durationScaled = duration_ms * _speedMultiplier;

  // Calculate steps
  int steps = max(MIN_STEPS, BASE_STEPS);
  steps = int(steps * _speedMultiplier);
  steps = max(steps, MIN_STEPS);

  float hipStep   = (hTo - hFrom) / float(steps);
  float kneeStep  = (kTo - kFrom) / float(steps);
  float ankleStep = (aTo - aFrom) / float(steps);
  
  float stepDelay = durationScaled / float(steps);

  float h = hFrom;
  float k = kFrom;
  float a = aFrom;

  for (int i = 0; i <= steps; ++i) {
    // Safety Constrain every single step
    float hCon = angleConstrain(h, HIP_MIN, HIP_MAX);
    float kCon = angleConstrain(k, KNEE_MIN, KNEE_MAX);
    float aCon = angleConstrain(a, ANKLE_MIN, ANKLE_MAX);

    setServoAngle(_hipCh, hCon);
    setServoAngle(_kneeCh, kCon);
    setServoAngle(_ankleCh, aCon);

    h += hipStep;
    k += kneeStep;
    a += ankleStep;

    delay((int)stepDelay);
  }
}

void LegController::moveToPose(float hipDeg, float kneeDeg, float ankleDeg, int duration_ms) {
  // Move from current implicit home logic isn't tracked globally here 
  // so we assume start from home for isolated moves or you track state externally.
  // For the Step Cycle, we track flows locally.
}

void LegController::stepCycle() {
  float h0 = _hipHome;
  float k0 = _kneeHome;
  float a0 = _ankleHome;

  // Phase 1: Lift (Bend knee 35, Raise ankle 18)
  // Note: Using the exact logic from your snippet
  float ankleLift = max(90.0 + ANKLE_MIN, a0 - 18.0);
  float kneeLift  = min(90.0 + KNEE_MAX, k0 + 35.0);
  float hipPrep   = h0; 
  
  moveLegSmooth(h0, hipPrep, k0, kneeLift, a0, ankleLift, LIFT_TIME_MS);

  // Phase 2: Swing (Hip forward 25)
  float hipForward = constrain(h0 + 25.0, 90.0 + HIP_MIN, 90.0 + HIP_MAX);
  moveLegSmooth(hipPrep, hipForward, kneeLift, kneeLift, ankleLift, ankleLift, SWING_TIME_MS);

  // Phase 3: Place (Straighten knee/ankle)
  moveLegSmooth(hipForward, hipForward, kneeLift, k0, ankleLift, a0, PLACE_TIME_MS);

  // Phase 4: Recover (Hip back to home)
  moveLegSmooth(hipForward, h0, k0, k0, a0, a0, RECOVER_TIME_MS);

  delay((int)(CYCLE_DELAY_MS * _speedMultiplier));
}
