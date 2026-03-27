#include "Bipedal.h"
#include <math.h>

BipedLeg::BipedLeg(Adafruit_PWMServoDriver& pwm,
                   uint8_t hipCh, uint8_t kneeCh, uint8_t ankleCh,
                   bool mirrorX)
    : _pwm(pwm),
      _hipCh(hipCh), _kneeCh(kneeCh), _ankleCh(ankleCh),
      _mirrorX(mirrorX),
      _hipOff(0.0f), _kneeOff(0.0f), _ankleOff(0.0f)
{}
void BipedLeg::setOffsets(float hipOff, float kneeOff, float ankleOff) {
    _hipOff   = hipOff;
    _kneeOff  = kneeOff;
    _ankleOff = ankleOff;
}


IKResult BipedLeg::calculateIK(float x, float y) const {
    IKResult res;
    res.valid = true;
    if (_mirrorX) x = -x;

    // ── Reach check 
    float r2   = x * x + y * y;
    float r    = sqrtf(r2);
    float maxR = L1 + L2;                  // fully extended
    float minR = fabsf(L1 - L2);          // fully folded (= 0 for equal links)

    if (r > maxR || r < minR) {
        res.hip   = 0.0f;
        res.knee  = 0.0f;
        res.ankle = 0.0f;
        res.valid = false;
        Serial.print(F("[IK] UNREACHABLE  r="));
        Serial.print(r, 2);
        Serial.print(F(" cm  target=("));
        Serial.print(x, 1); Serial.print(F(","));
        Serial.print(y, 1); Serial.println(F(")"));
        return res;
    }

    float cosQ2 = (r2 - L1 * L1 - L2 * L2) / (2.0f * L1 * L2);
    cosQ2 = clamp(cosQ2, -1.0f, 1.0f);    // guard floating-point rounding
    float q2_rad = acosf(cosQ2);           

    // ── Hip angle (elbow-backward selection for "<" geometry) ────────────
    float phi_vec = atan2f(x, -y);         // angle from downward vertical
    float delta   = atan2f(L2 * sinf(q2_rad), L1 + L2 * cosf(q2_rad));
    float q1_rad  = phi_vec - delta;

    // ── Convert to degrees ───────────────────────────────────────────────
    float hipDeg   = q1_rad  * (180.0f / PI);
    float kneeDeg  = q2_rad  * (180.0f / PI);  
    float ankleDeg = -(hipDeg + kneeDeg);       
  
    if (hipDeg < HIP_MIN || hipDeg > HIP_MAX) {
        Serial.print(F("[IK] Hip limit  "));
        Serial.print(hipDeg, 1); Serial.println(F(" deg — clamped"));
        hipDeg  = clamp(hipDeg, HIP_MIN, HIP_MAX);
        res.valid = false;
    }

    if (kneeDeg < KNEE_MIN || kneeDeg > KNEE_MAX) {
        Serial.print(F("[IK] Knee limit "));
        Serial.print(kneeDeg, 1); Serial.println(F(" deg — clamped"));
        kneeDeg  = clamp(kneeDeg, KNEE_MIN, KNEE_MAX);
        ankleDeg = -(hipDeg + kneeDeg);
        res.valid = false;
    }

    if (ankleDeg < ANKLE_MIN || ankleDeg > ANKLE_MAX) {
        Serial.print(F("[IK] Ankle limit "));
        Serial.print(ankleDeg, 1); Serial.println(F(" deg — clamped"));
        ankleDeg  = clamp(ankleDeg, ANKLE_MIN, ANKLE_MAX);
        res.valid = false;
    }

    res.hip   = hipDeg;
    res.knee  = kneeDeg;
    res.ankle = ankleDeg;
    return res;
}

bool BipedLeg::moveTo(float x, float y) {
    IKResult ik = calculateIK(x, y);
    driveJoints(ik.hip, ik.knee, ik.ankle);
    return ik.valid;
}

void BipedLeg::setServoRaw(uint8_t ch, float angleDeg) {
    float offset = 0.0f;
    if (ch == _hipCh)         offset = _hipOff;
    else if (ch == _kneeCh)   offset = _kneeOff;
    else if (ch == _ankleCh)  offset = _ankleOff;

    _pwm.setPWM(ch, 0, angleToPulse(angleDeg + offset));
}

void BipedLeg::driveJoints(float hipDeg, float kneeDeg, float ankleDeg) {
    _pwm.setPWM(_hipCh,   0, angleToPulse(hipDeg   + _hipOff));
    _pwm.setPWM(_kneeCh,  0, angleToPulse(kneeDeg  + _kneeOff));
    _pwm.setPWM(_ankleCh, 0, angleToPulse(ankleDeg + _ankleOff));
}

uint16_t BipedLeg::angleToPulse(float angleDeg) const {
    float servoAngle = angleDeg + 90.0f;
    servoAngle = clamp(servoAngle, 0.0f, 180.0f);
    return static_cast<uint16_t>(
        PULSE_MIN + (servoAngle / 180.0f) * static_cast<float>(PULSE_MAX - PULSE_MIN)
    );
}

float BipedLeg::clamp(float v, float lo, float hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}
