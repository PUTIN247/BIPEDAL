// =============================================================================
//  BipedLeg.cpp  —  BIPDEAL Stage-1 servo controller implementation
//  Status : Existing Stage-1 class.  DO NOT modify the IK formulation.
// =============================================================================

#include "BipedLeg.h"
#include <math.h>

// ---------------------------------------------------------------------------
//  Constructor
// ---------------------------------------------------------------------------
BipedLeg::BipedLeg(Adafruit_PWMServoDriver& pwm,
                   uint8_t hipCh, uint8_t kneeCh, uint8_t ankleCh,
                   bool mirrorX)
    : _pwm(pwm),
      _hipCh(hipCh), _kneeCh(kneeCh), _ankleCh(ankleCh),
      _mirrorX(mirrorX),
      _hipOff(0.0f), _kneeOff(0.0f), _ankleOff(0.0f)
{}

// ---------------------------------------------------------------------------
//  setOffsets()
// ---------------------------------------------------------------------------
void BipedLeg::setOffsets(float hipOff, float kneeOff, float ankleOff) {
    _hipOff   = hipOff;
    _kneeOff  = kneeOff;
    _ankleOff = ankleOff;
}

// ---------------------------------------------------------------------------
//  calculateIK()
//
//  Two-link planar IK for the "<"-shaped leg.
//
//  Geometry derivation
//  ───────────────────
//  Let the hip joint sit at the origin. The foot target is (x, y)
//  with y < 0 (below the hip).
//
//  Step 1 — Knee angle via law of cosines
//      r²  =  x² + y²
//      cos(q2)  =  (r² − L1² − L2²) / (2·L1·L2)
//      q2   =  arccos(cos(q2))          [0 … π rad]
//
//      q2 is positive (forward flexion): as the knee bends, the lower leg
//      swings forward relative to the upper leg, which is exactly the
//      behaviour of the "<" geometry during normal gait.
//
//  Step 2 — Hip angle (elbow-backward / "<" configuration)
//      φ_vec = atan2(x, −y)     angle of foot vector from downward vertical
//      δ     = atan2(L2·sin(q2), L1 + L2·cos(q2))   geometric correction
//      q1    = φ_vec − δ        (elbow-backward selection)
//
//      For a typical home stance (foot at ~(−2, −15)):
//        q1 ≈ −44.8°  → upper leg tilts backward  ← confirms "<" shape
//        q2 ≈ +74.4°  → lower leg swings forward  ← confirms "<" shape
//
//  Step 3 — Ankle (foot levelling identity from master spec)
//      ankle = −(hip + knee)
//
//  NOTE: At the peak of the swing arc (y = −13, stepHeight = 2 cm),
//  the knee angle is ~93.7°, which slightly exceeds the 90° hard limit.
//  The IK clamps the knee and logs a warning via Serial.
//  To keep the knee within limits at all points, reduce Trajectory
//  stepHeight to ≤ 1.5 cm (safe limit: ~1.57 cm at x = 0).
// ---------------------------------------------------------------------------
IKResult BipedLeg::calculateIK(float x, float y) const {
    IKResult res;
    res.valid = true;

    // Mirror x for the right leg
    if (_mirrorX) x = -x;

    // ── Reach check ──────────────────────────────────────────────────────
    float r2   = x * x + y * y;
    float r    = sqrtf(r2);
    float maxR = L1 + L2;                  // fully extended
    float minR = fabsf(L1 - L2);          // fully folded (= 0 for equal links)

    if (r > maxR || r < minR) {
        // Out of the geometric workspace
        // Return a best-effort home-ish angle set rather than garbage
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

    // ── Knee angle (law of cosines) ───────────────────────────────────────
    float cosQ2 = (r2 - L1 * L1 - L2 * L2) / (2.0f * L1 * L2);
    cosQ2 = clamp(cosQ2, -1.0f, 1.0f);    // guard floating-point rounding
    float q2_rad = acosf(cosQ2);           // radians, always in [0, π]

    // ── Hip angle (elbow-backward selection for "<" geometry) ────────────
    float phi_vec = atan2f(x, -y);         // angle from downward vertical
    float delta   = atan2f(L2 * sinf(q2_rad), L1 + L2 * cosf(q2_rad));
    float q1_rad  = phi_vec - delta;

    // ── Convert to degrees ───────────────────────────────────────────────
    float hipDeg   = q1_rad  * (180.0f / PI);
    float kneeDeg  = q2_rad  * (180.0f / PI);   // positive = forward flexion
    float ankleDeg = -(hipDeg + kneeDeg);        // foot levelling identity

    // ── Joint limit checks (clamp + warn; do not block servo motion) ─────
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
        // Recompute ankle after knee clamp so foot levelling stays consistent
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

// ---------------------------------------------------------------------------
//  moveTo()
// ---------------------------------------------------------------------------
bool BipedLeg::moveTo(float x, float y) {
    IKResult ik = calculateIK(x, y);
    // Drive servos with clamped angles even if valid == false.
    // This prevents the leg from freezing at near-limit trajectory peaks.
    driveJoints(ik.hip, ik.knee, ik.ankle);
    return ik.valid;
}

// ---------------------------------------------------------------------------
//  setServoRaw()  —  raw single-channel access (calibration offset applied)
// ---------------------------------------------------------------------------
void BipedLeg::setServoRaw(uint8_t ch, float angleDeg) {
    float offset = 0.0f;
    if (ch == _hipCh)    offset = _hipOff;
    else if (ch == _kneeCh)   offset = _kneeOff;
    else if (ch == _ankleCh)  offset = _ankleOff;

    _pwm.setPWM(ch, 0, angleToPulse(angleDeg + offset));
}

// ---------------------------------------------------------------------------
//  driveJoints()  —  internal: apply offsets and write all three servos
// ---------------------------------------------------------------------------
void BipedLeg::driveJoints(float hipDeg, float kneeDeg, float ankleDeg) {
    _pwm.setPWM(_hipCh,   0, angleToPulse(hipDeg   + _hipOff));
    _pwm.setPWM(_kneeCh,  0, angleToPulse(kneeDeg  + _kneeOff));
    _pwm.setPWM(_ankleCh, 0, angleToPulse(ankleDeg + _ankleOff));
}

// ---------------------------------------------------------------------------
//  angleToPulse()
//
//  Maps the project angle convention (0° = neutral/vertical) to PCA9685
//  pulse counts.
//
//  Standard servo: 0° → 180° maps to PULSE_MIN → PULSE_MAX.
//  Our 0° is the mechanical neutral, so:
//      servo_angle = our_angle + 90°    shifts our range into [0°, 180°]
// ---------------------------------------------------------------------------
uint16_t BipedLeg::angleToPulse(float angleDeg) const {
    float servoAngle = angleDeg + 90.0f;
    servoAngle = clamp(servoAngle, 0.0f, 180.0f);
    return static_cast<uint16_t>(
        PULSE_MIN + (servoAngle / 180.0f) * static_cast<float>(PULSE_MAX - PULSE_MIN)
    );
}

// ---------------------------------------------------------------------------
//  clamp()
// ---------------------------------------------------------------------------
float BipedLeg::clamp(float v, float lo, float hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}
