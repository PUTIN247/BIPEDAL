// =============================================================================
//  BipedLeg.h  —  BIPDEAL Bipedal Robot, Single-Leg Servo Controller
//  Status : Existing Stage-1 class.  DO NOT modify the IK formulation.
// =============================================================================
//
//  Coordinate frame  (origin = hip joint centre):
//    x  →  positive = forward in direction of motion
//    y  →  positive = upward  (ground line sits at y = -15 cm)
//
//  Leg geometry  ("< "-shaped, backward-bending knee):
//    Hip→Knee  slopes downward AND backward  (upper link)
//    Knee→Ankle slopes downward AND forward  (lower link)
//    L1 = L2 = 9.5 cm
//
//  Angle convention:
//    0°  = leg perfectly straight, vertical, all joints colinear
//    +ve = forward flexion   (direction of intended motion)
//    −ve = backward extension
//
//  Foot levelling identity:
//    ankle = −(hip + knee)   keeps the foot parallel to the ground
//
// =============================================================================

#pragma once
#include <Arduino.h>
#include <Adafruit_PWMServoDriver.h>

// ---------------------------------------------------------------------------
//  IKResult  —  returned by BipedLeg::calculateIK()
// ---------------------------------------------------------------------------
struct IKResult {
    float hip;      // degrees  hip angle
    float knee;     // degrees  knee angle  (+ = forward flexion)
    float ankle;    // degrees  ankle angle (auto-computed: -(hip+knee))
    bool  valid;    // false  if target is unreachable OR any limit is exceeded
                    // Angles are CLAMPED to limits even when valid==false,
                    // so servos can still be driven to the closest reachable pose.
};

// ---------------------------------------------------------------------------
//  BipedLeg
// ---------------------------------------------------------------------------
class BipedLeg {
public:
    // ---- Link lengths (cm) — match hardware; never change these ----------
    static constexpr float L1 = 9.5f;   // hip  → knee
    static constexpr float L2 = 9.5f;   // knee → ankle

    // ---- Joint limits (degrees) ------------------------------------------
    static constexpr float HIP_MIN    = -90.0f;
    static constexpr float HIP_MAX    =  60.0f;
    static constexpr float KNEE_MIN   = -90.0f;
    static constexpr float KNEE_MAX   =  90.0f;   // MUST NEVER be exceeded
    static constexpr float ANKLE_MIN  = -90.0f;
    static constexpr float ANKLE_MAX  =  70.0f;

    // ---- PCA9685 pulse-width range ---------------------------------------
    // Adjust to match your servo's physical 0°–180° pulse window.
    // Typical MG996R / SG90:  PULSE_MIN ≈ 150, PULSE_MAX ≈ 600
    static constexpr uint16_t PULSE_MIN = 150;
    static constexpr uint16_t PULSE_MAX = 600;

    // ---- Construction ----------------------------------------------------
    //  pwm      : shared Adafruit_PWMServoDriver instance (already begin()'d)
    //  hipCh    : PCA9685 channel assigned to the hip servo
    //  kneeCh   : PCA9685 channel assigned to the knee servo
    //  ankleCh  : PCA9685 channel assigned to the ankle servo
    //  mirrorX  : set true for the right leg to mirror the x-axis
    BipedLeg(Adafruit_PWMServoDriver& pwm,
             uint8_t hipCh, uint8_t kneeCh, uint8_t ankleCh,
             bool mirrorX = false);

    // ---- Calibration offsets -------------------------------------------
    //  Added to each computed angle before converting to pulse counts.
    //  Trim until every joint reads 0° (leg straight & vertical) at home.
    void setOffsets(float hipOff, float kneeOff, float ankleOff);

    // ---- Kinematics (pure computation, no servo motion) ----------------
    //  Solves two-link planar IK for the foot at (x, y).
    //  Returns angles in the project angle convention.
    //  result.valid  = true  ↔  target reachable AND all limits satisfied.
    //  result.valid  = false ↔  out of workspace or a limit was hit.
    //                           Clamped angles are still stored in the result.
    IKResult calculateIK(float x, float y) const;

    // ---- Drive leg to (x, y) -------------------------------------------
    //  Calls calculateIK, applies calibration offsets, drives all three servos.
    //  Returns true if target was fully reachable (valid IK, no limits hit).
    //  Even on false, servos are driven to the best clamped pose.
    bool moveTo(float x, float y);

    // ---- Raw servo access  ---------------------------------------------
    //  Drives ONE PCA9685 channel to angleDeg in the project convention.
    //  The calibration offset for that channel is applied automatically.
    //  Use sparingly — prefer moveTo() for normal gait control.
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
