// =============================================================================
//  GaitController.h  —  BIPDEAL Stage-2 Two-Leg Gait Coordinator
// =============================================================================
//
//  Manages the three-stage startup sequence and continuous walking gait
//  for both legs.  Each leg is driven by the same Trajectory object;
//  the right leg's phase is always 0.5 ahead of the left leg's phase.
//
//  State machine
//  ─────────────────────────────────────────────────────────────────────────
//
//  HOMING
//    Both legs smoothly gather from their assumed power-on positions
//    toward the symmetric home position (homeX, groundY).
//    Assumed start:  left = P0 (-4, -15),  right = P3 (3, -15).
//    These are the natural resting positions after a Stage-1 run and form
//    a stable standing pose; the "gather" motion is visually clean.
//    Duration: HOME_MS  (smoothstep easing applied).
//
//  PRE_GAIT
//    From home, left leg moves to P0 and right leg moves to P3.
//    This creates the initial left-back / right-forward asymmetry required
//    so that φ = 0 maps the left leg exactly to P0 and the right leg to P3
//    with zero position discontinuity.
//    Duration: PRE_GAIT_MS  (smoothstep easing applied).
//
//  WALKING
//    φ advances monotonically from millis().
//    Left  leg : φ_L = φ
//    Right leg : φ_R = (φ + 0.5) mod 1.0
//    Both legs are updated on every call to update().
//
//  Phase-to-position handoff at WALKING entry
//  ─────────────────────────────────────────────────────────────────────────
//    φ = 0.0 → traj.getFootPosition(0.0) = P0 (-4, -15)  ← left  at PRE_GAIT end ✓
//    φ+0.5   → traj.getFootPosition(0.5) = P3 ( 3, -15)  ← right at PRE_GAIT end ✓
//    No servo jump at the PRE_GAIT → WALKING transition.
//
//  Smooth interpolation
//  ─────────────────────────────────────────────────────────────────────────
//    HOMING and PRE_GAIT use cubic smoothstep:  f(t) = t²(3 − 2t).
//    This gives zero velocity at both endpoints (ease-in / ease-out)
//    using only 3 multiplications — no transcendental functions.
//
//  Foot position tracking
//  ─────────────────────────────────────────────────────────────────────────
//    The controller tracks the last commanded foot position for both legs.
//    Use getLeftFoot() / getRightFoot() for serial telemetry in the sketch.
//
// =============================================================================

#pragma once
#include <Arduino.h>
#include "Bipedal.h"
#include "Trajectory.h"

// ---------------------------------------------------------------------------
//  GaitController
// ---------------------------------------------------------------------------
class GaitController {
public:

    // ── Timing constants ─────────────────────────────────────────────────────
    //  These control the startup durations and gait speed.
    //  Geometry and phase logic are completely independent of these values.
    static constexpr uint32_t HOME_MS     =  800UL;   // both legs → home
    static constexpr uint32_t PRE_GAIT_MS =  600UL;   // home → (P0 , P3)
    static constexpr uint32_t CYCLE_MS    = 2000UL;   // one full gait cycle

    // ── State ─────────────────────────────────────────────────────────────────
    enum class State : uint8_t {
        HOMING   = 0,   // gathering both legs to home position
        PRE_GAIT = 1,   // spreading legs to initial step configuration
        WALKING  = 2    // continuous phase-based gait
    };

    // ── Construction ──────────────────────────────────────────────────────────
    //  left  : left  BipedLeg instance (mirrorX = false)
    //  right : right BipedLeg instance (mirrorX = true)
    //  traj  : shared Trajectory object — must remain valid for the lifetime
    //          of this GaitController
    GaitController(BipedLeg& left, BipedLeg& right, const Trajectory& traj);

    // ── Lifecycle ─────────────────────────────────────────────────────────────
    //  Call begin() once in setup() after the PCA9685 is initialised.
    //  Call update() on every loop() iteration that passes the rate limiter.
    //  Returns the current state so the sketch can log state transitions.
    void  begin();
    State update();

    // ── Accessors ─────────────────────────────────────────────────────────────
    State        getState()     const { return _state;      }
    bool         isWalking()    const { return _state == State::WALKING; }

    //  Master gait phase [0, 1) — meaningful only during WALKING.
    //  During HOMING / PRE_GAIT this returns 0.
    float        getPhase()     const { return _phi;        }

    //  Last commanded foot position for each leg.
    //  Valid in all states — use for serial telemetry.
    FootPosition getLeftFoot()  const { return _leftCurr;   }
    FootPosition getRightFoot() const { return _rightCurr;  }

private:

    BipedLeg&         _left;
    BipedLeg&         _right;
    const Trajectory& _traj;

    State         _state;
    uint32_t      _stateStart;    // millis() snapshot when current state began
    float         _phi;           // master gait phase [0, 1)

    // Interpolation endpoints for HOMING and PRE_GAIT transitions
    FootPosition  _leftFrom,  _rightFrom;
    FootPosition  _leftTo,    _rightTo;

    // Last commanded positions — updated by _moveBoth() on every servo write
    FootPosition  _leftCurr,  _rightCurr;

    // ── State entry helpers ───────────────────────────────────────────────────
    void _enterHoming();
    void _enterPreGait();
    void _enterWalking();

    // ── Per-state update handlers ─────────────────────────────────────────────
    void _doHoming  (uint32_t now);
    void _doPreGait (uint32_t now);
    void _doWalking (uint32_t now);

    // ── Servo write helper ────────────────────────────────────────────────────
    //  Drives both legs to (lp, rp) and updates the cached current positions.
    void _moveBoth(const FootPosition& lp, const FootPosition& rp);

    // ── Pure maths helpers (no side-effects) ──────────────────────────────────
    //  _smoothstep : cubic ease-in/ease-out, input & output both in [0, 1]
    //  _clamp01    : clamps v to [0.0, 1.0]
    //  _lerp       : linear interpolation between two FootPositions
    static float        _smoothstep(float t);
    static float        _clamp01   (float v);
    static FootPosition _lerp(const FootPosition& a,
                              const FootPosition& b, float t);
};
