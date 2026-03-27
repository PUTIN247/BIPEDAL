// =============================================================================
//  Trajectory.cpp  —  BIPDEAL Stage-1 Sinusoidal Gait Trajectory Generator
// =============================================================================

#include "Trajectory.h"
#include <math.h>

// ---------------------------------------------------------------------------
//  Internal helpers
// ---------------------------------------------------------------------------
static inline float lerp(float a, float b, float t) {
    return a + t * (b - a);
}

// ---------------------------------------------------------------------------
//  defaultParams()
// ---------------------------------------------------------------------------
Trajectory::Params Trajectory::defaultParams() {
    Params p;
    p.stepLength = 7.0f;                        // cm
    p.stepHeight = 2.0f;                        // cm
    p.groundY    = -15.0f;                      // cm
    //  homeRatio = homeX / (stepLength / 2)
    //            = −2.0 / (7.0 / 2)  =  −2.0 / 3.5  =  −0.571428...
    p.homeRatio  = -2.0f / 3.5f;               // → homeX = −2.0 cm
    return p;
}

// ---------------------------------------------------------------------------
//  Constructor
// ---------------------------------------------------------------------------
Trajectory::Trajectory(const Params& p) {
    setParams(p);
}

// ---------------------------------------------------------------------------
//  setParams()
// ---------------------------------------------------------------------------
void Trajectory::setParams(const Params& p) {
    _p = p;
    _recalcDerived();
}

// ---------------------------------------------------------------------------
//  _recalcDerived()
//  Called whenever parameters change; computes all derived constants.
// ---------------------------------------------------------------------------
void Trajectory::_recalcDerived() {
    _xMin  = -_p.stepLength * 0.5f;                    // cm
    _xMax  = +_p.stepLength * 0.5f;                    // cm
    _homeX =  _p.homeRatio  * (_p.stepLength * 0.5f);  // cm
}

// ---------------------------------------------------------------------------
//  getFootPosition(phi)
//
//  φ is wrapped into [0, 1) before processing, so the caller can pass a
//  raw millis()-based accumulator and the function handles wrap-around.
//
//  Swing phase  φ ∈ [0, 0.5)
//  ──────────────────────────
//    t  = φ / 0.5                           normalised 0 → 1 over swing
//    x  = lerp(xMin, xMax, t)              constant forward velocity
//    y  = groundY + stepHeight·sin(π·t)    sinusoidal lift
//
//    At t=0.0 : foot at (xMin, groundY)  — lift-off
//    At t=0.5 : foot at (midX,  groundY + stepHeight)  — peak
//    At t=1.0 : foot at (xMax, groundY)  — touch-down
//
//  Stance phase  φ ∈ [0.5, 1.0)
//  ──────────────────────────────
//    t  = (φ − 0.5) / 0.5                  normalised 0 → 1 over stance
//    x  = lerp(xMax, xMin, t)              constant backward velocity
//    y  = groundY                           foot on ground
//
//  Horizontal velocity proof
//  ──────────────────────────
//    dx/dφ_swing  =  (xMax − xMin) / 0.5  =  stepLength / 0.5  = +14 cm/unit
//    dx/dφ_stance = (xMin − xMax) / 0.5  =  stepLength / 0.5  = −14 cm/unit
//    |dx/dφ_swing| == |dx/dφ_stance|  ✓
//
// ---------------------------------------------------------------------------
FootPosition Trajectory::getFootPosition(float phi) const {
    // Wrap φ into [0, 1)
    phi = phi - floorf(phi);

    FootPosition fp;

    if (phi < 0.5f) {
        // ── Swing phase ──────────────────────────────────────────────────
        float t = phi / 0.5f;                              // 0 → 1

        fp.x = lerp(_xMin, _xMax, t);                     // linear forward
        fp.y = _p.groundY + _p.stepHeight * sinf(PI * t); // sinusoidal lift
    }
    else {
        // ── Stance phase ─────────────────────────────────────────────────
        float t = (phi - 0.5f) / 0.5f;                    // 0 → 1

        fp.x = lerp(_xMax, _xMin, t);                     // linear backward
        fp.y = _p.groundY;                                 // flat ground contact
    }

    return fp;
}

// ---------------------------------------------------------------------------
//  getHomePosition()
// ---------------------------------------------------------------------------
FootPosition Trajectory::getHomePosition() const {
    FootPosition hp;
    hp.x = _homeX;
    hp.y = _p.groundY;
    return hp;
}
