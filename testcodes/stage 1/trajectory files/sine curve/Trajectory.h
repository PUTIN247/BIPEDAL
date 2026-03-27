// =============================================================================
//  Trajectory.h  —  BIPDEAL Stage-1 Sinusoidal Gait Trajectory Generator
// =============================================================================
//
//  Produces foot coordinates  x(φ), y(φ)  for a single leg as a continuous
//  function of the normalised gait phase  φ ∈ [0, 1).
//
//  Phase layout
//  ─────────────────────────────────────────────────────────────────────────
//    φ ∈ [0.0, 0.5)  → SWING phase  (foot airborne, moves forward)
//    φ ∈ [0.5, 1.0)  → STANCE phase (foot on ground, moves backward)
//
//  Trajectory equations
//  ─────────────────────────────────────────────────────────────────────────
//  Let:
//    t_sw = φ / 0.5              (0→1 over swing)
//    t_st = (φ − 0.5) / 0.5     (0→1 over stance)
//
//  SWING   x(φ) = xMin + t_sw · stepLength       linear, forward travel
//          y(φ) = groundY + stepHeight · sin(π · t_sw)  sinusoidal lift
//
//  STANCE  x(φ) = xMax − t_st · stepLength       linear, backward travel
//          y(φ) = groundY                          ground contact (flat)
//
//  where   xMin = −stepLength / 2
//          xMax = +stepLength / 2
//
//  Horizontal-velocity constraint  (required for Stage-2 two-leg sync)
//  ─────────────────────────────────────────────────────────────────────────
//  dx/dφ  during swing  =  stepLength / 0.5  =  +2·stepLength
//  dx/dφ  during stance =  stepLength / 0.5  =  −2·stepLength
//  → Magnitudes are equal; directions are opposite.  ✓
//  Both phases traverse the full stepLength over exactly half a cycle,
//  so Stage-2 can phase-offset the second leg by 0.5 and guarantee
//  that the x-velocity of the airborne leg always matches (in magnitude)
//  the x-velocity of the grounded leg.
//
//  Home position
//  ─────────────────────────────────────────────────────────────────────────
//  homeX  = homeRatio · (stepLength / 2)
//  homeY  = groundY
//  With defaults:  homeRatio = −2/3.5 ≈ −0.5714 → homeX = −2.0 cm
//
//  homeRatio is deliberately kept configurable so the relationship between
//  home position and step range can be adjusted without touching the rest
//  of the trajectory logic.
//
// =============================================================================

#pragma once
#include <Arduino.h>

// ---------------------------------------------------------------------------
//  FootPosition  —  2-D foot coordinate in the hip-centred frame
// ---------------------------------------------------------------------------
struct FootPosition {
    float x;    // cm,  positive = forward
    float y;    // cm,  always negative in normal operation (below hip joint)
};

// ---------------------------------------------------------------------------
//  Trajectory
// ---------------------------------------------------------------------------
class Trajectory {
public:

    // ── Gait parameters ───────────────────────────────────────────────────
    struct Params {
        float stepLength;   // cm   total x-travel per cycle         default  7.0
        float stepHeight;   // cm   peak lift above the ground line   default  2.0
                            //      NOTE: at stepHeight = 2.0 cm the knee angle
                            //      reaches ~93.7° at the arc peak (x≈0, y=−13),
                            //      which slightly exceeds the 90° hard limit.
                            //      The IK clamps and logs a warning.
                            //      Set stepHeight ≤ 1.5 cm to stay fully within
                            //      limits across the entire trajectory.
        float groundY;      // cm   y-coordinate of the flat ground plane
                            //                                        default −15.0
        float homeRatio;    // —    homeX = homeRatio · (stepLength / 2)
                            //      −1 → xMin  |  0 → centre  |  +1 → xMax
                            //      default −0.5714 → homeX = −2.0 cm
    };

    // ── Construction ──────────────────────────────────────────────────────
    explicit Trajectory(const Params& p = defaultParams());

    // Re-configure at runtime; recomputes all derived values.
    void setParams(const Params& p);

    // ── Core query ────────────────────────────────────────────────────────
    // Returns foot position at normalised phase φ.
    // φ is accepted over any real range; it wraps automatically into [0, 1).
    FootPosition getFootPosition(float phi) const;

    // ── Convenience accessors ─────────────────────────────────────────────
    FootPosition getHomePosition() const;
    float        getXMin()    const { return _xMin;        }
    float        getXMax()    const { return _xMax;        }
    float        getGroundY() const { return _p.groundY;   }
    const Params& getParams() const { return _p;           }

    // ── Defaults ──────────────────────────────────────────────────────────
    static Params defaultParams();

private:
    Params _p;
    float  _xMin;       // −stepLength / 2
    float  _xMax;       // +stepLength / 2
    float  _homeX;      //  homeRatio  · (stepLength / 2)

    void _recalcDerived();
};
