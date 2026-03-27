// =============================================================================
//  Trajectory.h  —  BIPDEAL Stage-1 Cubic Bézier Gait Trajectory Generator
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
//  ── Swing Phase  (Cubic Bézier) ──────────────────────────────────────────
//
//  Let  t = φ / 0.5   →  t ∈ [0, 1] over the swing half-cycle.
//
//  The foot path is a cubic Bézier defined by four control points:
//
//      P0 = (p0x, p0y)  = (-4, -15)   lift-off  (on ground)
//      P1 = (p1x, p1y)  = (-4, -13)   tangent control — foot lifts vertically
//      P2 = (p2x, p2y)  = ( 3, -13)   tangent control — foot descends vertically
//      P3 = (p3x, p3y)  = ( 3, -15)   touch-down (on ground)
//
//  Parametric equations (Bernstein form):
//
//      x(t) = (1-t)³·P0x + 3(1-t)²t·P1x + 3(1-t)t²·P2x + t³·P3x
//      y(t) = (1-t)³·P0y + 3(1-t)²t·P1y + 3(1-t)t²·P2y + t³·P3y
//
//  Because P1x == P0x and P2x == P3x, the tangent at lift-off is purely
//  vertical (zero horizontal velocity) and the tangent at touch-down is
//  also purely vertical. The foot lifts straight up, travels at altitude,
//  and descends straight down — with no foot-drag at either transition.
//
//  ── Stance Phase  (Reversed Cubic Bézier) ────────────────────────────────
//
//  Let  t = (φ - 0.5) / 0.5   →  t ∈ [0, 1] over the stance half-cycle.
//
//  The stance foot path is the swing Bézier evaluated with control points
//  reversed (P3, P2, P1, P0):
//
//      x(t) = (1-t)³·P3x + 3(1-t)²t·P2x + 3(1-t)t²·P1x + t³·P0x
//      y(t) = groundY   (foot held flat on ground throughout stance)
//
//  This guarantees that:
//    - dx/dt at stance start  =  −dx/dt at swing end    (magnitudes match)
//    - dx/dt at stance end    =  −dx/dt at swing start  (magnitudes match)
//  The x-velocity profile is anti-symmetric: same magnitude, opposite
//  direction. This is the required horizontal-velocity constraint for
//  Stage-2 two-leg synchronisation (equal push-off and pull-through).
//
//  ── Velocity Continuity at Phase Boundaries ──────────────────────────────
//
//  At φ = 0 / 1  (swing start = stance end):
//      Swing  dx/dt → 3·(P1x − P0x) = 3·(-4 − -4) = 0   ✓ (vertical lift-off)
//      Stance dx/dt at t=1 → 3·(P0x − P1x) = 0           ✓ (vertical arrive)
//
//  At φ = 0.5  (swing end = stance start):
//      Swing  dx/dt at t=1 → 3·(P3x − P2x) = 3·(3 − 3) = 0   ✓
//      Stance dx/dt at t=0 → 3·(P2x − P3x) = 0                ✓
//
//  Both boundaries have zero horizontal velocity — no impulse, no jitter.
//
//  ── Home Position ────────────────────────────────────────────────────────
//  homeX = -2.0 cm,  homeY = groundY = -15.0 cm
//  This is the balanced standing pose. The leg moves here on startup before
//  the gait loop begins.
//
//  ── Computational Notes ──────────────────────────────────────────────────
//  Bézier evaluation uses only multiplications and additions (~20–30 FLOPs).
//  No trigonometric functions are called in the trajectory generator.
//  Execution time on Arduino Uno at 16 MHz is approximately 5–15 µs per
//  call, well within the 20 ms (50 Hz) update budget.
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
    //
    //  All four Bézier control points plus the home position are exposed as
    //  tunable parameters. In Stage 1 the defaults encode the specification
    //  exactly; they can be overridden with setParams() at runtime.
    //
    //  GEOMETRY RULES that must be respected when changing control points:
    //    • P0 and P3 must lie on the ground line (p0y == p3y == groundY).
    //    • P1x == P0x  → purely vertical lift-off tangent.
    //    • P2x == P3x  → purely vertical touch-down tangent.
    //    • P1y == P2y  → flat top of the arch; set to groundY + stepHeight.
    //  Violating these rules will introduce horizontal velocity at the
    //  phase boundaries and may cause servo jitter or ground scuffing.
    //
    struct Params {

        // ── Swing Bézier control points ──────────────────────────────────
        float p0x;   // cm   lift-off x           default  −4.0
        float p0y;   // cm   lift-off y (= groundY) default −15.0
        float p1x;   // cm   swing tangent x      default  −4.0  (== p0x)
        float p1y;   // cm   swing tangent y       default −13.0  (= groundY + 2)
        float p2x;   // cm   landing tangent x     default   3.0  (== p3x)
        float p2y;   // cm   landing tangent y     default −13.0  (= groundY + 2)
        float p3x;   // cm   touch-down x          default   3.0
        float p3y;   // cm   touch-down y (= groundY) default −15.0

        // ── Home position ────────────────────────────────────────────────
        float homeX;    // cm   default −2.0
        float homeY;    // cm   default −15.0  (= groundY)
    };

    // ── Construction ──────────────────────────────────────────────────────
    explicit Trajectory(const Params& p = defaultParams());

    // Re-configure at runtime.
    void setParams(const Params& p);

    // ── Core query ────────────────────────────────────────────────────────
    //  Returns foot position at normalised phase φ.
    //  φ wraps automatically into [0, 1) — pass raw millis()-based values.
    FootPosition getFootPosition(float phi) const;

    // ── Convenience accessors ─────────────────────────────────────────────
    FootPosition  getHomePosition() const;
    float         getXStart()  const { return _p.p0x; }   // lift-off x
    float         getXEnd()    const { return _p.p3x; }   // touch-down x
    float         getGroundY() const { return _p.p0y; }   // ground line y
    float         getPeakY()   const { return _p.p1y; }   // arch peak y
    const Params& getParams()  const { return _p;     }

    // ── Defaults ──────────────────────────────────────────────────────────
    static Params defaultParams();

private:
    Params _p;

    // Evaluates the cubic Bézier for one axis given four scalar control
    // values (a, b, c, d) and parameter t ∈ [0, 1].
    static float _bezier(float a, float b, float c, float d, float t);
};
