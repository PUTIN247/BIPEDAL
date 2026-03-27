// =============================================================================
//  Trajectory.cpp  —  BIPDEAL Stage-1 Cubic Bézier Gait Trajectory Generator
// =============================================================================
//
//  All trajectory maths is pure polynomial arithmetic.
//  No sin/cos calls — the Bézier evaluator uses only multiplications and
//  additions, keeping the Arduino Uno comfortably within its 16 MHz budget.
//
// =============================================================================

#include "Trajectory.h"

// ---------------------------------------------------------------------------
//  defaultParams()
//
//  Control points match the Stage-1 specification exactly:
//
//      P0 = (-4, -15)   lift-off point       on the ground line
//      P1 = (-4, -13)   swing tangent        directly above P0 → vertical lift-off
//      P2 = ( 3, -13)   landing tangent      directly above P3 → vertical touch-down
//      P3 = ( 3, -15)   touch-down point     on the ground line
//
//  Arch geometry:
//      step length  = P3x − P0x = 3 − (−4) = 7 cm
//      peak height  = |P1y − P0y| = |−13 − (−15)| = 2 cm above ground
//      ground line  = y = −15 cm
//
//  Home position:
//      x = −2.0 cm   (between lift-off and touch-down, biased toward P0)
//      y = −15.0 cm  (on the ground line)
// ---------------------------------------------------------------------------
Trajectory::Params Trajectory::defaultParams() {
    Params p;

    // Swing Bézier control points
    p.p0x =  -4.0f;    // lift-off
    p.p0y = -15.0f;
    p.p1x =  -4.0f;    // tangent above P0  (p1x == p0x → zero horizontal velocity at lift-off)
    p.p1y = -13.0f;
    p.p2x =   3.0f;    // tangent above P3  (p2x == p3x → zero horizontal velocity at touch-down)
    p.p2y = -13.0f;
    p.p3x =   3.0f;    // touch-down
    p.p3y = -15.0f;

    // Home position — balanced standing pose
    p.homeX = -2.0f;
    p.homeY = -15.0f;

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
}

// ---------------------------------------------------------------------------
//  _bezier()
//
//  Evaluates a cubic Bézier for one Cartesian axis.
//
//  Given four scalar control values (a, b, c, d) and parameter t ∈ [0, 1]:
//
//      B(t) = (1-t)³·a  +  3(1-t)²t·b  +  3(1-t)t²·c  +  t³·d
//
//  Expanded into Horner-like form to minimise multiplications:
//
//      u  = 1 - t
//      u² = u  * u
//      u³ = u² * u
//      t² = t  * t
//      t³ = t² * t
//
//      B  = u³·a  +  3·u²·t·b  +  3·u·t²·c  +  t³·d
//
//  Total cost: ~10 multiplications + 4 additions — no trig, no division.
// ---------------------------------------------------------------------------
float Trajectory::_bezier(float a, float b, float c, float d, float t) {
    float u   = 1.0f - t;
    float u2  = u  * u;
    float u3  = u2 * u;
    float t2  = t  * t;
    float t3  = t2 * t;

    return u3 * a
         + 3.0f * u2 * t  * b
         + 3.0f * u  * t2 * c
         +        t3      * d;
}

// ---------------------------------------------------------------------------
//  getFootPosition(phi)
//
//  φ is wrapped into [0, 1) before processing.
//
//  ── Swing phase  φ ∈ [0, 0.5) ───────────────────────────────────────────
//
//    t = φ / 0.5          (maps swing half-cycle to [0, 1])
//
//    x(t) = Bézier(P0x, P1x, P2x, P3x, t)
//    y(t) = Bézier(P0y, P1y, P2y, P3y, t)
//
//    At t = 0: foot at P0  (lift-off,  ground)
//    At t = 1: foot at P3  (touch-down, ground)
//    At t ≈ 0.5: foot near arch peak  y ≈ groundY + stepHeight
//
//  ── Stance phase  φ ∈ [0.5, 1.0) ────────────────────────────────────────
//
//    t = (φ − 0.5) / 0.5  (maps stance half-cycle to [0, 1])
//
//    The stance x-path is the swing Bézier with control points reversed:
//    x(t) = Bézier(P3x, P2x, P1x, P0x, t)
//
//    This is algebraically equivalent to evaluating the swing Bézier at
//    (1 − t), so the x-velocity profile is identical in magnitude to the
//    swing profile but in the opposite direction.
//
//    y is held at groundY — the foot remains flat on the ground.
//
//  ── Horizontal-velocity symmetry proof ───────────────────────────────────
//
//    The derivative of a cubic Bézier at t = 0 is:
//        dx/dt|_{t=0} = 3·(B1 − B0)
//    and at t = 1:
//        dx/dt|_{t=1} = 3·(B3 − B2)
//
//    Swing:
//        t=0 : 3·(P1x − P0x) = 3·(−4 − (−4)) =  0   ← vertical lift-off
//        t=1 : 3·(P3x − P2x) = 3·( 3 −   3 ) =  0   ← vertical touch-down
//
//    Stance (reversed order P3, P2, P1, P0):
//        t=0 : 3·(P2x − P3x) = 3·( 3 −   3 ) =  0   ← continues from swing end
//        t=1 : 3·(P0x − P1x) = 3·(−4 − (−4)) =  0   ← arrives at swing start
//
//    Throughout the mid-cycle the x-velocity magnitudes are mirror-symmetric:
//        swing  dx/dt at parameter  τ  =  −stance dx/dt at parameter  τ
//    This satisfies the Stage-2 requirement that both legs always apply
//    equal and opposite horizontal foot forces on the ground.
//
// ---------------------------------------------------------------------------
FootPosition Trajectory::getFootPosition(float phi) const {

    // Wrap φ into [0, 1) without using fmod (lighter on Uno)
    phi = phi - floorf(phi);

    FootPosition fp;

    if (phi < 0.5f) {
        // ── Swing phase ──────────────────────────────────────────────────
        //  Normalise to [0, 1] over the swing half-cycle.
        float t = phi / 0.5f;

        fp.x = _bezier(_p.p0x, _p.p1x, _p.p2x, _p.p3x, t);
        fp.y = _bezier(_p.p0y, _p.p1y, _p.p2y, _p.p3y, t);
    }
    else {
        // ── Stance phase ─────────────────────────────────────────────────
        //  Normalise to [0, 1] over the stance half-cycle.
        float t = (phi - 0.5f) / 0.5f;

        //  Reversed Bézier: control point order P3, P2, P1, P0
        //  x traces the exact mirror of the swing x-profile.
        fp.x = _bezier(_p.p3x, _p.p2x, _p.p1x, _p.p0x, t);

        //  y stays on the ground throughout stance.
        fp.y = _p.p0y;    // groundY  (== p3y by geometry rule)
    }

    return fp;
}

// ---------------------------------------------------------------------------
//  getHomePosition()
// ---------------------------------------------------------------------------
FootPosition Trajectory::getHomePosition() const {
    FootPosition hp;
    hp.x = _p.homeX;
    hp.y = _p.homeY;
    return hp;
}
