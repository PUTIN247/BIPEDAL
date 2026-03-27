
#pragma once
#include <Arduino.h>
struct FootPosition {
    float x;    // cm,  positive = forward
    float y;    // cm,  always negative in normal operation (below hip joint)
};

class Trajectory {
public:
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
    void setParams(const Params& p);
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
    static float _bezier(float a, float b, float c, float d, float t);
};
