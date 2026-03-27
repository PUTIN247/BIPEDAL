#include "Trajectory.h"
Trajectory::Params Trajectory::defaultParams() {
    Params p;
    p.p0x =  -4.0f;    // lift-off
    p.p0y = -15.0f;
    p.p1x =  -4.0f;    // tangent above P0  (p1x == p0x → zero horizontal velocity at lift-off)
    p.p1y = -13.0f;
    p.p2x =   3.0f;    // tangent above P3  (p2x == p3x → zero horizontal velocity at touch-down)
    p.p2y = -13.0f;
    p.p3x =   3.0f;    // touch-down
    p.p3y = -15.0f;

    p.homeX = -2.0f;
    p.homeY = -15.0f;

    return p;
}
Trajectory::Trajectory(const Params& p) {
    setParams(p);
}
void Trajectory::setParams(const Params& p) {
    _p = p;
}

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

FootPosition Trajectory::getFootPosition(float phi) const {

    phi = phi - floorf(phi);

    FootPosition fp;

    if (phi < 0.5f) {
        float t = phi / 0.5f;

        fp.x = _bezier(_p.p0x, _p.p1x, _p.p2x, _p.p3x, t);
        fp.y = _bezier(_p.p0y, _p.p1y, _p.p2y, _p.p3y, t);
    }
    else {
        float t = (phi - 0.5f) / 0.5f;
        fp.x = _bezier(_p.p3x, _p.p2x, _p.p1x, _p.p0x, t);
    }

    return fp;
}
FootPosition Trajectory::getHomePosition() const {
    FootPosition hp;
    hp.x = _p.homeX;
    hp.y = _p.homeY;
    return hp;
}
