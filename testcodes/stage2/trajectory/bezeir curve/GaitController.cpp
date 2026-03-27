#include "GaitController.h"

GaitController::GaitController(BipedLeg& left, BipedLeg& right,
                               const Trajectory& traj)
    : _left(left),
      _right(right),
      _traj(traj),
      _state(State::HOMING),
      _stateStart(0UL),
      _phi(0.0f),
      _leftFrom ({0.0f, 0.0f}), _rightFrom({0.0f, 0.0f}),
      _leftTo   ({0.0f, 0.0f}), _rightTo  ({0.0f, 0.0f}),
      _leftCurr ({0.0f, 0.0f}), _rightCurr({0.0f, 0.0f})
{}

void GaitController::begin() {
    _enterHoming();
}

GaitController::State GaitController::update() {
    uint32_t now = millis();
    switch (_state) {
        case State::HOMING:   _doHoming  (now);  break;
        case State::PRE_GAIT: _doPreGait (now);  break;
        case State::WALKING:  _doWalking (now);  break;
    }
    return _state;
}

void GaitController::_enterHoming() {
    _state      = State::HOMING;
    _stateStart = millis();

    // From: assumed power-on positions (Stage-1 cycle endpoints)
    _leftFrom  = { _traj.getXStart(),  _traj.getGroundY() };   // (-4, -15)
    _rightFrom = { _traj.getXEnd(),    _traj.getGroundY() };   // ( 3, -15)

    // To: symmetric home position
    _leftTo  = _traj.getHomePosition();                         // (-2, -15)
    _rightTo = _traj.getHomePosition();                         // (-2, -15)
}

void GaitController::_enterPreGait() {
    _state      = State::PRE_GAIT;
    _stateStart = millis();

    // From: home (where HOMING just left both legs)
    _leftFrom  = _traj.getHomePosition();                       // (-2, -15)
    _rightFrom = _traj.getHomePosition();                       // (-2, -15)

    // To: initial step configuration
    _leftTo  = { _traj.getXStart(), _traj.getGroundY() };      // P0 = (-4, -15)
    _rightTo = { _traj.getXEnd(),   _traj.getGroundY() };      // P3 = ( 3, -15)
}

void GaitController::_enterWalking() {
    _state      = State::WALKING;
    _stateStart = millis();
    _phi        = 0.0f;
}

void GaitController::_doHoming(uint32_t now) {
    // Normalise elapsed time to [0, 1]; apply smoothstep for ease-in/out.
    float raw_t = static_cast<float>(now - _stateStart)
                / static_cast<float>(HOME_MS);
    float t = _smoothstep(_clamp01(raw_t));

    _moveBoth(_lerp(_leftFrom,  _leftTo,  t),
              _lerp(_rightFrom, _rightTo, t));

    // raw_t comparison avoids clamping artefacts at the boundary.
    if (raw_t >= 1.0f) _enterPreGait();
}

void GaitController::_doPreGait(uint32_t now) {
    float raw_t = static_cast<float>(now - _stateStart)
                / static_cast<float>(PRE_GAIT_MS);
    float t = _smoothstep(_clamp01(raw_t));

    _moveBoth(_lerp(_leftFrom,  _leftTo,  t),
              _lerp(_rightFrom, _rightTo, t));

    if (raw_t >= 1.0f) _enterWalking();
}

void GaitController::_doWalking(uint32_t now) {
    // Cycle position within the current gait period [0, CYCLE_MS)
    uint32_t cyclePos = (now - _stateStart) % CYCLE_MS;
    _phi = static_cast<float>(cyclePos) / static_cast<float>(CYCLE_MS);

    // Right leg leads by half a cycle
    float phiR = _phi + 0.5f;
    if (phiR >= 1.0f) phiR -= 1.0f;

    _moveBoth(_traj.getFootPosition(_phi),
              _traj.getFootPosition(phiR));
}

void GaitController::_moveBoth(const FootPosition& lp,
                                const FootPosition& rp) {
    _left .moveTo(lp.x, lp.y);
    _right.moveTo(rp.x, rp.y);
    _leftCurr  = lp;
    _rightCurr = rp;
}
float GaitController::_smoothstep(float t) {
    return t * t * (3.0f - 2.0f * t);
}

float GaitController::_clamp01(float v) {
    if (v < 0.0f) return 0.0f;
    if (v > 1.0f) return 1.0f;
    return v;
}

FootPosition GaitController::_lerp(const FootPosition& a,
                                    const FootPosition& b,
                                    float t) {
    FootPosition fp;
    fp.x = a.x + t * (b.x - a.x);
    fp.y = a.y + t * (b.y - a.y);
    return fp;
}
