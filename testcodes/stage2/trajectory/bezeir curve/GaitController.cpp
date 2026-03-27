// =============================================================================
//  GaitController.cpp  —  BIPDEAL Stage-2 Two-Leg Gait Coordinator
// =============================================================================

#include "GaitController.h"

// ---------------------------------------------------------------------------
//  Constructor
// ---------------------------------------------------------------------------
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

// ---------------------------------------------------------------------------
//  begin()
//
//  Kick off the startup sequence.  Must be called in setup() after the
//  PCA9685 has been initialised with begin() and setPWMFreq().
// ---------------------------------------------------------------------------
void GaitController::begin() {
    _enterHoming();
}

// ---------------------------------------------------------------------------
//  update()
//
//  Advance the active state by one tick.
//  Call every UPDATE_INTERVAL_MS milliseconds in loop().
// ---------------------------------------------------------------------------
GaitController::State GaitController::update() {
    uint32_t now = millis();
    switch (_state) {
        case State::HOMING:   _doHoming  (now);  break;
        case State::PRE_GAIT: _doPreGait (now);  break;
        case State::WALKING:  _doWalking (now);  break;
    }
    return _state;
}

// ===========================================================================
//  State entry methods  —  called ONCE when entering each state
// ===========================================================================

// ---------------------------------------------------------------------------
//  _enterHoming()
//
//  Assumed power-on positions for both legs:
//    Left  → P0 = (xStart, groundY)  = (-4, -15)
//    Right → P3 = (xEnd,   groundY)  = ( 3, -15)
//
//  Rationale:
//    After a Stage-1 run the single tested leg finishes near these positions.
//    Choosing them as the start also produces a visually natural "gather"
//    where both feet move inward to the centre before the step spread.
//    If the legs are already at home the smoothstep gives a brief soft hold.
// ---------------------------------------------------------------------------
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

// ---------------------------------------------------------------------------
//  _enterPreGait()
//
//  From the home position, spread both legs to the initial step configuration:
//    Left  → P0 = (-4, -15)   back of the stride, ready to swing forward
//    Right → P3 = ( 3, -15)   front of the stride, ready to push backward
//
//  After this transition, entering WALKING at φ = 0 maps:
//    traj.getFootPosition(0.0) = P0  ← exactly where the left leg just arrived
//    traj.getFootPosition(0.5) = P3  ← exactly where the right leg just arrived
//  Zero position discontinuity guaranteed.
// ---------------------------------------------------------------------------
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

// ---------------------------------------------------------------------------
//  _enterWalking()
//
//  φ is reset to 0.  At φ = 0:
//    Left  leg → traj.getFootPosition(0.0)   = P0  ✓ matches PRE_GAIT end
//    Right leg → traj.getFootPosition(0.5)   = P3  ✓ matches PRE_GAIT end
//  The first call to _doWalking will compute exactly these positions,
//  so the servos do not move at the transition instant.
// ---------------------------------------------------------------------------
void GaitController::_enterWalking() {
    _state      = State::WALKING;
    _stateStart = millis();
    _phi        = 0.0f;
}

// ===========================================================================
//  Per-state update handlers  —  called every update() tick
// ===========================================================================

// ---------------------------------------------------------------------------
//  _doHoming()
// ---------------------------------------------------------------------------
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

// ---------------------------------------------------------------------------
//  _doPreGait()
// ---------------------------------------------------------------------------
void GaitController::_doPreGait(uint32_t now) {
    float raw_t = static_cast<float>(now - _stateStart)
                / static_cast<float>(PRE_GAIT_MS);
    float t = _smoothstep(_clamp01(raw_t));

    _moveBoth(_lerp(_leftFrom,  _leftTo,  t),
              _lerp(_rightFrom, _rightTo, t));

    if (raw_t >= 1.0f) _enterWalking();
}

// ---------------------------------------------------------------------------
//  _doWalking()
//
//  Master phase φ ∈ [0, 1) is derived from millis() via a clean modulo.
//  This is rollover-safe for the ~49-day Arduino millis() wrap.
//
//  Phase assignment:
//    φ_L = φ                       left  leg (leads the cycle)
//    φ_R = (φ + 0.5) mod 1.0       right leg (180° behind)
//
//  Trajectory::getFootPosition() wraps its argument internally, but we
//  do the explicit wrap here so getPhase() / serial telemetry are correct.
// ---------------------------------------------------------------------------
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

// ===========================================================================
//  Private helpers
// ===========================================================================

// ---------------------------------------------------------------------------
//  _moveBoth()
//
//  Drives both legs to the supplied foot positions and caches the results.
//  All servo writes pass through here so _leftCurr / _rightCurr are always
//  up to date for the serial telemetry accessors.
// ---------------------------------------------------------------------------
void GaitController::_moveBoth(const FootPosition& lp,
                                const FootPosition& rp) {
    _left .moveTo(lp.x, lp.y);
    _right.moveTo(rp.x, rp.y);
    _leftCurr  = lp;
    _rightCurr = rp;
}

// ---------------------------------------------------------------------------
//  _smoothstep()
//
//  Cubic Hermite smoothstep:  f(t) = t² · (3 − 2t)
//
//  Properties:
//    f(0) = 0,  f(1) = 1
//    f'(0) = 0, f'(1) = 0   ← zero velocity at both ends
//
//  Cost: 3 multiplications + 1 subtraction.  No trig, no sqrt.
// ---------------------------------------------------------------------------
float GaitController::_smoothstep(float t) {
    return t * t * (3.0f - 2.0f * t);
}

// ---------------------------------------------------------------------------
//  _clamp01()
// ---------------------------------------------------------------------------
float GaitController::_clamp01(float v) {
    if (v < 0.0f) return 0.0f;
    if (v > 1.0f) return 1.0f;
    return v;
}

// ---------------------------------------------------------------------------
//  _lerp()
//
//  Linear interpolation between two FootPositions.
//  Combined with smoothstep this produces ease-in/ease-out motion.
// ---------------------------------------------------------------------------
FootPosition GaitController::_lerp(const FootPosition& a,
                                    const FootPosition& b,
                                    float t) {
    FootPosition fp;
    fp.x = a.x + t * (b.x - a.x);
    fp.y = a.y + t * (b.y - a.y);
    return fp;
}
