#pragma once
#include <Arduino.h>
#include "Bipedal.h"
#include "Trajectory.h"

class GaitController {
public:

    static constexpr uint32_t HOME_MS     =  800UL;   // both legs → home
    static constexpr uint32_t PRE_GAIT_MS =  600UL;   // home → (P0 , P3)
    static constexpr uint32_t CYCLE_MS    = 2000UL;   // one full gait cycle

    // ── State 
    enum class State : uint8_t {
        HOMING   = 0,   // gathering both legs to home position
        PRE_GAIT = 1,   // spreading legs to initial step configuration
        WALKING  = 2    // continuous phase-based gait
    };

    // ── Construction
    GaitController(BipedLeg& left, BipedLeg& right, const Trajectory& traj);

    void  begin();
    State update();

    // ── Accessors 
    State        getState()     const { return _state;      }
    bool         isWalking()    const { return _state == State::WALKING; }
    float        getPhase()     const { return _phi;        }
    FootPosition getLeftFoot()  const { return _leftCurr;   }
    FootPosition getRightFoot() const { return _rightCurr;  }

private:

    BipedLeg&         _left;
    BipedLeg&         _right;
    const Trajectory& _traj;

    State         _state;
    uint32_t      _stateStart;    // millis() snapshot when current state began
    float         _phi;           // master gait phase [0, 1)
    FootPosition  _leftFrom,  _rightFrom;
    FootPosition  _leftTo,    _rightTo;
    FootPosition  _leftCurr,  _rightCurr;

    // ── State entry helpers ───────────────────────────────────────────────────
    void _enterHoming();
    void _enterPreGait();
    void _enterWalking();

    // ── Per-state update handlers ─────────────────────────────────────────────
    void _doHoming  (uint32_t now);
    void _doPreGait (uint32_t now);
    void _doWalking (uint32_t now);
    void _moveBoth(const FootPosition& lp, const FootPosition& rp);

    static float        _smoothstep(float t);
    static float        _clamp01   (float v);
    static FootPosition _lerp(const FootPosition& a,
                              const FootPosition& b, float t);
};
