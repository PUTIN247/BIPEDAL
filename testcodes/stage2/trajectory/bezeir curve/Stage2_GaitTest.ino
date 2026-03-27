// =============================================================================
//  Stage2_GaitTest.ino  —  BIPDEAL Stage-2: Two-Leg Coordinated Walking
// =============================================================================
//
//  Extends Stage-1 to drive both legs in coordinated anti-phase gait.
//  The GaitController manages the startup sequence and continuous walking;
//  loop() only needs to call gait.update() every 20 ms.
//
//  Startup sequence (automatic, no user interaction required)
//  ─────────────────────────────────────────────────────────────────────────
//  [1] HOMING   (800 ms)
//        Both legs gather smoothly to home position (-2, -15) cm.
//        Assumed start:  left = (-4, -15),  right = (3, -15).
//
//  [2] PRE_GAIT (600 ms)
//        From home, left moves to P0 (-4, -15) and right to P3 (3, -15).
//        This creates the initial asymmetry needed for clean gait entry.
//
//  [3] WALKING  (continuous)
//        Phase-based Bézier gait.
//        φ_L = φ         Left  leg: swing [0,0.5), stance [0.5,1.0)
//        φ_R = φ + 0.5   Right leg: 180° out of phase with left
//
//  Both homing and pre-gait transitions use cubic smoothstep (ease-in/out).
//  No velocity discontinuities occur at any state transition.
//
//  Hardware
//  ─────────────────────────────────────────────────────────────────────────
//  PCA9685   I2C address 0x40
//  Left  leg :  ch 0 hip  | ch 1 knee  | ch 2 ankle   mirrorX = false
//  Right leg :  ch 3 hip  | ch 4 knee  | ch 5 ankle   mirrorX = true
//
//  Serial telemetry (115200 baud)
//  ─────────────────────────────────────────────────────────────────────────
//  #define SERIAL_PLOT streams CSV each update tick.
//  Format during startup:  state, lx, ly, rx, ry
//  Format during walking:  phi,   lx, ly, rx, ry
//  Comment out SERIAL_PLOT to eliminate all Serial overhead in production.
//
// =============================================================================

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include "Bipedal.h"
#include "Trajectory.h"
#include "GaitController.h"

// =============================================================================
//  ── CONFIGURATION  (edit to match your hardware) ────────────────────────────
// =============================================================================

// ---- PCA9685 ----------------------------------------------------------------
#define PCA9685_ADDR   0x40
#define OSC_FREQUENCY  27000000UL     // measure with a scope; typical: 25–27 MHz

// ---- Left leg servo channels ------------------------------------------------
constexpr uint8_t L_CH_HIP    = 0;
constexpr uint8_t L_CH_KNEE   = 1;
constexpr uint8_t L_CH_ANKLE  = 2;

// ---- Right leg servo channels -----------------------------------------------
constexpr uint8_t R_CH_HIP    = 3;
constexpr uint8_t R_CH_KNEE   = 4;
constexpr uint8_t R_CH_ANKLE  = 5;

// ---- Calibration offsets (degrees) ------------------------------------------
//  Trim each offset until the joint physically reads 0° (leg perfectly
//  straight and vertical) when commanded to the home position.
//  Left and right legs are tuned independently.
constexpr float L_CAL_HIP    =  0.0f;
constexpr float L_CAL_KNEE   =  0.0f;
constexpr float L_CAL_ANKLE  =  0.0f;

constexpr float R_CAL_HIP    =  0.0f;
constexpr float R_CAL_KNEE   =  0.0f;
constexpr float R_CAL_ANKLE  =  0.0f;

// ---- Servo update rate ------------------------------------------------------
//  20 ms = 50 Hz, matching the PCA9685 PWM frequency.
//  Includes ~2 ms I2C overhead for 6 servo writes on Arduino Uno.
constexpr uint32_t UPDATE_INTERVAL_MS = 20UL;

// ---- Serial telemetry -------------------------------------------------------
//  Comment out to remove all Serial overhead in production.
#define SERIAL_PLOT

// =============================================================================
//  ── GLOBAL OBJECTS ──────────────────────────────────────────────────────────
// =============================================================================

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(PCA9685_ADDR);

//  mirrorX = false : left leg — x-axis positive = forward in direction of motion
//  mirrorX = true  : right leg — x-axis is mirrored inside BipedLeg::calculateIK
BipedLeg leftLeg (pwm, L_CH_HIP, L_CH_KNEE, L_CH_ANKLE, false);
BipedLeg rightLeg(pwm, R_CH_HIP, R_CH_KNEE, R_CH_ANKLE, true);

//  Single Trajectory instance shared by both legs.
//  Loaded with defaultParams() in setup(); override with traj.setParams() if needed.
Trajectory traj;

//  Two-leg coordinator — owns the state machine and phase logic.
GaitController gait(leftLeg, rightLeg, traj);

// =============================================================================
//  ── STATE CHANGE PRINTER  (Serial only) ─────────────────────────────────────
// =============================================================================
#ifdef SERIAL_PLOT
static void printStateChange(GaitController::State s) {
    switch (s) {
        case GaitController::State::HOMING:
            Serial.println(F("[STATE] HOMING   — both legs moving to home (-2, -15) cm"));
            break;
        case GaitController::State::PRE_GAIT:
            Serial.println(F("[STATE] PRE_GAIT — left → P0 (-4,-15)  right → P3 (3,-15)"));
            break;
        case GaitController::State::WALKING:
            Serial.println(F("[STATE] WALKING  — continuous gait started"));
            Serial.println(F("phi,lx_cm,ly_cm,rx_cm,ry_cm"));   // CSV header for Plotter
            break;
    }
}
#endif

// =============================================================================
//  ── SETUP ───────────────────────────────────────────────────────────────────
// =============================================================================
void setup() {

    Serial.begin(115200);
    while (!Serial) { /* wait for USB CDC on Leonardo/Micro; nop on Uno */ }

    Serial.println(F(""));
    Serial.println(F("╔══════════════════════════════════════════════════════╗"));
    Serial.println(F("║   BIPDEAL — Stage 2: Two-Leg Coordinated Walking     ║"));
    Serial.println(F("╚══════════════════════════════════════════════════════╝"));
    Serial.println(F(""));

    // ── Initialise I2C and PCA9685 ───────────────────────────────────────────
    Wire.begin();
    pwm.begin();
    pwm.setOscillatorFrequency(OSC_FREQUENCY);
    pwm.setPWMFreq(50);       // 50 Hz — standard servo refresh rate
    delay(10);                // let PCA9685 stabilise after power-on

    // ── Apply calibration offsets ────────────────────────────────────────────
    leftLeg .setOffsets(L_CAL_HIP, L_CAL_KNEE, L_CAL_ANKLE);
    rightLeg.setOffsets(R_CAL_HIP, R_CAL_KNEE, R_CAL_ANKLE);

    // ── Trajectory parameters (specification defaults) ───────────────────────
    //
    //  To adjust at runtime (e.g. to lower the arch peak if knee limit warnings
    //  appear, as documented in Bipedal.cpp):
    //      Trajectory::Params p = Trajectory::defaultParams();
    //      p.p1y = -13.5f;   // shift peak down by 0.5 cm
    //      p.p2y = -13.5f;   // keep arch flat (p1y == p2y rule)
    //      traj.setParams(p);
    //
    Trajectory::Params p = Trajectory::defaultParams();
    traj.setParams(p);

    // ── Print configuration ──────────────────────────────────────────────────
    Serial.println(F("  Trajectory : Cubic Bézier — shared by both legs"));
    Serial.println(F("  ─────────────────────────────────────────────────────"));
    Serial.print  (F("  P0  lift-off  = (")); Serial.print(p.p0x, 1);
    Serial.print  (F(", "));                  Serial.print(p.p0y, 1);
    Serial.println(F(") cm  ← Left  leg at PRE_GAIT end / WALKING φ=0"));

    Serial.print  (F("  P3  touch-dn  = (")); Serial.print(p.p3x, 1);
    Serial.print  (F(", "));                  Serial.print(p.p3y, 1);
    Serial.println(F(") cm  ← Right leg at PRE_GAIT end / WALKING φ=0.5"));

    Serial.print  (F("  Home         = (")); Serial.print(p.homeX, 1);
    Serial.print  (F(", "));                  Serial.print(p.homeY, 1);
    Serial.println(F(") cm"));

    Serial.println(F("  ─────────────────────────────────────────────────────"));
    Serial.print  (F("  Step length   = ")); Serial.print(p.p3x - p.p0x, 1);
    Serial.println(F(" cm"));
    Serial.print  (F("  Arch height   = ")); Serial.print(p.p0y - p.p1y, 1);
    Serial.println(F(" cm"));
    Serial.print  (F("  Gait cycle    = ")); Serial.print(GaitController::CYCLE_MS);
    Serial.println(F(" ms"));
    Serial.print  (F("  Homing        = ")); Serial.print(GaitController::HOME_MS);
    Serial.println(F(" ms  (smoothstep)"));
    Serial.print  (F("  Pre-gait      = ")); Serial.print(GaitController::PRE_GAIT_MS);
    Serial.println(F(" ms  (smoothstep)"));
    Serial.print  (F("  Update rate   = ")); Serial.print(1000UL / UPDATE_INTERVAL_MS);
    Serial.println(F(" Hz"));
    Serial.println(F(""));
    Serial.println(F("  Startup sequence begins …"));
    Serial.println(F(""));

#ifdef SERIAL_PLOT
    // Print the very first state banner before the first update tick
    printStateChange(GaitController::State::HOMING);
#endif

    // ── Start state machine ──────────────────────────────────────────────────
    gait.begin();
}

// =============================================================================
//  ── MAIN LOOP ───────────────────────────────────────────────────────────────
// =============================================================================
void loop() {

    // ── Non-blocking rate limiter ─────────────────────────────────────────────
    //  Subtraction-based check handles the millis() 49-day rollover safely.
    static uint32_t lastUpdate = 0;
    uint32_t now = millis();
    if (now - lastUpdate < UPDATE_INTERVAL_MS) return;
    lastUpdate = now;

    // ── Snapshot state before update ──────────────────────────────────────────
    GaitController::State prevState = gait.getState();

    // ── Advance state machine (drives servos internally) ──────────────────────
    GaitController::State currState = gait.update();

#ifdef SERIAL_PLOT

    // ── Announce state transitions ─────────────────────────────────────────────
    if (currState != prevState) {
        printStateChange(currState);
    }

    // ── Stream telemetry ───────────────────────────────────────────────────────
    //  During HOMING / PRE_GAIT: emit state index + actual commanded positions.
    //  During WALKING: emit phi + commanded positions (matches Serial Plotter).
    FootPosition lf = gait.getLeftFoot();
    FootPosition rf = gait.getRightFoot();

    if (currState == GaitController::State::WALKING) {
        // phi, lx, ly, rx, ry  — CSV for Arduino Serial Plotter
        Serial.print(gait.getPhase(), 3); Serial.print(F(","));
    } else {
        // state index (0=HOMING, 1=PRE_GAIT) instead of phi
        Serial.print(static_cast<uint8_t>(currState)); Serial.print(F(","));
    }

    Serial.print(lf.x, 2);  Serial.print(F(","));
    Serial.print(lf.y, 2);  Serial.print(F(","));
    Serial.print(rf.x, 2);  Serial.print(F(","));
    Serial.println(rf.y, 2);

#endif  // SERIAL_PLOT
}
