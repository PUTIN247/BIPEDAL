#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include "Bipedal.h"
#include "Trajectory.h"
#include "GaitController.h"

// ---- PCA9685 
#define PCA9685_ADDR   0x40
#define OSC_FREQUENCY  27000000UL     // measure with a scope; typical: 25–27 MHz

// ---- Left leg servo channels 
constexpr uint8_t L_CH_HIP    = 0;
constexpr uint8_t L_CH_KNEE   = 1;
constexpr uint8_t L_CH_ANKLE  = 2;

// ---- Right leg servo channels 
constexpr uint8_t R_CH_HIP    = 3;
constexpr uint8_t R_CH_KNEE   = 4;
constexpr uint8_t R_CH_ANKLE  = 5;

constexpr float L_CAL_HIP    =  0.0f;
constexpr float L_CAL_KNEE   =  0.0f;
constexpr float L_CAL_ANKLE  =  0.0f;

constexpr float R_CAL_HIP    =  0.0f;
constexpr float R_CAL_KNEE   =  0.0f;
constexpr float R_CAL_ANKLE  =  0.0f;

constexpr uint32_t UPDATE_INTERVAL_MS = 20UL;

#define SERIAL_PLOT


Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(PCA9685_ADDR);
BipedLeg leftLeg (pwm, L_CH_HIP, L_CH_KNEE, L_CH_ANKLE, false);
BipedLeg rightLeg(pwm, R_CH_HIP, R_CH_KNEE, R_CH_ANKLE, true);
Trajectory traj;

GaitController gait(leftLeg, rightLeg, traj);
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
    gait.begin();
}

void loop() {

    static uint32_t lastUpdate = 0;
    uint32_t now = millis();
    if (now - lastUpdate < UPDATE_INTERVAL_MS) return;
    lastUpdate = now;

    GaitController::State prevState = gait.getState();
    GaitController::State currState = gait.update();

#ifdef SERIAL_PLOT

    // ── Announce state transitions ─────────────────────────────────────────────
    if (currState != prevState) {
        printStateChange(currState);
    }
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
