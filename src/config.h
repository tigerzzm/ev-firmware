#pragma once
//==============================================================================
// EV firmware — hardware config & pin map
// Target: Raspberry Pi Pico 2 (RP2350).  Logic is 3.3 V and NOT 5 V-tolerant.
// Pin assignments follow claude/EV_Detailed_Design.md §2. Adjust freely.
//==============================================================================
#include <cstdint>

// ---- I2C0 (shared bus: BNO085 IMU + 16x2 LCD) -------------------------------
#define I2C_SDA_PIN        4    // GP4
#define I2C_SCL_PIN        5    // GP5   (3.3 V bus, 4.7k pull-ups)
#define BNO085_INT_PIN     6    // GP6   optional (data-ready)
#define BNO085_RST_PIN     7    // GP7   optional (clean reset); use -1 if unwired
#define LCD_I2C_ADDR       0x27

// ---- Distance: free-rolling FRONT encoder (undriven wheel, no slip) ----------
// Decoded in PIO. This single encoder serves BOTH the precise stop trigger and
// the ground-speed signal for the time/speed loop — no drive-motor encoder needed.
#define FRONT_ENCODER_A_PIN   10   // GP10  (level-shift if the encoder is 5 V!)
#define FRONT_ENCODER_B_PIN   11   // GP11

// ---- (OPTIONAL) drive-motor encoder — only if you ever want a tighter speed
//      loop than the free-roller gives. Left unwired by default. --------------
// #define DRIVE_ENCODER_A_PIN 12  // GP12
// #define DRIVE_ENCODER_B_PIN 13  // GP13

// ---- Motor driver: BTS7960 (single 550 rear drive) --------------------------
#define BTS7960_RPWM_PIN   16   // GP16  forward PWM
#define BTS7960_LPWM_PIN   17   // GP17  reverse / brake PWM
#define BTS7960_EN_PIN     18   // GP18  tie R_EN + L_EN together
// #define BTS7960_R_IS_PIN 26  // GP26 (ADC) optional current sense
// #define BTS7960_L_IS_PIN 27  // GP27 (ADC) optional current sense

// ---- Steering servo (retrofit on existing front pivot) ----------------------
#define SERVO_SIG_PIN      15   // GP15  50 Hz, 1000-2000 us; powered from 5 V rail

// ---- UI: rotary dial + buttons ----------------------------------------------
#define DIAL_CLK_PIN       20   // GP20
#define DIAL_DT_PIN        21   // GP21
#define DIAL_BTN_PIN       22   // GP22
#define START_BTN_PIN      14   // GP14  pressed by the #2-pencil trigger (vertical)
#define SET_BTN_PIN        19   // GP19  confirm distance/time

//==============================================================================
// Calibration constants — MEASURE THESE ON THE BENCH (see EV_Detailed_Design §7)
//==============================================================================
// Distance: metres of ground travel per encoder count of the free roller.
//   Roll the car a marked 5.000 m by hand, read counts; M_PER_COUNT = 5.0/counts.
//   Ballpark from the existing EV kit (code_ref/main_code): 7.3025 cm wheel,
//   1200 PPR -> circumference 22.94 cm -> ~0.000191 m/count. The free roller may
//   differ, so still calibrate. Ships 0.0 so an uncalibrated build reads 0 m.
constexpr float M_PER_COUNT      = 0.0f;    // <-- CALIBRATE (nonzero!) before running
constexpr float CM_PER_COUNT     = M_PER_COUNT * 100.0f;

// Geometry
constexpr float WHEELBASE_L_M    = 0.0f;    // <-- MEASURE front-axle to rear-axle (m)

// Gate bulge h (sagitta) for the arc path. The existing kit (code_ref/main_code
// getArcLength) derived it from vehicle width + can-bonus margin:
//   arcHeight = 100 - (vehicleWidth + canBonusError)/2  = 100 - (13 + 2)/2 = 92.5 cm.
// Narrower gate (larger effective bulge / tighter gap) = more can bonus; verify
// the car threads it before pushing tighter.
constexpr float GATE_BULGE_H_M   = 0.925f;

// Steering servo map
constexpr int   SERVO_CENTER_US  = 1500;    // <-- wheels dead straight
constexpr float SERVO_US_PER_RAD = 500.0f;  // <-- command +/- known angle, measure
constexpr int   SERVO_MIN_US     = 1000;
constexpr int   SERVO_MAX_US     = 2000;
constexpr float DELTA_MAX_RAD    = 0.6f;    // steering clamp

// Control loop
constexpr int   CONTROL_TICK_HZ  = 500;     // core1 estimator/control rate
constexpr int   CONTROL_TICK_US  = 1000000 / CONTROL_TICK_HZ;

// Target ranges (for UI entry clamping) — from the rules manual
constexpr float TARGET_DIST_MIN_M = 7.00f,  TARGET_DIST_MAX_M = 10.00f;
constexpr float TARGET_TIME_MIN_S = 10.0f,  TARGET_TIME_MAX_S = 20.0f;

// Stop / creep behaviour (the #1 scoring win: closed-loop stop on encoder count)
constexpr float CREEP_ZONE_M      = 0.20f;  // switch to closed-loop creep this far out
constexpr float CREEP_PWM         = 22.0f;  // low, guaranteed-to-move PWM %
constexpr int   BRAKE_PULSE_MS    = 100;    // active reverse brake pulse
// Creep stall guard (APPROACH): if the car isn't progressing toward the target,
// step the creep PWM up until it moves. Prevents a low battery / grippy tire from
// stalling short of the target.
constexpr float    CREEP_PWM_MAX         = 40.0f;  // don't exceed this while bumping
constexpr float    CREEP_PWM_STEP        = 3.0f;   // PWM% added per stall window
constexpr uint32_t CREEP_STALL_WINDOW_MS = 150;    // progress-check window
constexpr float    CREEP_STALL_MIN_CM    = 0.3f;   // expected min progress per window
