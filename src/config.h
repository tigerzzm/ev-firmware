#pragma once
//==============================================================================
// EV firmware — ALL global parameters.
//
// Every tunable number in the firmware lives in this file. Nothing else defines
// a magic constant; if you find one elsewhere, hoist it here.
//
// Target: Raspberry Pi Pico 2 (RP2350). Logic is 3.3 V and NOT 5 V-tolerant.
//
// LAYOUT
//   §1  MUST CALIBRATE   — ship at 0 on purpose; the car cannot run until set
//   §2  BENCH-MEASURED   — real hardware numbers, from docs/OPERATING_GUIDE.md
//   §3  SIGNS            — four one-line flips confirmed on the bench
//   §4  CONTROL TUNING   — gains and profile shape
//   §5  RULES & STRATEGY — from the rules manual; re-check each season
//   §6  PIN MAP          — wiring; see docs/WIRING_GUIDE.md
//   §7  BENCH TOOLS      — calibration-mode settings (cal.cpp)
//   §8  DERIVED          — computed; do not edit
//==============================================================================
#include <cstdint>

//==============================================================================
// §1  MUST CALIBRATE BEFORE THE FIRST RUN
//------------------------------------------------------------------------------
// These ship at 0.0 DELIBERATELY. An uncalibrated build reads zero distance and
// never reaches its stop condition — an intended tripwire, not a bug.
// Procedure: docs/OPERATING_GUIDE.md §1.1 and §1.2.
//==============================================================================

// Metres of ground travel per encoder count of the free roller.
//   Calibration menu -> "1 Encoder read". Zero, push a marked 5.000 m by hand,
//   read the count. The serial output does the division for you.
//   Encoder is a Signswise 600 P/R optical; the PIO decodes all four quadrature
//   edges, so one shaft revolution = 600 * 4 = 2400 counts.
//   Expect ~52,300 counts over 5 m. Near 26,000 means 2x decoding, near 13,000
//   means 1x — check the B-phase wiring before trusting the number.
constexpr float M_PER_COUNT      = 0.0f;    // <-- CALIBRATE (nonzero!)

// Front-axle centre to rear-axle centre, metres. Tape measure; no firmware.
// While this is 0.0 the steering feedforward atan(L*k) is identically zero and
// the lateral loop runs on feedback alone.
constexpr float WHEELBASE_L_M    = 0.0f;    // <-- MEASURE

//==============================================================================
// §2  BENCH-MEASURED HARDWARE NUMBERS
//==============================================================================

// ---- Steering servo map ----------------------------------------------------
// SERVO_US_PER_RAD is a SERVO + LINKAGE product, applied to the BEAM angle:
//   MG90S ~637 us/rad at the shaft x the 1:5.6 drag-link ratio ~= 3565 us/rad.
// The shipped 500.0f carries no knowledge of the linkage and is ~7x too small —
// the car would steer ~7x less than commanded. Measure it: from centre, jog a
// known offset in the servo-jog tool, protractor the beam, divide. Calibrate
// near +/-3 deg of beam; the four-bar map is non-linear at the extremes.
constexpr int   SERVO_CENTER_US  = 1500;    // <-- wheels dead straight
constexpr float SERVO_US_PER_RAD = 500.0f;  // <-- CALIBRATE (expect ~3500)
constexpr int   SERVO_MIN_US     = 1000;    // servo electrical range; also the
constexpr int   SERVO_MAX_US     = 2000;    //   outer mechanical backstop

// Travel limit either side of centre, microseconds. Tighter than the electrical
// range above, because the LINKAGE runs out long before the servo does — and a
// servo held against a mechanical stop sits at stall current indefinitely, which
// is how gear trains die. steerWriteUs() enforces this on every command, so the
// servo-jog tool cannot walk into a stop either.
//   +/-400 us is roughly +/-7 deg of beam at the design ratio (~3565 us/rad).
// Set to 0 to disable and use the full SERVO_MIN_US..SERVO_MAX_US range — do that
// only with the horn OFF the drag link, e.g. when exploring the true limits.
constexpr int   SERVO_TRAVEL_US  = 400;

// Steering clamp, radians of BEAM angle. The drag link delivers only about
// +/-7 deg (0.12 rad) — see docs/design/EV_Steering_Linkage_Design.md §10.
// Commanding more drives the servo into a mechanical stop and stalls it.
constexpr float DELTA_MAX_RAD    = 0.12f;   // ~7 deg  <-- VERIFY on the linkage

// ---- Drivetrain PWM levels (all %) -----------------------------------------
// Find these with calibration menu -> "2 Motor test". Walk the duty down until
// the car stops moving; that threshold is your dead-zone floor.
constexpr float    LAUNCH_PWM     = 45.0f;  // open-loop kick to break stiction
constexpr uint32_t LAUNCH_MS      = 150;    // ...for this long
constexpr float    BRAKE_PWM      = 60.0f;  // active reverse brake level
constexpr int      BRAKE_PULSE_MS = 100;    // ...for this long
// PROVISIONAL: 10% is a guess below the dead zone. At the previous 22% the creep
// exceeded commanded cruise duty at the slow end of the rules envelope (D=7 m /
// T=20 s cruises near 17%), so the car accelerated into its own stop. Set this a
// couple of points ABOVE the measured floor.
constexpr float CREEP_PWM         = 10.0f;  // low, guaranteed-to-move duty

// Creep stall guard (APPROACH): if the car isn't progressing toward the target,
// step the creep duty up until it moves. Prevents a low battery or a grippy tyre
// stalling the car short of the target.
constexpr float    CREEP_PWM_MAX         = 40.0f;  // ceiling while bumping
constexpr float    CREEP_PWM_STEP        = 3.0f;   // added per stall window
constexpr uint32_t CREEP_STALL_WINDOW_MS = 150;    // progress-check window
constexpr float    CREEP_STALL_MIN_CM    = 0.3f;   // min progress per window

// Feedforward speed->PWM slope, % per (m/s). PLACEHOLDER — a pure guess, not a
// measurement. Replace with interpolation over a real table from calibration
// menu -> "4 PWM->speed". See pwmForSpeed() in speed.cpp.
constexpr float PWM_PER_MPS      = 40.0f;   // <-- CALIBRATE

// Optional inner speed loop (MotorController in speed.cpp). All zero = disabled;
// the longitudinal loop is feedforward-only until these are tuned.
constexpr float SPEED_KP = 0.0f, SPEED_KI = 0.0f, SPEED_KD = 0.0f;
constexpr float SPEED_KS = 0.0f, SPEED_KV = 0.0f, SPEED_KA = 0.0f;

//==============================================================================
// §3  SIGNS — confirm each on the bench, then flip if wrong
//------------------------------------------------------------------------------
// Each is a one-line change. A wrong sign presents as a tuning problem and can
// burn hours, so verify them individually and early.
//==============================================================================
// Turn the car left by hand: heading should INCREASE.
constexpr double HEADING_SIGN   = +1.0;
// Push the car forward: the encoder count should go UP.
// (Or just swap the encoder's green and white wires.)
constexpr int    ENCODER_SIGN   = +1;
// Turn the dial clockwise: the displayed value should go UP.
constexpr int    DIAL_DIR_SIGN  = +1;
// Push the car off the arc by hand: the servo should steer it BACK, not further.
constexpr float  STEER_FB_SIGN  = +1.0f;

//==============================================================================
// §4  CONTROL TUNING
//==============================================================================

// ---- Lateral (Stanley) — steer.cpp -----------------------------------------
constexpr float STEER_KH        = 1.0f;   // heading-error gain   <-- TUNE
constexpr float STEER_KC        = 1.0f;   // cross-track gain     <-- TUNE
constexpr float STEER_SPEED_EPS = 0.2f;   // m/s floor in atan2, avoids a
                                          // divide-by-zero at a standstill

// ---- Longitudinal trapezoid — speed.cpp ------------------------------------
constexpr float ACCEL_FRAC      = 0.15f;  // fraction of L spent accelerating
constexpr float DECEL_FRAC      = 0.10f;  // fraction of L spent ramping down
// Floor the ramps at this fraction of peak so the car never coasts toward v=0
// (which would stall cruise before the closed-loop creep takes over, and makes
// the time integral of ds/v diverge). The precise stop is APPROACH's job.
constexpr float TRAPEZOID_FLOOR_FRAC = 0.25f;

// Live LCD telemetry during a run. An LCD write is SLOW — each character is four
// I2C byte-transactions plus strobe delays, so a 16-char line costs on the order
// of 10 ms at 100 kHz. That is fine against a plant timescale of L/v = 0.35-1.0 s,
// but it must not run every control tick. 0 disables run-time LCD updates.
constexpr uint32_t LCD_RUN_UPDATE_MS = 250;   // 4 Hz

// ---- Loop rates ------------------------------------------------------------
constexpr int   CONTROL_TICK_HZ = 500;    // core1 estimator/control rate

// Ground-speed estimate from the free roller (odo.cpp). Differentiating distance
// at the full 500 Hz tick is dominated by encoder quantisation (one count is
// most of a m/s at 2 ms), so the estimator re-samples on a longer window and
// then low-passes the result.
constexpr float SPEED_SAMPLE_MIN_S = 0.020f;  // ~50 Hz resample window
constexpr float SPEED_FILTER_TAU_S = 0.100f;  // single-pole time constant

//==============================================================================
// §5  RULES & STRATEGY — from the rules manual, re-check each season
//------------------------------------------------------------------------------
// Basis is the 2026 Division C manual; the 2027 manual posts 8 Sept 2026.
// See docs/design/EV_Rules_Scoring_Model.md.
//==============================================================================
constexpr float TARGET_DIST_MIN_M = 7.00f,  TARGET_DIST_MAX_M = 10.00f;
constexpr float TARGET_TIME_MIN_S = 10.0f,  TARGET_TIME_MAX_S = 20.0f;

// Gate bulge h (sagitta) for the arc path, metres. Derived from vehicle width
// plus can-bonus margin: 100 - (13 + 2)/2 = 92.5 cm. A tighter gap earns more
// can bonus; verify the car threads it before pushing tighter.
constexpr float GATE_BULGE_H_M   = 0.925f;

// Run-2 stop correction. The rules give two runs and keep the better, so run 1
// is a free calibration run. Measure the PHYSICAL error (Measurement Point ->
// Target Point, with a tape) and enter it NEGATED: stopped 3 cm past the target
// -> -0.030. Applied to the stop distance ONLY, never to pathPlan() — replanning
// the arc would change curvature and could break the can-gate line. This single
// number absorbs MP offset, encoder track offset, and brake overshoot at once.
constexpr float STOP_OFFSET_MIN_M = -0.50f;
constexpr float STOP_OFFSET_MAX_M =  0.50f;

// Switch to closed-loop creep this far from the stop target.
constexpr float CREEP_ZONE_M      = 0.20f;

// Runaway watchdog: abort after targetT * FACTOR + PAD seconds. Guards against
// an uncalibrated M_PER_COUNT, where distance reads 0 and the stop never fires.
constexpr float RUN_TIMEOUT_FACTOR = 2.0f;
constexpr float RUN_TIMEOUT_PAD_S  = 5.0f;

//==============================================================================
// §5b  UI MODE
//------------------------------------------------------------------------------
// 1 = dial entry. The run asks for D, T and the stop offset on the LCD; targets
//     are announced after impound, so this is the fast path at a competition
//     (~10 s per change vs ~90 s for an edit/rebuild/reflash cycle).
// 0 = no dial. The run skips entry entirely and uses the DEFAULT_* values below —
//     you change those here and reflash. Bench tools (cal.cpp) still work, using
//     SET = increment (auto-repeats, wraps) and START = confirm.
//
// The dial is worth more at the BENCH than at the competition: the servo jog in
// particular is far nicer with live rotation than with a repeat-press button.
//==============================================================================
#define USE_DIAL_ENTRY 1

// Used only when USE_DIAL_ENTRY == 0. Clamped to the §5 legal ranges at runtime.
constexpr float DEFAULT_TARGET_D_M    = 8.00f;
constexpr float DEFAULT_TARGET_T_S    = 15.0f;
// Run-2 correction: measure MP -> Target with a tape after run 1 and enter it
// NEGATED here (stopped 3 cm past -> -0.030f), then rebuild and reflash.
constexpr float DEFAULT_STOP_OFFSET_M = 0.0f;

//==============================================================================
// §6  PIN MAP — see docs/WIRING_GUIDE.md
//==============================================================================

// Pin choices below match the KEYESTUDIO KS3017 Pico IO shield's DEDICATED
// 4-pin I2C connectors, which carry SDA/SCL/3V3/GND together. The shield's 3-pin
// S/3V3/GND rows expose every GPIO too, but an I2C device on those would need its
// power and ground duplicated across two headers.

// ---- I2C0 — BNO085 IMU ONLY ------------------------------------------------
// The IMU has this bus to itself. It is read from core1 at the control tick.
// Shield connector: "I2C0".
#define I2C_SDA_PIN       20    // GP20 (pin 26)
#define I2C_SCL_PIN       21    // GP21 (pin 27)  3.3 V bus
#define BNO085_INT_PIN     6    // GP6  (pin 9)   optional; UNUSED by firmware
#define BNO085_RST_PIN     7    // GP7  (pin 10)  optional clean reset

// ---- I2C1 — 16x2 LCD ONLY --------------------------------------------------
// Deliberately a SEPARATE controller from the IMU. Sharing one bus meant core0
// (LCD) and core1 (IMU) could collide, which poseSetActive() had to gate against
// — and that gate is why nothing could be displayed during a run. On its own bus
// the LCD can be written at any time.
// Shield connector: "I2C1". RP2350 mux allows I2C1 SDA {2,6,10,14,18,26} /
// SCL {3,7,11,15,19,27}, so GP14/GP15 is a legal pair.
#define LCD_I2C_INST       i2c1
#define LCD_SDA_PIN       14    // GP14 (pin 19)
#define LCD_SCL_PIN       15    // GP15 (pin 20)
#define LCD_I2C_ADDR       0x27  // some backpacks are 0x3F

// ---- Distance: free-rolling FRONT encoder (undriven wheel, no slip) ---------
// Decoded in PIO. This single encoder serves BOTH the precise stop trigger and
// the ground-speed signal — no drive-motor encoder needed.
// A and B MUST be consecutive GPIOs: the PIO reads `pin` and `pin+1`.
// Signswise 600 P/R, NPN open-collector, 5-24 V: power at 5 V and pull A/B up to
// 3.3 V (4.7k). NEVER pull up to 5 V — that puts 5 V on the GPIO.
#define FRONT_ENCODER_A_PIN   10   // GP10 (pin 14)  green
#define FRONT_ENCODER_B_PIN   11   // GP11 (pin 15)  white

// ---- (OPTIONAL) drive-motor encoder — only for a tighter inner speed loop ---
// #define DRIVE_ENCODER_A_PIN 12  // GP12
// #define DRIVE_ENCODER_B_PIN 13  // GP13

// ---- Motor driver: BTS7960 / IBT-2 (single 550 rear drive) -----------------
// Two half-bridges, one motor. Enables are separate GPIOs but are always written
// together — a single motor has no useful mode with one side disabled.
#define BTS7960_RPWM_PIN   16   // GP16 (pin 21)  forward PWM
#define BTS7960_LPWM_PIN   17   // GP17 (pin 22)  reverse / brake PWM
#define BTS7960_R_EN_PIN   18   // GP18 (pin 24)  right half-bridge inhibit
#define BTS7960_L_EN_PIN   26   // GP26 (pin 31)  left half-bridge inhibit
// GP27/GP28 remain free for ADC (e.g. battery voltage). R_IS/L_IS unconnected.

// ---- Onboard LED: heartbeat ------------------------------------------------
// PICO_DEFAULT_LED_PIN is GP25 on a Pico 1 and Pico 2 (it is on the wireless chip
// on a Pico W, where this is not defined and the heartbeat compiles out).
// Blinks from core1 so it proves the ESTIMATOR is running, not just that power is
// on — a solid or dark LED means core1 has stopped.
constexpr uint32_t HEARTBEAT_MS = 500;   // toggle interval; 0 disables

// ---- Steering servo --------------------------------------------------------
// GP5 = PWM slice 2 channel B. Slice 2 channel A is GP4 (START), which is a plain
// GPIO input and never set to GPIO_FUNC_PWM — so the servo owns the slice.
// Signal only: V+ comes from the 5 V rail, never a 3-pin header's 3V3 pin.
#define SERVO_SIG_PIN       5   // GP5  (pin 7)   50 Hz, 1000-2000 us

// ---- UI: rotary dial + buttons (all active-low, internal pull-ups) ----------
#define DIAL_CLK_PIN        2   // GP2  (pin 4)
#define DIAL_DT_PIN         3   // GP3  (pin 5)
#define DIAL_BTN_PIN       22   // GP22 (pin 29)  cycles the increment
#define START_BTN_PIN       4   // GP4  (pin 6)   #2-pencil trigger, VERTICAL only
#define SET_BTN_PIN        19   // GP19 (pin 25)  confirm / abort / hold-at-boot

//==============================================================================
// §7  BENCH TOOLS — calibration mode (cal.cpp), entered by HOLDING SET at boot
//==============================================================================
// PWM->speed probe: the car accelerates through CAL_WARMUP_M (discarded launch
// transient), then average speed is timed over exactly CAL_MEASURE_M. Floor
// needed is warmup + measure + braking, about 3.5 m per data point.
constexpr float CAL_WARMUP_M  =  1.00f;
constexpr float CAL_MEASURE_M =  2.00f;
constexpr float CAL_TIMEOUT_S = 20.0f;
constexpr float CAL_PWM_MIN   = 10.0f;
constexpr float CAL_PWM_MAX   = 100.0f;
constexpr uint32_t MOTOR_TEST_MS = 1500;   // motor-test burst length
// Hold SET this long at the "EV ready" screen to drop into the bench menu, so
// calibration is reachable WITHOUT a power cycle. Rebooting re-enumerates USB and
// drops the serial monitor (some monitors then latch onto the wrong port), which
// makes bench work needlessly painful.
constexpr uint32_t CAL_ENTRY_HOLD_MS = 2000;

//==============================================================================
// §8  DERIVED — computed from the above; do not edit
//==============================================================================
constexpr float CM_PER_COUNT    = M_PER_COUNT * 100.0f;
constexpr int   CONTROL_TICK_US = 1000000 / CONTROL_TICK_HZ;
