#pragma once
//==============================================================================
// EV firmware — ALL global parameters. 2027 rules (bottle task).
//
// Every tunable number in the firmware lives in this file. Nothing else defines
// a magic constant; if you find one elsewhere, hoist it here.
//
// The run (docs/design/EV_Detailed_Design.md §0, rules docs/rules/EV_Rules_2027):
//   START -> start delay -> forward on y=0 -> capture the bottle -> push it past
//   the Bottle Line -> stop -> reverse until the REAR Measurement Point is on the
//   Target Point. All x positions below are the rear MP's x, Start = 0.
//
// Target: Raspberry Pi Pico (RP2040, PICO_BOARD "pico"). Logic is 3.3 V and NOT
// 5 V-tolerant.
//
// LAYOUT
//   §1  MUST MEASURE     — ship at 0 on purpose; the car refuses to ARM until set
//   §2  BENCH-MEASURED   — real hardware numbers, from docs/OPERATING_GUIDE.md
//   §3  SIGNS            — four one-line flips confirmed on the bench
//   §4  CONTROL TUNING   — gains, segment speeds, accelerations
//   §5  RULES & STRATEGY — from the 2027 rules manual; re-check each season
//   §6  PIN MAP          — wiring; see docs/WIRING_GUIDE.md
//   §7  BENCH TOOLS      — calibration-mode settings (cal.cpp)
//   §8  DERIVED          — computed; do not edit
//==============================================================================
#include <cstdint>

//==============================================================================
// §1  MUST MEASURE BEFORE THE FIRST RUN
//------------------------------------------------------------------------------
// These ship at 0.0 DELIBERATELY. sm.cpp refuses to ARM while any of them is
// still 0 and names the missing one on the LCD — an uncalibrated car never moves.
//==============================================================================

// Metres of ground travel per encoder count of the free roller, FORWARD.
//   Calibration menu -> "1 Encoder read". START to zero, push a marked 5.000 m
//   forward by hand; serial prints the constant from the NET count. Don't use the
//   F or R counts for this: any overshoot-and-ease-back during the push inflates
//   both and biases the constant low.
//   Encoder is a Signswise 600 P/R optical; the PIO decodes all four quadrature
//   edges, so one shaft revolution = 600 * 4 = 2400 counts.
//   Expect ~52,300 counts over 5 m. Near 26,000 means 2x decoding, near 13,000
//   means 1x — check the B-phase wiring before trusting the number.
constexpr float M_PER_COUNT      = 0.0f;    // <-- CALIBRATE (nonzero!)

// Same, measured pulling the car BACKWARD 5.000 m: START again at the far mark,
// pull back, and serial prints it from the net count.
// Leave at 0 to use M_PER_COUNT for both directions.
//
// BOTH scales set the final MP (2 pt/cm), and the FORWARD one matters more:
//   final MP = forward counts * M_PER_COUNT - reverse counts * M_PER_COUNT_REV
// The forward leg is the long one (~D + B, 6-12 m) and the reverse leg the short
// one (0.2-2.6 m), so a 0.2% error in M_PER_COUNT moves the final MP ~2 cm, while
// the same error in M_PER_COUNT_REV moves it only a few mm. Average several rolls.
constexpr float M_PER_COUNT_REV  = 0.0f;    // <-- CALIBRATE (or leave 0)

// Pusher contact face -> rear MP, along the car, metres. The contact face is the
// deepest point of the pusher's mouth, where the bottle sits when seated. Enters
// every plan: capture point and forward stop both depend on it.
constexpr float L_P_M            = 0.0f;    // <-- MEASURE

// Free-roller contact patch -> rear MP, along the car, metres. Turns the pose
// (which tracks the encoder wheel) into the MP position the reverse steering aims.
constexpr float ENC_FROM_MP_M    = 0.0f;    // <-- MEASURE

// Servo microseconds per radian of BEAM angle. A SERVO + LINKAGE product:
//   MG90S ~637 us/rad at the shaft x the 1:5.6 drag-link ratio ~= 3565 us/rad.
// Measure it: from centre, jog a known offset in "3 Servo jog", protractor the
// beam, divide. Calibrate near +/-3 deg of beam; the four-bar map is non-linear at
// the extremes. A wrong value doesn't just steer weakly: the estimator integrates
// the roller along the angle it THINKS it commanded, so the pose drifts and the
// steering corrects errors that aren't there. Expect ~3500.
constexpr float SERVO_US_PER_RAD = 0.0f;    // <-- CALIBRATE (expect ~3500)

// Front axle centreline -> rear axle centreline, metres (NOT the overall length).
// Needed ONLY when pure pursuit is selected for either leg (STEER_FWD_LAW or
// STEER_REV_LAW, §4); sm.cpp refuses to arm with it selected while this is 0.
// Expect ~0.44.
constexpr float WHEELBASE_L_M    = 0.0f;    // <-- MEASURE (pure pursuit only)

//==============================================================================
// §2  BENCH-MEASURED HARDWARE NUMBERS
//==============================================================================

// ---- Steering servo map (SERVO_US_PER_RAD is in §1) -------------------------
constexpr int   SERVO_CENTER_US  = 1500;    // <-- wheels dead straight
constexpr int   SERVO_MIN_US     = 1000;    // servo electrical range; also the
constexpr int   SERVO_MAX_US     = 2000;    //   outer mechanical backstop

// Travel limit either side of centre, microseconds. Tighter than the electrical
// range above, because the LINKAGE runs out long before the servo does — and a
// servo held against a mechanical stop sits at stall current indefinitely.
// steerWriteUs() enforces this on every command, so the servo-jog tool cannot
// walk into a stop either.
//   +/-400 us is roughly +/-7 deg of beam at the design ratio (~3565 us/rad).
// Set to 0 to disable — only with the horn OFF the drag link.
constexpr int   SERVO_TRAVEL_US  = 400;

// Steering clamp, radians of BEAM angle. The drag link delivers only about
// +/-7 deg (0.12 rad) — see docs/design/EV_Steering_Linkage_Design.md §10.
constexpr float DELTA_MAX_RAD    = 0.12f;   // ~7 deg  <-- VERIFY on the linkage

// ---- Drive feedforward: speed -> PWM % -------------------------------------
// pwm = SPEED_FF_OFFSET_PCT + SPEED_FF_PCT_PER_MPS * |v|, then the speed loop
// (§4) corrects the rest. Fill these from calibration menu -> "4 PWM->speed":
// fit a line through the (speed, PWM) points; the intercept is the offset.
// PLACEHOLDERS until then — the feedback loop carries the error meanwhile.
constexpr float SPEED_FF_OFFSET_PCT  = 5.0f;   // <-- CALIBRATE (dead-zone edge)
constexpr float SPEED_FF_PCT_PER_MPS = 40.0f;  // <-- CALIBRATE
constexpr float PWM_MAX_PCT          = 90.0f;  // never command above this

// ---- Encoder direction split (odo_split.h) ----------------------------------
// Counts are scaled by M_PER_COUNT or M_PER_COUNT_REV according to the direction
// of travel. A change of direction is only accepted after this many counts
// against the current one, so edge jitter at a standstill (servo buzz, the
// turnaround hold) nets out instead of being scaled twice. 8 counts ~ 0.8 mm.
constexpr int32_t ODO_DIR_HYST_COUNTS = 8;
// After draining the encoder FIFO, wait at most this many polls for one FRESH
// sample (odo.cpp). A fresh entry arrives within ~10 PIO cycles; the bound only
// matters if the state machine has stopped (a few us).
constexpr int     ODO_FRESH_SPIN      = 200;   // runs with IRQs off: keep it short

// ---- Starting from standstill ----------------------------------------------
// Kick: a short burst to break stiction at the two standstill starts (launch and
// the start of the reverse leg). Ends after KICK_MS or once the car has moved
// KICK_DONE_M, whichever is first. Find the stiction level with "2 Motor test".
constexpr float    KICK_PWM    = 35.0f;
constexpr uint32_t KICK_MS     = 250;
constexpr float    KICK_DONE_M = 0.01f;

// ---- Stopping ---------------------------------------------------------------
// Final stop (end of the reverse leg): drive the OPPOSITE way at this duty until
// one BRAKE_POLL_MS poll shows no further progress, capped at the max time, then
// hold. Ending on the first still poll means the pulse cannot keep driving a car
// that has already stopped. 0 = no counter-pulse, dynamic-brake hold only.
constexpr float    FINAL_BRAKE_PWM    = 40.0f;  // <-- TUNE
constexpr uint32_t FINAL_BRAKE_MAX_MS = 120;
// Encoder poll period while braking. A window with no new count means the car is
// below ~1/(BRAKE_POLL_MS * counts_per_m) = ~0.02 m/s; the hold finishes from there.
constexpr uint32_t BRAKE_POLL_MS      = 5;
// More counts than this in one poll = still moving, worth a counter-pulse. Fewer
// is edge jitter; at the planned final creep (V_REV_END 0.05 m/s ~ 2.5 counts per
// poll) the counter-pulse still fires, so FINAL_BRAKE_PWM stays a live knob.
constexpr long     BRAKE_MOVING_COUNTS = 2;   // above +/-1 jitter, below V_REV_END (~2.5)
// Abort stop (SET pressed, watchdog): harder and longer, because the car may be
// at cruise speed rather than creeping into a planned stop.
constexpr float    ABORT_BRAKE_PWM    = 60.0f;
constexpr uint32_t ABORT_BRAKE_MAX_MS = 300;
// After a stop the motor is held shorted (driveHold: both low-side FETs on), which
// resists rolling, and the wheels stay at their last angle. Both are kept until the
// next ARMED: the car may still be on the track waiting to be measured, and
// re-centring the steering or freeing the drive could move the MP (2 pt/cm).
// The final position is read for the report after this settle time.
constexpr uint32_t STOP_SETTLE_MS     = 500;

//==============================================================================
// §3  SIGNS — confirm each on the bench, then flip if wrong
//------------------------------------------------------------------------------
// Each is a one-line change. A wrong sign presents as a tuning problem and can
// burn hours, so verify them individually and early.
//==============================================================================
// Turn the car left by hand: heading should INCREASE ("6 IMU heading").
constexpr double HEADING_SIGN   = +1.0;
// Push the car forward: the encoder count should go UP ("1 Encoder read").
// (Or just swap the encoder's green and white wires.)
constexpr int    ENCODER_SIGN   = +1;
// Turn the dial clockwise: the displayed value should go UP.
constexpr int    DIAL_DIR_SIGN  = +1;
// Servo direction: a POSITIVE steering angle must point the front wheels LEFT.
// "7 Steer response" -> FWD mode: turn the car LEFT by hand and the wheels must
// steer RIGHT (back toward the line). If they steer left, flip this.
constexpr float  STEER_DIR_SIGN = +1.0f;

//==============================================================================
// §4  CONTROL TUNING
//==============================================================================

// ---- Steering, forward leg: which law holds the line y = 0 -------------------
// Both are in steer_law.cpp and both are simulated closed-loop in host_check.
//   Stanley      corrects heading error + sideways error at the front axle.
//                Lines up quickly; two gains. Can get twitchy at low speed.
//   PurePursuit  aims at a point on the line PP_LOOKAHEAD_M ahead of the rear
//                axle. Very smooth; one knob. Needs WHEELBASE_L_M (§1).
// Switch here, rebuild, and compare lateral error at the bottle on the floor.
enum class SteerLaw { Stanley, PurePursuit };
constexpr SteerLaw STEER_FWD_LAW = SteerLaw::Stanley;

// Stanley:  delta = KH * (0 - theta) - atan2(KC * y, |v| + EPS)
// (Reverse Stanley, below, uses the same three constants.)
constexpr float STEER_KH        = 1.0f;   // heading-error gain     <-- TUNE
constexpr float STEER_KC        = 1.0f;   // cross-track gain        <-- TUNE
constexpr float STEER_SPEED_EPS = 0.2f;   // m/s floor in atan2 (no /0 at rest)

// Pure pursuit (either leg):  delta = atan(2 * L * sin(alpha) / Ld). On a
// straight line it is always stable, forward or reverse, and settles in about
// 3-4 lookaheads of travel; a 1 deg start error peaks at ~5 mm sideways with
// Ld = 0.8 m. Smaller Ld corrects faster but steers harder. Must stay longer than
// the wheelbase (see §8): shorter is harsh, and the bench steer test's left/right
// check stops meaning anything in forward.
constexpr float PP_LOOKAHEAD_M  = 0.8f;   // <-- TUNE

// ---- Steering, reverse leg: which law -------------------------------------------
// The forward laws can't simply run backwards: in reverse a left wheel turns the
// car RIGHT, so forward Stanley used as-is drives the car off the line.
//   AimAtTarget  turns the car so its rear MP points at the Target:
//                  theta_des = atan2(y_mp, x_mp - x_target)
//                  delta     = -KR * (theta_des - theta)
//                Needs no wheelbase. Inside REV_STEER_HOLD_M of the Target the aim
//                angle becomes ill-conditioned, so the wheels freeze there.
//   PurePursuit  pure pursuit with the goal point PP_LOOKAHEAD_M BEHIND the rear
//                axle on the centerline. Never runs out of goal, so it keeps
//                steering right up to the stop. Needs WHEELBASE_L_M (§1).
//   Stanley      Stanley adapted for reverse: the sideways error is measured at
//                the rear MP (the LEADING end when backing up) and the heading
//                term flips sign:  delta = KH * theta - atan2(KC * y_mp, |v| + EPS)
//                Same gains as forward Stanley. Needs no wheelbase.
// All three land within ~1 cm in simulation; compare them on the floor.
//
// A complete Stanley run:      STEER_FWD_LAW = Stanley,     STEER_REV_LAW = Stanley
// A complete pure-pursuit run: STEER_FWD_LAW = PurePursuit, STEER_REV_LAW = PurePursuit
enum class RevSteerLaw { AimAtTarget, PurePursuit, Stanley };
constexpr RevSteerLaw STEER_REV_LAW = RevSteerLaw::AimAtTarget;

constexpr float STEER_KR         = 1.0f;  // AimAtTarget gain  <-- TUNE (keep low)
constexpr float REV_STEER_HOLD_M = 0.15f; // AimAtTarget freeze zone

// ---- Speed loop (PI on free-roller ground speed) — speed.cpp ----------------
// Output is a PWM magnitude in the commanded direction only; it can slow the car
// to zero drive but never drive it backward, so a forward leg cannot reverse.
constexpr float SPEED_KP    = 30.0f;  // % per (m/s) of error     <-- TUNE
constexpr float SPEED_KI    = 60.0f;  // % per (m/s * s)          <-- TUNE
constexpr float SPEED_I_MAX = 40.0f;  // integrator clamp, %

// ---- Segment speeds, m/s ---------------------------------------------------
// Chosen for ACCURACY and BOTTLE SAFETY, not for the Target Time: time is only
// 0.5 pt/s and the start delay absorbs slack. When a run would not fit inside T,
// plan.cpp speeds up cruise and reverse first (up to their _MAX), and raises the
// push only if that still isn't enough. The push is the long stretch, so with
// these nominal speeds it is raised in most legal runs: test bottles at V_PUSH_MAX.
constexpr float V_CRUISE_NOM = 0.60f;  // Start -> capture zone
constexpr float V_CRUISE_MAX = 1.60f;
constexpr float V_CAPTURE    = 0.20f;  // through first contact: seat, don't kick
constexpr float V_PUSH       = 0.60f;  // pushing the bottle
// The push is the long segment (captured at 2-4 m, pushed to D + B, up to ~12 m),
// so it sets whether the run fits inside T: at 0.45 m/s most legal runs finish
// several seconds late. A steady push does not tip the bottle at any speed (tip
// needs contact height > r/mu); ACCELERATION does, and A_PUSH/A_RELEASE limit it.
// Verify on the bench at V_PUSH_MAX before trusting it — a tipped bottle costs
// ~420 pts, a second of time costs 0.5. Lower this if bottles wobble or tip.
constexpr float V_PUSH_MAX   = 1.50f;
constexpr float V_PUSH_END   = 0.06f;  // floor of the push decel ramp
constexpr float V_REV        = 0.35f;  // reverse leg
constexpr float V_REV_MAX    = 0.80f;
constexpr float V_REV_END    = 0.05f;  // floor of the final decel ramp

// ---- Accelerations, m/s^2 --------------------------------------------------
constexpr float A_ACCEL    = 0.8f;   // speeding up, cruise and reverse
constexpr float A_DECEL    = 0.8f;   // cruise -> capture speed
constexpr float A_PUSH     = 0.4f;   // speeding up WITH the bottle (anti-tip)
// Push decel. Must stay well below mu*g (~3-4 m/s^2) so the bottle stays pressed
// against the pusher and stops with it: S_SLIDE ~ 0 and repeatable.
constexpr float A_RELEASE  = 0.6f;
constexpr float A_REV_STOP = 0.5f;   // final decel into the Target

// ---- Run geometry tuning, metres -------------------------------------------
constexpr float CAPTURE_LEAD_M = 0.15f;  // slow to V_CAPTURE this far before contact
constexpr float SEAT_MARGIN_M  = 0.03f;  // stay slow this far past contact
// Turnaround dwell between the forward stop and the reverse leg. The rules end a
// run after 3 s stationary (§8.p) — see the static_assert in §8.
constexpr uint32_t TURN_MS     = 400;

// ---- Timing -----------------------------------------------------------------
// Start delay = T - t_motion_est. T_BIAS_S corrects the motion-time model:
// set it to (measured motion time - estimated) averaged over a few full runs
// (both are printed after every run).
constexpr float T_BIAS_S = 0.0f;       // <-- CALIBRATE
// Planner numerics (plan.cpp): bisection steps when finding the smallest speed-up
// (30 halvings of a <3x range: ~1e-9, far below anything physical), and the slack
// allowed when deciding a plan "fits" inside T.
constexpr int   PLAN_BISECT_ITERS = 30;
constexpr float PLAN_TIME_TOL_S   = 1e-3f;
// Run event log (sm.cpp): a run logs ~10 events; the rest is room for IMU and
// abort entries. Entries past this are dropped.
constexpr int   RUN_LOG_CAPACITY  = 24;

// Live LCD telemetry is written ONLY while the car is stationary (start delay):
// an LCD write is ~30 ms of blocking I2C, which mid-motion would delay the stop.
constexpr uint32_t LCD_RUN_UPDATE_MS = 250;   // 4 Hz
// ...and not within this long of the launch, so a write can't make it late.
// 300 ms: even a write on a failing bus (~270 ms, every byte timing out) ends
// before the launch.
constexpr uint32_t LCD_LAUNCH_GUARD_MS = 300;
// A delay-screen write taking longer than this means the LCD bus is failing (a
// healthy one is ~30 ms); the delay loop then stops writing the LCD (sm.cpp).
constexpr uint32_t LCD_SLOW_WRITE_MS = 60;

// ---- Pose zero at START (pose.cpp) -------------------------------------------
// While the estimator runs, core1 owns the IMU and captures the heading zero.
// If it doesn't answer within this time (IMU stuck mid-read), the run goes
// encoder-only instead of steering on an unzeroed heading. Normally ~2 ms (one
// core1 tick). Kept short: it runs after the pencil press, so it eats into the
// start delay, and SET isn't polled while it waits.
constexpr uint32_t POSE_ZERO_TIMEOUT_MS = 400;
// When core0 zeroes with the estimator idle (bench tools), it first reads this
// many reports, 2 ms apart: the BNO085 queues reports, so the first read after an
// idle spell can be an OLD heading.
constexpr int      IMU_DRAIN_READS      = 25;
constexpr uint32_t IMU_DRAIN_SPACING_MS = 2;

// ---- Loop rates ------------------------------------------------------------
constexpr int   CONTROL_TICK_HZ = 500;    // core1 estimator + core0 control

// Ground-speed estimate from the free roller (odo.cpp). Differentiating distance
// at the full 500 Hz tick is dominated by encoder quantisation, so the estimator
// re-samples on a longer window and then low-passes the result.
constexpr float SPEED_SAMPLE_MIN_S = 0.020f;  // ~50 Hz resample window
constexpr float SPEED_FILTER_TAU_S = 0.100f;  // single-pole time constant

// ---- IMU health ---------------------------------------------------------------
// imuHealthy() is false once the newest Game Rotation Vector sample is older than
// this. The report streams at 100 Hz, so 150 ms is ~15 missed samples. On loss
// the run DEGRADES (servo centred, encoder only) rather than stopping: a mid-track
// stop ends the run (§8.p).
constexpr uint32_t IMU_STALE_MS  = 150;
constexpr uint32_t IMU_WARN_MS   = 5000;  // serial warning + failure count
// A heading gap longer than this can't be trusted for the rest of the run: the
// zero would be on a sample from before the car was placed, or the estimator has
// integrated position blind for too long. Steering then stays straight (sm.cpp).
// Shorter gaps (an I2C retry) only pause steering until samples resume.
constexpr uint32_t IMU_TRUST_GAP_MS = 500;
// After a chip reset, or a re-enable that failed, retry enabling the heading
// report this often until it streams again. Without a retry, one failed re-enable
// left the IMU dead until a power cycle.
constexpr uint32_t IMU_RETRY_MS  = 500;

//==============================================================================
// §5  RULES & STRATEGY — 2027 Division C manual (docs/rules/EV_Rules_2027.txt)
//==============================================================================
constexpr float TARGET_DIST_MIN_M = 5.00f,  TARGET_DIST_MAX_M = 9.00f;   // §8.h
constexpr float BOTTLE_LINE_MIN_M = 1.00f,  BOTTLE_LINE_MAX_M = 3.00f;   // §8.j
constexpr float TARGET_TIME_MIN_S = 10.0f,  TARGET_TIME_MAX_S = 20.0f;   // §8.i
constexpr float VEHICLE_LEN_MAX_M = 0.80f;                               // §3.d

// Value-entry steps (ui.cpp): the rules intervals, coarsest first; the dial
// button cycles through them. D and B: 0.25 m regional, 0.10 state, 0.01 national
// (§8.h, §8.j), plus 1.0 m to travel fast. T: always 0.5 s steps (§8.i).
constexpr float ENTRY_STEPS_M[] = {0.25f, 0.10f, 0.01f, 1.0f};
constexpr float ENTRY_STEPS_S[] = {0.5f, 1.0f};

// Bottle CENTRE position from Start, by tournament level (§7.c.ii.5).
constexpr float BOTTLE_X_REGIONAL_M = 2.0f;
constexpr float BOTTLE_X_STATE_M    = 3.0f;
constexpr float BOTTLE_X_NATIONAL_M = 4.0f;
// Bottle radius: 6-7 cm diameter (§8.k).
constexpr float R_BOTTLE_M = 0.033f;

// Aim the bottle's NEAR side this far past the Bottle Line. Fully past earns -20
// (§5.f) and a few cm past costs only 1 pt/cm, so aim past, not at, the line.
constexpr float A_PAST_M  = 0.05f;
// How far the bottle slides on after the pusher stops. ~0 with a gentle release.
constexpr float S_SLIDE_M = 0.0f;      // <-- MEASURE on a competition-like floor

// Run-2 corrections. The better of two runs counts, so run 1 calibrates run 2.
// Each is a CHANGE to the value already set (the dial keeps the last one):
//   MP offset:     measure where the rear MP stopped. Stopped 3 cm BEYOND the
//                  Target (bottle side) -> subtract 0.030; 2 cm short (Start side)
//                  -> add 0.020. From 0 that gives -0.030 / +0.020.
//   Bottle offset: + pushes the bottle further. Bottle near side 2 cm short of
//                  where you wanted it -> add 0.020.
// Both apply to their own stop only: the MP offset never moves the forward stop,
// and the bottle offset never moves the final MP stop.
constexpr float STOP_OFFSET_MIN_M = -0.50f;
constexpr float STOP_OFFSET_MAX_M =  0.50f;

// Shortest reverse leg the plan accepts — only enough to guarantee the car really
// backs up. Rules give B >= 1.00 with a car <= 0.80 long, but run-2 offsets and
// S_SLIDE_M on a long car can legitimately bring it near 0.2 m, and refusing those
// plans would block run 2. A leg shorter than REV_STEER_HOLD_M is fine: AimAtTarget
// then just holds the wheels where the forward leg left them (on the line).
constexpr float REV_MIN_M = 0.05f;
// Travel the encoder must see before the result screen calls it a run. A few mm:
// more than one count of jitter from the pencil press, less than any real launch.
// (§4.d.vii.4.a: a vehicle that doesn't move when triggered has not had a run.)
constexpr float MOVED_MIN_M = 0.003f;
// ARMED ignores START for this long after it appears (sm.cpp): time for core1 to
// drain the IMU's queued reports so the heading zero is current.
constexpr uint32_t ARMED_SETTLE_MS = 300;
// START (at ARMED) and SET (as the abort) must stay pressed this long to count.
// A real press lasts tens of ms; a noise spike from the motor or servo wiring is
// one sample. Run Time still counts from the moment the START press began.
constexpr uint32_t BUTTON_QUALIFY_MS = 6;
// After a button is released, ignore it this long: a contact bounce on release
// would otherwise read as a NEW press and skip the next entry screen (keeping
// last run's value for it without anyone touching it).
constexpr uint32_t BUTTON_RELEASE_SETTLE_MS = 50;
// Shortest push (seated -> forward stop) the plan accepts.
constexpr float PUSH_MIN_M = 0.10f;

// ---- Guards -----------------------------------------------------------------
// Runaway watchdog: abort after delay + t_motion_est * FACTOR + PAD seconds.
constexpr float RUN_TIMEOUT_FACTOR = 2.0f;
constexpr float RUN_TIMEOUT_PAD_S  = 5.0f;
// Stuck at low speed: if NO encoder count arrives for STALL_KICK_MS while a
// moving state is driving, re-apply the stiction kick (KICK_PWM). At slow targets
// (end of the reverse ramp, ~0.05 m/s) the speed error is tiny, so the PI alone
// winds up too slowly to break stiction before the dead-encoder check fires.
constexpr uint32_t STALL_KICK_MS = 150;
// ...as a RAMP up to KICK_PWM at this rate, not a step: the car may be mid-push
// with the bottle seated (a jolt can tip it), or near xFinal (a step lurches it
// past the Target). 150 %/s reaches 35 % in ~0.23 s.
constexpr float    STALL_BOOST_PCT_PER_S = 150.0f;
// "Stuck" means less than this much progress in the leg's direction within
// STALL_KICK_MS. Above one count, so roller jitter doesn't count as moving; well
// under what even the slowest ramp floor covers in that time (0.05 m/s -> 7.5 mm).
constexpr float    STUCK_MIN_M = 0.002f;
// Dead-encoder watchdog: abort if there is still NO count after the re-kick has
// had its chance. Without counts the speed loop sees v = 0 and winds up, so this
// must stay short. A moving car produces hundreds of counts per second; a car that
// never moved at the launch means no run occurred (§4.d.vii.4.a).
constexpr uint32_t ENC_DEAD_MS = 500;
// Stall watchdog: abort if a moving state makes less than PROGRESS_MIN_M of
// progress in STALL_ABORT_MS (slow creep that isn't getting anywhere). The run is
// over at 3 s stationary anyway.
constexpr uint32_t STALL_ABORT_MS = 2000;
constexpr float    PROGRESS_MIN_M = 0.01f;
// Wrong-way guard: abort if the car moves this far against the leg's direction.
// Travelling the wrong way from the Start is a Failed Run (§8.r.i); on the reverse
// leg, creeping forward would push the bottle.
constexpr float WRONG_WAY_M = 0.03f;
// Rules §8.p: a run ends after 3 s stationary. Planned stationary time must stay
// far below that (checked in §8).
constexpr uint32_t RULE_STOP_MS = 3000;

//==============================================================================
// §5b  UI MODE
//------------------------------------------------------------------------------
// 1 = dial entry. The run asks for D, B, T, level and the two offsets on the LCD;
//     targets are announced after impound (§4.b), so this is the fast path.
// 0 = no dial. The run uses the DEFAULT_* values below — edit and reflash.
//     Bench tools still work, using SET = increment and START = confirm.
//==============================================================================
#define USE_DIAL_ENTRY 1

// How long a status or error screen stays up before the UI moves on.
constexpr uint32_t UI_MSG_MS  = 2500;   // errors: NOT CALIBRATED, PLAN ERROR
constexpr uint32_t UI_PLAN_MS = 1500;   // the plan summary before ARMED

// Used as the first-run defaults with the dial, and as the values without it.
constexpr float DEFAULT_TARGET_D_M    = 7.00f;
constexpr float DEFAULT_BOTTLE_LINE_M = 2.00f;
constexpr float DEFAULT_TARGET_T_S    = 15.0f;
constexpr int   DEFAULT_LEVEL         = 0;      // 0 regional, 1 state, 2 national
constexpr float DEFAULT_MP_OFFSET_M   = 0.0f;
constexpr float DEFAULT_BTL_OFFSET_M  = 0.0f;

//==============================================================================
// §6  PIN MAP — see docs/WIRING_GUIDE.md
//==============================================================================

// Pin choices below match the KEYESTUDIO KS3017 Pico IO shield's DEDICATED
// 4-pin I2C connectors, which carry SDA/SCL/3V3/GND together.

// ---- I2C0 — BNO085 IMU ONLY ------------------------------------------------
// The IMU has this bus to itself. It is read from core1 at the control tick.
#define I2C_SDA_PIN       20    // GP20 (pin 26)
#define I2C_SCL_PIN       21    // GP21 (pin 27)  3.3 V bus
#define BNO085_INT_PIN     6    // GP6  (pin 9)   optional; UNUSED by firmware
#define BNO085_RST_PIN     7    // GP7  (pin 10)  optional clean reset

// ---- I2C1 — 16x2 LCD ONLY --------------------------------------------------
// A SEPARATE controller from the IMU, so the two can never collide.
#define LCD_I2C_INST       i2c1
#define LCD_SDA_PIN       14    // GP14 (pin 19)
#define LCD_SCL_PIN       15    // GP15 (pin 20)
#define LCD_I2C_ADDR       0x27  // some backpacks are 0x3F
// Per-byte write timeout. One byte at 100 kHz takes ~0.2 ms; this only bounds a
// bus held low by a bad cable (lcd.cpp).
constexpr uint32_t LCD_I2C_TIMEOUT_US = 2000;

// ---- Distance: free-rolling FRONT encoder (undriven wheel, no slip) ---------
// Decoded in PIO, both directions. A and B MUST be consecutive GPIOs.
// Signswise 600 P/R, NPN open-collector, 5-24 V: power at 5 V and pull A/B up to
// 3.3 V (4.7k). NEVER pull up to 5 V — that puts 5 V on the GPIO.
#define FRONT_ENCODER_A_PIN   10   // GP10 (pin 14)  green
#define FRONT_ENCODER_B_PIN   11   // GP11 (pin 15)  white

// ---- Motor driver: BTS7960 / IBT-2 (single 550 rear drive) -----------------
#define BTS7960_RPWM_PIN   16   // GP16 (pin 21)  forward PWM
#define BTS7960_LPWM_PIN   17   // GP17 (pin 22)  reverse PWM
#define BTS7960_R_EN_PIN   18   // GP18 (pin 24)  right half-bridge inhibit
#define BTS7960_L_EN_PIN   26   // GP26 (pin 31)  left half-bridge inhibit

// ---- Onboard LED: heartbeat ------------------------------------------------
// Blinks from core1 so it proves the ESTIMATOR is running, not just that power is
// on — a solid or dark LED means core1 has stopped.
constexpr uint32_t HEARTBEAT_MS = 500;   // toggle interval; 0 disables

// ---- Steering servo --------------------------------------------------------
// GP5 = PWM slice 2 channel B. Slice 2 channel A is GP4 (START), a plain GPIO
// input — so the servo owns the slice. V+ from the 5 V rail, never 3V3.
#define SERVO_SIG_PIN       5   // GP5  (pin 7)   50 Hz, 1000-2000 us

// ---- UI: rotary dial + buttons (all active-low, internal pull-ups) ----------
#define DIAL_CLK_PIN        2   // GP2  (pin 4)
#define DIAL_DT_PIN         3   // GP3  (pin 5)
#define DIAL_BTN_PIN       22   // GP22 (pin 29)  cycles the increment
#define START_BTN_PIN       4   // GP4  (pin 6)   #2-pencil trigger, VERTICAL only
#define SET_BTN_PIN        19   // GP19 (pin 25)  confirm / abort / hold-for-bench

//==============================================================================
// §7  BENCH TOOLS — calibration mode (cal.cpp)
//==============================================================================
// PWM->speed probe: the car accelerates through CAL_WARMUP_M (discarded launch
// transient), then average speed is timed over exactly CAL_MEASURE_M.
constexpr float CAL_WARMUP_M  =  1.00f;
constexpr float CAL_MEASURE_M =  2.00f;
constexpr float CAL_TIMEOUT_S = 20.0f;
constexpr float CAL_PWM_MIN   = 5.0f;
constexpr float CAL_PWM_MAX   = 100.0f;
constexpr uint32_t MOTOR_TEST_MS = 1500;   // motor-test burst length
// The marked length rolled by hand in "1 Encoder read". Measure a different one
// (less floor space) by changing it here; the printed constants follow.
constexpr float CAL_REF_LENGTH_M = 5.000f;
// "7 Steer response", REV mode: the target sits this far behind where the MP was
// when the pose was zeroed, so the reverse law has something to aim at.
constexpr float CAL_STEER_REV_TARGET_M = 1.0f;
// Hold SET this long at the "EV ready" screen to open the bench menu without a
// power cycle (a reboot drops the USB serial monitor).
constexpr uint32_t CAL_ENTRY_HOLD_MS = 2000;

//==============================================================================
// §8  DERIVED — computed from the above; do not edit
//==============================================================================
constexpr float M_PER_COUNT_REV_EFF =
    (M_PER_COUNT_REV > 0.0f) ? M_PER_COUNT_REV : M_PER_COUNT;
constexpr int   CONTROL_TICK_US = 1000000 / CONTROL_TICK_HZ;
// The steering limit the servo can actually deliver: DELTA_MAX_RAD, or less if
// SERVO_TRAVEL_US cuts in first (0.12 rad needs ~428 us at ~3565 us/rad, but
// travel stops at 400 us). The laws clamp to this, so they never plan on more
// authority than the wheels have. Until SERVO_US_PER_RAD is measured, it is
// just DELTA_MAX_RAD.
constexpr float DELTA_LIMIT_RAD =
    (SERVO_US_PER_RAD > 0.0f && SERVO_TRAVEL_US > 0 &&
     SERVO_TRAVEL_US / SERVO_US_PER_RAD < DELTA_MAX_RAD)
        ? SERVO_TRAVEL_US / SERVO_US_PER_RAD : DELTA_MAX_RAD;

static_assert(TURN_MS + KICK_MS < RULE_STOP_MS / 2,
              "turnaround + reverse kick must stay far below the 3 s stop rule");
static_assert(STALL_ABORT_MS < RULE_STOP_MS,
              "stall abort must fire before the rules end the run");
static_assert(V_PUSH_MAX >= V_PUSH && V_CRUISE_MAX >= V_CRUISE_NOM && V_REV_MAX >= V_REV,
              "each _MAX speed must be >= its nominal");
static_assert(V_CAPTURE > 0.0f && V_PUSH_END > 0.0f && V_REV_END > 0.0f,
              "ramp floors must be > 0 or the car stalls before its stop");
constexpr bool USES_PURE_PURSUIT =
    STEER_FWD_LAW == SteerLaw::PurePursuit || STEER_REV_LAW == RevSteerLaw::PurePursuit;
// The §1 values STEERING needs, by name, or nullptr if all are set. Used by the
// bench steer test, which must work before the pusher exists (no L_P_M yet).
// Names fit the LCD's second line.
constexpr const char *steeringCalibrationMissing() {
  return M_PER_COUNT <= 0.0f                          ? "M_PER_COUNT=0"
       : ENC_FROM_MP_M <= 0.0f                        ? "ENC_FROM_MP_M=0"
       : SERVO_US_PER_RAD <= 0.0f                     ? "SERVO_US/RAD=0"
       : (USES_PURE_PURSUIT && WHEELBASE_L_M <= 0.0f) ? "WHEELBASE_L_M=0"
       : nullptr;
}
// Everything a RUN needs: steering plus the pusher geometry. sm.cpp refuses to arm
// until this is nullptr.
constexpr const char *calibrationMissing() {
  return L_P_M <= 0.0f ? "L_P_M=0" : steeringCalibrationMissing();
}

static_assert(!USES_PURE_PURSUIT || WHEELBASE_L_M < PP_LOOKAHEAD_M,
              "pure pursuit: PP_LOOKAHEAD_M must be longer than WHEELBASE_L_M");
static_assert(STALL_KICK_MS + 1000.0f * KICK_PWM / STALL_BOOST_PCT_PER_S < ENC_DEAD_MS &&
              STALL_KICK_MS + KICK_MS < ENC_DEAD_MS && ENC_DEAD_MS < STALL_ABORT_MS,
              "dead-encoder check: give the stall re-kick a chance, fire before the stall check");
