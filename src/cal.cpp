#include "cal.h"
#include "config.h"
#include "drive.h"
#include "lcd.h"
#include "hardware/gpio.h"
#include <algorithm>
#include "imu.h"
#include "pose.h"
#include "odo.h"
#include "utils.h"
#include <cmath>
#include "steer.h"
#include "steer_law.h"
#include "ui.h"
#include "pico/stdlib.h"
#include <cstdio>

// Average speed is timed over a fixed DISTANCE rather than read from
// odoSpeedMps(). Over a 2 m window, distance/time is exact — no filter lag and
// no quantisation worth worrying about. Run it in BOTH directions: the reverse
// leg runs on the same feedforward line (config.h §2), so check they agree.
void calPwmProbe() {
  static const char *dirs[] = {"1 Forward", "2 Reverse", "3 Back"};
  float pwm = 30.0f;

  printf("\n=== PWM -> speed calibration ===\n");
  printf("Pick a direction, dial a duty %%, press START. Needs ~3.5 m of floor.\n");
  printf("SET aborts a run in progress. Fit PWM = offset + slope*v through the points\n");
  printf("and enter them as SPEED_FF_OFFSET_PCT / SPEED_FF_PCT_PER_MPS.\n\n");

  while (true) {
    const int choice = uiMenuSelect("PWM->speed", dirs, 3);
    if (choice == 2) return;
    const int dir = (choice == 1) ? -1 : +1;
    pwm = uiEnterPwmPercent(pwm);

    // The whole measurement is distance-based, so an uncalibrated M_PER_COUNT
    // would simply hang here. Fail loudly instead.
    if (M_PER_COUNT <= 0.0f) {
      printf("[CAL] M_PER_COUNT is 0 — do the 5.000 m distance calibration FIRST.\n");
      uiShow("M_PER_COUNT=0", "dist cal first");
      sleep_ms(2500);
      continue;
    }

    char l1[17];
    snprintf(l1, sizeof l1, "%s %.0f%% ready", dir > 0 ? "FWD" : "REV", pwm);
    uiShow(l1, "START=go SET=bk");
    bool backOut = false;
    while (!uiStartPressed()) {
      if (uiSetPressed()) { uiWaitSetRelease(); backOut = true; break; }   // wrong choice / floor not clear
      sleep_ms(2);
    }
    if (backOut) continue;

    steerCenter();              // wheels straight — this is a straight-line test
    odoZero();

    const uint32_t t0 = to_ms_since_boot(get_absolute_time());
    uint32_t tA = 0, tB = 0;    // window start/end timestamps
    bool aborted = false;

    if (dir > 0) driveForward(pwm); else driveReverse(pwm);
    // The same runaway guards as a run: without them a slipped encoder belt, a
    // loose connector or a flipped sign (distance never grows the commanded way)
    // drives on at the dialled duty until CAL_TIMEOUT_S — tens of metres.
    float    progD  = 0.0f;
    uint32_t progMs = t0;
    const char *why = nullptr;
    HeldButton setHeld;                      // a noise sample must not abort the point
    while (true) {
      const uint32_t now = to_ms_since_boot(get_absolute_time());
      const float    d   = dir * odoDistanceM();     // distance in the run direction
      if (tA == 0 && d >= CAL_WARMUP_M) tA = now;                      // launch done
      if (tA != 0 && d >= CAL_WARMUP_M + CAL_MEASURE_M) { tB = now; break; }
      if (d - progD >= PROGRESS_MIN_M) { progD = d; progMs = now; }
      if (setHeld.update(uiSetPressed(), now))     { why = "SET";        break; }
      if (d < -WRONG_WAY_M)                        { why = "wrong way";  break; }
      // Same window as a run's stall check, not the dead-encoder one: the slow
      // points just above stiction are the ones that find the dead-zone offset.
      if (now - progMs > STALL_ABORT_MS)           { why = "no progress"; break; }
      if ((now - t0) / 1000.0f > CAL_TIMEOUT_S)    { why = "timeout";    break; }
      sleep_ms(2);
    }
    aborted = (why != nullptr);

    // Wrong way usually means a flipped encoder sign or swapped motor leads, and then
    // "against the measured motion" is the way the car is already going: hold only.
    const bool wrongWay = why && why[0] == 'w';
    driveBrakeToStop(wrongWay ? 0.0f : ABORT_BRAKE_PWM, ABORT_BRAKE_MAX_MS);
    sleep_ms(STOP_SETTLE_MS);
    driveCoast();

    if (aborted || tB == 0) {
      printf("[CAL] %s pwm=%3.0f%%  ABORTED: %s (travelled %.2f m)\n",
             dir > 0 ? "FWD" : "REV", pwm, why ? why : "?", odoDistanceM());
      uiShow("ABORTED", why ? why : "check path/PWM");
      sleep_ms(2000);
      continue;
    }

    const float dt = (tB - tA) / 1000.0f;
    const float v  = CAL_MEASURE_M / dt;
    printf("[CAL] %s pwm=%3.0f%%  v=%.3f m/s   (%.2f m in %.2f s)\n",
           dir > 0 ? "FWD" : "REV", pwm, v, CAL_MEASURE_M, dt);
    char l2[17];
    snprintf(l1, sizeof l1, "%s PWM %.0f%%", dir > 0 ? "FWD" : "REV", pwm);
    snprintf(l2, sizeof l2, "%.3f m/s", v);
    uiShow(l1, l2);
    sleep_ms(3000);
  }
}

//==============================================================================
// Motor + driver test
//==============================================================================
// Runs a short burst at a dialled PWM and reports the ENCODER delta alongside it.
// That single number checks four things at once: the driver responds, the motor
// turns, it turns the right way, and the encoder counts the right way. Walking
// the PWM down until the car stops moving also gives you the real dead-zone
// floor; KICK_PWM (config.h §2) should sit comfortably above it in both directions.
void calMotorTest() {
  static const char *items[] = {"1 Forward", "2 Reverse", "3 Back"};
  float pwm = 20.0f;

  while (true) {
    const int choice = uiMenuSelect("Motor test", items, 3);
    if (choice == 2) return;
    const bool reverse = (choice == 1);

    pwm = uiEnterPwmPercent(pwm);

    char l1[17];
    snprintf(l1, sizeof l1, "%s %.0f%%", reverse ? "REV" : "FWD", pwm);
    uiShow(l1, "START=go SET=bk");
    while (true) {
      if (uiStartPressed()) break;
      if (uiSetPressed()) { sleep_ms(200); return; }
      sleep_ms(2);
    }

    steerCenter();                            // straight: a run may have left them angled
    odoZero();                                // so the distance uses the run's own scaling
    const uint64_t t0 = to_ms_since_boot(get_absolute_time());

    if (reverse) driveReverse(pwm);
    else         driveForward(pwm);

    while (to_ms_since_boot(get_absolute_time()) - t0 < MOTOR_TEST_MS) {
      if (uiSetPressed()) break;              // let go early
      sleep_ms(2);
    }
    driveCoast();                             // coast, don't brake — we want to
    sleep_ms(400);                            // see how far it actually rolled

    const DirSplit snap = odoSnapshot();     // one sample for both numbers
    const long  dc = snap.net();
    const float dm = snap.metres(M_PER_COUNT, M_PER_COUNT_REV_EFF);
    printf("[MOT] %s pwm=%3.0f%%  counts=%+ld  dist=%+.3f m%s\n",
           reverse ? "REV" : "FWD", pwm, dc, dm,
           (dc == 0) ? "   <-- NO MOVEMENT" : "");
    char l2[17];
    snprintf(l1, sizeof l1, "cnt %+ld", dc);
    snprintf(l2, sizeof l2, "%+.3f m", dm);
    uiShow(l1, l2);
    sleep_ms(2500);
  }
}

//==============================================================================
// Servo jog
//==============================================================================
// The dial moves the servo immediately, one increment per detent. Use it to find
// true centre before fitting the horn, to walk out to the mechanical limits, and
// to measure beam angle against pulse width (that ratio IS SERVO_US_PER_RAD).
void calServoTest() {
  uiRotaryResync();                         // no phantom first step
  // Jog only inside the window steerWriteUs() really sends (the travel limit, then
  // the electrical range). Showing a pulse width beyond it would make the
  // SERVO_US_PER_RAD measurement divide by an offset the servo never got.
  const int jogLo = std::max(SERVO_MIN_US, SERVO_TRAVEL_US > 0 ? SERVO_CENTER_US - SERVO_TRAVEL_US : SERVO_MIN_US);
  const int jogHi = std::min(SERVO_MAX_US, SERVO_TRAVEL_US > 0 ? SERVO_CENTER_US + SERVO_TRAVEL_US : SERVO_MAX_US);
  int us = SERVO_CENTER_US;
  int incIdx = 0;
  static const int incs[] = {10, 1, 100};

  printf("\n=== Servo jog ===\n");
  printf("Dial moves the servo LIVE. Dial-button cycles step, SET exits.\n");
  printf("WARNING: the linkage allows only ~+/-7 deg of beam. With the drag link\n");
  printf("attached, stop as soon as it stiffens — holding against a mechanical\n");
  printf("stop stalls the servo. Safest with the horn OFF the link.\n\n");

  uiShow("Servo jog", "link OFF is safe");
  sleep_ms(2000);

  steerWriteUs(us);
  bool lastSet  = uiSetPressed();
  bool lastDial = uiDialPressed();
  auto render = [&]() {
    char l1[17], l2[17];
    snprintf(l1, sizeof l1, "us %4d", us);
    snprintf(l2, sizeof l2, "step %d  d%+d", incs[incIdx], us - SERVO_CENTER_US);
    lcdClear(); lcdPrintAt(0, 0, l1); lcdPrintAt(0, 1, l2);
    printf("[SRV] us=%4d  (centre %+d)\n", us, us - SERVO_CENTER_US);
  };
  render();

#if USE_DIAL_ENTRY
  // Dial jogs, dial-button cycles the step, SET exits.
  while (true) {
    const int step = uiRotaryStep();
    if (step != 0) {
      us = std::clamp(us + step * incs[incIdx], jogLo, jogHi);
      steerWriteUs(us);
      render();
    }

    const bool db = uiDialPressed();
    if (!lastDial && db) {                      // press edge: cycle step size
      incIdx = (incIdx + 1) % 3;
      render();
      sleep_ms(200);
    }
    lastDial = db;

    const bool st = uiSetPressed();
    if (!lastSet && st) {                       // press edge
      sleep_ms(50);
      uiWaitSetRelease();
      steerWriteUs(SERVO_CENTER_US);            // leave it centred
      printf("[SRV] exit — returned to centre %d us\n", SERVO_CENTER_US);
      return;
    }
    lastSet = st;
    sleep_ms(2);
  }
#else
  // No dial: SET jogs (auto-repeats, wraps at the top so you can get back down),
  // a START tap cycles the step size, and holding START for 1 s exits. Each
  // repeat writes the servo, so you still get live movement to watch.
  (void)lastDial;
  uiShow("SET=jog START=", "tap step/hold x");
  sleep_ms(1800);
  render();
  uint32_t heldSince = 0, lastRepeat = 0, startDown = 0;

  while (true) {
    const uint32_t now = to_ms_since_boot(get_absolute_time());

    const bool sp = uiSetPressed();
    bool bump = false;
    if (!lastSet && sp) { bump = true; heldSince = now; lastRepeat = now; }
    else if (sp && (now - heldSince) > 500) {
      const uint32_t period = (now - heldSince > 2000) ? 40 : 150;
      if (now - lastRepeat >= period) { bump = true; lastRepeat = now; }
    }
    lastSet = sp;
    if (bump) {
      us += incs[incIdx];
      if (us > jogHi) us = jogLo;                  // wrap
      steerWriteUs(us);
      render();
    }

    if (uiStartPressed()) {
      if (startDown == 0) startDown = now;
      if (now - startDown > 1000) {               // held = exit
        while (uiStartPressed()) sleep_ms(5);
        steerWriteUs(SERVO_CENTER_US);
        printf("[SRV] exit — returned to centre %d us\n", SERVO_CENTER_US);
        return;
      }
    } else if (startDown != 0) {                  // released short = tap
      incIdx = (incIdx + 1) % 3;
      render();
      startDown = 0;
    }
    sleep_ms(2);
  }
#endif
}


//==============================================================================
// Live encoder readout
//==============================================================================
// Reads raw counts rather than odoDistanceM(), which reads 0 while M_PER_COUNT is
// uncalibrated — the exact situation this mode exists to fix. Forward and reverse
// counts are shown separately: push the car 5.000 m FORWARD for M_PER_COUNT, then
// pull it 5.000 m BACK for M_PER_COUNT_REV.
void calEncoderReadout() {
  printf("\n=== Encoder readout ===\n");
  printf("START zeroes the counts, SET exits.\n");
  printf("START at the start mark, push %.3f m forward -> M_PER_COUNT.\n", (double)CAL_REF_LENGTH_M);
  printf("START again at the far mark, pull %.3f m back -> M_PER_COUNT_REV.\n\n", (double)CAL_REF_LENGTH_M);

  odoZero();
  long lastF = -1, lastR = 1, lastN = 1;
  int  lastAB = -1;
  bool lastStart = uiStartPressed();
  bool lastSet   = uiSetPressed();

  while (true) {
    const DirSplit snap = odoSnapshot();   // one sample, so f, r and n agree
    const long f = snap.fwd;
    const long r = snap.rev;
    const long n = snap.net();
    // Raw A/B levels. gpio_get() reads the pad input regardless of the pin being
    // under PIO control, so this distinguishes "encoder dead" from "decoder not
    // working".
    const int a  = gpio_get(FRONT_ENCODER_A_PIN) ? 1 : 0;
    const int b  = gpio_get(FRONT_ENCODER_B_PIN) ? 1 : 0;
    const int ab = (a << 1) | b;

    if (f != lastF || r != lastR || n != lastN || ab != lastAB) {
      lastF = f; lastR = r; lastN = n; lastAB = ab;
      char l1[17], l2[17];
      snprintf(l1, sizeof l1, "F%+ld R%+ld", f, r);
      snprintf(l2, sizeof l2, "A%d B%d net%+ld", a, b, n);
      lcdClear(); lcdPrintAt(0, 0, l1); lcdPrintAt(0, 1, l2);
      printf("[ENC] A=%d B=%d  fwd=%+ld  rev=%+ld  net=%+ld", a, b, f, r, n);
      // Calibrate on the NET count since the last zero, never on fwd or rev alone:
      // any back-and-forth during the hand push (overshooting the mark and easing
      // back) adds the same counts to BOTH, which would bias both constants low.
      if (n > 0) printf("   if this was %.3f m forward: M_PER_COUNT=%.8f",
                        (double)CAL_REF_LENGTH_M, CAL_REF_LENGTH_M / (float)n);
      if (n < 0) printf("   if this was %.3f m back: M_PER_COUNT_REV=%.8f",
                        (double)CAL_REF_LENGTH_M, CAL_REF_LENGTH_M / (float)(-n));
      printf("\n");
    }

    const bool sp = uiStartPressed();
    if (!lastStart && sp) { odoZero(); lastF = -1; }   // press edge = zero
    lastStart = sp;

    const bool st = uiSetPressed();
    if (!lastSet && st) {
      uiWaitSetRelease();
      return;
    }
    lastSet = st;

    sleep_ms(20);
  }
}


//==============================================================================
// Input test
//==============================================================================
// All five UI inputs are active-low with internal pull-ups: 1 = released,
// 0 = pressed. Rotating the dial should toggle CLK and DT out of phase.
void calInputTest() {
  printf("\n=== Input test ===  (1 = released, 0 = pressed)\n");
  printf("Press each button and rotate the dial. Hold START 1.5 s to exit.\n\n");

  int last = -1;
  uint32_t startDown = 0;
  while (true) {
    const int set  = gpio_get(SET_BTN_PIN)   ? 1 : 0;
    const int strt = gpio_get(START_BTN_PIN) ? 1 : 0;
    const int dial = gpio_get(DIAL_BTN_PIN)  ? 1 : 0;
    const int clk  = gpio_get(DIAL_CLK_PIN)  ? 1 : 0;
    const int dt   = gpio_get(DIAL_DT_PIN)   ? 1 : 0;
    const int code = (set<<4)|(strt<<3)|(dial<<2)|(clk<<1)|dt;

    if (code != last) {
      last = code;
      char l1[17], l2[17];
      snprintf(l1, sizeof l1, "SET%d STRT%d DIAL%d", set, strt, dial);
      snprintf(l2, sizeof l2, "CLK%d DT%d exit=ST", clk, dt);
      lcdClear(); lcdPrintAt(0, 0, l1); lcdPrintAt(0, 1, l2);
      printf("[IN ] SET=%d START=%d DIAL=%d | CLK=%d DT=%d\n",
             set, strt, dial, clk, dt);
    }

    const uint32_t now = to_ms_since_boot(get_absolute_time());
    if (!strt) {                       // START held (active low)
      if (startDown == 0) startDown = now;
      if (now - startDown > 1500) {
        while (!gpio_get(START_BTN_PIN)) sleep_ms(5);
        printf("[IN ] exit\n");
        return;
      }
    } else {
      startDown = 0;
    }
    sleep_ms(20);
  }
}


//==============================================================================
// IMU heading readout
//==============================================================================
void calImuTest() {
  printf("\n=== IMU heading ===\n");
  // Do NOT gate entry on imuHealthy(). That flag is a FRESHNESS check — it only
  // stays true while something keeps calling imuYawDeg(), and in this menu nothing
  // does, so it goes stale after IMU_STALE_MS and reads false on a perfectly
  // healthy chip. Prime it with a few polls, then just display the live state and
  // let the reader judge.
  uiShow("IMU heading", "priming...");
  for (int i = 0; i < 30 && !imuHealthy(); i++) {
    (void)imuYawDeg();
    sleep_ms(20);
  }
  if (!imuHealthy()) {
    printf("[IMU] still not streaming after priming — check I2C wiring.\n");
  }
  printf("Turn the car LEFT by hand: 'rel' must INCREASE. 'rel' is the heading as\n");
  printf("the run sees it (HEADING_SIGN applied), so after a flip and a rebuild this\n");
  printf("screen confirms it. If it decreases, flip HEADING_SIGN in config.h §3.\n");
  printf("START zeroes the reference, SET exits.\n\n");

  float zero = (float)imuYawDeg();
  float lastShown = 1e9f;
  bool lastStart = uiStartPressed(), lastSet = uiSetPressed();

  while (true) {
    const float yaw = (float)imuYawDeg();
    // Relative to the captured zero, wrapped to +/-180 so it reads naturally
    // rather than jumping 359 -> 0 as you rotate through the wrap point, and with
    // HEADING_SIGN applied exactly as pose.cpp does — otherwise a flip could never
    // be confirmed here.
    const float rel = (float)(HEADING_SIGN * utils::angleError(yaw, zero, false));

    if (std::fabs(rel - lastShown) > 0.2f) {
      lastShown = rel;
      char l1[17], l2[17];
      snprintf(l1, sizeof l1, "yaw %6.1f %s", (double)yaw, imuHealthy() ? "OK " : "BAD");
      snprintf(l2, sizeof l2, "rel %+6.1f ST=0", (double)rel);
      lcdClear(); lcdPrintAt(0, 0, l1); lcdPrintAt(0, 1, l2);
      printf("[IMU] yaw=%7.2f  rel=%+7.2f  healthy=%d\n",
             (double)yaw, (double)rel, imuHealthy() ? 1 : 0);
    }

    const bool sp = uiStartPressed();
    if (!lastStart && sp) { zero = yaw; lastShown = 1e9f; }   // press edge = re-zero
    lastStart = sp;

    const bool st = uiSetPressed();
    if (!lastSet && st) { uiWaitSetRelease(); return; }
    lastSet = st;

    sleep_ms(30);
  }
}


//==============================================================================
// Steering response test
//==============================================================================
// Runs the REAL control laws (steer_law.cpp) against the live pose and drives the
// servo, so you can rotate or slide the car by hand and watch it react. Rotating
// in place is enough: theta changes even when the car doesn't roll.
//   FWD mode: turn the car LEFT  -> wheels must steer RIGHT (back to the line).
//   REV mode: turn the car LEFT  -> wheels must steer LEFT. (In reverse a left
//             wheel turns the car right, back toward the aim.) The target sits
//             CAL_STEER_REV_TARGET_M behind the MP's position at the last zero.
// Rotating in place, both modes depend only on HEADING_SIGN x STEER_DIR_SIGN, so
// they flip TOGETHER and cannot tell those two signs apart. Verify HEADING_SIGN
// first with "6 IMU heading"; then a wrong FWD means STEER_DIR_SIGN. FWD right but
// REV wrong is not a sign problem at all — it points at the reverse law or the
// rear geometry (ENC_FROM_MP_M, or WHEELBASE_L_M for pure pursuit), so don't flip
// anything to "fix" it. Each mode runs the law config.h selects for that leg, and
// the LCD names it: FWD STAN / FWD PP, REV AIM / REV PP / REV STAN.
// DialBtn toggles FWD/REV, START re-zeroes, SET exits.
void calSteerTest() {
  // Without these the servo can't move (or pure pursuit computes 0), so the sign
  // check would show nothing and blame nothing. Name the missing value instead.
  // (M_PER_COUNT too: the cross-track term needs real distances.)
  const char *missing = steeringCalibrationMissing();   // not L_P_M: no pusher needed
  if (missing) {
    printf("[STR] can't run the steer test: %s (config.h §1)\n", missing);
    uiShow("NOT CALIBRATED", missing);
    sleep_ms(UI_MSG_MS);
    return;
  }
  printf("\n=== Steering response ===\n");
  printf("Check HEADING_SIGN with '6 IMU heading' FIRST.\n");
  printf("FWD: turn the car LEFT -> wheels steer RIGHT. If not, flip STEER_DIR_SIGN.\n");
  printf("REV: turn the car LEFT -> wheels steer LEFT. If FWD is right but REV is\n");
  printf("wrong, don't flip a sign: suspect the reverse law or the rear geometry\n");
  printf("(ENC_FROM_MP_M, or WHEELBASE_L_M with pure pursuit).\n");
  printf("DialBtn = FWD/REV, START = re-zero, SET = exit.\n\n");

  uiShow("Steer test", "priming...");
  for (int i = 0; i < 30 && !imuHealthy(); i++) { (void)imuYawDeg(); sleep_ms(20); }

  poseZeroAtArm();
  poseSetActive(true);              // estimator runs on core1 from here
  sleep_ms(200);                    // let a few ticks land

  bool rev = false;
  float delta = 0.0f;
  // Each mode runs whichever law config.h selects; the LCD names it.
  const char *fwdName = (STEER_FWD_LAW == SteerLaw::PurePursuit) ? "PP" : "STAN";
  const char *revName = (STEER_REV_LAW == RevSteerLaw::PurePursuit) ? "PP"
                      : (STEER_REV_LAW == RevSteerLaw::Stanley)     ? "STAN" : "AIM";
  const float xTarget = -CAL_STEER_REV_TARGET_M;   // REV mode: aim behind the start
  bool lastStart = uiStartPressed(), lastSet = uiSetPressed(), lastDial = uiDialPressed();
  uint32_t lastDraw = 0;

  odoSpeedReset();
  while (true) {
    odoSpeedTick(odoDistanceM());   // the speed estimate lives on this core (odo.h)
    const Position ro = poseGet();
    delta = rev ? steerLawReverse((float)ro.x, (float)ro.y, (float)ro.theta,
                                  std::fabs(odoSpeedMps()), xTarget, delta)
                : steerLawForward((float)ro.y, (float)ro.theta, std::fabs(odoSpeedMps()));
    steerSetAngle(delta);

    const uint32_t now = to_ms_since_boot(get_absolute_time());
    if (now - lastDraw >= 150) {                 // LCD writes are slow; throttle
      lastDraw = now;
      char l1[17], l2[17];
      snprintf(l1, sizeof l1, "%s %-4s th%+5.1f", rev ? "REV" : "FWD", rev ? revName : fwdName,
               (double)(ro.theta * 180.0 / M_PI));
      snprintf(l2, sizeof l2, "y%+5.2f d%+5.1f", (double)ro.y,
               (double)(delta * 180.0f / (float)M_PI));
      lcdClear(); lcdPrintAt(0, 0, l1); lcdPrintAt(0, 1, l2);
      printf("[STR] %s %-4s theta=%+7.1f deg  y=%+6.3f m  delta=%+6.2f deg (%s)\n",
             rev ? "REV" : "FWD", rev ? revName : fwdName, (double)(ro.theta * 180.0 / M_PI), (double)ro.y,
             (double)(delta * 180.0f / (float)M_PI), delta > 0 ? "LEFT" : delta < 0 ? "RIGHT" : "centre");
    }

#if USE_DIAL_ENTRY
    const bool db = uiDialPressed();
    if (!lastDial && db) { rev = !rev; delta = 0.0f; sleep_ms(BUTTON_RELEASE_SETTLE_MS); }
    lastDial = db;
#else
    (void)lastDial;
#endif

    const bool sp = uiStartPressed();
    if (!lastStart && sp) {
#if !USE_DIAL_ENTRY
      // No dial: START also toggles FWD/REV, so the reverse check stays reachable.
      rev = !rev; delta = 0.0f;
#endif
      printf(poseZeroAtArm() ? "[STR] pose re-zeroed\n"
                             : "[STR] re-zero FAILED: IMU not answering on core1\n");      // (odoZero inside the zero also restarts the speed estimate.)
    }
    lastStart = sp;

    const bool st = uiSetPressed();
    if (!lastSet && st) {
      uiWaitSetRelease();
      poseSetActive(false);
      steerCenter();
      printf("[STR] exit — estimator off, servo centred\n");
      return;
    }
    lastSet = st;
    sleep_ms(10);
  }
}

//==============================================================================
// Menu
//==============================================================================
void calMenu() {
  // A run ends with the motor shorted (driveHold) so the car can't move while it
  // is measured. Release it here: the bench tools push the car by hand, and
  // pushing against a braked motor skids the wheels and corrupts M_PER_COUNT.
  driveCoast();
  static const char *items[] = {"1 Encoder read", "2 Motor test",
                                "3 Servo jog", "4 PWM->speed",
                                "5 Input test", "6 IMU heading",
                                "7 Steer response", "8 Exit to run"};
  printf("\n=== BENCH CALIBRATION MENU ===\n");
  while (true) {
    switch (uiMenuSelect("Bench menu", items, 8)) {
      case 0: calEncoderReadout(); break;
      case 1: calMotorTest();      break;
      case 2: calServoTest();      break;
      case 3: calPwmProbe();       break;
      case 4: calInputTest();      break;
      case 5: calImuTest();        break;
      case 6: calSteerTest();      break;
      default:
        printf("[CAL] leaving calibration\n");
        return;                    // back to whoever called us
    }
  }
}
