#include "cal.h"
#include "config.h"
#include "drive.h"
#include "lcd.h"
#include "hardware/gpio.h"
#include <algorithm>
#include "imu.h"
#include "path.h"
#include "pose.h"
#include "odo.h"
#include "utils.h"
#include <cmath>
#include "steer.h"
#include "ui.h"
#include "pico/stdlib.h"
#include <cstdio>

// Average speed is timed over a fixed DISTANCE rather than read from
// odoSpeedMps(). Over a 2 m window, distance/time is exact — no filter lag and
// no quantisation worth worrying about — whereas the filtered estimate would lag
// by roughly its time constant.
void calPwmProbe() {
  float pwm = 30.0f;

  printf("\n=== PWM -> speed calibration ===\n");
  printf("Dial a duty %%, press START. Needs ~3.5 m of clear floor per point.\n");
  printf("SET aborts a run in progress.\n\n");

  while (true) {
    uiShow("PWM->speed cal", "SET to enter");
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
    snprintf(l1, sizeof l1, "PWM %.0f%% ready", pwm);
    uiShow(l1, "START to run");
    while (!uiStartPressed()) sleep_ms(2);

    steerSetAngle(0.0f);        // wheels straight — this is a straight-line test
    odoZero();
    odoSpeedReset();

    const uint64_t t0 = to_ms_since_boot(get_absolute_time());
    uint64_t tA = 0, tB = 0;    // window start/end timestamps
    bool aborted = false;

    driveForward(pwm);
    while (true) {
      const uint64_t now = to_ms_since_boot(get_absolute_time());
      const float    d   = odoDistanceM();
      if (tA == 0 && d >= CAL_WARMUP_M) tA = now;                      // launch done
      if (tA != 0 && d >= CAL_WARMUP_M + CAL_MEASURE_M) { tB = now; break; }
      if (uiSetPressed())                          { aborted = true; break; }
      if ((now - t0) / 1000.0f > CAL_TIMEOUT_S)    { aborted = true; break; }
      sleep_ms(2);
    }

    driveBrake();
    sleep_ms(BRAKE_PULSE_MS);
    driveCoast();

    if (aborted || tB == 0) {
      printf("[CAL] pwm=%3.0f%%  ABORTED (travelled %.2f m)\n", pwm, odoDistanceM());
      uiShow("ABORTED", "check path/PWM");
      sleep_ms(2000);
      continue;
    }

    const float dt = (tB - tA) / 1000.0f;
    const float v  = CAL_MEASURE_M / dt;
    printf("[CAL] pwm=%3.0f%%  v=%.3f m/s   (%.2f m in %.2f s)\n",
           pwm, v, CAL_MEASURE_M, dt);
    char l2[17];
    snprintf(l1, sizeof l1, "PWM %.0f%%", pwm);
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
// floor, which is what CREEP_PWM should sit just above.
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

    const long     c0 = odoCountSigned();
    const uint64_t t0 = to_ms_since_boot(get_absolute_time());

    if (reverse) driveReverse(pwm);
    else         driveForward(pwm);

    while (to_ms_since_boot(get_absolute_time()) - t0 < MOTOR_TEST_MS) {
      if (uiSetPressed()) break;              // let go early
      sleep_ms(2);
    }
    driveCoast();                             // coast, don't brake — we want to
    sleep_ms(400);                            // see how far it actually rolled

    const long  dc = odoCountSigned() - c0;
    const float dm = (float)dc * M_PER_COUNT;
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
      us = std::clamp(us + step * incs[incIdx], SERVO_MIN_US, SERVO_MAX_US);
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
      while (uiSetPressed()) sleep_ms(5);       // wait for release
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
      if (us > SERVO_MAX_US) us = SERVO_MIN_US;    // wrap
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
// Deliberately does its own zero-tracking rather than using odoDistanceM(), which
// reads 0 while M_PER_COUNT is uncalibrated — the exact situation this mode
// exists to fix.
void calEncoderReadout() {
  printf("\n=== Encoder readout ===\n");
  printf("START zeroes the count, SET exits.\n");
  printf("To calibrate: zero, push the car a MEASURED distance, read the count.\n\n");

  long c0   = odoCountSigned();
  long last = c0 + 1;              // force a first render
  int  lastAB = -1;
  bool lastStart = uiStartPressed();
  bool lastSet   = uiSetPressed();

  while (true) {
    const long c  = odoCountSigned();
    const long dc = c - c0;
    // Raw A/B levels. gpio_get() reads the pad input regardless of the pin being
    // under PIO control, so this shows the wires even when the PIO isn't counting
    // — which distinguishes "encoder dead" from "decoder not working".
    const int a  = gpio_get(FRONT_ENCODER_A_PIN) ? 1 : 0;
    const int b  = gpio_get(FRONT_ENCODER_B_PIN) ? 1 : 0;
    const int ab = (a << 1) | b;

    if (dc != last || ab != lastAB) {
      last = dc; lastAB = ab;
      char l1[17], l2[17];
      snprintf(l1, sizeof l1, "cnt %+ld", dc);
      if (M_PER_COUNT > 0.0f)
        snprintf(l2, sizeof l2, "A%d B%d %+.3fm", a, b, (double)((float)dc * M_PER_COUNT));
      else
        snprintf(l2, sizeof l2, "A%d B%d  cal=0", a, b);
      lcdClear(); lcdPrintAt(0, 0, l1); lcdPrintAt(0, 1, l2);
      // Print the constant a 5.000 m reference roll would imply — scale it
      // yourself if you used a different reference length.
      if (dc != 0) {
        printf("[ENC] A=%d B=%d  counts=%+ld   M_PER_COUNT if this was 5.000 m = %.8f\n",
               a, b, dc, 5.0f / (float)(dc < 0 ? -dc : dc));
      } else {
        printf("[ENC] A=%d B=%d  counts=%+ld\n", a, b, dc);
      }
    }

    const bool sp = uiStartPressed();
    if (!lastStart && sp) { c0 = odoCountSigned(); last = c0 + 1; }   // press edge = zero
    lastStart = sp;

    const bool st = uiSetPressed();
    if (!lastSet && st) {
      while (uiSetPressed()) sleep_ms(5);
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
  // does, so it goes stale after IMU_TIMEOUT_MS and reads false on a perfectly
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
  printf("Turn the car LEFT by hand: 'rel' must INCREASE.\n");
  printf("If it decreases, flip HEADING_SIGN in config.h §3.\n");
  printf("START zeroes the reference, SET exits.\n\n");

  float zero = (float)imuYawDeg();
  float lastShown = 1e9f;
  bool lastStart = uiStartPressed(), lastSet = uiSetPressed();

  while (true) {
    const float yaw = (float)imuYawDeg();
    // Relative to the captured zero, wrapped to +/-180 so it reads naturally
    // rather than jumping 359 -> 0 as you rotate through the wrap point.
    const float rel = (float)utils::angleError(yaw, zero, false);

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
    if (!lastSet && st) { while (uiSetPressed()) sleep_ms(5); return; }
    lastSet = st;

    sleep_ms(30);
  }
}


//==============================================================================
// Steering response test
//==============================================================================
// Rotating the car in place is enough to exercise this: theta changes even when
// ds is zero, so the heading term responds without the car going anywhere.
void calSteerTest() {
  printf("\n=== Steering response ===\n");
  printf("Rotate the car by hand and watch the servo.\n");
  printf("Turn the car RIGHT -> the servo must steer LEFT to correct (and vice\n");
  printf("versa). If it drives the SAME way you turned, flip STEER_FB_SIGN.\n");
  printf("START re-zeroes the pose, SET exits.\n\n");

  const ArcPlan plan = pathPlan(8.0f, GATE_BULGE_H_M);
  printf("[STR] nominal arc D=8.00 h=%.3f -> R=%.2f L=%.3f k=%.4f\n",
         (double)GATE_BULGE_H_M, (double)plan.R, (double)plan.L, (double)plan.k);

  uiShow("Steer test", "priming...");
  for (int i = 0; i < 30 && !imuHealthy(); i++) { (void)imuYawDeg(); sleep_ms(20); }

  poseZeroAtArm();
  poseSetActive(true);              // estimator runs on core1 from here
  sleep_ms(200);                    // let a few ticks land

  bool lastStart = uiStartPressed(), lastSet = uiSetPressed();
  uint32_t lastDraw = 0;

  while (true) {
    const Position p   = poseGet();
    const float    ect = pathCrosstrack(plan, p);
    // Same commanded angle the run loop would produce, and it drives the servo.
    const float    d   = steerControl(plan, p, odoSpeedMps());
    const int      us  = SERVO_CENTER_US + (int)(d * SERVO_US_PER_RAD);

    const uint32_t now = to_ms_since_boot(get_absolute_time());
    if (now - lastDraw >= 150) {                 // LCD writes are slow; throttle
      lastDraw = now;
      char l1[17], l2[17];
      snprintf(l1, sizeof l1, "th%+5.0f ct%+5.2f",
               (double)(p.theta * 180.0 / M_PI), (double)ect);
      snprintf(l2, sizeof l2, "d%+5.1f us%4d",
               (double)(d * 180.0f / (float)M_PI), us);
      lcdClear(); lcdPrintAt(0, 0, l1); lcdPrintAt(0, 1, l2);
      printf("[STR] theta=%+7.1f deg  ct=%+6.3f m  delta=%+6.2f deg  us=%4d\n",
             (double)(p.theta * 180.0 / M_PI), (double)ect,
             (double)(d * 180.0f / (float)M_PI), us);
    }

    const bool sp = uiStartPressed();
    if (!lastStart && sp) { poseZeroAtArm(); printf("[STR] pose re-zeroed\n"); }
    lastStart = sp;

    const bool st = uiSetPressed();
    if (!lastSet && st) {
      while (uiSetPressed()) sleep_ms(5);
      poseSetActive(false);
      steerWriteUs(SERVO_CENTER_US);
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
