#pragma once
//==============================================================================
// pose_track — the dead-reckoning step, from the two MEASURED inputs only: the
// roller's distance (encoder) and the body heading (IMU). Pure logic, header-only,
// tested in host_check.
//
// It deliberately does NOT use the steering angle. The servo command is not a
// measurement: a centre pulse off by 0.5 deg would make the roller "roll" 0.5 deg
// off its real path on every tick, and that error grows with distance (~13 cm over
// a 9 m leg in the host sim). Instead it tracks the REAR AXLE, which always rolls
// along the body heading whatever the front wheels do, and places the roller
// rigidly in front of it:
//   rear   += ds * (cos theta, sin theta)
//   roller  = rear + L * (cos theta, sin theta)          L = wheelbase
// ds is the roller's step, not the rear axle's; they differ by cos(delta), so the
// error is second order in the steering angle (~1 mm over 10 m at 0.02 rad).
//
// With L = 0 (WHEELBASE_L_M not measured) the track IS the roller, and the roller's
// y reads low by L * sin(theta) — a few mm at the headings a straight run holds,
// and it does not grow with distance.
//==============================================================================
#include <cmath>

struct HeadingTrack {
  double xRear = 0.0, yRear = 0.0;

  // Roller at (xRoller, 0) with the heading just zeroed (theta = 0).
  void reset(double xRoller, double wheelbase) { reset(xRoller, 0.0, wheelbase, 0.0); }
  // Roller at (xRoller, yRoller) with the body at heading theta0 (aim trim).
  void reset(double xRoller, double yRoller, double wheelbase, double theta0) {
    xRear = xRoller - wheelbase * std::cos(theta0);
    yRear = yRoller - wheelbase * std::sin(theta0);
  }

  void step(double ds, double theta) {
    xRear += ds * std::cos(theta);
    yRear += ds * std::sin(theta);
  }

  double rollerX(double theta, double wheelbase) const { return xRear + wheelbase * std::cos(theta); }
  double rollerY(double theta, double wheelbase) const { return yRear + wheelbase * std::sin(theta); }
};
