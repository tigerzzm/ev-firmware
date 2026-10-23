#pragma once
//==============================================================================
// steer_law — the lateral control laws. Pure math (no hardware) so host_check can
// close the loop around them in a bicycle-model simulation.
//
// Convention: delta > 0 points the front wheels LEFT; theta > 0 is the car turned
// LEFT of the +x axis; y > 0 is LEFT of the centerline. steer.cpp maps delta onto
// the servo (STEER_DIR_SIGN), so these laws never need a sign flip.
//
// Kinematics (front-steered bicycle, wheelbase L): theta' = v * tan(delta) / L.
// Going forward (v > 0) a left delta turns the car left; in REVERSE (v < 0) the
// same delta turns it RIGHT. That is why a forward law can't just run backwards.
//==============================================================================

//------------------------------------------------------------------------------
// The two entry points. Both take the pose as poseGet() returns it — the ROLLER,
// which rides the front axle — and dispatch to the law config.h §4 selects. These
// are what the state machine and the bench steer test call.
//------------------------------------------------------------------------------

// Forward leg, holding the line y = 0 (STEER_FWD_LAW).
float steerLawForward(float yRoller, float theta, float speedAbs);

// Reverse leg, backing the rear MP onto (xTarget, 0) along the line (STEER_REV_LAW).
// `lastDelta` is the previous command (AimAtTarget holds it near the target).
float steerLawReverse(float xRoller, float yRoller, float theta, float speedAbs,
                      float xTarget, float lastDelta);

//------------------------------------------------------------------------------
// The individual laws, with explicit inputs so tests can drive each one directly.
// All are clamped to +/-DELTA_LIMIT_RAD (DELTA_MAX_RAD, or less if the servo
// travel limit cuts in first; config.h §8).
//------------------------------------------------------------------------------

// Stanley, forward, at the front axle:
//   delta = KH * (0 - theta) - atan2(KC * yFront, |v| + EPS)
// Both terms steer TOWARD the line: car left of it (y > 0) -> steer right, car
// pointing left (theta > 0) -> steer right.
float steerLawStanley(float yFront, float theta, float speedAbs);

// Pure pursuit, forward, from the REAR axle. Goal point: on the line, `lookahead`
// metres from the rear axle, AHEAD of the car. Steers onto the circle through it
// that is tangent to the car's heading:
//   alpha = atan2(-yRear, sqrt(Ld^2 - yRear^2)) - theta
//   delta = atan(2 * wheelbase * sin(alpha) / Ld)
// If the rear axle is more than Ld off the line, it aims square at the line.
float steerLawPurePursuit(float yRear, float theta, float lookahead, float wheelbase);

// Aim-at-target, reverse: point the car so that backing up carries the rear MP
// to (xTarget, 0). theta_des is the heading whose REAR points at the target:
//   theta_des = atan2(yMp, xMp - xTarget);  delta = -KR * (theta_des - theta)
// Within REV_STEER_HOLD_M of the target the aim is ill-conditioned, so it returns
// `lastDelta` (holds the wheels where they are).
float steerLawAimAtTarget(float xMp, float yMp, float theta, float xTarget, float lastDelta);

// Stanley, reverse, at the rear MP (the LEADING end when backing up):
//   delta = KH * theta - atan2(KC * yMp, |v| + EPS)
// Two changes from forward Stanley, both forced by reverse kinematics:
//   - the sideways error is taken at the leading end. At the trailing front axle
//     the car's own swing feeds back the wrong way and it diverges;
//   - the heading term flips sign, because in reverse a left wheel turns the car
//     right.
// The sideways term keeps its sign, because two flips cancel. Car left of the
// line: steer right, same as forward. In reverse that turns the car LEFT, and a
// car pointing left while backing up carries its rear to the RIGHT, toward the line.
// Linearised it is stable for any positive gains, and more so the further the
// reference point sits behind the rear axle — the MP is behind it. Uses the same
// STEER_KH / STEER_KC / STEER_SPEED_EPS as forward Stanley.
float steerLawStanleyReverse(float yMp, float theta, float speedAbs);

// Pure pursuit, reverse, from the rear axle (the LEADING axle when backing up).
// Goal point: on the line, `lookahead` metres BEHIND the rear axle. The arc maths
// doesn't care which way the car moves: the circle through the rear axle, tangent
// to the heading, through a point at car-frame sideways offset yCar has curvature
// 2 * yCar / Ld^2, and the bicycle model turns that into a steer. No goal-point
// singularity near the target, so it steers right up to the stop.
float steerLawPurePursuitReverse(float yRear, float theta, float lookahead, float wheelbase);
