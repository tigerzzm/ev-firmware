#include "utils.h"

#include <cmath>

/**
 * Calculates the error between two angles.
 * BY DEFAULT, EXPECTS ANGLES IN DEGREES
 */
double utils::angleError(double angle1, double angle2, bool radians) {
  return std::remainder(angle1 - angle2, radians ? 2 * M_PI : 360);
}

/**
 * Returns the angle in the range [0, 2PI]
 */
double utils::angleSquish(double angle, bool radians) {
  // `circle` was a float holding 2*pi, which is ~1.7e-7 larger than the true
  // value — a needless precision loss in a double-precision function.
  const double circle = radians ? (2.0 * M_PI) : 360.0;

  // fmod first, so this is O(1). The previous `while (angle < 0) angle += circle`
  // was unbounded: a large negative input would spin for many thousands of
  // iterations, and this runs on the 500 Hz estimator tick.
  double a = std::fmod(angle, circle);
  if (a < 0) a += circle;
  return a;
}

/**
 * Converts degrees to radians
 */
double utils::degToRad(double deg) { return deg * M_PI / 180; }

/**
 * Converts radians to degrees
 */
double utils::radToDeg(double rad) { return rad * 180 / M_PI; }

/**
 * @brief Slew rate limiter
 *
 * @param target target value
 * @param current current value
 * @param maxChange maximum change. No maximum if set to 0
 * @return float - the limited value
 */
float utils::slew(float target, float current, float maxChange) {
  float change = target - current;
  if (maxChange == 0)
    return target;
  if (change > maxChange)
    change = maxChange;
  else if (change < -maxChange)
    change = -maxChange;
  return current + change;
}

/**
 * @brief Get the signed curvature of a circle that intersects the first pose
 * and the second pose
 *
 * @note The circle will be tangent to the theta value of the first pose
 * @note The curvature is signed. Positive curvature means the circle is going
 * clockwise, negative means counter-clockwise
 * @note Theta has to be in radians and in standard form. That means 0 is right
 * and increases counter-clockwise
 *
 * @param pose the first pose
 * @param other the second pose
 * @return float curvature
 */
float utils::getCurvature(Position pose, Position other) {
  // calculate whether the pose is on the left or right side of the circle
  float side = utils::sgn(std::sin(pose.theta) * (other.x - pose.x) -
                          std::cos(pose.theta) * (other.y - pose.y));
  // calculate center point and radius
  float a = -std::tan(pose.theta);
  float c = std::tan(pose.theta) * pose.x - pose.y;
  float x = std::fabs(a * other.x + other.y + c) / std::sqrt((a * a) + 1);
  float d = std::hypot(other.x - pose.x, other.y - pose.y);

  // return curvature
  return side * ((2 * x) / (d * d));
}

/**
 * @brief Single-pole low-pass filter step (see utils.h).
 *
 * alpha = dt / (tau + dt) is the exact discretisation of a first-order lag for
 * a sample taken dt after the last one, so an irregular tick rate stays correct.
 */
float utils::lowpass(float prev, float sample, float dt, float tau) {
  if (dt <= 0.0f) return prev;
  if (tau <= 0.0f) return sample;
  const float alpha = dt / (tau + dt);
  return prev + alpha * (sample - prev);
}
