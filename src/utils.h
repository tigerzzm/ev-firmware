#pragma once

// Harvested from robotour-pico (src/utils.h), unchanged except the include:
//   robotour pulled in "chassis.h"; the EV build only needs the Position type,
//   so this now includes "position.h" directly (no differential-drive chassis).
#include "position.h"

namespace utils {

/**
 * Calculates the error between two angles.
 * BY DEFAULT EXPECTS ANGLES IN DEGREES
 */
double angleError(double angle1, double angle2, bool radians = false);

/**
 * Returns the angle in the range [0, 2PI]
 */
double angleSquish(double angle, bool radians = true);

/**
 * Converts degrees to radians
 */
double degToRad(double deg);

/**
 * Converts radians to degrees
 */
double radToDeg(double rad);

/**
 * @brief Slew rate limiter
 *
 * @param target target value
 * @param current current value
 * @param maxChange maximum change. No maximum if set to 0
 * @return float - the limited value
 */
float slew(float target, float current, float maxChange);

/**
 * @brief Get the signed curvature of a circle that intersects the first pose
 * and the second pose. (Kept for the path/steering follower.)
 */
float getCurvature(Position pose, Position other);

template <typename T> constexpr T sgn(T value) {
  return value < 0 ? -1 : (value > 0 ? 1 : 0);
}

}; // namespace utils
