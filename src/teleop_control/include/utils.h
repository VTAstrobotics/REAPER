#ifndef UTILS_H
#define UTILS_H

/**
 * @brief Allowed amount of difference for two floats to be equal
 */
constexpr float APPROX_FUDGE = 0.01;

/**
 * @brief Determine if two floats are about equal
 */
constexpr bool approx(float f1, float f2) {
  return ((f1) > (f2) ? (f1) - (f2) < APPROX_FUDGE : (f2) - (f1) < APPROX_FUDGE);
}

#endif
