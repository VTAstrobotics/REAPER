#ifndef UTILS_H
#define UTILS_H

#include "rclcpp/rclcpp.hpp" // technically not needed to compile, but needed for clang-tidy (linter)

/**
 * @brief Allowed amount of difference for two floats to be equal
 */
constexpr float APPROX_FUDGE = 0.01;

/**
 * @brief Determine if two doubles are about equal
 */
constexpr bool approx(double f1, double f2)
{
  return ((f1) > (f2) ? (f1) - (f2) < APPROX_FUDGE
                      : (f2) - (f1) < APPROX_FUDGE);
}

/**
 * Checks if the requested power is in [-1, 1]
 * @param pwr is the requested duty cycle
 * @return true if in bounds, fales if out of bounds
 */
bool pwr_in_bounds(double pwr) { return pwr >= -1 && pwr <= 1; }

/**
 *
 * @param result_param is the parameter of the result to set
 * @param result_val is the val to set it to
 * @param logger is the logger to use
 * @param print_prefix is the prefix on print statements so we know where it
 * came from
 * @return true if goal succeeded, false if failed
 */
bool goal_done_helper(float& result_param, float result_val,
                      const rclcpp::Logger& logger, const char* print_prefix)
{
  if (rclcpp::ok()) {
    result_param = result_val;

    RCLCPP_DEBUG(logger, "%s: Goal succeeded", print_prefix);
    return true;
  }

  RCLCPP_ERROR(logger, "%s: Goal failed", print_prefix);
  return false;
}

#endif
