#pragma once

#include <cmath>
#include <cstdint>
#include <string>
#include <string_view>
#include <vector>

#include "pros/motor_group.hpp"

// Default Thresholds
constexpr int MAX_MOTOR_TEMP = 53.0;

namespace sfx {
namespace motorChecks {

/**
 * @struct checkTemp
 * @brief Results of a passive thermal check.
 */
struct checkTemp {
  bool overheated = false;
  double maxTemp = 0.0;
  std::vector<std::int8_t> motorsOverheated;
};

/**
 * @brief Check if a motor group is overheating (Passive).
 */
checkTemp checkMotorOverheat(pros::MotorGroup &mg,
                             float maxTemp = MAX_MOTOR_TEMP);

/**
 * @brief Format temperature results into a readable string.
 * @param name The name of the subsystem (e.g., "Intake").
 * @param t The result structure.
 * @param useANSI If true, includes color codes.
 * @return std::string The formatted report.
 */
std::string formatTempCheckResult(std::string_view name, const checkTemp &t,
                                  bool useANSI = true);

/**
 * @struct scanMotorReturn
 * @brief Comprehensive results combining thermal and electrical checks.
 * Uses composition to include thermal results.
 */
struct scanMotorReturn {
  // Includes thermal results directly
  checkTemp thermal;

  // Current / Stall results
  bool stalled = false;
  int maxCurrent = 0; // mA
  std::vector<std::int8_t> motorsStalled;

  // Overall status
  bool passed = false;
};

// --- Constants for Sampling (Used by scanMotor) ---
constexpr int SAMPLE_COUNT = 10;
constexpr int STALL_REQUIRED = 6;
constexpr int SAMPLE_DELAY_MS = 20;

/**
 * @brief Active diagnostic scan (Blocking ~200ms).
 * Checks for stalls and calls checkMotorOverheat internally.
 */
scanMotorReturn scanMotor(pros::MotorGroup &mg, int maxAmpDraw = 2000,
                          double maxVelocity = 80,
                          double maxTemp = MAX_MOTOR_TEMP);

/**
 * @brief Format full scan results into a readable string.
 */
std::string formatMotorScanResult(std::string_view name,
                                  const scanMotorReturn &r,
                                  bool useAnsi = true);

/**
 * @brief Utility to format port lists.
 */
std::string formatMotorPorts(const std::vector<std::int8_t> &ports);

} // namespace motorChecks
} // namespace sfx