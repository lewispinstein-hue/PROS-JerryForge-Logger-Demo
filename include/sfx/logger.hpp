#pragma once
/**
 * @file logger.hpp
 * @brief Comprehensive logging system for robot telemetry (specifically made
 * for LemLib), diagnostics, and debugging information.
 *
 * NOTE: Refactored into a Singleton class (sfx::Logger).
 * Usage:
 *   auto& logger = sfx::Logger::get_instance();
 *   logger.initialize(cfg);              // optional; recommended
 *   logger.setRobot({ chassisPtr, leftPtr, rightPtr });
 *   logger.start();
 */

#include "lemlib/chassis/chassis.hpp"
#include "pros/rtos.hpp"

#include <atomic>
#include <cstdint>
#include <cstdio>
#include <memory>
#include <string>
#include <vector>

namespace sfx {

// LOG_SOURCE can be overridden per translation unit.
// If not provided, default is "[USER]".
// System/internal files can suppress source tags with:
//   #define LOG_SOURCE nullptr
//   #include "sfx/logger.hpp"
#ifndef LOG_SOURCE
#define LOG_SOURCE "[USER]"
#endif

#define _SFX_CURRENT_SOURCE LOG_SOURCE

/**
 * @defgroup LoggingMacros Logging Macros
 * @{
 */
#define LOG_DEBUG(fmt, ...)                                                    \
  sfx::Logger::get_instance().log_message(                                     \
      sfx::LogLevel::LOG_LEVEL_DEBUG, _SFX_CURRENT_SOURCE, fmt, ##__VA_ARGS__)

#define LOG_INFO(fmt, ...)                                                     \
  sfx::Logger::get_instance().log_message(                                     \
      sfx::LogLevel::LOG_LEVEL_INFO, _SFX_CURRENT_SOURCE, fmt, ##__VA_ARGS__)

#define LOG_WARN(fmt, ...)                                                     \
  sfx::Logger::get_instance().log_message(                                     \
      sfx::LogLevel::LOG_LEVEL_WARN, _SFX_CURRENT_SOURCE, fmt, ##__VA_ARGS__)

#define LOG_ERROR(fmt, ...)                                                    \
  sfx::Logger::get_instance().log_message(                                     \
      sfx::LogLevel::LOG_LEVEL_ERROR, _SFX_CURRENT_SOURCE, fmt, ##__VA_ARGS__)

#define LOG_FATAL(fmt, ...)                                                    \
  sfx::Logger::get_instance().log_message(                                     \
      sfx::LogLevel::LOG_LEVEL_FATAL, _SFX_CURRENT_SOURCE, fmt, ##__VA_ARGS__)
/** @} */

#define SHARED(obj)                                                            \
  std::shared_ptr<std::remove_reference_t<decltype(obj)>>(&obj, [](void *) {})

/**
 * @struct MutexGuard
 * @brief RAII wrapper for handling PROS mutexes.
 */
struct MutexGuard {
  pros::Mutex &m;
  bool locked = false;

  explicit inline MutexGuard(pros::Mutex &m) : m(m) { locked = m.take(); }
  explicit inline MutexGuard(pros::Mutex &m, uint32_t timeout) : m(m) {
    locked = m.take(timeout);
  }

  ~MutexGuard() {
    if (locked)
      m.give();
  }

  bool isLocked() const { return locked; }

  MutexGuard(const MutexGuard &) = delete;
  MutexGuard &operator=(const MutexGuard &) = delete;
};

/**
 * @enum LogLevel
 * @brief Enumeration of log levels for filtering log output.
 */
enum LogLevel {
  LOG_LEVEL_NONE = 0,
  LOG_LEVEL_DEBUG,
  LOG_LEVEL_INFO,
  LOG_LEVEL_WARN,
  LOG_LEVEL_ERROR,
  LOG_LEVEL_FATAL
};

// --------------------------------------------------------------------------
// Configuration Constants (unchanged)
// --------------------------------------------------------------------------
constexpr uint32_t thermalCheckInterval = 15000;
constexpr uint32_t batteryCheckInterval = 20000;
constexpr uint32_t taskPrintInterval = 30000;
constexpr uint16_t waitForStdInTimeout = 15000;

constexpr uint32_t SD_FLUSH_INTERVAL_MS = 1000;
constexpr uint32_t AUTO_SAVE_INTERVAL = 30000;

const double LOW_BATTERY_THRESHOLD = 30.0;
const double CRITICAL_VOLTAGE_THRESHOLD = 11000;

/**
 * @class Logger
 * @brief Singleton Logger class. Functionality mirrors the original
 * implementation.
 */
class Logger {
public:
  using LogLevel = ::sfx::LogLevel;
  struct loggerConfig {
    std::atomic<bool> runThermalWatchdog{true};
    std::atomic<bool> printLemlibPose{true};
    std::atomic<bool> printBatteryData{true};
    std::atomic<bool> onlyPrintOverheatedMotors{true};
    std::atomic<bool> printMotorWatchdogWarnings{true};
    std::atomic<bool> printPROSTasks{false};
    std::atomic<bool> logToTerminal{true};
    std::atomic<bool> logToSD{false};
    std::atomic<bool> outputForJerryio{false};
  };

  struct RobotRef {
    std::shared_ptr<lemlib::Chassis> chassis;
    std::shared_ptr<pros::MotorGroup> Left_Drivetrain;
    std::shared_ptr<pros::MotorGroup> Right_Drivetrain;
  };

  static Logger &get_instance();

  // Lifecycle
  void initialize(const loggerConfig &cfg);
  void start();
  void pause();
  void unpause();

  uint32_t status() const;

  // Config setters/getters (names kept close)
  void setRunThermalWatchdog(bool v);
  void setPrintLemlibPose(bool v);
  void setPrintBatteryData(bool v);
  void setOnlyPrintOverheatedMotors(bool v);
  void setPrintWatchdogWarnings(bool v);
  void setPrintProsTasks(bool v);
  void setLogToTerminal(bool v); // can change anytime
  void setLogToSD(bool v);       // locked after start()
  void setOutputForJerryio(bool v);
  void setWaitForStdIn(bool v);

  bool getRunThermalWatchdog() const {
    return config_.runThermalWatchdog.load();
  }
  bool getPrintLemlibPose() const { return config_.printLemlibPose.load(); }
  bool getPrintBatteryData() const { return config_.printBatteryData.load(); }
  bool getOnlyPrintOverheatedMotors() const {
    return config_.onlyPrintOverheatedMotors.load();
  }
  bool getPrintWatchdogWarnings() const {
    return config_.printMotorWatchdogWarnings.load();
  }
  bool getPrintProsTasks() const { return config_.printPROSTasks.load(); }
  bool getLogToTerminal() const { return config_.logToTerminal.load(); }
  bool getLogToSD() const { return config_.logToSD.load(); }
  bool getOutputForJerryio() const { return config_.outputForJerryio.load(); }

  // Min level
  void setLoggerMinLevel(LogLevel level);

  // Robot + motors
  bool setRobot(RobotRef ref);
  void registerMotor(std::string name, pros::MotorGroup *motor);

  // Logging
  void log_message(LogLevel level, const char *source, const char *fmt, ...);
  void logToSD(const char *levelStr, const char *fmt, ...);

  // Battery
  void printBatteryInfo();

private:
  Logger() = default;
  Logger(const Logger &) = delete;
  Logger &operator=(const Logger &) = delete;

  // Internal helpers (names per your convention)
  void Update();

  void printLemlibPose_();
  void printThermalWatchdog_(uint32_t now);
  void printBatteryWatchdog_(uint32_t now);
  void printProsTasks_(uint32_t now);
  void handleAutoSaveSd_();
  void printRunningTasks_();

  bool checkRobotConfig_(bool checkLemLib = true);
  bool initSDLogger_();
  void makeTimestampedFilename_(size_t len);

  const char *levelToString_(LogLevel level) const;

  void copyConfigFrom_(const loggerConfig &cfg);

  struct MotorMonitor {
    std::string name;
    std::shared_ptr<pros::MotorGroup> group = nullptr;
  };

private:
  // State moved from globals
  loggerConfig config_{};
  LogLevel minLogLevel_ = LogLevel::LOG_LEVEL_INFO;

  pros::Mutex logToSdMutex_;
  pros::Mutex loggerMutex_;
  pros::Mutex generalMutex_;

  uint32_t lastFlush_ = 0;
  FILE *sdFile_ = nullptr;
  char currentFilename_[128] = "";

  bool waitForSTDin_ = false;
  bool started_ = false;
  bool sdLocked_ = false; // logToSD locked after start

  // Robot refs
  std::shared_ptr<lemlib::Chassis> pChassis_ = nullptr;
  std::shared_ptr<pros::MotorGroup> pLeftDrivetrain_ = nullptr;
  std::shared_ptr<pros::MotorGroup> pRightDrivetrain_ = nullptr;

  std::vector<MotorMonitor> internalMotorsToScan_;

  std::unique_ptr<pros::Task> task_;
};

} // namespace sfx
