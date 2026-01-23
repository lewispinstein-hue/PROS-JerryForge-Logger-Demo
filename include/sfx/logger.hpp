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
      sfx::LogLevel::DEBUG, _SFX_CURRENT_SOURCE, fmt, ##__VA_ARGS__)

#define LOG_INFO(fmt, ...)                                                     \
  sfx::Logger::get_instance().log_message(                                     \
      sfx::LogLevel::INFO, _SFX_CURRENT_SOURCE, fmt, ##__VA_ARGS__)

#define LOG_WARN(fmt, ...)                                                     \
  sfx::Logger::get_instance().log_message(                                     \
      sfx::LogLevel::WARN, _SFX_CURRENT_SOURCE, fmt, ##__VA_ARGS__)

#define LOG_ERROR(fmt, ...)                                                    \
  sfx::Logger::get_instance().log_message(                                     \
      sfx::LogLevel::ERROR, _SFX_CURRENT_SOURCE, fmt, ##__VA_ARGS__)

#define LOG_FATAL(fmt, ...)                                                    \
  sfx::Logger::get_instance().log_message(                                     \
      sfx::LogLevel::FATAL, _SFX_CURRENT_SOURCE, fmt, ##__VA_ARGS__)
/** @} */

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

/** * @enum LogLevel
 * @brief Enumeration of log levels for filtering log output.
 */
enum LogLevel { NONE = 0, DEBUG, INFO, WARN, ERROR, FATAL };

// --------------------------------------------------------------------------
// Configuration Constants
// --------------------------------------------------------------------------
constexpr uint32_t thermalCheckInterval = 15000;
constexpr uint32_t batteryCheckInterval = 20000;
constexpr uint32_t taskPrintInterval = 30000;
constexpr uint16_t waitForStdInTimeout = 15000;

constexpr uint32_t SD_FLUSH_INTERVAL_MS = 1000;

const double LOW_BATTERY_THRESHOLD = 30.0;
const double CRITICAL_VOLTAGE_THRESHOLD = 11000;

// ---------- Generic variable watches ----------
using WatchId = uint16_t;

/**
 * @def Helper to turn a pointer into an std::shared_ptr. Meant for
 * logger.setRobot, which expects a shared_ptr type
 */
#define SHARED(obj)                                                            \
  std::shared_ptr<std::remove_reference_t<decltype(obj)>>(&obj, [](void *) {})

/**
 * @def Helper to make logger.watch LevelOverride struct more readable
 */
#define PREDICATE(func) sfx::as_predicate<int>([](int v) { return func; })

template <class T> struct LevelOverride {
  using value_type = T;
  LogLevel elevatedLevel = LogLevel::WARN;
  std::function<bool(const T &)> predicate;
};

// Helper: turn any predicate callable into std::function<bool(const T&)>
template <class T, class Pred>
std::function<bool(const T &)> as_predicate(Pred &&p) {
  return std::function<bool(const T &)>(std::forward<Pred>(p));
}

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
    std::atomic<bool> printWatches{true};
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

  // Config setters/getters
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
  void setPrintWatches(bool v);

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
  bool getPrintWatches() const { return config_.printWatches.load(); }

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

  // Normal
  template <class Getter>
  WatchId
  watch(std::string label, LogLevel baseLevel, uint32_t intervalMs,
        Getter &&getter,
        auto ov = LevelOverride<std::decay_t<std::invoke_result_t<Getter &>>>{},
        std::string fmt = {}) {
    using T = std::decay_t<std::invoke_result_t<Getter &>>;

    return addWatch<T>(std::move(label), baseLevel, intervalMs,
                       std::forward<Getter>(getter), std::move(ov),
                       std::move(fmt));
  }
  // Overload with onChange
  template <class Getter>
  WatchId
  watch(std::string label, LogLevel baseLevel, bool onChange, Getter &&getter,
        auto ov = LevelOverride<std::decay_t<std::invoke_result_t<Getter &>>>{},
        std::string fmt = {}) {

    using T = std::decay_t<std::invoke_result_t<Getter &>>;

    return addWatch<T>(std::move(label), baseLevel,
                       /*intervalMs=*/0, // ignored when onChange=true
                       std::forward<Getter>(getter), std::move(ov),
                       std::move(fmt), onChange);
  }

private:
  Logger() = default;
  Logger(const Logger &) = delete;
  Logger &operator=(const Logger &) = delete;

  // Internal helpers
  void Update();

  void printLemlibPose_();
  void printThermalWatchdog_();
  void printBatteryWatchdog_();
  void printProsTasks_();
  void printRunningTasks_();

  bool checkRobotConfig_(bool checkLemLib = true);
  bool configCheck();
  bool initSDLogger_();
  void makeTimestampedFilename_(size_t len);
  void waitForStartChar();
  const char *levelToString_(LogLevel level) const;

  void copyConfigFrom_(const loggerConfig &cfg);

  struct Watch {
    WatchId id{};
    std::string label;
    LogLevel baseLevel{LogLevel::INFO};
    uint32_t intervalMs{1000};
    uint32_t lastPrintMs{0};
    std::string fmt;

    bool onChange = false;
    std::optional<std::string> lastValue;

    std::function<std::pair<LogLevel, std::string>()> eval;
  };

  WatchId nextId_ = 1;
  std::unordered_map<WatchId, Watch> watches_;

  // --- rendering helpers ---
  static std::string renderValue(const std::string &v, const std::string &) {
    return v;
  }
  static std::string renderValue(const char *v, const std::string &) {
    return v ? std::string(v) : std::string("(null)");
  }
  static std::string renderValue(bool v, const std::string &) {
    return v ? "true" : "false";
  }

  template <class T>
  static std::string renderValue(const T &v, const std::string &fmt) {
    if constexpr (std::is_arithmetic_v<T>) {
      if (!fmt.empty()) {
        char buf[256];
        if constexpr (std::is_floating_point_v<T>) {
          std::snprintf(buf, sizeof(buf), fmt.c_str(), (double)v);
        } else {
          std::snprintf(buf, sizeof(buf), fmt.c_str(), (long long)v);
        }
        return std::string(buf);
      }
      return std::to_string(v);
    } else {
      return std::string("<unrenderable>");
    }
  }

  // --- core builder ---
  template <class T, class Getter>
  WatchId addWatch(std::string label, LogLevel baseLevel, uint32_t intervalMs,
                   Getter &&getter, LevelOverride<T> ov, std::string fmt,
                   bool onChange = false) {
    Watch w;
    w.id = nextId_++;
    w.label = std::move(label);
    w.baseLevel = baseLevel;
    w.intervalMs = intervalMs;
    w.onChange = onChange;
    w.fmt = std::move(fmt);

    using G = std::decay_t<Getter>;
    G g = std::forward<Getter>(getter); // store callable by value

    // Capture fmt by value (not by reference to w), and move ov in.
    const std::string fmtCopy = w.fmt;

    w.eval = [baseLevel, fmtCopy, g = std::move(g),
              ov =
                  std::move(ov)]() mutable -> std::pair<LogLevel, std::string> {
      T v = static_cast<T>(g());

      LogLevel lvl = baseLevel;
      if (ov.predicate && ov.predicate(v))
        lvl = ov.elevatedLevel;

      std::string out = renderValue(v, fmtCopy);
      return {lvl, std::move(out)};
    };

    WatchId id = w.id;
    watches_.emplace(id, std::move(w));
    return id;
  }

  void printWatches();

  struct MotorMonitor {
    std::string name;
    std::shared_ptr<pros::MotorGroup> group = nullptr;
  };

  // State moved from globals
  loggerConfig config_{};
  LogLevel minLogLevel_ = LogLevel::INFO;

  pros::Mutex logToSdMutex_;
  pros::Mutex loggerMutex_;
  pros::Mutex generalMutex_;

  uint32_t lastFlush_ = 0;
  FILE *sdFile_ = nullptr;
  char currentFilename_[128] = "";

  bool waitForSTDin_ = false;
  bool started_ = false;
  bool sdLocked_ = false; // logToSD locked

  // Robot refs
  std::shared_ptr<lemlib::Chassis> pChassis_ = nullptr;
  std::shared_ptr<pros::MotorGroup> pLeftDrivetrain_ = nullptr;
  std::shared_ptr<pros::MotorGroup> pRightDrivetrain_ = nullptr;

  std::vector<MotorMonitor> internalMotorsToScan_;

  std::unique_ptr<pros::Task> task_;
};
} // namespace sfx
