#pragma once
/**
 * @file logger.hpp
 * @brief Logging + telemetry utilities for PROS/LemLib robots.
 *
 * This header provides:
 * - A singleton logger (sfx::Logger) that can print to the PROS terminal and/or
 *   write to an SD card file.
 * - Convenience log macros (LOG_INFO, LOG_WARN, ...) that annotate messages with
 *   a per-translation-unit source tag (LOG_SOURCE).
 * - A lightweight "watch" system to periodically print variable values (or only
 *   when values change), with optional log-level elevation predicates.
 *
 * Where to use it:
 * - Robot bring-up, debugging, telemetry, and quick diagnosis on-field.
 * - Periodic status reporting (battery, task list) during development and test.
 *
 * When to use it:
 * - When you need structured, rate-limited logging without sprinkling printfs.
 * - When you want a single place to control log verbosity and outputs.
 *
 * @note This logger is designed for PROS + LemLib projects. It expects PROS RTOS
 *       primitives (pros::Task, pros::Mutex) and (optionally) LemLib chassis
 *       pointers for pose printing.
 *
 * Basic setup example:
 * @code
 * #include "sfx/logger.hpp"
 *
 * void initialize() {
 *   auto& logger = sfx::Logger::get_instance();
 *
 *   sfx::Logger::loggerConfig cfg{};
 *   cfg.logToTerminal = true;
 *   cfg.logToSD = false;
 *   logger.initialize(cfg);          // optional; recommended
 *
 *   // Optional: provide robot references used by watchdogs/pose printing.
 *   logger.setRobot({ SHARED(chassis), SHARED(leftDrive), SHARED(rightDrive) });
 *
 *   logger.start();
 * }
 * @endcode
 */

#include "lemlib/chassis/chassis.hpp"
#include "pros/motor_group.hpp"
#include "pros/rtos.hpp"



namespace sfx {

/**
 * @name Source tagging
 * @{
 *
 * LOG_SOURCE can be overridden per translation unit to annotate all log macros
 * with a short source tag (e.g., "[DRIVE]" or "[AUTO]").
 *
 * Example:
 * @code
 * #define LOG_SOURCE "[AUTO]"
 * #include "sfx/logger.hpp"
 *
 * void autonomous() {
 *   LOG_INFO("Starting autonomous");
 * }
 * @endcode
 *
 * To suppress the source tag from macros in a translation unit:
 * @code
 * #define LOG_SOURCE nullptr
 * #include "sfx/logger.hpp"
 * @endcode
 */
#ifndef LOG_SOURCE
/// @brief Default source tag used by the LOG_* macros when not overridden.
#define LOG_SOURCE "[USER]"
#endif

/// @brief Internal macro alias used by LOG_* macros.
#define _SFX_CURRENT_SOURCE LOG_SOURCE
/** @} */

// --------------------------------------------------------------------------
// Convenience log macros
// --------------------------------------------------------------------------

/**
 * @defgroup LoggingMacros Logging Macros
 * @brief Convenience wrappers around sfx::Logger::log_message().
 *
 * Where to use them:
 * - Most call-sites that just want to log something quickly.
 *
 * When to use them:
 * - Prefer LOG_INFO/WARN/ERROR over calling log_message() directly unless you
 *   need a custom source pointer or you are writing logger internals.
 *
 * @note These macros use LOG_SOURCE to tag messages per translation unit.
 * @{
 */

/// @brief Log a DEBUG-level message (usually noisy, for development).
#define LOG_DEBUG(fmt, ...)                                                    \
  sfx::Logger::get_instance().log_message(                                     \
      sfx::LogLevel::DEBUG, _SFX_CURRENT_SOURCE, fmt, ##__VA_ARGS__)

/// @brief Log an INFO-level message (normal operational breadcrumbs).
#define LOG_INFO(fmt, ...)                                                     \
  sfx::Logger::get_instance().log_message(                                     \
      sfx::LogLevel::INFO, _SFX_CURRENT_SOURCE, fmt, ##__VA_ARGS__)

/// @brief Log a WARN-level message (unexpected but recoverable situations).
#define LOG_WARN(fmt, ...)                                                     \
  sfx::Logger::get_instance().log_message(                                     \
      sfx::LogLevel::WARN, _SFX_CURRENT_SOURCE, fmt, ##__VA_ARGS__)

/// @brief Log an ERROR-level message (failure that likely affects behavior).
#define LOG_ERROR(fmt, ...)                                                    \
  sfx::Logger::get_instance().log_message(                                     \
      sfx::LogLevel::ERROR, _SFX_CURRENT_SOURCE, fmt, ##__VA_ARGS__)

/// @brief Log a FATAL-level message (serious failure; usually precedes a stop).
#define LOG_FATAL(fmt, ...)                                                    \
  sfx::Logger::get_instance().log_message(                                     \
      sfx::LogLevel::FATAL, _SFX_CURRENT_SOURCE, fmt, ##__VA_ARGS__)
/** @} */

/**
 * @struct MutexGuard
 * @brief RAII helper for PROS mutex take/give.
 *
 * Where to use it:
 * - Inside logger internals or any code that must reliably release a pros::Mutex.
 *
 * When to use it:
 * - Prefer this over manual take()/give() when exceptions/early returns are
 *   possible (even if you don't use exceptions, early returns are common).
 *
 * @note If the mutex cannot be taken, the guard will be "unlocked" and will not
 *       call give() in the destructor.
 */
struct MutexGuard {
  /// @brief Mutex reference managed by this guard.
  pros::Mutex &m;
  bool locked = false;
  explicit inline MutexGuard(pros::Mutex &m) : m(m) { locked = m.take(); }
  explicit inline MutexGuard(pros::Mutex &m, uint32_t timeout) : m(m) {
    locked = m.take(timeout);
  }
  ~MutexGuard() { if (locked) m.give(); }

  bool isLocked() const { return locked; }

  MutexGuard(const MutexGuard &) = delete;
  MutexGuard &operator=(const MutexGuard &) = delete;
};

/**
 * @enum LogLevel
 * @brief Log severity levels used for filtering and formatting.
 *
 * @note Ordering matters: higher values are considered "more severe".
 */
enum LogLevel {
  NONE = 0, /// The lowest log level. Used for simply disabling logger.
  DEBUG,    /// Used for info related to startup and diagnostics
  INFO,     /// The most frequently used log level. 
  WARN,     /// Used for logs still not dangerous, but that should stand out
  ERROR,    /// Used when something has gone wrong.
  FATAL     /// Used only for serious failures; often precedes a force stop.
}; 

// --------------------------------------------------------------------------
// Configuration constants (timings / thresholds)
// --------------------------------------------------------------------------

/// @brief How often to evaluate motor thermal state (ms).
constexpr uint32_t thermalCheckInterval = 15000;

/// @brief How often to print/refresh battery data (ms).
constexpr uint32_t batteryCheckInterval = 20000;

/// @brief How often to print task listing info (ms).
constexpr uint32_t taskPrintInterval = 30000;

/// @brief Timeout while waiting for stdin start char (ms).
constexpr uint16_t waitForStdInTimeout = 15000;

/// @brief SD file flush interval (ms).
constexpr uint32_t SD_FLUSH_INTERVAL_MS = 1000;

/// @brief Battery percentage threshold to warn the driver.
const double LOW_BATTERY_THRESHOLD = 30.0;

/// @brief Voltage threshold (mV) used to indicate critical low voltage.
const double CRITICAL_VOLTAGE_THRESHOLD = 11700;

// ---------- Generic variable watches ----------

/// @brief Identifier for a registered watch entry.
using WatchId = uint64_t;

/**
 * @brief shared
 * @brief Wrap a stack/global object pointer into a non-owning std::shared_ptr.
 *
 * Where to use it:
 * - For Logger::setRobot(), which expects std::shared_ptr references.
 *
 * @note This does NOT take ownership. The deleter is a no-op. The referenced
 *       object must outlive the shared_ptr and any Logger usage.
 *
 * Example:
 * @code
 * lemlib::Chassis chassis(...);
 * pros::MotorGroup left(...), right(...);
 *
 * logger.setRobot({
 *   .chassis = sfx::shared(chassis),
 *   .Left_Drivetrain = sfx::shared(left),
 *   .Right_Drivetrain = sfx::shared(right)
 * });
 * @endcode
 */
 
template <class T>
std::shared_ptr<std::remove_reference_t<T>> shared(T& obj) {
  using U = std::remove_reference_t<T>;
  return std::shared_ptr<U>(std::addressof(obj), [](U*) {}); // no-op deleter
}

/**
 * @def PREDICATE
 * @brief Helper for building a LevelOverride predicate with an int input.
 *
 * Where to use it:
 * - When using watch() with integer-like values and you want a concise predicate.
 *
 * @note This macro is limited to predicates over int32_t. For other types, use
 *       sfx::as_predicate<Typename>(expression) directly.
 */
#define PREDICATE(func) \
sfx::as_predicate<int32_t>([](int32_t v) { return func; })

/**
 * @struct LevelOverride
 * @brief Optional log-level override applied to a watch sample.
 *
 * A watch has a base log level (e.g., INFO). If predicate(expression) evaluates to
 * true, the watch sample is emitted at elevatedLevel instead.
 *
 * Where to use it:
 * - In watches where you want "normal" printing at INFO, but highlight abnormal
 *   values at WARN/ERROR.
 */
template <class T> struct LevelOverride {
  using value_type = T;

  /// @brief Level used when predicate returns true.
  LogLevel elevatedLevel = LogLevel::WARN;

  /// @brief Predicate to decide if a sample should be emitted at elevatedLevel.
  std::function<bool(const T &)> predicate;

  /// @brief An optional label that prints instead of the regular when the predicate is true.
  std::string label;
};

/**
 * @brief Convert an arbitrary predicate callable into std::function<bool(const T&)>.
 *
 * Where to use it:
 * - To pass lambdas/functions into LevelOverride in a type-erased form.
 *
 * @tparam T Predicate input type.
 * @tparam Pred Callable type (lambda, function pointer, functor).
 * @param p Predicate callable.
 * \return A std::function wrapper calling p(const T&).
 */
template <class T, class Pred>
std::function<bool(const T &)> as_predicate(Pred &&p) {
  return std::function<bool(const T &)>(std::forward<Pred>(p));
}

/**
 * @class Logger
 * @brief Singleton logging + telemetry manager.
 *
 * Where to use it:
 * - As the single source of truth for logging configuration.
 * - As a central place for periodic telemetry (pose, battery, tasks, watches).
 *
 * When to use it:
 * - Prefer it for on-robot debug output instead of scattered printf calls.
 * - Use watches for values you want sampled at a controlled cadence.
 *
 */
class Logger {
public:
  using LogLevel = ::sfx::LogLevel;

  /**
   * @struct loggerConfig
   * @brief Runtime configuration for Logger output and periodic reporters.
   *
   * @note Most fields are atomic so they can be toggled while running.
   */
  struct loggerConfig {
    std::atomic<bool> runThermalWatchdog{true};          ///< @brief Enable motor thermal watchdog printing.
    std::atomic<bool> printTelemetry{true};             ///< @brief Enable LemLib pose printing (requires chassis ref).
    std::atomic<bool> printBatteryData{true};            ///< @brief Enable battery status printing.
    std::atomic<bool> onlyPrintOverheatedMotors{true};   ///< @brief When true, omit non-overheated motors in thermal output.
    std::atomic<bool> printMotorWatchdogWarnings{true};  ///< @brief Emit warnings when motors appear unhealthy/overheated.
    std::atomic<bool> printPROSTasks{false};             ///< @brief Print PROS task list periodically.
    std::atomic<bool> logToTerminal{true};               ///< @brief Print logs to the terminal.
    std::atomic<bool> logToSD{true};                     ///< @brief Write logs to SD (locked after start()).
    std::atomic<bool> outputForViewer{false};           ///< @brief Output formatting mode for JerryIO tools.
    std::atomic<bool> printWatches{true};                ///< @brief Print registered watches.
  };

  /**
   * @struct RobotRef
   * @brief References to robot components used by telemetry helpers.
   *
   * Provide these via setRobot() so pose/motor watchdogs can run.
   */
  struct RobotRef {
    std::shared_ptr<lemlib::Chassis>  chassis;           ///< @brief LemLib chassis used for pose telemetry.
    std::shared_ptr<pros::MotorGroup> Left_Drivetrain;   ///< @brief Left drivetrain motors for thermal scanning.
    std::shared_ptr<pros::MotorGroup> Right_Drivetrain;  ///< @brief Right drivetrain motors for thermal scanning.
  };

  /**
   * @brief Access the singleton logger instance.
   * \return Reference to the global Logger instance.
   */
  static Logger &get_instance();

  // ------------------------------------------------------------------------
  // Lifecycle
  // ------------------------------------------------------------------------

  /**
   * @brief Start the logger background task (periodic telemetry + watches).
   *
   * When to use it:
   * - Call once after configuration and (optionally) setRobot().
   *
   * @note SD logging may become locked after start() if a failure is detected.
   */
  void start();

  /// @brief Pause periodic printing without destroying the logger task.
  void pause();

  /// @brief Resume after pause().
  void unpause();

  /**
   * @brief Get a compact status bitmask / state code.
   * \return Implementation-defined status value.
   *
   * @note The bitmap returned is from FreeRTOS Task Status Enum (pros::task_state_e_t).
   */
  uint32_t status() const;

  // ------------------------------------------------------------------------
  // Config setters/getters
  // ------------------------------------------------------------------------

  /// @brief Enable/disable the thermal watchdog.
  void setRunThermalWatchdog(bool v);

  /// @brief Enable/disable LemLib pose printing.
  void setPrintTelemetry(bool v);

  /// @brief Enable/disable battery printing.
  void setPrintBatteryData(bool v);

  /// @brief If true, only print overheated motors in thermal output.
  void setOnlyPrintOverheatedMotors(bool v);

  /// @brief Enable/disable watchdog warning lines.
  void setPrintWatchdogWarnings(bool v);

  /// @brief Enable/disable PROS task list printing.
  void setPrintProsTasks(bool v);

  /**
   * @brief Enable/disable terminal logging.
   *
   * @note This can typically be changed at runtime.
   */
  void setLogToTerminal(bool v);

  /**
   * @brief Enable/disable SD logging.
   *
   * @note Many implementations lock SD logging after start() to avoid file
   *       lifecycle issues. If that applies, calls after start() may no-op.
   */
  void setLogToSD(bool v);

  /// @brief Enable/disable JerryIO output formatting.
  void setOutputForViewer(bool v);

  /// @brief Configure whether to wait for a start character on stdin.
  void setWaitForStdIn(bool v);

  /// @brief Enable/disable printing of registered watches.
  void setPrintWatches(bool v);

  /// @brief Get whether the thermal watchdog is enabled.
  bool getRunThermalWatchdog() const { return config_.runThermalWatchdog.load(); }

  /// @brief Get whether pose printing is enabled.
  bool getPrintTelemtry() const { return config_.printTelemetry.load(); }

  /// @brief Get whether battery printing is enabled.
  bool getPrintBatteryData() const { return config_.printBatteryData.load(); }

  /// @brief Get whether non-overheated motors should be suppressed in thermal output.
  bool getOnlyPrintOverheatedMotors() const {
    return config_.onlyPrintOverheatedMotors.load();
  }

  /// @brief Get whether watchdog warnings are enabled.
  bool getPrintWatchdogWarnings() const {
    return config_.printMotorWatchdogWarnings.load();
  }

  /// @brief Get whether PROS tasks printing is enabled.
  bool getPrintProsTasks() const { return config_.printPROSTasks.load(); }

  /// @brief Get whether terminal logging is enabled.
  bool getLogToTerminal() const { return config_.logToTerminal.load(); }

  /// @brief Get whether SD logging is enabled.
  bool getLogToSD() const { return config_.logToSD.load(); }

  /// @brief Get whether JerryIO formatting is enabled.
  bool getoutputForViewer() const { return config_.outputForViewer.load(); }

  /// @brief Get whether watches are printed.
  bool getPrintWatches() const { return config_.printWatches.load(); }

  // ------------------------------------------------------------------------
  // Log filtering
  // ------------------------------------------------------------------------

  /**
   * @brief Set the minimum log level that will be emitted.
   *
   * Where to use it:
   * - Whenever you want to filter out logs that are not important to you.
   */
  void setLoggerMinLevel(LogLevel level);

  // ------------------------------------------------------------------------
  // Robot + motors
  // ------------------------------------------------------------------------

  /**
   * @brief Provide robot component references used by telemetry helpers.
   * @param ref Chassis/drivetrain refs (shared_ptr).
   * \return True if refs were accepted (e.g., non-null and consistent).
   *
   * @note If you do not call this, pose printing and some watchdog features may
   *       be disabled or will no-op.
   */
  bool setRobot(RobotRef ref);

  /**
   * @brief Register an additional motor group for monitoring.
   * @param name Human-readable name for telemetry output.
   * @param motor Motor group pointer to monitor.
   *
   * @note If you pass a raw pointer, ensure its lifetime exceeds the Logger's.
   */
  void registerMotor(std::string name, pros::MotorGroup *motor);

  // ------------------------------------------------------------------------
  // Logging
  // ------------------------------------------------------------------------

  /**
   * @brief Emit a formatted log message.
   *
   * Use the LOG_* macros unless you need a custom source pointer.
   *
   * @param level Log severity.
   * @param source Optional source tag string (may be nullptr).
   * @param fmt printf-style format string.
   */
  void log_message(LogLevel level, const char *source, const char *fmt, ...);

  /**
   * @brief Write a formatted log line to the SD log file.
   *
   * @note This is typically called by log_message() when SD logging is enabled.
   */
  void logToSD(const char *levelStr, const char *fmt, ...);

  // ------------------------------------------------------------------------
  // Battery helpers
  // ------------------------------------------------------------------------

  /**
   * @brief Print battery information immediately.
   *
   * Where to use it:
   * - Manual "check now" calls outside the periodic watchdog.
   */
  void printBatteryInfo();

  // ------------------------------------------------------------------------
  // Watches
  // ------------------------------------------------------------------------

  /**
   * @brief Register a periodic watch on a getter function.
   *
   * The getter is sampled every intervalMs and printed at baseLevel, unless
   * the optional override predicate elevates the level.
   *
   * @tparam Getter Callable that returns the value to render (numeric/bool/string/cstr).
   * @param label Display label for the watch.
   * @param baseLevel Level used for normal samples.
   * @param intervalMs Sampling/print interval in ms.
   * @param getter Callable returning a value.
   * @param ov Optional LevelOverride (type inferred from getter).
   * @param fmt Optional printf-style format for numeric values (e.g. "%.2f").
   * \return WatchId that can be used to identify the watch internally.
   *
   * Example:
   * @code
   * auto& logger = sfx::Logger::get_instance();
   * logger.watch("intake_rpm", sfx::LogLevel::INFO, 1000,
   * [&](){ return left_mg.get_actual_velocity(); },
   * sfx::LevelOverride<double>{ sfx::LogLevel::WARN,
   * PREDICATE(v > 550;) },
   * "%.0f");
   * @endcode
   */
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

  /**
   * @brief Register a watch that prints only when the rendered value changes.
   *
   * @tparam Getter Callable that returns the value to render.
   * @param label Display label for the watch.
   * @param baseLevel Level used for normal samples.
   * @param onChange If true, prints only on value change (interval ignored).
   * @param getter Callable returning a value.
   * @param ov Optional LevelOverride (type inferred from getter).
   * @param fmt Optional printf-style format for numeric values.
   * \return WatchId of the registered watch.
   */
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

  /// @brief Background update loop invoked by the logger task.
  void Update();

  /// @brief Print LemLib pose (requires chassis ref).
  void printLemlibPose_();

  /// @brief Print motor thermal watchdog telemetry.
  void printThermalWatchdog_();

  /// @brief Print battery watchdog telemetry.
  void printBatteryWatchdog_();

  /// @brief Print PROS task telemetry.
  void printProsTasks_();

  /// @brief Print currently running tasks (implementation-defined).
  void printRunningTasks_();

  /// @brief Validate that required robot references are present.
  bool checkRobotConfig_();

  /// @brief Initialize SD logger file handle and state.
  bool initSDLogger_();

  /// @brief Generate a timestamped filename into currentFilename_.
  void makeTimestampedFilename_();

  /// @brief Optional gate: wait for a start char before beginning output.
  void waitForStartChar();

  /**
   * @brief Convert a LogLevel to a printable string.
   * @param level Log level to convert.
   * \return C-string representation of the level.
   */
  const char *levelToString_(LogLevel level) const;

  /**
   * @struct Watch
   * @brief Internal watch record.
   */
  struct Watch {
    WatchId id{};                       ///< @brief Watch identifier.
    std::string label;                  ///< @brief Watch display label.
    LogLevel baseLevel{LogLevel::INFO}; ///< @brief Base log level for normal samples.
    uint32_t intervalMs{1000};          ///< @brief Print interval (ms) when not onChange.
    uint32_t lastPrintMs{0};            ///< @brief Last print timestamp (ms).
    std::string fmt;                    ///< @brief Optional numeric format string.

    bool onChange = false;             ///< @brief If true, prints only when value changes.
    std::optional<std::string> lastValue = std::nullopt;; ///< @brief Last rendered value (for onChange).

    /// @brief Computes (level, rendered string) for the current sample.
    std::function<std::tuple<LogLevel, std::string, std::string>()> eval;
  };

  /// @brief Next watch id to assign.
  WatchId nextId_ = 1;

  /// @brief Watch registry keyed by WatchId.
  std::unordered_map<WatchId, Watch> watches_;

  // --- rendering helpers ---

  /**
   * @brief Render a std::string as-is.
   * \return The rendered string.
   */
  static std::string renderValue(const std::string &v, const std::string &) {
    return v;
  }

  /**
   * @brief Render a C-string safely.
   * \return "(null)" if v is nullptr, otherwise v as std::string.
   */
  static std::string renderValue(const char *v, const std::string &) {
    return v ? std::string(v) : std::string("(null)");
  }

  /**
   * @brief Render a boolean as "true"/"false".
   * \return Rendered boolean string.
   */
  static std::string renderValue(bool v, const std::string &) {
    return v ? "true" : "false";
  }

  /**
   * @brief Render arithmetic types using an optional printf-style format.
   *
   * @tparam T Value type.
   * @param v Value to render.
   * @param fmt Optional printf-style format string.
   * \return Rendered value string.
   *
   * @note Non-arithmetic types fall back to "<unrenderable>".
   */
  template <class T>
  static std::string renderValue(const T& v, const std::string& fmt) {
    if constexpr (std::is_floating_point_v<T>) {
      if (!fmt.empty()) {
        char buf[256];
        std::snprintf(buf, sizeof(buf), fmt.c_str(), (double)v);
        return std::string(buf);
      }
      return std::to_string((double)v);
    } else if constexpr (std::is_integral_v<T>) {
      (void)fmt; // ignore fmt for integrals
      return std::to_string((long long)v);
    } else {
      return std::string("<unrenderable>");
    }
  }


  // --- core builder ---

  /**
   * @brief Internal watch registration routine.
   *
   * @tparam T Watch value type.
   * @tparam Getter Getter callable type.
   * @param label Display label.
   * @param baseLevel Base log level.
   * @param intervalMs Interval in ms (ignored when onChange=true).
   * @param getter Getter callable.
   * @param ov Optional override predicate/level.
   * @param fmt Optional numeric format.
   * @param onChange If true, print only on change.
   * \return Assigned WatchId.
   */
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
    const std::string labelCopy = w.label;

    w.eval = [baseLevel, labelCopy, fmtCopy, g = std::move(g), 
              ov = std::move(ov)]() mutable -> 
              std::tuple<LogLevel, std::string, std::string> {

      T v = static_cast<T>(g());

      const bool tripped = (ov.predicate && ov.predicate(v));

      LogLevel lvl = baseLevel;
      if (tripped) lvl = ov.elevatedLevel;

      std::string rawOut = renderValue(v, fmtCopy);

      // Get default label
      std::string displayOut = labelCopy;

      if (tripped && !ov.label.empty()) {
        displayOut = ov.label;
      }
      return {lvl, std::move(rawOut), std::move(displayOut)};
    };

    WatchId id = w.id;
    watches_.emplace(id, std::move(w));
    return id;
  }

  /// @brief Print all watches that are due (and/or changed).
  void printWatches();

  /**
   * @struct MotorMonitor
   * @brief Internal record of a motor group to scan.
   */
  struct MotorMonitor {
    std::string name;                          ///< @brief Display name.
    std::shared_ptr<pros::MotorGroup> group = nullptr; ///< @brief Motor group pointer (shared ownership if provided).
  };

  // ------------------------------------------------------------------------
  // Internal state
  // ------------------------------------------------------------------------

  loggerConfig config_{};           
  LogLevel minLogLevel_ = LogLevel::INFO;

  pros::Mutex logToSdMutex_;       
  pros::Mutex loggerMutex_;       
  pros::Mutex generalMutex_;      

  uint32_t lastFlush_ = 0;   
  FILE *sdFile_ = nullptr;      
  char currentFilename_[128] = "";   
  const char *date = __DATE__;
  bool waitForSTDin_ = false;        
  bool started_ = false;             
  bool sdLocked_ = false;    
  bool configSet_ = false;       

  // Robot refs
  std::shared_ptr<lemlib::Chassis> pChassis_ = nullptr; 
  std::shared_ptr<pros::MotorGroup> pLeftDrivetrain_ = nullptr; 
  std::shared_ptr<pros::MotorGroup> pRightDrivetrain_ = nullptr; 

  std::vector<MotorMonitor> internalMotorsToScan_; 

  std::unique_ptr<pros::Task> task_;
};

} // namespace sfx