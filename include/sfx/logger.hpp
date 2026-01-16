#pragma once

/**
 * @file logger.hpp
 * @brief Comprehensive logging system for robot telemetry (specifically made
 * for LemLib), diagnostics, and debugging information.
 *
 * @note This header provides a complete logging framework that interfaces with
 * the PROS terminal to output color-coded telemetry data. The logging system
 * uses ANSI color codes for terminal output, making it easy to distinguish
 * between different log levels (INFO, WARN, ERROR).
 *
 * @note Even if you are outputting logs into an SD card, it is recommended to
 * open the terminal on first setup to watch for errors.
 * * @note To use the logger effectively, set up a shell function that automates
 * connecting to the robot and starting the logging task. Add the proslog()
 * function to your
 * ~/.zshrc file (see example below).
 *
 * @example Setup shell function in ~/.zshrc:
 * @code{.bash}
 * # ---------------------------------------------------
 * # PROS Logging Function + Auto Start
 * # Checks for connection. If found:
 * # 1. Creates log file
 * # 2. Sends 'Y' to robot to start the task
 * # 3. Saves output to file
 * # ---------------------------------------------------
 * proslog() {
 * if pros lsusb | grep -q "usbmodem"; then
 * mkdir -p Logs
 * local logfile="Logs/$(date +%Y-%m-%d_%H-%M-%S).log"
 * * echo -e "\033[0;32m[INFO]:\033[0m Robot Connected!"
 * echo -e "\033[0;32m[INFO]:\033[0m Sending handshake 'Y'..."
 * echo -e "\033[0;32m[INFO]:\033[0m Logging clean data to $logfile"
 * echo -e "\033[0;32m[INFO]:\033[0m Press Ctrl+C to stop."
 * * # 1. { echo "Y"; cat -u; } -> Sends "Y", then keeps the pipe open for your
 * keyboard. # 2. | script ... -> Feeds that "Y" into the PROS terminal as if
 * you typed it. # 3. | tee ... -> Splits the output to Screen and File
 * Cleaners.
 * * { echo "Y"; cat -u; } | script -q /dev/null pros terminal | tee >(perl -pe
 * 'BEGIN{$|=1} s/\x1b\[[0-9;]*[mGK]//g' > "$logfile")
 * * else
 * echo -e "\033[0;31m[ERROR]:\033[0m No V5 device detected."
 * fi
 * }
 * @endcode
 * After setup, start logging by running: proslog
 */

#include "lemlib/chassis/chassis.hpp" // For lemlib::ControllerSettings
#include "pros/abstract_motor.hpp"    // For MotorGroup
#include "sfx/motorChecks.hpp"        // For overheating checks
#include <atomic>
#include <cstdarg>
#include <cstring>


namespace sfx {
namespace Logger {

// 1. If the User manually defined a name (e.g. #define LOG_SOURCE "Lift"), use
// it.
#ifdef LOG_SOURCE
#define _SFX_CURRENT_SOURCE LOG_SOURCE

// 2. Otherwise, use our auto-detector function
#else
// Determine which file macro is available (Modern GCC vs Standard)
#ifdef __FILE_NAME__
#define _SFX_FILE_MACRO __FILE_NAME__
#else
#define _SFX_FILE_MACRO __FILE__
#endif

// Pass the filename to our constexpr function to decide if it's
// [USER] or [SYSTEM]

#define _SFX_CURRENT_SOURCE                                                    \
  sfx::Logger::internal::resolveLogSource(_SFX_FILE_MACRO)
#endif

namespace internal {

// Helper: Checks if a string ends with a suffix (e.g. "logger.hpp")
constexpr bool ends_with(std::string_view str, std::string_view suffix) {
  return str.size() >= suffix.size() &&
         str.compare(str.size() - suffix.size(), suffix.size(), suffix) == 0;
}

// The Logic: Returns the specific name for system files, otherwise "[USER]"
constexpr const char *resolveLogSource(const char *filepath) {
  std::string_view path(filepath);

  // Whitelist: Allowed System Files
  if (ends_with(path, "logger.hpp"))
    return "logger.hpp";
  if (ends_with(path, "infoSink.cpp"))
    return "infoSink.cpp";

  // Default for everything else
  return "[USER]";
}

} // namespace internal

/**
 * @struct MutexGuard
 * @brief RAII wrapper for handling PROS mutexes.
 * * @note Automatically takes the mutex upon construction and gives it upon
 * destruction to ensure thread safety even if exceptions occur.
 */
struct MutexGuard {
  pros::Mutex &m;
  explicit MutexGuard(pros::Mutex &m, uint32_t timeout = 10) : m(m) {
    m.take(timeout);
  }
  ~MutexGuard() { m.give(); }
};

/**
 * @defgroup LoggingMacros Logging Macros
 * @brief Printf-style logging macros with color-coded output and timestamps.
 *
 * @note These macros can be called from anywhere in your code after the logger
 * is initialized. They automatically include timestamps (seconds since program
 * start) and appropriate log level prefixes. Output uses ANSI color codes: INFO
 * (green), WARN (yellow/orange), ERROR (red).
 *
 * @note All macros use printf-style formatting with variadic arguments,
 * compatible with standard C format strings.
 *
 * @example Usage throughout your code:
 * @code{.cpp}
 * LOG_INFO("Robot initialized successfully");
 * LOG_WARN("Motor temperature: %.1f C", temp);
 * LOG_ERROR("Battery voltage critical: %.2fV", voltage);
 * @endcode
 *
 * @{
 */
#define LOG_DEBUG(fmt, ...)                                                    \
  sfx::Logger::log_message(sfx::Logger::LogLevel::LOG_LEVEL_DEBUG,             \
                           _SFX_CURRENT_SOURCE, fmt, ##__VA_ARGS__)

#define LOG_INFO(fmt, ...)                                                     \
  sfx::Logger::log_message(sfx::Logger::LogLevel::LOG_LEVEL_INFO,              \
                           _SFX_CURRENT_SOURCE, fmt, ##__VA_ARGS__)

#define LOG_WARN(fmt, ...)                                                     \
  sfx::Logger::log_message(sfx::Logger::LogLevel::LOG_LEVEL_WARN,              \
                           _SFX_CURRENT_SOURCE, fmt, ##__VA_ARGS__)

#define LOG_ERROR(fmt, ...)                                                    \
  sfx::Logger::log_message(sfx::Logger::LogLevel::LOG_LEVEL_ERROR,             \
                           _SFX_CURRENT_SOURCE, fmt, ##__VA_ARGS__)

#define LOG_FATAL(fmt, ...)                                                    \
  sfx::Logger::log_message(sfx::Logger::LogLevel::LOG_LEVEL_FATAL,             \
                           _SFX_CURRENT_SOURCE, fmt, ##__VA_ARGS__)
/** @} */

/**
 * @brief Internal function for logging to SD card.
 *
 * @note This function is called automatically by the logging system.
 * Should not be called directly by user code.
 *
 * @param levelStr Log level string ("INFO", "WARN", "ERROR")
 * @param fmt Printf-style format string
 * @param ... Variadic arguments for format string
 */
void logToSD(const char *levelStr, const char *fmt, ...);

/**
 * @brief Initializes the SD card logger system.
 *
 * @note Checks for SD card presence, creates directories if needed, and opens
 * a timestamped log file.
 *
 * @return true if SD logger initialized successfully, false otherwise
 */
extern bool initSDLogger();

/**
 * @enum LogLevel
 * @brief Enumeration of log levels for filtering log output.
 */
enum LogLevel {
  LOG_LEVEL_NONE = 0, ///< Stop all logger printing
  LOG_LEVEL_DEBUG,    ///< Debug level - verbose, dev-only info
  LOG_LEVEL_INFO,     ///< Info level - general information messages
  LOG_LEVEL_WARN,     ///< Warning level - warnings and non-critical issues
  LOG_LEVEL_ERROR,    ///< Error level - errors and critical issues
  LOG_LEVEL_FATAL     ///< Fatal level - unrecoverable errors
};

/**
 * @brief Minimum log level threshold for filtering log output.
 * @note Defaults to LOG_LEVEL_INFO (shows all messages).
 */
inline LogLevel minLogLevel = LOG_LEVEL_INFO;

/**
 * @brief Core logging dispatch function.
 * @param level Severity of the log message.
 * @param file Source file name (optional).
 * @param fmt Format string.
 */
void log_message(LogLevel level, const char *file, const char *fmt, ...);

/**
 * @brief Sets the minimum log level threshold.
 *
 * @note Can be called at any time to change filtering behavior.
 *
 * @param level The minimum log level to display (messages below this level are
 * filtered)
 *
 * @example Filter to only show errors:
 * @code{.cpp}
 * setLoggerMinLevel(LOG_LEVEL_ERROR);
 * @endcode
 */
inline void setLoggerMinLevel(LogLevel level) {
  minLogLevel = level;
  LOG_DEBUG("SetLoggerMinLevel set to: %d", level);
}

// --------------------------------------------------------------------------
// Configuration Constants
// --------------------------------------------------------------------------

/** @brief Interval in ms between thermal checks (Default: 15s). */
constexpr uint32_t thermalCheckInterval = 15000;

/** @brief Interval in ms between battery checks (Default: 20s). */
constexpr uint32_t batteryCheckInterval = 20000;

/** @brief Interval in ms between task list prints (Default: 30s). */
constexpr uint32_t taskPrintInterval = 30000;

/** @brief How long to wait (ms) for input char before continuing automatically.
 */
constexpr uint16_t waitForStdInTimeout = 15000;

/**
 * @struct loggerConfig
 * @brief Structure containing configuration parameters for the logger system.
 *
 * @note This structure should be configured BEFORE calling startLogger() during
 * initialization. It can also be modified at runtime using the setter
 * functions.
 */
struct loggerConfig {
  /** @brief Enable monitoring motor temperatures? */
  std::atomic<bool> runThermalWatchdog{true};

  /** @brief Enable printing robot pose (X, Y, Theta) to terminal? */
  std::atomic<bool> printLemlibPose{true};

  /** @brief Enable monitoring battery voltage and capacity? */
  std::atomic<bool> printBatteryData{true};

  /** @brief If true, only print thermal logs when motors are actually hot. */
  std::atomic<bool> onlyPrintOverheatedMotors{true};

  /** @brief If true, print warnings when motors are warm but not yet critical.
   */
  std::atomic<bool> printMotorWatchdogWarnings{true};

  /** @brief Enable periodic printing of all running FreeRTOS tasks? */
  std::atomic<bool> printPROSTasks{false};

  /** @brief Enable outputting logs to the V5 Terminal? */
  std::atomic<bool> logToTerminal{true};

  /** @brief Enable saving logs to the SD card? */
  std::atomic<bool> logToSD{true};

  /** @brief If true, format output as CSV for JerryIO external tool
   * compatibility. */
  std::atomic<bool> outputForJerryio{false};
};

extern loggerConfig Config;

// --------------------------------------------------------------------------
// Setters & Getters
// --------------------------------------------------------------------------

void setRunThermalWatchdog(bool v);         ///< Toggle thermal monitoring
void setPrintLemlibPose(bool v);            ///< Toggle pose (X, Y, Theta, drivetrain speed) printing
void setPrintBatteryData(bool v);           ///< Toggle battery voltage/capacity logging
void setOnlyPrintOverheatedMotors(bool v);  ///< If true, hide thermal logs unless hot
void setPrintWatchdogWarnings(bool v);      ///< Toggle "approaching limit" warnings
void setPrintProsTasks(bool v);             ///< Toggle periodic Task list printing
void setLogToTerminal(bool v);              ///< Toggle output to V5 Terminal
void setLogToSD(bool v);                    ///< Toggle output to SD Card file
void setOutputForJerryio(bool v);           ///< Toggle CSV format for external tools
void setWaitForStdIn(bool v);               ///< If true, wait for user input before starting

inline bool getRunThermalWatchdog() { return Config.runThermalWatchdog.load(); }               ///< Get thermal monitoring state
inline bool getPrintLemlibPose() { return Config.printLemlibPose.load(); }                     ///< Get pose printing state
inline bool getPrintBatteryData() { return Config.printBatteryData.load(); }                   ///< Get battery logging state
inline bool getOnlyPrintOverheatedMotors() { return Config.onlyPrintOverheatedMotors.load(); } ///< Get thermal filter state
inline bool getPrintWatchdogWarnings() { return Config.printMotorWatchdogWarnings.load(); }    ///< Get thermal warning state
inline bool getPrintProsTasks() { return Config.printPROSTasks.load(); }                       ///< Get task printing state
inline bool getLogToTerminal() { return Config.logToTerminal.load(); }                         ///< Get terminal logging state
inline bool getLogToSD() { return Config.logToSD.load(); }                                     ///< Get SD logging state
inline bool getOutputForJerryio() { return Config.outputForJerryio.load(); }  

// --------------------------------------------------------------------------
// SD Card & System
// --------------------------------------------------------------------------

/**
 * @brief Buffer/flush frequency to the SD card in milliseconds (default:
 * 1000ms).
 * @note Keep between 500ms and 2500ms to balance IO usage and RAM buffer size.
 */
constexpr uint32_t SD_FLUSH_INTERVAL_MS = 1000;

/** @brief Frequency to close and reopen the file to ensure data safety. */
constexpr uint32_t AUTO_SAVE_INTERVAL = 30000;

/** @brief Global flag indicating if logger waits for stdin input. */
extern bool waitForSTDin;

/**
 * @brief Checks if the logger task has been started.
 * @return true if logger has started, false otherwise.
 */
extern bool loggerStatus();

// --------------------------------------------------------------------------
// Battery Thresholds
// --------------------------------------------------------------------------

/**
 * @brief Battery capacity threshold for low battery warning (percentage).
 * @note Default: 30.0 (30%).
 */
const double LOW_BATTERY_THRESHOLD = 30.0;

/**
 * @brief Battery voltage threshold for critical voltage error (millivolts).
 * @note Default: 11000 (11.0V).
 */
const double CRITICAL_VOLTAGE_THRESHOLD = 11000;

/**
 * @brief Checks and logs battery capacity and voltage.
 * @note Called automatically if printBatteryData is enabled.
 */
void printBatteryInfo();

// --------------------------------------------------------------------------
// Robot Configuration
// --------------------------------------------------------------------------

/**
 * @struct RobotRef
 * @brief Structure containing pointers to robot components required for
 * logging.
 *
 * @note All members MUST be non-null pointers to fully initialized objects that
 * remain valid for the logger's lifetime. Objects should be defined globally.
 *
 * @example Setup with global objects:
 * @code{.cpp}
 * void initialize() {
 * Logger::RobotRef config;
 * config.chassis = &chassis;
 * config.lateralSettings = &lateralSettings;
 * config.angularSettings = &angularSettings;
 * config.Left_Drivetrain = &left_motors;
 * config.Right_Drivetrain = &right_motors;
 * Logger::setRobot(config);
 * Logger::startLogger();
 * }
 * @endcode
 */
struct RobotRef {
  /** @brief Pointer to LemLib chassis for pose data (X, Y, Theta). Must be
   * valid and global. */
  lemlib::Chassis *chassis;

  /** @brief Pointer to lateral PID settings (dumped at startup). Must be valid
   * and global. */
  lemlib::ControllerSettings *lateralSettings;

  /** @brief Pointer to angular PID settings (dumped at startup). Must be valid
   * and global. */
  lemlib::ControllerSettings *angularSettings;

  /** @brief Pointer to left drivetrain motor group for velocity logging. */
  pros::MotorGroup *Left_Drivetrain;

  /** @brief Pointer to right drivetrain motor group for velocity logging. */
  pros::MotorGroup *Right_Drivetrain;
};

/**
 * @brief Configures the logger with pointers to robot components.
 *
 * @note MUST be called EXACTLY ONCE during initialization, BEFORE
 * startLogger().
 *
 * @param Config RobotRef structure with all required pointers.
 * @return true if successful, false if failed (nullptr detected or called
 * twice).
 */
extern bool setRobot(RobotRef Config);

/**
 * @brief Initializes and starts the background logging task.
 *
 * @note REQUIRES setRobot() to be called FIRST with valid RobotRef
 * configuration.
 * @note The logger task waits for stdin input ('Y' character) before starting
 * if waitForStdIn is true.
 *
 * @example
 * @code{.cpp}
 * void initialize() {
 * Logger::RobotRef config = { ... };
 * Logger::setRobot(config);
 * Logger::startLogger();
 * }
 * @endcode
 */
void startLogger();

// --------------------------------------------------------------------------
// Motor Monitoring
// --------------------------------------------------------------------------

/**
 * @struct MotorMonitor
 * @brief Structure representing a motor group to be monitored for thermal
 * status.
 */
struct MotorMonitor {
  /** @brief Human-readable name identifying the motor group (e.g. "Intake"). */
  std::string name;

  /** @brief Pointer to the motor group to monitor. Must remain valid. */
  pros::MotorGroup *group;
};

/**
 * @brief Registers a motor group to be monitored for thermal status.
 *
 * @note Should be called before startLogger().
 *
 * @param name Human-readable name for the motor group.
 * @param motor Pointer to the global motor group object.
 *
 * @example
 * @code{.cpp}
 * Logger::registerMotor("Left Drive", &left_motors);
 * Logger::registerMotor("Intake", &intake);
 * @endcode
 */
void registerMotor(std::string name, pros::MotorGroup *motor);

} // namespace Logger
} // namespace sfx