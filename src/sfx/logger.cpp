#include "sfx/motorChecks.hpp"
#include <cstdarg>
#include <cstring>

#define LOG_SOURCE nullptr
#include "sfx/logger.hpp"

namespace sfx {
namespace Logger {

static uint32_t lastFlush = 0;

static FILE *sdFile = nullptr;
static char currentFilename[128] = "";

bool waitForSTDin = false;
bool loggerStarted = false;

// Define the mutexes and config struct
loggerConfig Config;
pros::Mutex logToSdMutex;
pros::Mutex loggerMutex;
pros::Mutex generalMutex;

static std::shared_ptr<lemlib::Chassis> pChassis = nullptr;
static std::shared_ptr<pros::MotorGroup> pLeftDrivetrain = nullptr;
static std::shared_ptr<pros::MotorGroup> pRightDrivetrain = nullptr;

static std::unique_ptr<pros::Task> logger;

struct MotorMonitor {
  std::string name;
  std::shared_ptr<pros::MotorGroup> group = nullptr;
};

static std::vector<MotorMonitor>
    internalMotorsToScan; // Holds motors to watchdog scan

void setRunThermalWatchdog(bool v) {
  Config.runThermalWatchdog.store(v);
  LOG_DEBUG("setRunThermalWatchdog set to: %d", v);
}

void setPrintLemlibPose(bool v) {
  Config.printLemlibPose.store(v);
  LOG_DEBUG("setPrintLemlibPose set to: %d", v);
}

void setPrintBatteryData(bool v) {
  Config.printBatteryData.store(v);
  LOG_DEBUG("setPrintBatteryData set to: %d", v);
}

void setOnlyPrintOverheatedMotors(bool v) {
  Config.onlyPrintOverheatedMotors.store(v);
  LOG_DEBUG("onlyPrintOverheatedMotors set to: %d", v);
}

void setPrintWatchdogWarnings(bool v) {
  Config.printMotorWatchdogWarnings.store(v);
  LOG_DEBUG("printWatchdogWarnings set to: %d", v);
}

void setPrintProsTasks(bool v) {
  Config.printPROSTasks.store(v);
  LOG_DEBUG("printProsTasks set to: %d", v);
}

void setLogToTerminal(bool v) {
  Config.logToTerminal.store(v);
  LOG_DEBUG("logToTerminal set to: %d", v);
}

void setLogToSD(bool v) {
  Config.logToSD.store(v);
  LOG_DEBUG("logToSD set to: %d", v);
}

void setOutputForJerryio(bool v) {
  Config.outputForJerryio.store(v);
  LOG_DEBUG("outputForJerryio set to: %d", v);
}

void setWaitForStdIn(bool v) {
  if (Config.logToSD.load() && !Config.logToTerminal.load()) {
    LOG_WARN("waitForStdIn value is not settable; LogToSD is enabled.");
    return;
  }

  if (loggerStatus()) {
    LOG_WARN(
        "setWaitForStdIn() called after logger start — ignored. Set value: %d",
        v);
    return;
  }
  waitForSTDin = v;
}

void setLoggerMinLevel(LogLevel level) {
  minLogLevel = level;
  LOG_DEBUG("SetLoggerMinLevel set to: %d", level);
}

void registerMotor(std::string name, pros::MotorGroup *motor) {
  // Thread safety
  MutexGuard m(generalMutex);
  if (!m.isLocked())
    return;

  // Add the new motor to our internal list
  try {
    if (motor != nullptr) {
      std::shared_ptr<pros::MotorGroup> newMotor =
          std::shared_ptr<pros::MotorGroup>(motor);
      internalMotorsToScan.push_back({name, newMotor});
    } else
      LOG_WARN("registerMotor called with nullptr arguments!");
  } catch (std::exception &e) {
    LOG_ERROR("Exception adding motor to thermal watchdog: %s", e.what());
  }
}

const char *levelToString(LogLevel level) {
  switch (level) {
  case LogLevel::LOG_LEVEL_DEBUG:
    return "DEBUG";
  case LogLevel::LOG_LEVEL_INFO:
    return "INFO";
  case LogLevel::LOG_LEVEL_WARN:
    return "WARN";
  case LogLevel::LOG_LEVEL_ERROR:
    return "ERROR";
  case LogLevel::LOG_LEVEL_FATAL:
    return "FATAL";
  default:
    return "UNKNOWN";
  }
}

void log_message(LogLevel level, const char *source, const char *fmt, ...) {
  if (level < minLogLevel)
    return;

  // Thread safety
  MutexGuard m(loggerMutex);
  if (!m.isLocked())
    return;

  char buffer[512];
  va_list args;
  va_start(args, fmt);
  vsnprintf(buffer, sizeof(buffer), fmt, args);
  va_end(args);

  double time = pros::millis() / 1000.0;

  // Determine colors AND reset codes based on settings
  const char *color = "";
  const char *reset = "";

  // Only use colors if we are not in JerryIO mode
  if (Config.logToTerminal.load() && !Config.outputForJerryio.load()) {
    reset = "\033[0m"; // Enable reset code
    switch (level) {
    case LogLevel::LOG_LEVEL_DEBUG:
      color = "\033[36m";
      break;
    case LogLevel::LOG_LEVEL_INFO:
      color = "\033[32m";
      break;
    case LogLevel::LOG_LEVEL_WARN:
      color = "\x1b[0;38;5;214m";
      break;
    case LogLevel::LOG_LEVEL_ERROR:
      color = "\033[31m";
      break;
    case LogLevel::LOG_LEVEL_FATAL:
      color = "\033[0;31;2m";
      break;
    default:
      break;
    }
  }

  if (source != nullptr && source[0] != '\0') {
    printf("%s[%.2f] %s[%s] %s:%s %s\n", reset, time, color,
           levelToString(level), source, reset, buffer);
  } else {
    printf("%s[%.2f] %s[%s]:%s %s\n", reset, time, color, levelToString(level),
           reset, buffer);
  }

  // Log to SD Card
  if (Config.logToSD.load()) {
    logToSD(levelToString(level), "%s", buffer);
  }
}

bool setRobot(RobotRef ref) {
  static bool isConfigSet = false;
  if (isConfigSet) {
    LOG_WARN("setRobot(RobotRef ref) called twice!\n");
    return false;
  }
  isConfigSet = true;

  if (!ref.chassis || !ref.Left_Drivetrain || !ref.Right_Drivetrain) {
    LOG_FATAL("setRobot(RobotRef ref) called with nullptr arguments!\n");
    return false;
  }

  try {
    // Take ownership via shared_ptr
    pChassis = std::shared_ptr<lemlib::Chassis>(ref.chassis);
    pLeftDrivetrain = std::shared_ptr<pros::MotorGroup>(ref.Left_Drivetrain);
    pRightDrivetrain = std::shared_ptr<pros::MotorGroup>(ref.Right_Drivetrain);
  } catch (const std::exception &e) {
    LOG_FATAL("Failed to allocate memory for robot objects: %s\n", e.what());
    return false;
  }

  LOG_INFO("setRobot(RobotRef ref) successfully set variables!");
  return true;
}

bool checkRobotConfig(bool checkLemLib = true) {
  // Thread safety
  MutexGuard m(generalMutex, TIMEOUT_MAX);

  bool allValid = true;

  if (pChassis == nullptr && checkLemLib) {
    LOG_FATAL("Chassis pointer is NULL!\n");
    allValid = false;
  }
  if (pLeftDrivetrain == nullptr) {
    LOG_FATAL("Left Drivetrain pointer is NULL!\n");
    allValid = false;
  }
  if (pRightDrivetrain == nullptr) {
    LOG_FATAL("Right Drivetrain pointer is NULL!\n");
    allValid = false;
  }

  return allValid;
}

extern "C" void vTaskList(char *pcWriteBuffer);

void printRunningTasks() {
  // Thread safety
  MutexGuard m(generalMutex);
  if (!m.isLocked())
    return;

  char buffer[1024];
  vTaskList(buffer);

  LOG_INFO("\n--- Task List ---\n");
  LOG_INFO("Name                         	State  Prio   Stack   Num\n");
  LOG_INFO("-------------------------------------------------------\n");
  LOG_INFO("%s", buffer);
  LOG_INFO("-------------------------------------------------------\n");
}

static void makeTimestampedFilename(size_t len) {
  // Thread safety
  MutexGuard m(generalMutex, TIMEOUT_MAX);

  time_t now = time(0);
  struct tm *tstruct = localtime(&now);

  if (tstruct->tm_year < 100) {
    // Save directly to the global variable currentFilename
    printf("[DEBUG]: VEX RTC Inaccurate. Falling back to program duration.\n");
    snprintf(currentFilename, sizeof(currentFilename), "/usd/%u-%u.log",
             pros::millis() / 1000, pros::millis());
  } else {
    printf("[DEBUG]: VEX RTC Plausible. Creating file name with date.\n");
    strftime(currentFilename, sizeof(currentFilename),
             "/usd/%Y-%m-%d_%H-%M.log", tstruct);
  }
}

void pauseLogger() {
  uint32_t status = loggerStatus();
  if (status != pros::E_TASK_STATE_DELETED &&
      status != pros::E_TASK_STATE_INVALID &&
      status != pros::E_TASK_STATE_SUSPENDED &&
      status != pros::E_TASK_STATE_BLOCKED) {
    logger->suspend();
  } else
    LOG_INFO("Logger cannot be paused as it is not in a running state.");
}

void unpauseLogger() {
  uint32_t status = loggerStatus();
  if (status != pros::E_TASK_STATE_DELETED &&
      status != pros::E_TASK_STATE_INVALID &&
      status == pros::E_TASK_STATE_SUSPENDED &&
      status != pros::E_TASK_STATE_BLOCKED) {
    logger->resume();
  } else
    LOG_INFO("Logger cannot be unpaused as it is not paused.");
}

uint32_t loggerStatus() { return logger->get_state(); }

// Public
bool initSDLogger() {
  // Thread safety
  MutexGuard m(generalMutex);
  if (!m.isLocked())
    return false;

  if (pros::usd::is_installed()) {
    printf("[DEBUG]: SD Card installed\n");
    pros::delay(500);
  } else {
    printf("[DEBUG]: SD Card not installed, rechecking...\n");
    for (int i = 0; i < 10; i++) {
      if (pros::usd::is_installed()) {
        printf("[DEBUG]: SD Card installed! Attempt: %d/10", i);
        break;
      }
      printf("[DEBUG]: Rechecking SD card installment... Attempts: %d/10", i);
      pros::delay(100);
    }
  }

  if (!pros::usd::is_installed()) {
    printf(
        "[DEBUG]: SD Card not installed after 10 attemps. Aborting SD card.\n");
    return false;
  }

  makeTimestampedFilename(sizeof(currentFilename));

  sdFile = fopen(currentFilename, "w");
  if (!sdFile) {
    printf("[DEBUG]: File could not be opened. Aborting.\n");
    return false;
  }
  printf("[DEBUG]: File successfully opened.\n");
  fprintf(sdFile, "=== Logger initialized at %.2fs ===\n",
          pros::millis() / 1000.0);
  fflush(sdFile);
  return true;
}

void logToSD(const char *levelStr, const char *fmt, ...) {
  if (!sdFile) {
    printf(
        "[ERROR]: Writing to SD card failed!. FILE * is null. Message lost!\n");
    printf("Message error level was: %s\n", levelStr);
    return;
  }
  // Thread safety
  MutexGuard m(logToSdMutex);
  if (!m.isLocked())
    return;

  // 1. Write to RAM buffer
  fprintf(sdFile, "[%.2f] [%s]: ", pros::millis() / 1000.0, levelStr);

  va_list args;
  va_start(args, fmt);
  vfprintf(sdFile, fmt, args);
  va_end(args);

  fprintf(sdFile, "\n");

  // 2. CHECK: Is this an error?
  bool isError = (strcmp(levelStr, "ERROR") == 0);

  // 3. FLUSH LOGIC
  if (isError || (pros::millis() - lastFlush >= SD_FLUSH_INTERVAL_MS)) {
    fflush(sdFile);
    lastFlush = pros::millis();
  }
}

void printBatteryInfo() {
  // Thread safety
  MutexGuard m(generalMutex);
  if (!m.isLocked())
    return;

  double capacity = pros::battery::get_capacity();
  double voltage = pros::battery::get_voltage();

  if (voltage < CRITICAL_VOLTAGE_THRESHOLD) {
    LOG_ERROR("Low Voltage Detected (CRITICAL): %.2fV (Capacity: %.0f%%)",
              (voltage / 1000.0), capacity);
  } else if (capacity < LOW_BATTERY_THRESHOLD) {
    LOG_WARN("Low Battery: %.0f%% | Voltage: %.2fV", capacity,
             (voltage / 1000.0));
  } else {
    LOG_INFO("Battery Good: %.0f%% | Voltage: %.2fV", capacity,
             (voltage / 1000.0));
  }
}

void startLogger() {
  static bool hasLoggerStarted = false;
  if (hasLoggerStarted) {
    LOG_WARN("Function: startLogger() called a second time! Function "
             "aborted.\n");
    return;
  }
  hasLoggerStarted = true;
  loggerStarted = hasLoggerStarted;

  // RUNTIME CHECK: Initialize SD card if enabled in Config
  if (Config.logToSD.load()) {
    bool success = initSDLogger();
    if (!success) {
      // If SD was requested but failed, we warn but don't necessarily kill the
      // logger if terminal is also active.
      Config.logToSD.store(false);
      LOG_FATAL("initSDCard failed! Unable to initialize SD card.\n");
    } else {
      LOG_INFO("Successfully initialized SD card!\n");
    }
  }

  // Calculate divide factor to normalize return velocity to ±127
  // Static initialization ensures this only runs once
  static pros::MotorGears drivetrain_gearset = pLeftDrivetrain->get_gearing();
  static double divide_factor_drivetrainRPM = 1;
  switch (drivetrain_gearset) {
  case pros::MotorGears::rpm_100:
    divide_factor_drivetrainRPM = 100.0;
    break;
  case pros::MotorGears::rpm_200:
    divide_factor_drivetrainRPM = 200.0;
    break;
  case pros::MotorGears::rpm_600:
    divide_factor_drivetrainRPM = 600.0;
    break;
  default:
    divide_factor_drivetrainRPM = 300.0; // safest fallback
  }

  // Lambda helper to cap the output at ±127
  auto norm = [&](double rpm) {
    double v = (rpm / divide_factor_drivetrainRPM) * 127.0;
    if (v > 127)
      v = 127;
    if (v < -127)
      v = -127;
    return v;
  };

  pros::Task logger(
      [&]() {
        char startChar;
        uint32_t startTime = pros::millis();

        if (Config.logToTerminal.load() && waitForSTDin) {
          LOG_INFO("Waiting for handshake (Y) with timeout...\n");

          uint32_t startTime = pros::millis();

          while ((pros::millis() - startTime) < waitForStdInTimeout) {
            int c = getchar(); // Get one character from the serial buffer

            if (c != EOF) { // EOF means the buffer is currently empty
              char startChar = (char)c;
              if (startChar == 'Y') {
                LOG_INFO("startChar received! Starting logger...");
                break;
              }
            }
            if ((pros::millis() - startTime) < waitForStdInTimeout - 21) {
              LOG_WARN("Logger auto-started after timeout.\n");
            }
            pros::delay(20); // Small delay to prevent CPU hogging
          }

        } else {
          LOG_INFO("Logger started automatically!\n");
        }

        pros::delay(1000); // controller RX settle

        // Check setup and allow for chassis * to be nullptr if not using lemlib
        if (Config.printLemlibPose.load()) {
          if (!checkRobotConfig()) {
            LOG_FATAL("At least one pointer set by setRobot(RobotRef "
                      "ref) is nullptr. Aborting!\n");
            LOG_INFO("You can disable LemLib logging to allow logger to run "
                     "without setting up its config.");
            return;
          } else {
            LOG_INFO("All pointers set by setRobot(RobotRef ref) seem "
                     "to be valid.");
          }
        } else if (!checkRobotConfig(false)) {
          LOG_FATAL("At least one pointer set by setRobot(RobotRef "
                    "ref) is nullptr. Aborting!\n");
          return;
        } else
          LOG_INFO("All pointers set by setRobot(RobotRef ref) seem "
                   "to be valid.");

        try {
          // ----- Logging ----- //
          if (Config.outputForJerryio.load()) {
            if (!Config.printLemlibPose.load()) {
              LOG_FATAL("You MUST have LemLib logging enabled to use "
                        "outputForJerryio!");
              LOG_FATAL("Enable LemLib logging or disable outputForJerryio");
              return;
            }

            // THIS IS FOR LOGGING TO .log. We use a special format for this
            while (true) {
              float normalizedTheta = fmod(pChassis->getPose().theta, 360.0);

              LOG_INFO("[DATA],%.2f,%.2f,%.2f,%.1f,%.1f", pChassis->getPose().x,
                       pChassis->getPose().y, normalizedTheta,
                       norm(pLeftDrivetrain->get_actual_velocity()),
                       norm(pRightDrivetrain->get_actual_velocity()));

              // RUNTIME CHECK: Delay
              if (Config.logToTerminal.load()) {
                pros::delay(120); // main loop delay for bandwidth safety
              } else {
                pros::delay(80); // Faster logging if just SD card
              }
            }
          }

          // Creation of reference times for spaced checks
          uint32_t lastThermalCheck, lastBatteryCheck, lastTasksPrint,
              lastAutoSave = pros::millis();

          while (true) {
            if (Config.printLemlibPose.load()) {
              LOG_INFO(
                  "Pose X: %.2f Y: %.2f Theta: %.2f | LVel: %.1f RVel: %.1f",
                  pChassis->getPose().x, pChassis->getPose().y,
                  pChassis->getPose().theta,
                  norm(pLeftDrivetrain->get_actual_velocity()),
                  norm(pRightDrivetrain->get_actual_velocity()));
            }

            uint32_t now = pros::millis();

            // Thermal check every X ms
            if (Config.runThermalWatchdog.load() &&
                (now - lastThermalCheck) >= thermalCheckInterval) {
              for (auto &entry : internalMotorsToScan) {
                auto status =
                    motorChecks::checkMotorOverheat(*entry.group, 55.0);

                if (status.overheated) {
                  std::string ports;
                  for (auto p : status.motorsOverheated)
                    ports += std::to_string(p) + ", ";
                  if (!ports.empty())
                    ports.pop_back();

                  if (status.maxTemp >= 65) {
                    LOG_ERROR("%s CRITICAL TEMP! Max: %.0fC | Ports: %s",
                              entry.name.c_str(), status.maxTemp,
                              ports.c_str());
                  } else {
                    LOG_WARN("%s Overheating! Max: %.0fC | Ports: %s",
                             entry.name.c_str(), status.maxTemp, ports.c_str());
                  }
                } else if (Config.printMotorWatchdogWarnings.load() &&
                           status.maxTemp >= 50.0) {
                  LOG_WARN("%s is warm/approaching throttle. (Max: %.0fC)",
                           entry.name.c_str(), status.maxTemp);
                } else if (!Config.onlyPrintOverheatedMotors.load()) {
                  LOG_INFO("%s MotorGroup OK (Max: %.0fC)", entry.name.c_str(),
                           status.maxTemp);
                }
              }
              lastThermalCheck = now;
            }

            // Battery check every Y ms
            if (Config.printBatteryData.load() &&
                (now - lastBatteryCheck) >= batteryCheckInterval) {
              printBatteryInfo();
              lastBatteryCheck = now;
            }

            if (Config.printPROSTasks.load() &&
                (now - lastTasksPrint) >= taskPrintInterval) {
              printRunningTasks();
              lastTasksPrint = now;
            }

            if (pros::millis() - lastAutoSave > AUTO_SAVE_INTERVAL &&
                Config.logToSD.load()) {
              LOG_INFO("Attempting to close/open SD card...");
              if (sdFile != nullptr) {
                // Force the buffer clear
                fflush(sdFile);
                // 1. Get the current filename
                fclose(sdFile);
                // 2. Reopen in APPEND mode so we don't overwrite
                sdFile = fopen(currentFilename, "a");
                if (sdFile != nullptr) {
                  LOG_INFO("SD Card successfully reopened.");
                  fprintf(sdFile, "[SYSTEM]: Auto-save successful.\n");
                } else {
                  LOG_FATAL("SD Card could not be reopened; disabling SD card "
                            "logging.");
                  Config.logToSD.store(false);
                }
                fflush(sdFile);
                lastAutoSave = pros::millis();
                // No need for a LOG_INFO here, it would spam the log
              } else
                LOG_ERROR(
                    "sdFile has become nullptr, disabling SD card logging.");
              Config.logToSD.store(false);
            }

            fflush(stdout);

            // RUNTIME CHECK: Delay
            if (Config.logToTerminal.load()) {
              pros::delay(120); // main loop delay for bandwidth safety
            } else {
              pros::delay(80); // Faster logging if just SD card
            }
          }
        } catch (const std::exception &e) {
          LOG_FATAL("Logger crashed: %s", e.what());
          loggerStarted = false;
        }
      },
      TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "SFX Logger");
}
} // namespace Logger
} // namespace sfx