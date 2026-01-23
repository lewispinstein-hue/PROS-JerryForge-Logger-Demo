#include "pros/rtos.hpp"
#define LOG_SOURCE nullptr
#include "sfx/logger.hpp"
#include "sfx/motorChecks.hpp"

#include <cmath>
#include <cstdarg>
#include <cstring>
#include <ctime>

namespace sfx {

Logger &Logger::get_instance() {
  static Logger instance;
  return instance;
}

void Logger::copyConfigFrom_(const Logger::loggerConfig &cfg) {
  config_.runThermalWatchdog.store(cfg.runThermalWatchdog.load());
  config_.printLemlibPose.store(cfg.printLemlibPose.load());
  config_.printBatteryData.store(cfg.printBatteryData.load());
  config_.onlyPrintOverheatedMotors.store(cfg.onlyPrintOverheatedMotors.load());
  config_.printMotorWatchdogWarnings.store(
      cfg.printMotorWatchdogWarnings.load());
  config_.printPROSTasks.store(cfg.printPROSTasks.load());
  config_.logToTerminal.store(cfg.logToTerminal.load());
  config_.logToSD.store(cfg.logToSD.load());
  config_.outputForJerryio.store(cfg.outputForJerryio.load());
}

void Logger::initialize(const Logger::loggerConfig &cfg) {
  // Thread safety
  MutexGuard m(generalMutex_, TIMEOUT_MAX);
  if (!m.isLocked())
    return;

  // Copy atomics (assignment operator is deleted)
  copyConfigFrom_(cfg);

  // SD init happens here if enabled (per your requirement)
  if (config_.logToSD.load()) {
    bool success = initSDLogger_();
    if (!success) {
      config_.logToSD.store(false);
      LOG_FATAL("initSDCard failed! Unable to initialize SD card.\n");
    } else {
      LOG_INFO("Successfully initialized SD card!\n");
    }
  }
}

void Logger::setRunThermalWatchdog(bool v) {
  config_.runThermalWatchdog.store(v);
  LOG_DEBUG("setRunThermalWatchdog set to: %d", v);
}

void Logger::setPrintLemlibPose(bool v) {
  config_.printLemlibPose.store(v);
  LOG_DEBUG("setPrintLemlibPose set to: %d", v);
}

void Logger::setPrintBatteryData(bool v) {
  config_.printBatteryData.store(v);
  LOG_DEBUG("setPrintBatteryData set to: %d", v);
}

void Logger::setOnlyPrintOverheatedMotors(bool v) {
  config_.onlyPrintOverheatedMotors.store(v);
  LOG_DEBUG("onlyPrintOverheatedMotors set to: %d", v);
}

void Logger::setPrintWatchdogWarnings(bool v) {
  config_.printMotorWatchdogWarnings.store(v);
  LOG_DEBUG("printWatchdogWarnings set to: %d", v);
}

void Logger::setPrintProsTasks(bool v) {
  config_.printPROSTasks.store(v);
  LOG_DEBUG("printProsTasks set to: %d", v);
}

void Logger::setLogToTerminal(bool v) {
  config_.logToTerminal.store(v);
  LOG_DEBUG("logToTerminal set to: %d", v);
}

void Logger::setLogToSD(bool v) {
  if (started_ || sdLocked_) {
    LOG_WARN("setLogToSD() called after logger start — ignored. Set value: %d",
             v);
    return;
  }
  config_.logToSD.store(v);
  LOG_DEBUG("logToSD set to: %d", v);
}

void Logger::setOutputForJerryio(bool v) {
  config_.outputForJerryio.store(v);
  LOG_DEBUG("outputForJerryio set to: %d", v);
}

void Logger::setWaitForStdIn(bool v) {
  if (config_.logToSD.load() && !config_.logToTerminal.load()) {
    LOG_WARN("waitForStdIn value is not settable; LogToSD is enabled.");
    return;
  }
  if (status()) {
    LOG_WARN(
        "setWaitForStdIn() called after logger start — ignored. Set value: %d",
        v);
    return;
  }
  waitForSTDin_ = v;
}

void Logger::setLoggerMinLevel(LogLevel level) {
  minLogLevel_ = level;
  LOG_DEBUG("SetLoggerMinLevel set to: %d", (int)level);
}

bool Logger::setRobot(RobotRef ref) {
  static bool isConfigSet = false;
  if (isConfigSet) {
    LOG_WARN("setRobot(RobotRef) called twice!");
    return false;
  }
  isConfigSet = true;

  if (!ref.Left_Drivetrain || !ref.Right_Drivetrain) {
    LOG_FATAL("setRobot(RobotRef) called with nullptr drivetrain arguments!");
    return false;
  }

  // chassis is allowed to be nullptr now
  if (!ref.chassis) {
    LOG_WARN(
        "setRobot(): chassis is nullptr (OK if printLemlibPose is disabled).");
  }

  pChassis_ = ref.chassis;
  pLeftDrivetrain_ = ref.Left_Drivetrain;
  pRightDrivetrain_ = ref.Right_Drivetrain;

  LOG_INFO("setRobot() successfully set variables!");
  return true;
}

void Logger::registerMotor(std::string name, pros::MotorGroup *motor) {
  MutexGuard m(generalMutex_);
  if (!m.isLocked())
    return;

  try {
    if (motor != nullptr) {
      // Preserve original semantics: wrap provided pointer in shared_ptr.
      // NOTE: This assumes caller manages lifetime appropriately.
      std::shared_ptr<pros::MotorGroup> newMotor =
          std::shared_ptr<pros::MotorGroup>(motor);
      internalMotorsToScan_.push_back({name, newMotor});
    } else {
      LOG_WARN("registerMotor called with nullptr arguments!");
    }
  } catch (std::exception &e) {
    LOG_ERROR("Exception adding motor to thermal watchdog: %s", e.what());
  }
}

const char *Logger::levelToString_(LogLevel level) const {
  switch (level) {
  case LogLevel::DEBUG:
    return "DEBUG";
  case LogLevel::INFO:
    return "INFO";
  case LogLevel::WARN:
    return "WARN";
  case LogLevel::ERROR:
    return "ERROR";
  case LogLevel::FATAL:
    return "FATAL";
  default:
    return "UNKNOWN";
  }
}

void Logger::log_message(LogLevel level, const char *source, const char *fmt,
                         ...) {
  if (level < minLogLevel_)
    return;

  MutexGuard m(loggerMutex_);
  if (!m.isLocked())
    return;

  char buffer[512];
  va_list args;
  va_start(args, fmt);
  vsnprintf(buffer, sizeof(buffer), fmt, args);
  va_end(args);

  double time = pros::millis() / 1000.0;

  const char *color = "";
  const char *reset = "";

  if (config_.logToTerminal.load() && !config_.outputForJerryio.load()) {
    reset = "\033[0m";
    switch (level) {
    case LogLevel::DEBUG:
      color = "\033[36m";
      break;
    case LogLevel::INFO:
      color = "\033[32m";
      break;
    case LogLevel::WARN:
      color = "\x1b[0;38;5;214m";
      break;
    case LogLevel::ERROR:
      color = "\033[31m";
      break;
    case LogLevel::FATAL:
      color = "\033[0;31;2m";
      break;
    default:
      break;
    }
  }

  if (source != nullptr && source[0] != '\0') {
    printf("%s[%.2f] %s[%s] %s:%s %s\n", reset, time, color,
           levelToString_(level), source, reset, buffer);
  } else {
    printf("%s[%.2f] %s[%s]:%s %s\n", reset, time, color, levelToString_(level),
           reset, buffer);
  }

  if (config_.logToSD.load()) {
    logToSD(levelToString_(level), "%s", buffer);
  }
}

extern "C" void vTaskList(char *pcWriteBuffer);

void Logger::printRunningTasks_() {
  MutexGuard m(generalMutex_);
  if (!m.isLocked())
    return;

  char buffer[1024];
  vTaskList(buffer);

  printf("\n--- Task List ---\n");
  printf("Name                         	State  Prio   Stack   Num\n");
  printf("-------------------------------------------------------\n");
  printf("%s", buffer);
  printf("-------------------------------------------------------\n");
}

void Logger::makeTimestampedFilename_(size_t /*len*/) {
  MutexGuard m(generalMutex_, TIMEOUT_MAX);

  time_t now = time(0);
  struct tm *tstruct = localtime(&now);

  if (tstruct->tm_year < 100) {
    printf("[DEBUG]: VEX RTC Inaccurate. Falling back to program duration.\n");
    snprintf(currentFilename_, sizeof(currentFilename_), "/usd/%u-%u.log",
             pros::millis() / 1000, pros::millis());
  } else {
    printf("[DEBUG]: VEX RTC Plausible. Creating file name with date.\n");
    strftime(currentFilename_, sizeof(currentFilename_),
             "/usd/%Y-%m-%d_%H-%M.log", tstruct);
  }
}

bool Logger::initSDLogger_() {
  MutexGuard m(generalMutex_);
  if (!m.isLocked())
    return false;

  if (pros::usd::is_installed()) {
    printf("[DEBUG]: SD Card installed\n");
    pros::delay(500);
  } else {
    printf("[DEBUG]: SD Card not installed, rechecking...\n");
    for (int i = 0; i < 10; i++) {
      if (pros::usd::is_installed()) {
        printf("[DEBUG]: SD Card installed! Attempt: %d/10\n", i);
        break;
      }
      printf("[DEBUG]: Rechecking SD card installment... Attempts: %d/10\n", i);
      pros::delay(100);
    }
  }

  if (!pros::usd::is_installed()) {
    printf(
        "[DEBUG]: SD Card not installed after 10 attemps. Aborting SD card.\n");
    return false;
  }

  makeTimestampedFilename_(sizeof(currentFilename_));

  sdFile_ = fopen(currentFilename_, "w");
  if (!sdFile_) {
    printf("[DEBUG]: File could not be opened. Aborting.\n");
    return false;
  }
  printf("[DEBUG]: File successfully opened.\n");
  fprintf(sdFile_, "=== Logger initialized at %.2fs ===\n",
          pros::millis() / 1000.0);
  fflush(sdFile_);
  return true;
}

void Logger::logToSD(const char *levelStr, const char *fmt, ...) {
  if (!sdFile_) {
    printf("[%d] [ERROR]: Writing to SD card failed!. FILE * is null. Message "
           "lost! ",
           pros::millis() / 1000);
    printf("Message error level was: %s\n", levelStr);
    return;
  }

  MutexGuard m(logToSdMutex_);
  if (!m.isLocked())
    return;

  fprintf(sdFile_, "[%.2f] [%s]: ", pros::millis() / 1000.0, levelStr);

  va_list args;
  va_start(args, fmt);
  vfprintf(sdFile_, fmt, args);
  va_end(args);

  fprintf(sdFile_, "\n");

  bool isError = (strcmp(levelStr, "ERROR") == 0);

  if (isError || (pros::millis() - lastFlush_ >= SD_FLUSH_INTERVAL_MS)) {
    fflush(sdFile_);
    lastFlush_ = pros::millis();
  }
}

void Logger::printBatteryInfo() {
  MutexGuard m(generalMutex_);
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

bool Logger::checkRobotConfig_(bool checkLemLib) {
  MutexGuard m(generalMutex_, TIMEOUT_MAX);

  bool allValid = true;

  if (pChassis_ == nullptr && checkLemLib) {
    LOG_FATAL("Chassis pointer is NULL!\n");
    allValid = false;
  }
  if (pLeftDrivetrain_ == nullptr) {
    LOG_FATAL("Left Drivetrain pointer is NULL!\n");
    allValid = false;
  }
  if (pRightDrivetrain_ == nullptr) {
    LOG_FATAL("Right Drivetrain pointer is NULL!\n");
    allValid = false;
  }

  return allValid;
}

uint32_t Logger::status() const {
  if (!task_)
    return pros::E_TASK_STATE_INVALID;
  return task_->get_state();
}

void Logger::pause() {
  uint32_t st = status();
  if (st != pros::E_TASK_STATE_DELETED && st != pros::E_TASK_STATE_INVALID &&
      st != pros::E_TASK_STATE_SUSPENDED && st != pros::E_TASK_STATE_BLOCKED) {
    task_->suspend();
  } else {
    LOG_INFO("Logger cannot be paused as it is not in a running state.");
  }
}

void Logger::unpause() {
  uint32_t st = status();
  if (st != pros::E_TASK_STATE_DELETED && st != pros::E_TASK_STATE_INVALID &&
      st == pros::E_TASK_STATE_SUSPENDED && st != pros::E_TASK_STATE_BLOCKED) {
    task_->resume();
  } else {
    LOG_INFO("Logger cannot be unpaused as it is not paused.");
  }
}

void Logger::start() {
  if (started_) {
    LOG_WARN("Function: start() called a second time! Function aborted.\n");
    return;
  }
  started_ = true;
  sdLocked_ = true;

  // If user didn't call initialize(), SD init used to happen here.
  // Preserve behavior by initializing SD now if requested and not already open.
  if (config_.logToSD.load() && sdFile_ == nullptr) {
    bool success = initSDLogger_();
    if (!success) {
      config_.logToSD.store(false);
      LOG_FATAL("initSDCard failed! Unable to initialize SD card.\n");
    } else {
      LOG_INFO("Successfully initialized SD card!\n");
    }
  }

  // Calculate divide factor to normalize return velocity to ±127 (unchanged)
  static pros::MotorGears drivetrain_gearset =
      pLeftDrivetrain_ ? pLeftDrivetrain_->get_gearing()
                       : pros::MotorGears::invalid;
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
    divide_factor_drivetrainRPM = 300.0;
  }

  // Lambda helper to cap the output at ±127 (unchanged)
  auto norm = [&](double rpm) {
    double v = (rpm / divide_factor_drivetrainRPM) * 127.0;
    if (v > 127)
      v = 127;
    if (v < -127)
      v = -127;
    return v;
  };

  // Create task that runs Update (Update uses same structure as original code)
  task_ = std::make_unique<pros::Task>(
      [this, norm]() mutable {
        // Preserve original task body (moved into Update-like logic with same
        // checks). We keep the original code structure here to avoid behavior
        // changes. NOTE: Helper extraction happens inside Update(). Store norm
        // lambda in thread local via capture; pass to pose printing via
        // member-lambda usage inside Update.
        (void)norm; // norm is used in Update() via re-computation; kept for
                    // capture symmetry.
        this->Update();
      },
      TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "SFX Logger");
}

void Logger::printWatches() {
  uint32_t nowMs = pros::millis();

  for (auto &[id, w] : watches_) {
    // Gate evaluation frequency for ALL watches
    if (w.lastPrintMs != 0 && (nowMs - w.lastPrintMs) < w.intervalMs) {
      continue;
    }
    w.lastPrintMs = nowMs;

    if (!w.eval)
      continue;

    auto [lvl, valueStr] = w.eval();

    if (w.onChange) {
      if (w.lastValue && *w.lastValue == valueStr) {
        continue;
      }
      w.lastValue = valueStr;
    }

    switch (lvl) {
    case LogLevel::DEBUG:
      LOG_DEBUG("%s %s", w.label.c_str(), valueStr.c_str());
      break;
    case LogLevel::INFO:
      LOG_INFO("%s %s", w.label.c_str(), valueStr.c_str());
      break;
    case LogLevel::WARN:
      LOG_WARN("%s %s", w.label.c_str(), valueStr.c_str());
      break;
    case LogLevel::ERROR:
      LOG_ERROR("%s %s", w.label.c_str(), valueStr.c_str());
      break;
    default:
      LOG_INFO("%s %s", w.label.c_str(), valueStr.c_str());
      break;
    }
  }
}

// --- Helper extractions (minimal; called by Update) ---
void Logger::printLemlibPose_() {
  // This function is intentionally implemented inside Update() to preserve
  // exact norm() capture.
}

void Logger::printThermalWatchdog_() {
  for (auto &entry : internalMotorsToScan_) {
    auto status = motorChecks::checkMotorOverheat(*entry.group, 55.0);

    if (status.overheated) {
      std::string ports;
      for (auto p : status.motorsOverheated)
        ports += std::to_string(p) + ", ";
      if (!ports.empty())
        ports.pop_back();

      if (status.maxTemp >= 65) {
        LOG_ERROR("%s CRITICAL TEMP! Max: %.0fC | Ports: %s",
                  entry.name.c_str(), status.maxTemp, ports.c_str());
      } else {
        LOG_WARN("%s Overheating! Max: %.0fC | Ports: %s", entry.name.c_str(),
                 status.maxTemp, ports.c_str());
      }
    } else if (config_.printMotorWatchdogWarnings.load() &&
               status.maxTemp >= 50.0) {
      LOG_WARN("%s is warm/approaching throttle. (Max: %.0fC)",
               entry.name.c_str(), status.maxTemp);
    } else if (!config_.onlyPrintOverheatedMotors.load()) {
      LOG_INFO("%s MotorGroup OK (Max: %.0fC)", entry.name.c_str(),
               status.maxTemp);
    }
  }
}

void Logger::handleAutoSaveSd_() {
  LOG_INFO("Attempting to close/open SD card...");
  if (sdFile_ != nullptr) {
    fflush(sdFile_);
    fclose(sdFile_);
    sdFile_ = fopen(currentFilename_, "a");
    if (sdFile_ != nullptr) {
      LOG_INFO("SD Card successfully reopened.");
      fprintf(sdFile_, "[SYSTEM]: Auto-save successful.\n");
    } else {
      LOG_FATAL("SD Card could not be reopened; disabling SD card logging.");
      config_.logToSD.store(false);
    }
    fflush(sdFile_);
  } else {
    LOG_ERROR("sdFile has become nullptr, disabling SD card logging.");
  }
  config_.logToSD.store(false);
}

void Logger::Update() {
  static pros::MotorGears drivetrain_gearset =
      pLeftDrivetrain_ ? pLeftDrivetrain_->get_gearing()
                       : pros::MotorGears::invalid;
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
    divide_factor_drivetrainRPM = 300.0;
  }
  auto norm = [&](double rpm) {
    double v = (rpm / divide_factor_drivetrainRPM) * 127.0;
    if (v > 127)
      v = 127;
    if (v < -127)
      v = -127;
    return v;
  };

  try {
    char startChar;
    uint32_t startTime = pros::millis();

    if (config_.logToTerminal.load() && waitForSTDin_) {
      LOG_INFO("Waiting for handshake (Y) with timeout...\n");

      uint32_t startTime2 = pros::millis();

      while ((pros::millis() - startTime2) < waitForStdInTimeout) {
        int c = getchar();

        if (c != EOF) {
          char startChar2 = (char)c;
          if (startChar2 == 'Y') {
            LOG_INFO("startChar received! Starting logger...");
            break;
          }
        }
        if ((pros::millis() - startTime2) < waitForStdInTimeout - 21) {
          LOG_WARN("Logger auto-started after timeout.\n");
        }
        pros::delay(20);
      }

    } else {
      LOG_INFO("Logger started automatically!");
    }

    pros::delay(1000);

    if (config_.printLemlibPose.load()) {
      if (!checkRobotConfig_()) {
        LOG_FATAL("At least one pointer set by setRobot(RobotRef ref) is "
                  "nullptr. Aborting!\n");
        LOG_INFO("You can disable LemLib logging to allow logger to run "
                 "without setting up its config.");
        return;
      } else {
        LOG_INFO(
            "All pointers set by setRobot(RobotRef ref) seem to be valid.");
      }
    } else if (!checkRobotConfig_(false)) {
      LOG_FATAL("At least one pointer set by setRobot(RobotRef ref) is "
                "nullptr. Aborting!\n");
      return;
    } else {
      LOG_INFO("All pointers set by setRobot(RobotRef ref) seem to be valid.");
    }

    // ----- Logging ----- //
    if (config_.outputForJerryio.load()) {
      if (!config_.printLemlibPose.load()) {
        LOG_FATAL(
            "You MUST have LemLib logging enabled to use outputForJerryio!");
        LOG_FATAL("Enable LemLib logging or disable outputForJerryio");
        return;
      }

      while (true) {
        float normalizedTheta = fmod(pChassis_->getPose().theta, 360.0);

        LOG_INFO("[DATA],%.2f,%.2f,%.2f,%.1f,%.1f", pChassis_->getPose().x,
                 pChassis_->getPose().y, normalizedTheta,
                 norm(pLeftDrivetrain_->get_actual_velocity()),
                 norm(pRightDrivetrain_->get_actual_velocity()));

        if (config_.logToTerminal.load()) {
          pros::delay(120);
        } else {
          pros::delay(80);
        }
      }
    }

    uint32_t lastThermalCheck, lastBatteryCheck, lastTasksPrint,
        lastAutoSave = pros::millis();

    while (true) {
      if (config_.printLemlibPose.load()) {
        LOG_INFO("Pose X: %.2f Y: %.2f Theta: %.2f | LVel: %.1f RVel: %.1f",
                 pChassis_->getPose().x, pChassis_->getPose().y,
                 pChassis_->getPose().theta,
                 norm(pLeftDrivetrain_->get_actual_velocity()),
                 norm(pRightDrivetrain_->get_actual_velocity()));
      }

      uint32_t now = pros::millis();

      if (config_.runThermalWatchdog.load() &&
          (now - lastThermalCheck) >= thermalCheckInterval) {
        printThermalWatchdog_();
        lastThermalCheck = now;
      }

      if (config_.printBatteryData.load() &&
          (now - lastBatteryCheck) >= batteryCheckInterval) {
        printBatteryInfo();
        lastBatteryCheck = now;
      }

      if (config_.printPROSTasks.load() &&
          (now - lastTasksPrint) >= taskPrintInterval) {
        printRunningTasks_();
        lastTasksPrint = now;
      }

      if (pros::millis() - lastAutoSave > AUTO_SAVE_INTERVAL &&
          config_.logToSD.load()) {
        handleAutoSaveSd_();
        lastAutoSave = pros::millis();
      }

      printWatches();

      fflush(stdout);

      if (config_.logToTerminal.load()) {
        pros::delay(120);
      } else {
        pros::delay(80);
      }
    }
  } catch (const std::exception &e) {
    LOG_FATAL("Logger crashed: %s", e.what());
    started_ = false;
  }
}

} // namespace sfx
