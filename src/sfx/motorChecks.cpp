#include "sfx/motorChecks.hpp"

namespace sfx {
namespace motorChecks {

// --- Helper: ANSI Codes ---
namespace colors {
constexpr const char *RED = "\033[31m";
constexpr const char *GREEN = "\033[32m";
constexpr const char *YELLOW = "\033[33m";
constexpr const char *MAGENTA = "\033[35m";
constexpr const char *RESET = "\033[0m";
} // namespace colors

checkTemp checkMotorOverheat(pros::MotorGroup &mg, float maxTemp) {
  checkTemp params;

  double maxTempSeen = 0;

  for (int index = 0; index < mg.size(); index++) {
    double temp = mg.get_temperature(index);
    maxTempSeen = std::max(maxTempSeen, temp);

    if (temp >= maxTemp) {
      params.motorsOverheated.push_back(mg.get_port(index));
    }
  }

  params.maxTemp = maxTempSeen;
  params.overheated = !params.motorsOverheated.empty();
  return params;
}

std::string formatCheckTemp(std::string_view name, const checkTemp &t,
                                  bool useAnsi) {
  std::stringstream ss;

  // Helper lambda to conditionally apply color
  auto c = [&](const char *color) -> const char * {
    return useAnsi ? color : "";
  };

  ss << "-------| " << name << " |-------\n";

  // Status Line
  ss << "Temp Status: ";
  if (t.overheated) {
    ss << c(colors::RED) << "OVERHEATED" << c(colors::RESET);
  } else {
    ss << c(colors::GREEN) << "OK" << c(colors::RESET);
  }
  ss << "\n";

  // Max Temp
  ss << "Max Temp: " << std::fixed << std::setprecision(1) << t.maxTemp
     << " C\n";

  // Details
  if (t.overheated && !t.motorsOverheated.empty()) {
    ss << c(colors::YELLOW) << "Overheated motors ("
       << t.motorsOverheated.size() << "): " << c(colors::RESET);

    for (auto port : t.motorsOverheated) {
      ss << (int)port << " ";
    }
    ss << "\n";
  } else {
    ss << "Overheated motors: none\n";
  }
  ss << "\n";
  
  return ss.str();
}

scanMotorReturn scanMotor(pros::MotorGroup &mg, int maxAmpDraw,
                          double maxVelocity, double maxTemp) {
  // 1. Start Active Test
  mg.move(80);

  // 2. Perform Passive Temp Check (DRY: reusing existing function)
  scanMotorReturn result;
  result.thermal = checkMotorOverheat(mg, maxTemp);

  // 3. Perform Active Stall Check
  std::vector<int> stallHits(mg.size(), 0);

  for (int sample = 0; sample < SAMPLE_COUNT; sample++) {
    for (int i = 0; i < mg.size(); i++) {
      int current = mg.get_current_draw(i);
      double velocity = std::fabs(mg.get_actual_velocity(i));

      result.maxCurrent = std::max(result.maxCurrent, current);

      if (current > maxAmpDraw && velocity < maxVelocity) {
        stallHits[i]++;
      }
    }
    pros::delay(SAMPLE_DELAY_MS);
  }

  // SAFETY: Always stop motors after test
  mg.move(0);

  // 4. Evaluate Stall Results
  for (int i = 0; i < mg.size(); i++) {
    if (stallHits[i] >= STALL_REQUIRED) {
      result.motorsStalled.push_back(mg.get_port(i));
    }
  }

  result.stalled = !result.motorsStalled.empty();

  // Overall Pass/Fail
  result.passed = (!result.stalled && !result.thermal.overheated);

  return result;
}

std::string formatMotorScanReturn(std::string_view name,
                                  const scanMotorReturn &r, bool useANSI) {
  std::stringstream ss;

  auto c = [&](const char *color) -> const char * {
    return useANSI ? color : "";
  };

  ss << "-------| " << name << " |-------\n";

  // Overall Status
  ss << "Overall: ";
  if (r.passed) {
    ss << c(colors::GREEN) << "PASS" << c(colors::RESET);
  } else {
    ss << c(colors::RED) << "FAIL" << c(colors::RESET);
  }
  ss << "\n";

  // Temperature Section 
  ss << "Max Temp: " << std::fixed << std::setprecision(1) << r.thermal.maxTemp
     << " C\n";

  if (r.thermal.overheated) {
    ss << c(colors::YELLOW) << "Overheating motors ("
       << r.thermal.motorsOverheated.size() << "): " << c(colors::RESET)
       << formatMotorPorts(r.thermal.motorsOverheated) << "\n";
  } else {
    ss << "Overheating motors: none\n";
  }

  // Current/Stall Section
  ss << "Max Current: " << r.maxCurrent << " mA\n";

  if (r.stalled) {
    ss << c(colors::MAGENTA) << "Stalled motors (" << r.motorsStalled.size()
       << "): " << c(colors::RESET) << formatMotorPorts(r.motorsStalled)
       << "\n";
  } else {
    ss << "Stalled motors: none\n";
  }

  ss << "-------| " << name << " |-------";
  return ss.str();
}

std::string formatMotorPorts(const std::vector<std::int8_t> &ports) {
  if (ports.empty())
    return "None";

  std::stringstream ss;
  for (size_t i = 0; i < ports.size(); i++) {
    ss << (int)ports[i];
    if (i + 1 < ports.size())
      ss << ", ";
  }
  return ss.str();
}

} // namespace motorChecks
} // namespace sfx