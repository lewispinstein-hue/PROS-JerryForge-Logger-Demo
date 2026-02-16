#pragma once
/** 
 * @file Chassis.hpp
 * @brief Shared-ownership wrapper for lemlib::Chassis.
 * Use this in place of lemlib::Chassis to ensure shared ownership.
 * This wrapper preserves the lemlib::Chassis interface via v().
 * Intended to standardize chassis ownership across the project.
 * Prefer sfx::Chassis for all user-facing chassis instances.
*/

#include "lemlib/chassis/chassis.hpp"
#include <memory>
#include <utility>

namespace sfx {

/** 
 * @class Chassis
 * @brief Wrapper around lemlib::Chassis with shared ownership.
 * Holds a std::shared_ptr to the underlying lemlib::Chassis.
 * Use this instead of lemlib::Chassis directly in user code.
 * Access the full lemlib::Chassis API through v().
*/
class Chassis {
public:
  using LemChassis = lemlib::Chassis;

  Chassis() = default;

  Chassis(lemlib::Drivetrain drivetrain,
          lemlib::ControllerSettings linearSettings,
          lemlib::ControllerSettings angularSettings,
          lemlib::OdomSensors sensors,
          lemlib::DriveCurve* throttleCurve = &lemlib::defaultDriveCurve,
          lemlib::DriveCurve* steerCurve = &lemlib::defaultDriveCurve)
      : chassis_(std::make_shared<LemChassis>(
            drivetrain, linearSettings, angularSettings, sensors, throttleCurve,
            steerCurve)) {}

  explicit Chassis(std::shared_ptr<LemChassis> chassis)
      : chassis_(std::move(chassis)) {}

  /**
   * @brief Get the shared_ptr to the underlying lemlib::Chassis.
   * Example:
   * @code
   * sfx::Chassis c(drivetrain, linear, angular, sensors);
   * auto ptr = c.shared();
   * @endcode
  */
  std::shared_ptr<LemChassis> shared() const { return chassis_; }
  LemChassis* get() const { return chassis_.get(); }

  /**
   * @brief Access the underlying lemlib::Chassis; same functionality as lemlib::Chassis.
   * Example:
   * @code
   * sfx::Chassis c(drivetrain, linear, angular, sensors);
   * c.v().calibrate();
   * @endcode
  */
  LemChassis& v() const { return *chassis_; }
  LemChassis& operator*() const { return *chassis_; }
  LemChassis* operator->() const { return chassis_.get(); }

private:
  std::shared_ptr<LemChassis> chassis_ = nullptr;
};

} // namespace sfx
