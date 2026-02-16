#pragma once
/**  
 * @file Motor.hpp
 * @brief Shared-ownership wrapper for pros::Motor.
 * Use this in place of pros::Motor to ensure shared ownership.
 * This wrapper preserves the pros::Motor interface via v().
 * Intended to standardize motor ownership across the project.
 * Prefer sfx::Motor for all user-facing motor instances.
*/

#include "pros/device.hpp"
#include "pros/motors.hpp"
#include <cstdint>
#include <memory>
#include <utility>

namespace sfx {

/** 
 * @class Motor
 * @brief Wrapper around pros::Motor with shared ownership.
 * Holds a std::shared_ptr to the underlying pros::Motor.
 * Use this instead of pros::Motor directly in user code.
 * Access the full pros::Motor API through v().
*/
class Motor {
public:
  using ProsMotor = pros::Motor;

  Motor() = default;

  Motor(const std::int8_t port,
        const pros::v5::MotorGears gearset = pros::v5::MotorGears::invalid,
        const pros::v5::MotorUnits encoder_units = pros::v5::MotorUnits::invalid)
      : motor_(std::make_shared<ProsMotor>(port, gearset, encoder_units)) {}

  Motor(const pros::Device& device) : Motor(device.get_port()) {}

  explicit Motor(std::shared_ptr<ProsMotor> motor)
      : motor_(std::move(motor)) {}
  
  /**
   * @brief Get the shared_ptr to the underlying pros::Motor.
   * Example:
   * @code
   * sfx::Motor m(1);
   * auto ptr = m.shared();
   * @endcode
  */
  std::shared_ptr<ProsMotor> shared() const { return motor_; }
  ProsMotor* get() const { return motor_.get(); }

  /** 
   * @brief Access the underlying pros::Motor; same functionality as pros::Motor.
   * Example:
   * @code
   * sfx::Motor m(1);
   * m.v().move(100);
   * @endcode
  */
  ProsMotor& v() const { return *motor_; }
  ProsMotor& operator*() const { return *motor_; }
  ProsMotor* operator->() const { return motor_.get(); }

private:
  std::shared_ptr<ProsMotor> motor_ = nullptr;
};

} // namespace sfx
