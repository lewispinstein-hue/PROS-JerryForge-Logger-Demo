#pragma once
/** 
 * @file MotorGroup.hpp
 * @brief Shared-ownership wrapper for pros::MotorGroup.
 * Use this in place of pros::MotorGroup to ensure shared ownership.
 * This wrapper preserves the pros::MotorGroup interface via v().
 * Intended to standardize motor group ownership across the project.
 * Prefer sfx::MotorGroup for all user-facing motor groups.
*/

#include "pros/abstract_motor.hpp"
#include "pros/motor_group.hpp"
#include <cstdint>
#include <vector>
#include <memory>
#include <utility>

namespace sfx {

/**
 * @class MotorGroup
 * @brief Wrapper around pros::MotorGroup with shared ownership.
 * Holds a std::shared_ptr to the underlying pros::MotorGroup.
 * Use this instead of pros::MotorGroup directly in user code.
 * Access the full pros::MotorGroup API through v().
*/
class MotorGroup {
public:
  using ProsMotorGroup = pros::MotorGroup;

  MotorGroup() = default;

  MotorGroup(const std::initializer_list<std::int8_t> ports,
             const pros::v5::MotorGears gearset = pros::v5::MotorGears::invalid,
             const pros::v5::MotorUnits encoder_units = pros::v5::MotorUnits::invalid)
      : group_(std::make_shared<ProsMotorGroup>(ports, gearset, encoder_units)) {}

  MotorGroup(const std::vector<std::int8_t>& ports,
             const pros::v5::MotorGears gearset = pros::v5::MotorGears::invalid,
             const pros::v5::MotorUnits encoder_units = pros::v5::MotorUnits::invalid)
      : group_(std::make_shared<ProsMotorGroup>(ports, gearset, encoder_units)) {}

  explicit MotorGroup(pros::AbstractMotor& motor_group)
      : group_(std::make_shared<ProsMotorGroup>(motor_group)) {}

  explicit MotorGroup(std::shared_ptr<ProsMotorGroup> group)
      : group_(std::move(group)) {}

  /**
   * @brief Get the shared_ptr to the underlying pros::MotorGroup.
   * Example:
   * @code
   * sfx::MotorGroup mg({1, -2, 3});
   * auto ptr = mg.shared();
   * @endcode
  */
  std::shared_ptr<ProsMotorGroup> shared() const { return group_; }
  ProsMotorGroup* get() const { return group_.get(); }

  /**
   * @brief Access the underlying pros::MotorGroup; same functionality as pros::MotorGroup.
   * Example:
   * @code
   * sfx::MotorGroup mg({1, -2, 3});
   * mg.v().move(100);
   * @endcode
   */
  ProsMotorGroup& v() const { return *group_; }
  ProsMotorGroup& operator*() const { return *group_; }
  ProsMotorGroup* operator->() const { return group_.get(); }

private:
  std::shared_ptr<ProsMotorGroup> group_ = nullptr;
};

} // namespace sfx
