#pragma once

#include "Shared/MotorGroup.hpp"       // IWYU pragma: keep
#include "Shared/Chassis.hpp"     // IWYU pragma: keep
#include "Shared/Motor.hpp"       // IWYU pragma: keep
#include "logger.hpp"       // IWYU pragma: keep
#include "motorChecks.hpp" // IWYU pragma: keep
#include "screen.hpp"      // IWYU pragma: keep

// Shorten sfx::Logger::LogLevel::Level into LogLevel::Level
using LogLevel = sfx::Logger::LogLevel;

// Shorten sfx::screen::Manager into screen::Manager
namespace screen = sfx::screen;

// Shorten sfx::screen::ButtonID into ButtonID
using ButtonId = sfx::screen::ButtonId;

// Shorten sfx::motorChecks
namespace motorChecks = sfx::motorChecks;
