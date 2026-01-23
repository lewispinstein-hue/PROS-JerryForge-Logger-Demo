#include "main.h"
// Get access to LemLib's headers
#include "lemlib/api.hpp"

// Get access to Sfx's headers
#include "lemlib/chassis/chassis.hpp"
#include "sfx/api.hpp"

// Creating motors and controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

pros::MotorGroup left_mg({1, -2, 3},
                 pros::MotorGearset::blue,
                 pros::v5::MotorUnits::degrees); // Creates a motor group with forwards ports 1 & 3 and reversed port 2

pros::MotorGroup right_mg({-4, 5, -6},
                 pros::MotorGearset::blue, 
                 pros::v5::MotorUnits::degrees); // Creates a motor group with forwards port 5 and reversed ports 4 & 6

// Setup LemLib 

// create an imu on port 10
pros::Imu imu(10);

// drivetrain settings
lemlib::Drivetrain drivetrain(&left_mg, // left motor group
                              &right_mg, // right motor group
                              10, // 10 inch track width
                              lemlib::Omniwheel::NEW_4, // using new 4" omnis
                              360, // drivetrain rpm is 360
                              2 // horizontal drift is 2 (for now)
);

// tracking wheels are the built in IME's in the motors
lemlib::TrackingWheel leftVerticalTrackingWheel(&left_mg,
                                                lemlib::Omniwheel::NEW_325, -10, 400);

lemlib::TrackingWheel rightVerticalTrackingWheel(&right_mg,
                                                 lemlib::Omniwheel::NEW_325,
                                                 10, 400);

// odometry settings
lemlib::OdomSensors sensors(&leftVerticalTrackingWheel, // vertical tracking wheel 1, set to null
                            &rightVerticalTrackingWheel, // vertical tracking wheel 2, set to nullptr as we are using IMEs
                            nullptr, // horizontal tracking wheel 1
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu // inertial sensor
);

// lateral PID controller
lemlib::ControllerSettings
    lateral_controller(10,  // proportional gain (kP)
                       0,   // integral gain (kI)
                       3,   // derivative gain (kD)
                       3,   // anti windup
                       1,   // small error range, in inches
                       100, // small error range timeout, in milliseconds
                       3,   // large error range, in inches
                       500, // large error range timeout, in milliseconds
                       20   // maximum acceleration (slew)
    );

// angular PID controller
lemlib::ControllerSettings
    angular_controller(2,   // proportional gain (kP)
                       0,   // integral gain (kI)
                       10,  // derivative gain (kD)
                       3,   // anti windup
                       1,   // small error range, in degrees
                       100, // small error range timeout, in milliseconds
                       3,   // large error range, in degrees
                       500, // large error range timeout, in milliseconds
                       0    // maximum acceleration (slew)
    );

// Finally, create the chassis
lemlib::Chassis chassis(drivetrain, // drivetrain settings
                        lateral_controller, // lateral PID settings
                        angular_controller, // angular PID settings
                        sensors // odometry sensors
);

/*
 * 3. SFX (LIBRARY) SETUP
 * ----------------------
 * Initialize the screen manager and logger.
 */
sfx::screen::Manager display;

void initialize() {
  // --- Logger Setup ---
  auto& logger = sfx::Logger::get_instance();
  // 1. Register motors you want to monitor
  logger.registerMotor("Left Drive", nullptr);
  logger.registerMotor("Right Drive", &right_mg);

  // 2. Pass the chassis and controllers to the logger so it can record them
  logger.setRobot({
    .chassis = nullptr,
    .Left_Drivetrain = SHARED(left_mg),
    .Right_Drivetrain = SHARED(right_mg)
  });

  // 3. Configure Logging Behaviors
  logger.setLogToTerminal(true);      // Print logs to the computer via USB
  logger.setLogToSD(false);           // Save logs to the SD card (Recommended!)
  logger.setPrintProsTasks(true);     // Log when PROS tasks start/stop
  logger.setOnlyPrintOverheatedMotors(false); // If true, hides cool motors
  logger.setPrintLemlibPose(false);
  logger.setLoggerMinLevel(LogLevel::LOG_LEVEL_INFO); // Hide "Debug" messages

  // 4. Calibrate & Start
  chassis.calibrate(); // Calibrate IMU
  logger.start(); // Start the background logging task

  // Print startup success to the brain screen
  display.clearScreen();
  display.printToScreen("System Ready!");
  display.printToScreen("Battery: {:.1f}%", pros::battery::get_capacity());
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */

/*
 * OPCONTROL
 * ---------
 * Demonstration of Screen, Buttons, and Drive Control.
 */
void opcontrol() {
  // -- Example: Autonomous Selector --
  display.clearScreen();
  display.printToScreen("Select Mode: (Tap a button below)");
  
  // Set button labels
  display.printToScreen("Left: Run a test movement");
  display.printToScreen("Middle: Run a test turn");
  display.printToScreen("Right: Check Motor Temps");
  display.drawBottomButtons(false); 

  // Reset Pose for the demo
  chassis.setPose(0, 0, 0);

  // Wait for user input
  sfx::screen::ButtonId button_pushed = display.waitForBottomButtonTap();

  // Handle the button press
  switch (button_pushed) {
    case ButtonId::LEFT:
      // Demo: Move forward
      display.printToScreen(true, "Running", "Move To Point");
      chassis.moveToPoint(0, 10, 1000);
      break;

    case ButtonId::MIDDLE:
      // Demo: Turn
      display.printToScreen(true, "Running", "Turn To Heading");
      chassis.turnToHeading(180, 1000);
      break;

    case ButtonId::RIGHT: {
      // Demo: Check Temperatures using SFX
      // 1. Get the overheat status
      auto status = sfx::motorChecks::checkMotorOverheat(left_mg);

      // 2. Format it into a nice string
      std::string output = sfx::motorChecks::formatCheckTemp("Left Drive", status, false);

      // 3. Print to screen
      display.printToScreen(true, "Results", "{}", output);
      break;
    }
    default:
      break;
  }
  
  display.printToScreen("Touch the screen to continue...");
  // Simple holder to constantly re-check for screen touch to continue
  while (!display.waitForScreenTouch(100)) pros::delay(10);

  // -- Main Drive Loop --
  display.printToScreen(true, "Driver Control", "Active");
  
  while (true) {
    LOG_INFO("AAA");
      // Get joystick values
      int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
      int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

      // Simple Deadband (prevents robot from creeping if joystick drifts)
      if (abs(leftY) < 10) leftY = 0;
      if (abs(rightX) < 10) rightX = 0;

      // Move the robot (Curvature Drive)
      chassis.curvature(leftY, rightX);

      // Save resources
      pros::delay(20);
  }
}