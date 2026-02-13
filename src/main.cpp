#include "main.h"
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/chassis/trackingWheel.hpp"
#include "liblvgl/lvgl.h"
#include "pros/abstract_motor.hpp"
#include "pros/adi.hpp"
#include "pros/misc.h"
#include "pros/motor_group.hpp"
#include "pros/motors.h"

pros::MotorGroup aleft({9, -2, -4}, pros::MotorGearset::blue,
                       pros::v5::MotorUnits::degrees);
pros::MotorGroup aright({-19, 13, 14}, pros::MotorGearset::blue,
                        pros::v5::MotorUnits::degrees);

pros::Motor intake(-18);
pros::Motor middle(-10);
pros::Motor top(16);

pros::adi::Pneumatics match('a', false);
pros::adi::Pneumatics arm('b', false);
pros::adi::Pneumatics middlePneu('c', false);

lemlib::Drivetrain DT(&aleft, &aright, 12.72, lemlib::Omniwheel::NEW_325, 480,
                      8);

pros::IMU inertial1(20);

lemlib::OdomSensors sensors(nullptr, nullptr, nullptr, nullptr, &inertial1);

int startTime;

// lateral PID controller
lemlib::ControllerSettings
    lateral_controller(6,   // proportional gain (kP)
                       0,   // integral gain (kI)
                       5,   // derivative gain (kD)
                       3,   // anti windup
                       1,   // small error range, in inches
                       100, // small error range timeout, in milliseconds
                       3,   // large error range, in inches
                       500, // large error range timeout, in milliseconds
                       8    // maximum acceleration (slew)
    );

// angular PID controller
lemlib::ControllerSettings
    angular_controller(4.2, // proportional gain (kP)
                       0,   // integral gain (kI)
                       38,  // derivative gain (kD)
                       3,   // anti windup
                       1,   // small error range, in degrees
                       100, // small error range timeout, in milliseconds
                       3,   // large error range, in degrees
                       500, // large error range timeout, in milliseconds
                       6    // maximum acceleration (slew)
    );

lemlib::Chassis chassis(DT, lateral_controller, angular_controller, sensors);

pros::Controller userInput(pros::E_CONTROLLER_MASTER);

enum class autonPose { LEFT, RIGHT, SOLO };

autonPose selectedAuton;

float mm_In(float mm) { return mm / 25.4; }
class score_State {
public:
  // ** Different scoring and intake states
  score_State() {}; // Intake and store blocks
  void store() {
    middle.move(-127);
    top.move(-127);
  }

  // Score blocks on long goals
  void loadTop() {
    if (!middlePneu.is_extended()) {
      middlePneu.extend();
    }
    middle.move(-127);
    top.move(127);
  }

  // Score blocks on central upper goal
  void loadMiddle() {
    if (middlePneu.is_extended()) {
      middlePneu.retract();
    }
    middle.move(-127);
    top.move(60);
  }

  // score blocks on central lower goal
  void loadBottom() {
    if (!middlePneu.is_extended()) {
      middlePneu.extend();
    }
    middle.move(127);
    top.move(-127);
  }
  // Stop motors
  void cancel() {
    if (!middlePneu.is_extended()) {
      middlePneu.extend();
    }
    middle.brake();
    top.brake();
  }
};

score_State Score_State;

void scoring() {
  while (true) {
    // Scoring and intake control
    if (userInput.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
      Score_State.store();
    } else if (userInput.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
      Score_State.loadTop();
    } else if (userInput.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)) {
      Score_State.loadBottom();
    } else if (userInput.get_digital(pros::E_CONTROLLER_DIGITAL_B)) {
      Score_State.loadMiddle();
    } else {
      Score_State.cancel();
    }

    // Pneumatics toggle
    if (userInput.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2)) {
      match.toggle();
    }
    if (userInput.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1)) {
      arm.toggle();
    }
    pros::delay(20);
  }
}

class leftAuton {
public:
  leftAuton();

  void alignLowerLeft() {
    chassis.moveToPose(-3, 29, 1000, -90, {.horizontalDrift = 8, .lead = 0.3});
    chassis.moveToPose(
        18, 29, -90, 1500,
        {.forwards = false, .horizontalDrift = 8, .lead = 0.2, .maxSpeed = 50});
    chassis.setPose(0, 0, 0);
  }
  void matchloadLowerLeft() {
    chassis.moveToPose(0, 24, 0, 1000);
    // chassis.waitUntil(3);
    // Score_State.store();
    // pros::delay(500);
    // chassis.moveToPoint(0, 20, 1000, {.forwards = false});
  }
};

void leftAutonInit() { selectedAuton = autonPose::LEFT; }

void initialize() {
  chassis.calibrate(); // calibrate sensors
  chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
  // lv_init();
  selectedAuton = autonPose::LEFT;
}

void disabled() {}

void competition_initialize() {}

void autonomous() {
  chassis.setPose(0, 0, 0);
  match.extend();
  arm.extend();
}

void opcontrol() {
  // Ball management task
  pros::Task scoreTask(scoring);
  while (true) {
    // Collect and store controller input
    int leftY = userInput.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
    int rightX = userInput.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

    // Apply controller input for movement using split arcade controls
    chassis.arcade(leftY, rightX, true, 0.40);

    pros::delay(10);
  }
}