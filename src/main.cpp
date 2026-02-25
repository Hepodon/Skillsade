#include "main.h"
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/chassis/trackingWheel.hpp"
#include "pros/adi.hpp"
#include "pros/misc.h"

using namespace pros;

MotorGroup aleft({-10, -9, -8});
MotorGroup aright({1, 2, 3});

MotorGroup middle({7, -4});

adi::Pneumatics match('a', false);
adi::Pneumatics arm('h', false);
adi::Pneumatics tripstate('e', false);
adi::Pneumatics tripstate2('b', false);

Rotation vertRot(5);

lemlib::TrackingWheel vert(&vertRot, lemlib::Omniwheel::NEW_2, 0.75);

IMU imu(6);

lemlib::Drivetrain DT(&aleft, &aright, 4, 12, 12, 6);

lemlib::OdomSensors sensors(&vert, nullptr, nullptr, nullptr, &imu);

// lateral PID controller
lemlib::ControllerSettings
    lateral_controller(5,   // proportional gain (kP)
                       0,   // integral gain (kI)
                       5.5, // derivative gain (kD)
                       3,   // anti windup
                       0.5, // small error range, in inches
                       100, // small error range timeout, in milliseconds
                       1.5, // large error range, in inches
                       200, // large error range timeout, in milliseconds
                       20   // maximum acceleration (slew)
    );

// angular PID controller
lemlib::ControllerSettings
    angular_controller(4.45, // proportional gain (kP)
                       0,    // integral gain (kI)
                       31.5, // derivative gain (kD)
                       3,    // anti windup
                       1,    // small error range, in degrees
                       100,  // small error range timeout, in milliseconds
                       3,    // large error range, in degrees
                       500,  // large error range timeout, in milliseconds
                       0     // maximum acceleration (slew)
    );

lemlib::Chassis chassis(DT, lateral_controller, angular_controller, sensors);

Controller userInput(E_CONTROLLER_MASTER);

void initialize() { chassis.calibrate(); }

void disabled() {}

void competition_initialize() {}

bool in = false;
bool mid = false;
bool top = false;
bool out = false;

void scoreTop() {
  if (!top) {
    tripstate.extend();
    tripstate2.extend();
    middle.move(127);
    top = true;
    mid = false;
    out = false;
    in = false;
  } else {
    middle.brake();
    top = false;
    mid = false;
    out = false;
    in = false;
  }
}

void intake() {
  if (!in) {
    tripstate.extend();
    tripstate2.retract();
    middle.move(127);
    in = true;
    top = false;
    mid = false;
    out = false;
  } else {
    middle.brake();
    top = true;
    mid = false;
    out = false;
    in = false;
  }
}

void scoreMid() {
  if (!mid) {
    tripstate.retract();
    tripstate2.retract();
    middle.move(127);
    mid = true;
    top = false;
    out = false;
    in = false;
  } else {
    middle.brake();
    top = true;
    mid = false;
    out = false;
    in = false;
  }
}

void outtake() {
  out = true;
  top = false;
  mid = false;
  in = false;
  while (userInput.get_digital(DIGITAL_B)) {
    tripstate.extend();
    tripstate2.extend();
    middle.move(-127);
  }
  middle.brake();

  out = false;
  top = false;
  mid = false;
  in = false;
}

void autonomous() {
  chassis.moveToPoint(0, 48, 10000);
  chassis.waitUntilDone();
  middle.move(50);
}

void opcontrol() {
  while (true) {
    chassis.arcade(userInput.get_analog(E_CONTROLLER_ANALOG_LEFT_Y),
                   userInput.get_analog(E_CONTROLLER_ANALOG_RIGHT_X), true,
                   0.4);

    if (userInput.get_digital_new_press(DIGITAL_L1)) {
      arm.toggle();
    }
    if (userInput.get_digital_new_press(DIGITAL_L2)) {
      match.toggle();
    }

    if (userInput.get_digital_new_press(DIGITAL_R2)) {
      scoreTop();
    } else if (userInput.get_digital_new_press(DIGITAL_R1)) {
      intake();
    } else if (userInput.get_digital_new_press(DIGITAL_A)) {
      scoreMid();
    } else if (userInput.get_digital(DIGITAL_B)) {
      outtake();
    } else if (userInput.get_digital_new_press(DIGITAL_DOWN)) {
      tripstate.toggle();
    } else if (userInput.get_digital_new_press(DIGITAL_UP)) {
      tripstate2.toggle();
    }

    delay(5);
  }
}