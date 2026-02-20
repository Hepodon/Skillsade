#include "main.h"
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/chassis/trackingWheel.hpp"
#include "liblvgl/core/lv_obj.h"
#include "liblvgl/core/lv_obj_pos.h"
#include "liblvgl/core/lv_obj_scroll.h"
#include "liblvgl/core/lv_obj_style.h"
#include "liblvgl/display/lv_display.h"
#include "liblvgl/misc/lv_area.h"
#include "liblvgl/misc/lv_color.h"
#include "pros/abstract_motor.hpp"
#include "pros/adi.hpp"
#include "pros/distance.hpp"
#include "pros/misc.h"
#include "pros/motor_group.hpp"
#include "pros/motors.h"
#include "pros/optical.hpp"
#include <cmath>
#include <cstdio>

using namespace pros;
using namespace std;

MotorGroup aleft({-8, -7, 19}, pros::MotorGearset::blue,
                 pros::v5::MotorUnits::degrees);
MotorGroup aright({9, 17, -3}, pros::MotorGearset::blue,
                  pros::v5::MotorUnits::degrees);
Motor intake(18);
Motor topOut(6);
Motor sorter(12);

adi::Pneumatics match('a', false);
adi::Pneumatics arm('h', false);

Distance rightFront(0);
Distance rightBack(0);
Distance leftFront(0);
Distance leftBack(0);
Distance front(0);
Distance matchDist(0);

Rotation vertRotation(-5);

Optical colorSensor();

lemlib::Drivetrain DT(&aleft, &aright, 12.72, lemlib::Omniwheel::NEW_325, 450,
                      8);

IMU inertial1(4);
IMU inertial2(-11);

lemlib::TrackingWheel leftVert(&vertRotation, lemlib::Omniwheel::NEW_2, 0.0, 1);

lemlib::OdomSensors sensors(&leftVert, nullptr, nullptr, nullptr, &inertial1);

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

lemlib::Chassis chassis(DT, lateral_controller, angular_controller, sensors);

pros::Controller userInput(pros::E_CONTROLLER_MASTER);

void avgIMU() {
  while (true) {
    inertial1.set_heading((inertial1.get_heading() + inertial2.get_heading()) /
                          2);
    pros::delay(5);
  }
}

bool leftAuton = false;

void setPoseFromAllSensors() {
  const double FIELD_W = 3658.0; // mm
  const double FIELD_H = 3658.0; // mm

  // Distance from center of bot to sensors
  const int SIDE_OFFSET = 90.0;
  const int FRONT_OFFSET = 85.0;

  // Distance between front and back sensors
  const int sensorSpacing = 180.0;

  // Read sensors
  double lf = leftFront.get();
  double lb = leftBack.get();
  double rf = rightFront.get();
  double rb = rightBack.get();
  double f = front.get();

  // Angle from walls
  double thetaLeft = atan2(lf - lb, sensorSpacing) * 180 / M_PI;
  double thetaRight = atan2(rf - rb, sensorSpacing) * 180 / M_PI;

  double theta = leftAuton ? thetaLeft : thetaRight;

  // Heading
  double imuHeading = inertial1.get_heading();
  double heading = imuHeading - theta;

  // X position
  double xLeft = (lf + lb) / 2 + SIDE_OFFSET;
  double xRight = FIELD_W - ((rf + rb) / 2 + SIDE_OFFSET);
  double x = (xLeft + xRight) / 2;

  // Y position
  double y = FIELD_H - (f + FRONT_OFFSET);

  // Convert to inches
  chassis.setPose(x / 25.4, y / 25.4, heading);
}

class score_State {
public:
  // Different scoring and intake states
  score_State() {};
  // Intake and store blocks
  void store() {
    topOut.move(127);
    sorter.move(-127);
    intake.move(-127);
  }

  // Score blocks on long goals
  void loadTop() {
    topOut.move(-127);
    sorter.move(-127);
    intake.move(-127);
  }

  // Score blocks on central upper goal
  void loadMiddle() {
    topOut.move(127);
    sorter.move(127);
    intake.move(-127);
  }

  // Score blocks on central upper goal
  void loadMiddleSLOW() {
    topOut.move(80);
    sorter.move(80);
    intake.move(-80);
  }

  // score blocks on central lower goal
  void loadBottom() {
    topOut.move(127);
    sorter.move(127);
    intake.move(127);
  }
  // Stop motors
  void cancel() {}
};

score_State balls;

void scoring() {
  while (true) {

    if (userInput.get_digital(DIGITAL_R2)) {
      balls.store();
    } else if (userInput.get_digital(DIGITAL_R1)) {
      balls.loadTop();
    } else if (userInput.get_digital(DIGITAL_B)) {
      balls.loadMiddle();
    } else if (userInput.get_digital(DIGITAL_A)) {
      balls.loadBottom();
    }
    pros::delay(20);
  }
}

void screenInit() {}

void initialize() {
  lv_init();
  chassis.calibrate();
  setPoseFromAllSensors();
  chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
  pros::Task averagingTask(avgIMU);
  screenInit();
}

void disabled() {}

void competition_initialize() {}

void autonomous() {
  while (true) {
    setPoseFromAllSensors();
    pros::delay(20);
  }
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