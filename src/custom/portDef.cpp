#include "lemlib/chassis/chassis.hpp"
#include "lemlib/chassis/trackingWheel.hpp"
#include "main.h"

using namespace pros;

float motorPorts[9][3] = {{-8, 0, 0}, {-7, 0, 0}, {-19, 0, 0},
                          {9, 0, 0},  {17, 0, 0}, {3, 0, 0},
                          {11, 0, 0}, {12, 0, 0}, {6, 0, 0}};

MotorGroup aleft({static_cast<signed char>(motorPorts[0][0]),
                  static_cast<signed char>(motorPorts[1][0]),
                  static_cast<signed char>(motorPorts[2][0])},
                 MotorGearset::blue, v5::MotorUnits::degrees);
MotorGroup aright({static_cast<signed char>(motorPorts[3][0]),
                   static_cast<signed char>(motorPorts[4][0]),
                   static_cast<signed char>(motorPorts[5][0])},
                  MotorGearset::blue, v5::MotorUnits::degrees);

Motor intake(static_cast<signed char>(motorPorts[6][0]));
Motor topOut(static_cast<signed char>(motorPorts[7][0]));
Motor sorter(static_cast<signed char>(motorPorts[8][0]));

adi::Pneumatics match('a', false);
adi::Pneumatics arm('h', false);

Distance Dleft(0);
Distance Dright(0);
Distance Dfront(0);
Distance DbackR(0);
Distance DbackL(0);

Rotation vertRotation(-5);

v5::Optical colorSensorMatch(0);
v5::Optical colorSensorScore(0);

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
Controller userInput(E_CONTROLLER_MASTER);

void avgIMU() {
  while (true) {
    inertial1.set_heading((inertial1.get_heading() + inertial2.get_heading()) /
                          2);
    delay(5);
  }
}