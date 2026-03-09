#include "lemlib/chassis/chassis.hpp"
#include "lemlib/chassis/trackingWheel.hpp"
#include "main.h"

using namespace pros;

float motorPorts[8][3] = {{-10, 0, 0}, {-9, 0, 0}, {-8, 0, 0}, {1, 0, 0},
                          {2, 0, 0},   {3, 0, 0},  {7, 0, 0},  {-4, 0, 0}};

MotorGroup aleft({static_cast<signed char>(motorPorts[0][0]),
                  static_cast<signed char>(motorPorts[1][0]),
                  static_cast<signed char>(motorPorts[2][0])},
                 MotorGearset::blue, v5::MotorUnits::degrees);
MotorGroup aright({static_cast<signed char>(motorPorts[3][0]),
                   static_cast<signed char>(motorPorts[4][0]),
                   static_cast<signed char>(motorPorts[5][0])},
                  MotorGearset::blue, v5::MotorUnits::degrees);
pros::MotorGroup middle({static_cast<signed char>(motorPorts[6][0]),
                         static_cast<signed char>(motorPorts[7][0])});

adi::Pneumatics match('a', false);
adi::Pneumatics arm('h', false);
pros::adi::Pneumatics tripstate('b', false);
pros::adi::Pneumatics tripstate2('e', false);

Distance Dleft(0);
Distance Dright(0);
Distance Dfront(0);
Distance DbackR(0);
Distance DbackL(0);

Rotation vertRotation(5);

v5::Optical colorSensorMatch(0);
v5::Optical colorSensorScore(0);

lemlib::Drivetrain DT(&aleft, &aright, 12.72, lemlib::Omniwheel::NEW_325, 450,
                      8);

IMU inertial1(6);

lemlib::TrackingWheel leftVert(&vertRotation, lemlib::Omniwheel::NEW_2, 0.0, 1);

lemlib::OdomSensors sensors(&leftVert, nullptr, nullptr, nullptr, &inertial1);
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