#pragma once
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/chassis/trackingWheel.hpp"
#include "main.h"

extern float motorPorts[9][3];

extern pros::MotorGroup aleft;
extern pros::MotorGroup aright;

extern pros::Motor intake;
extern pros::Motor topOut;
extern pros::Motor sorter;

extern pros::adi::Pneumatics match;
extern pros::adi::Pneumatics arm;

extern pros::Distance Dleft;
extern pros::Distance Dright;
extern pros::Distance Dfront;
extern pros::Distance DbackR;
extern pros::Distance DbackL;

extern pros::Rotation vertRotation;

extern pros::v5::Optical colorSensorMatch;
extern pros::v5::Optical colorSensorScore;

extern lemlib::Drivetrain DT;

extern pros::IMU inertial1;
extern pros::IMU inertial2;

extern lemlib::TrackingWheel leftVert;

extern lemlib::OdomSensors sensors;

// lateral PID controller
extern lemlib::ControllerSettings lateral_controller;

// angular PID controller
extern lemlib::ControllerSettings angular_controller;

extern lemlib::Chassis chassis;

extern pros::Controller userInput;

extern void avgIMU();