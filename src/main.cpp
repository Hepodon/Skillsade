#include "main.h"
#include "RclTracking.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/chassis/trackingWheel.hpp"
#include "liblvgl/core/lv_obj.h"
#include "liblvgl/display/lv_display.h"
#include "liblvgl/misc/lv_types.h"
#include "pros/abstract_motor.hpp"
#include "pros/adi.h"
#include "pros/adi.hpp"
#include "pros/distance.hpp"
#include "pros/misc.h"
#include "pros/motor_group.hpp"
#include "pros/motors.h"
#include "pros/motors.hpp"
#include "pros/optical.hpp"
#include <cmath>
#include <cstdio>

using namespace pros;
using namespace std;

float motorPorts[9][3] = {{-8, 0, 0}, {7, 0, 0},   {-19, 0, 0},
                          {9, 0, 0},  {-17, 0, 0}, {3, 0, 0},
                          {18, 0, 0}, {12, 0, 0},  {6, 0, 0}};

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

Distance rightFront(0);
Distance rightBack(0);
Distance leftFront(0);
Distance leftBack(0);
Distance front(0);
Distance matchDist(0);

RclSensor F(&front, 0, 20, 0, 1);
RclSensor LB(&leftBack, 0, 20, 0, 1);
RclSensor LF(&leftFront, 0, 20, 0, 1);
RclSensor RF(&rightFront, 0, 20, 0, 1);
RclSensor RB(&rightBack, 0, 20, 0, 1);

Rotation vertRotation(-5);

Optical colorSensorMatch();
Optical colorSensorScore();

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

RclTracking rcl(&chassis);

Controller userInput(E_CONTROLLER_MASTER);

void avgIMU() {
  while (true) {
    inertial1.set_heading((inertial1.get_heading() + inertial2.get_heading()) /
                          2);
    delay(5);
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

enum teamColor { red, blue };
enum where { top, middle, middleSlow };

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
  void cancel() {
    topOut.move(0);
    sorter.move(0);
    intake.move(0);
  }

  void intakeUntilOppCol(teamColor color, int timeout) {
    store();
    int time = 0;
    if (color == red) {
      while ((colorSensorMatch().get_hue() <= 15 ||
              colorSensorMatch().get_hue() >= 345) &&
             time < timeout) {
        delay(10);
        time += 10;
      }
    } else {
      while ((colorSensorMatch().get_hue() <= 240 ||
              colorSensorMatch().get_hue() >= 200) &&
             time < timeout) {
        delay(10);
        time += 10;
      }
    }
  }

  void scoreUntilOppCol(teamColor color, int timeout, where where = top) {
    if (where == top) {
      loadTop();
    } else if (where == middle) {
      loadMiddle();
    } else {
      loadMiddleSLOW();
    }
    int time = 0;
    if (color == red) {
      while ((colorSensorMatch().get_hue() <= 15 ||
              colorSensorMatch().get_hue() >= 345) &&
             time < timeout) {
        delay(10);
        time += 10;
      }
    } else {
      while ((colorSensorMatch().get_hue() <= 240 ||
              colorSensorMatch().get_hue() >= 200) &&
             time < timeout) {
        delay(10);
        time += 10;
      }
    }
  }
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
    } else if (userInput.get_digital(DIGITAL_X)) {
      balls.loadMiddleSLOW();
    } else {
      balls.cancel();
    }
    delay(20);
  }
}

void screenInit() {}

void diagnosticsUpdater() {
  while (true) {
    for (int i = 0; i < 9; i++) {
      motorPorts[i][1] = Motor(i).get_temperature();
      motorPorts[i][2] = Motor(i).get_position();
    }
  }
}

enum auton { Left, Right, rSolo, skills };

auton selected;

void rightAuton(lv_event_t *e) { selected = Right; }

void initialize() {

  lv_init();
  ////////////////////////

  lv_obj_t *autonScreen = lv_obj_create(NULL);
  lv_obj_t *trackingScreen = lv_obj_create(NULL);
  lv_obj_t *diagScreen = lv_obj_create(NULL);
  lv_obj_t *uiScreen = lv_obj_create(NULL);

  lv_obj_t *label = uiScreen;
  lv_obj_t *title = uiScreen;

  title = lv_label_create(uiScreen);
  lv_label_set_text(title, "VEX VITALS");
  lv_obj_set_style_text_font(title, &lv_font_montserrat_30, 0);
  lv_obj_center(title);

  lv_obj_t *STARTButton = lv_obj_create(uiScreen);

  lv_obj_set_size(STARTButton, 200, 50);

  lv_obj_align_to(STARTButton, NULL, LV_ALIGN_CENTER, 0, 100);

  lv_obj_set_style_radius(STARTButton, 20, 0);

  lv_obj_add_event_cb(STARTButton, rightAuton, LV_EVENT_RELEASED, NULL);

  lv_obj_set_style_bg_color(STARTButton, lv_palette_main(LV_PALETTE_GREEN),
                            LV_STATE_DEFAULT);

  lv_obj_set_style_bg_color(STARTButton, lv_palette_darken(LV_PALETTE_GREEN, 3),
                            LV_STATE_PRESSED);

  ////////////////////////
  chassis.calibrate();
  chassis.setBrakeMode(E_MOTOR_BRAKE_HOLD);
  Task averagingTask(avgIMU);
  lv_screen_load(uiScreen);
}

void disabled() {}

void competition_initialize() {}

void autonomous() {
  rcl.startTracking();
  colorSensorMatch().set_led_pwm(100);
  colorSensorScore().set_led_pwm(100);

  switch (selected) {
  case Left:
    break;
  case Right:
    break;
  case rSolo:
    break;
  case skills:
    break;
  }
}

void opcontrol() {
  // Ball management task
  Task scoreTask(scoring);
  while (true) {
    // Collect and store controller input
    int leftY = userInput.get_analog(E_CONTROLLER_ANALOG_LEFT_Y);
    int rightX = userInput.get_analog(E_CONTROLLER_ANALOG_RIGHT_X);

    // Apply controller input for movement using split arcade controls
    chassis.arcade(leftY, rightX, true, 0.40);

    delay(10);
  }
}