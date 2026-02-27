#include "main.h"
#include "RclTracking.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/chassis/trackingWheel.hpp"
#include "liblvgl/core/lv_obj.h"
#include "liblvgl/core/lv_obj_pos.h"
#include "liblvgl/core/lv_obj_scroll.h"
#include "liblvgl/core/lv_obj_tree.h"
#include "liblvgl/display/lv_display.h"
#include "liblvgl/misc/lv_area.h"
#include "liblvgl/misc/lv_types.h"
#include "pros/abstract_motor.hpp"
#include "pros/adi.hpp"
#include "pros/distance.hpp"
#include "pros/misc.h"
#include "pros/motor_group.hpp"
#include "pros/motors.h"
#include "pros/motors.hpp"
#include "pros/optical.hpp"
#include <cmath>
#include <cstdio>
#include <thread>

using namespace pros;
using namespace std;

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

Distance leftD(4);
Distance rightD(21);
Distance leftFront(15);
Distance rightFront(5);
Distance matchDist(20);

Rotation vertRotation(10);
bool done = true;

v5::Optical colorSensorMatch(13);
v5::Optical colorSensorScore(14);

lemlib::Drivetrain DT(&aleft, &aright, 12.72, lemlib::Omniwheel::NEW_325, 450,
                      8);

IMU inertial1(2);
IMU inertial2(16);

lemlib::TrackingWheel leftVert(&vertRotation, lemlib::Omniwheel::NEW_2, 0.0, 1);

lemlib::OdomSensors sensors(&leftVert, nullptr, nullptr, nullptr, &inertial1);

// lateral PID controller
lemlib::ControllerSettings
    lateral_controller(4.925, // proportional gain (kP)
                       0,     // integral gain (kI)
                       4,     // derivative gain (kD)
                       3,     // anti windup
                       1,     // small error range, in inches
                       100,   // small error range timeout, in milliseconds
                       3,     // large error range, in inches
                       300,   // large error range timeout, in milliseconds
                       20     // maximum acceleration (slew)
    );

// angular PID controller
lemlib::ControllerSettings
    angular_controller(4.49, // proportional gain (kP)
                       0,    // integral gain (kI)
                       32.5, // derivative gain (kD)
                       3,    // anti windup
                       1,    // small error range, in degrees
                       100,  // small error range timeout, in milliseconds
                       2,    // large error range, in degrees
                       300,  // large error range timeout, in milliseconds
                       0     // maximum acceleration (slew)
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

bool leftAuton = false;
void setPoseFromAllSensors() {
  const double FIELD_W = 3658.0; // mm
  const double FIELD_H = 3658.0; // mm

  const double SIDE_OFFSET = 90.0;
  const double FRONT_OFFSET = 85.0;
  const double sensorSpacing = 146.0;

  double lf = leftFront.get();
  double rf = rightFront.get();

  // Angle relative to wall
  double theta = atan2(lf - rf, sensorSpacing) * 180.0 / M_PI;

  // Correct heading using inertial
  double correctedHeading = inertial1.get_heading() - theta;

  // Position from front wall
  double avgFront = (lf + rf) / 2.0;
  double y = FIELD_H - (avgFront + FRONT_OFFSET);

  // X would require side sensors
  double x = rightD.get() *
             cos(correctedHeading); // Placeholder until side sensors used
  x /= 25.4;
  y /= 25.4;

  chassis.setPose(x, y, correctedHeading);
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

  void loadTopSlow() {
    topOut.move(-100);
    sorter.move(-100);
    intake.move(-100);
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
    done = false;
    int time = 0;
    if (color == red) {
      while ((colorSensorMatch.get_hue() <= 70 ||
              colorSensorMatch.get_hue() >= 290) &&
             time < timeout) {
        delay(100);
        time += 100;
      }
    } else {
      while ((colorSensorMatch.get_hue() <= 240 ||
              colorSensorMatch.get_hue() >= 200) &&
             time < timeout) {
        delay(10);
        time += 10;
      }
    }
    done = true;
  }

  void scoreUntilOppCol(teamColor color, int timeout, where where = top) {
    if (where == top) {
      loadTopSlow();
    } else if (where == middle) {
      loadMiddle();
    } else {
      loadMiddleSLOW();
    }
    done = false;
    int time = 0;
    if (color == red) {
      while ((colorSensorMatch.get_hue() <= 100 ||
              colorSensorMatch.get_hue() >= 290) &&
             time < timeout) {
        delay(5);
        time += 5;
      }
      loadMiddle();
    } else {
      while ((colorSensorMatch.get_hue() <= 240 ||
              colorSensorMatch.get_hue() >= 200) &&
             time < timeout) {
        delay(10);
        time += 10;
      }
    }
  }
};

score_State balls;

bool inScore = false;
bool topScore = false;
bool midScore = false;
bool bottomScore = false;

void scoring() {
  while (true) {

    if (userInput.get_digital_new_press(DIGITAL_R2)) {
      if (!inScore) {
        inScore = true;
        topScore = false;
        midScore = false;
        bottomScore = false;
        balls.store();
      } else {
        balls.cancel();
        inScore = false;
        topScore = false;
        midScore = false;
        bottomScore = false;
      }
    } else if (userInput.get_digital_new_press(DIGITAL_R1)) {
      if (!topScore) {
        inScore = false;
        topScore = true;
        midScore = false;
        bottomScore = false;
        balls.loadTop();
      } else {
        balls.cancel();
        inScore = false;
        topScore = false;
        midScore = false;
        bottomScore = false;
      }
    } else if (userInput.get_digital_new_press(DIGITAL_B)) {
      if (!midScore) {
        inScore = false;
        topScore = false;
        midScore = true;
        bottomScore = false;
        balls.loadMiddle();
      } else {
        balls.cancel();
        inScore = false;
        topScore = false;
        midScore = false;
        bottomScore = false;
      }
    } else if (userInput.get_digital_new_press(DIGITAL_A)) {
      if (!bottomScore) {
        inScore = false;
        topScore = false;
        midScore = false;
        bottomScore = true;
        balls.loadBottom();
      } else {
        balls.cancel();
        inScore = false;
        topScore = false;
        midScore = false;
        bottomScore = false;
      }
    } else if (userInput.get_digital_new_press(DIGITAL_X)) {
      if (!midScore) {
        inScore = false;
        topScore = false;
        midScore = true;
        bottomScore = false;
        balls.loadMiddleSLOW();
      } else {
        balls.cancel();
        inScore = false;
        topScore = false;
        midScore = false;
        bottomScore = false;
      }
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

auton selected = Right;
void rightAuton(lv_event_t *e) {
  selected = Right;
  lv_obj_clean(lv_screen_active());
}

void createLvglButton(lv_obj_t *button, const char *text,
                      lv_event_cb_t event_cb, int width, int height,
                      lv_obj_t *base, lv_align_t align, int x_ofs, int y_ofs,
                      lv_palette_t color = LV_PALETTE_BLUE) {
  lv_obj_set_size(button, width, height);
  lv_obj_align_to(button, base, align, x_ofs, y_ofs);
  lv_obj_set_style_radius(button, 20, 0);
  lv_obj_set_style_bg_color(button, lv_palette_main(color), LV_STATE_DEFAULT);
  lv_obj_set_style_bg_color(button, lv_palette_darken(color, 3),
                            LV_STATE_PRESSED);

  lv_obj_t *label = lv_label_create(button);
  lv_label_set_text(label, text);
  lv_obj_align_to(label, button, LV_ALIGN_CENTER, 0, 0);

  lv_obj_add_event_cb(button, event_cb, LV_EVENT_RELEASED, NULL);

  lv_obj_set_scrollbar_mode(button, LV_SCROLLBAR_MODE_OFF);
  lv_obj_set_scrollbar_mode(label, LV_SCROLLBAR_MODE_OFF);

  // lv_obj_set_style_bg_opa(button, LV_OPA_TRANSP, 0);
  // lv_obj_set_style_bg_opa(label, LV_OPA_TRANSP, 0);
}

bool abc = true;

void initialize() {
  if (!abc) {

    lv_init();

    lv_obj_clean(lv_screen_active());
    ///////////////////////

    lv_obj_t *autonScreen = lv_obj_create(NULL);
    lv_obj_t *trackingScreen = lv_obj_create(NULL);
    lv_obj_t *diagScreen = lv_obj_create(NULL);
    lv_obj_t *uiScreen = lv_obj_create(NULL);

    lv_obj_set_scrollbar_mode(uiScreen, LV_SCROLLBAR_MODE_OFF);
    lv_obj_set_scrollbar_mode(autonScreen, LV_SCROLLBAR_MODE_OFF);
    lv_obj_set_scrollbar_mode(diagScreen, LV_SCROLLBAR_MODE_OFF);
    lv_obj_set_scrollbar_mode(trackingScreen, LV_SCROLLBAR_MODE_OFF);

    lv_obj_t *rightButton = lv_button_create(autonScreen);
    lv_obj_t *leftButton = lv_button_create(autonScreen);
    lv_obj_t *rSoloButton = lv_button_create(autonScreen);

    lv_obj_t *blueButton = lv_button_create(autonScreen);
    lv_obj_t *redButton = lv_button_create(autonScreen);

    lv_obj_set_size(autonScreen, 480, 272);

    lv_obj_set_style_bg_opa(rSoloButton, LV_OPA_TRANSP, 0);
    lv_obj_set_style_bg_opa(rightButton, LV_OPA_TRANSP, 0);
    lv_obj_set_style_bg_opa(leftButton, LV_OPA_TRANSP, 0);

    lv_obj_set_style_bg_opa(blueButton, LV_OPA_TRANSP, 0);
    lv_obj_set_style_bg_opa(redButton, LV_OPA_TRANSP, 0);

    lv_obj_align_to(rightButton, autonScreen, LV_ALIGN_CENTER, 0, 0);
    lv_obj_align_to(leftButton, autonScreen, LV_ALIGN_CENTER, 0, 60);
    lv_obj_align_to(rSoloButton, autonScreen, LV_ALIGN_CENTER, 0, -60);

    lv_obj_align_to(redButton, autonScreen, LV_ALIGN_LEFT_MID, 0, 0);
    lv_obj_align_to(blueButton, autonScreen, LV_ALIGN_RIGHT_MID, 0, 0);

    createLvglButton(leftButton, "Left", rightAuton, 350, 50, autonScreen,
                     LV_ALIGN_CENTER, 0, 0, LV_PALETTE_RED);

    createLvglButton(rightButton, "Right", rightAuton, 350, 50, autonScreen,
                     LV_ALIGN_CENTER, 0, 55, LV_PALETTE_RED);

    createLvglButton(rSoloButton, "Right Solo", rightAuton, 350, 50,
                     autonScreen, LV_ALIGN_CENTER, 0, -55, LV_PALETTE_RED);

    // createLvglButton(redButton, "Right Solo", rightAuton, 350, 50,
    // autonScreen,
    //                  LV_ALIGN_CENTER, 0, -55, LV_PALETTE_RED);
    // createLvglButton(rSoloButton, "Right Solo", rightAuton, 350, 50,
    // autonScreen,
    //                  LV_ALIGN_CENTER, 0, -55, LV_PALETTE_RED);

    lv_screen_load(autonScreen);
    ////////////////////////
  } else {
    pros::lcd::initialize(); // initialize brain screen
  }
  // Task imuTask(avgIMU);
  chassis.calibrate();
  colorSensorMatch.set_led_pwm(100);
  colorSensorScore.set_led_pwm(100);
  chassis.setBrakeMode(E_MOTOR_BRAKE_COAST);
}

void disabled() {}

void competition_initialize() {}

void autonomous() {
  match.extend();
  setPoseFromAllSensors();
  Task avgTask(avgIMU);
  colorSensorMatch.set_led_pwm(100);
  colorSensorScore.set_led_pwm(100);

  switch (selected) {
  case Left:
    break;
  case Right:
    while (true) {
      // print robot location to the brain screen
      pros::lcd::print(0, "X: %f    matchHue: %f", chassis.getPose().x,
                       colorSensorMatch.get_hue()); // x
      pros::lcd::print(1, "Y: %f    topHue: %f", chassis.getPose().y,
                       colorSensorScore.get_hue());              // y
      pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading

      pros::lcd::print(3, "left DS: %d", leftD.get());          // heading
      pros::lcd::print(4, "Right DS: %d", rightD.get());        // heading
      pros::lcd::print(5, "leftFront DS: %d", leftFront.get()); // heading
      pros::lcd::print(6, "rightFront: %d", rightFront.get());  // heading

      pros::lcd::print(7, "match DS: %d", matchDist.get()); // heading
      // delay to save resources
      pros::delay(100);
    }
    // chassis.turnToHeading(90, 1000);
    // chassis.turnToHeading(0, 1000);
    // chassis.turnToHeading(180, 1000);
    // chassis.waitUntilDone();
    // balls.store();

    break;
  case rSolo:
    break;
  case skills:
    break;
  }
}

void opcontrol() {
  // print position to brain screen
  pros::Task screen_task([&]() {
    while (true) {
      // print robot location to the brain screen
      pros::lcd::print(0, "X: %f    matchHue: %f", chassis.getPose().x,
                       colorSensorMatch.get_hue()); // x
      pros::lcd::print(1, "Y: %f    topHue: %f", chassis.getPose().y,
                       colorSensorScore.get_hue());              // y
      pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading

      pros::lcd::print(3, "left DS: %d", leftD.get());          // heading
      pros::lcd::print(4, "Right DS: %d", rightD.get());        // heading
      pros::lcd::print(5, "leftFront DS: %d", leftFront.get()); // heading
      pros::lcd::print(6, "rightFront: %d", rightFront.get());  // heading

      pros::lcd::print(7, "match DS: %d", matchDist.get()); // heading
      // delay to save resources
      pros::delay(100);
    }
  });
  // Ball management task
  Task scoreTask(scoring);
  while (true) {
    // Collect and store controller input
    int leftY = userInput.get_analog(E_CONTROLLER_ANALOG_LEFT_Y);
    int rightX = userInput.get_analog(E_CONTROLLER_ANALOG_RIGHT_X);

    // Apply controller input for movement using split arcade controls
    chassis.arcade(leftY, rightX, true, 0.40);

    if (userInput.get_digital_new_press(DIGITAL_L2)) {
      match.toggle();
    }
    if (userInput.get_digital_new_press(DIGITAL_L1)) {
      arm.toggle();
    }

    delay(10);
  }
}