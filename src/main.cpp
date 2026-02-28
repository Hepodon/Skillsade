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
    lateral_controller(4.93, // proportional gain (kP)
                       0,    // integral gain (kI)
                       4,    // derivative gain (kD)
                       3,    // anti windup
                       1,    // small error range, in inches
                       100,  // small error range timeout, in milliseconds
                       3,    // large error range, in inches
                       300,  // large error range timeout, in milliseconds
                       20    // maximum acceleration (slew)
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

const double FIELD_W = 3658.0; // mm
const double FIELD_H = 3658.0; // mm

void avgIMU() {
  while (true) {
    inertial1.set_heading((inertial1.get_heading() + inertial2.get_heading()) /
                          2);
    delay(5);
  }
}

constexpr double MM_TO_IN = 1.0 / 25.4;
constexpr double FIELD_W_MM = 3658.0;
constexpr double FIELD_H_MM = 3658.0;

double clamp(double v, double lo, double hi) { return fmax(lo, fmin(v, hi)); }

double confidenceFromDist(double mm) {
  if (mm < 20 || mm > 2000)
    return 0.0;
  if (mm < 60 || mm > 1500)
    return 0.4;
  return 1.0;
}

double confidenceFromAngle(double deg, double ideal, double maxErr = 15.0) {
  double err = fabs(deg - ideal);
  return clamp(1.0 - err / maxErr, 0.0, 1.0);
}

double blend(double current, double measured, double confidence) {
  return current + (measured - current) * confidence;
}

float updateThetaFromFrontAlways() {
  constexpr double SENSOR_SPACING_MM = 146.0;

  double lf = leftFront.get();
  double rf = rightFront.get();

  double thetaRad = atan2(lf - rf, SENSOR_SPACING_MM);
  double thetaDeg = thetaRad * 180.0 / M_PI;

  return thetaDeg;
}

void updateXFromRightAlways() {
  constexpr double SIDE_OFFSET_MM = 0.0;

  double rightDist = rightD.get();
  double distConfidence = confidenceFromDist(rightDist);

  auto pose = chassis.getPose();
  double thetaRad = pose.theta * M_PI / 180.0;

  double angleConfidence = clamp(cos(thetaRad), 0.0, 1.0);
  double confidence = distConfidence * angleConfidence;

  double projected = (rightDist + SIDE_OFFSET_MM) * fabs(cos(thetaRad));
  double measuredXmm = FIELD_W_MM - projected;
  double measuredXin = measuredXmm * MM_TO_IN;

  double blendedX = blend(pose.x, measuredXin, confidence);

  chassis.setPose(blendedX, pose.y, pose.theta);
}

void updateXFromLeftAlways() {
  constexpr double SIDE_OFFSET_MM = 0.0;

  double leftDist = leftD.get();

  auto pose = chassis.getPose();
  double thetaRad = pose.theta * M_PI / 180.0;

  double projected = (leftDist + SIDE_OFFSET_MM) * fabs(cos(thetaRad));
  double measuredXin = projected * MM_TO_IN;

  chassis.setPose(measuredXin, pose.y, pose.theta);
}

void updateYFromFrontAlways() {
  constexpr double FRONT_OFFSET_MM = 85.0;

  double lf = leftFront.get();
  double rf = rightFront.get();

  auto pose = chassis.getPose();
  double thetaRad = pose.theta * M_PI / 180.0;

  double avgFront = (lf + rf) / 2.0;
  double projected = (avgFront + FRONT_OFFSET_MM) * fabs(cos(thetaRad));

  double measuredYmm = FIELD_H_MM - projected;
  double measuredYin = measuredYmm * MM_TO_IN;
  chassis.setPose(pose.x, measuredYin, pose.theta);
}

void updateXFromFrontAlways() {
  constexpr double FRONT_OFFSET_MM = 85.0;

  double lf = leftFront.get();
  double rf = rightFront.get();

  auto pose = chassis.getPose();
  double thetaRad = pose.theta * M_PI / 180.0;

  double avgFront = (lf + rf) / 2.0;
  double projected = (avgFront + FRONT_OFFSET_MM) * fabs(cos(thetaRad));

  double measuredXmm = projected;
  double measuredXin = measuredXmm * MM_TO_IN;
  chassis.setPose(measuredXin, pose.y, pose.theta);
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
    store();
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

  void intakeUntilDone(int timeout) {
    done = false;
    store();
    int time = 0;
    while ((matchDist.get() < 255) && time < timeout) {
      delay(150);
      time += 100;
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
  chassis.calibrate();
  // setPoseFromAllSensors();
  Task imuTask(avgIMU);
  colorSensorMatch.set_led_pwm(100);
  colorSensorScore.set_led_pwm(100);
  chassis.setBrakeMode(E_MOTOR_BRAKE_HOLD);
}

void disabled() {}

void competition_initialize() {}

void autonomous() { /*
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
   });*/
  match.extend();
  colorSensorMatch.set_led_pwm(80);

  switch (selected) {
  case Left:
    break;
  case Right:
    chassis.moveToPoint(-4, 35, 3000);
    delay(200);
    chassis.turnToPoint(5, 35, 3000);
    chassis.moveToPose(6, 36, 90, 3000);
    delay(200);
    chassis.moveToPoint(-27, 3, 3000, {.forwards = false});
    delay(200);
    chassis.moveToPose(-105, 31, -53, 5000, {.lead = 0.5});
    delay(200);
    chassis.moveToPose(-80, 37, 270, 3000, {.forwards = false, .minSpeed = 60});
    balls.loadTop();
    delay(200);
    balls.cancel();
    chassis.moveToPose(-114, 36, 270, 3000);
    delay(200);
    chassis.moveToPose(-83, 30, 270, 3000);
    delay(200);
    chassis.moveToPose(-93, -66, 142, 3000);
    delay(200);
    chassis.moveToPose(-105, -3, 268, 3000);
    delay(200);
    break;
  case rSolo:
    break;
  case skills:
    break;
  }
}//hi

void opcontrol() {

  arm.extend();
  // print position to brain
  float f;
  pros::Task screen_task([&]() {
    while (true) {
      f = updateThetaFromFrontAlways();
      // print robot location to the brain screen
      pros::lcd::print(0, "X: %f    matchHue: %f", chassis.getPose().x,
                       colorSensorMatch.get_hue()); // x
      pros::lcd::print(1, "Y: %f    topHue: %f", chassis.getPose().y,
                       colorSensorScore.get_hue()); // y
      pros::lcd::print(2, "Theta: %f    DTheta :%f", chassis.getPose().theta,
                       f); // heading

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