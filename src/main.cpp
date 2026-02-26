#include "main.h"
#include "RclTracking.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/chassis/trackingWheel.hpp"
#include "liblvgl/core/lv_obj.h"
#include "liblvgl/core/lv_obj_pos.h"
#include "liblvgl/core/lv_obj_scroll.h"
#include "liblvgl/core/lv_obj_tree.h"
#include "liblvgl/display/lv_display.h"
#include "liblvgl/lvgl.h"
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

pros::MotorGroup aleft({-10, -9, -8});
pros::MotorGroup aright({1, 2, 3});

pros::MotorGroup middle({7, -4});

pros::adi::Pneumatics match('a', false);
pros::adi::Pneumatics arm('h', false);
pros::adi::Pneumatics tripstate('b', false);
pros::adi::Pneumatics tripstate2('e', false);

pros::Rotation vertRot(5);

lemlib::TrackingWheel vert(&vertRot, lemlib::Omniwheel::NEW_2, 0.75);

pros::IMU imu(6);

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

pros::Controller userInput(pros::E_CONTROLLER_MASTER);

using namespace pros;

enum auton { Left, Right, rSolo, skills };

auton selected;

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
    // print position to brain screen
    pros::Task screen_task([&]() {
      while (true) {
        // print robot location to the brain screen
        pros::lcd::print(0, "X: %f", chassis.getPose().x);         // x
        pros::lcd::print(1, "Y: %f", chassis.getPose().y);         // y
        pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
        // delay to save resources
        pros::delay(20);
      }
    });
  }
  chassis.calibrate();
  chassis.setBrakeMode(E_MOTOR_BRAKE_COAST);
}

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
  middle.move(-127);
  while (userInput.get_digital(DIGITAL_B)) {
    delay(5);
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