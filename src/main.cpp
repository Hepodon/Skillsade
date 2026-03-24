#include "main.h"
#include "controls.hpp"
#include "graphics.hpp"
#include "liblvgl/display/lv_display.h"
#include "liblvgl/lvgl.h"
#include "liblvgl/widgets/line/lv_line.h"
#include "mcl.hpp"
#include "portDef.hpp"
#include "pros/screen.hpp"
#include <cmath>
#include <cstdio>

constexpr size_t PARTICLES = 3000;
ad::MCL<PARTICLES> mcl;
ad::Point last_odom_pos{0.0f, 0.0f};
bool mcl_initialized = false;

using namespace pros;

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

void odom() {
  if (!mcl_initialized)
    return;

  auto pose = chassis.getPose();

  float odom_x = pose.x;
  float odom_y = pose.y;

  ad::Point current_odom{odom_x, odom_y};

  float dx = current_odom.x - last_odom_pos.x;
  float dy = current_odom.y - last_odom_pos.y;

  last_odom_pos = current_odom;

  float std_dev = std::hypot(dx, dy) / 4.0f;
  mcl.predict(dx, dy, std_dev);

  std::vector<ad::Reading> readings;

  float theta = inertial1.get_rotation() * M_PI / 180.0f;
  ad::Rotation heading = ad::rad(theta);

  ad::Position robot_pos{current_odom.x, current_odom.y, heading};

  auto add_sensor = [&](Distance &sensor, ad::Position offset) {
    auto v = sensor.get();
    if (!v)
      return;

    float d = v;

    float bound = d < 7.874015f ? 0.590551f : 0.05f * d;
    constexpr float K = 3.0f;
    float std_dev = std::max(bound / K, 1e-6f);

    ad::Position rel = offset.rotate(robot_pos.theta);
    ad::Point ray_pt =
        rel.point() + ad::Point{rel.theta.cos(), rel.theta.sin()};

    readings.emplace_back(d, std_dev, rel.point(), ray_pt);
  };

  add_sensor(Dleft, ad::Position{-5.0f, 0.0f, ad::deg(180)});
  add_sensor(Dright, ad::Position{5.0f, 0.0f, ad::deg(0)});
  add_sensor(Dfront, ad::Position{0.0f, 5.0f, ad::deg(90)});
  add_sensor(DbackL, ad::Position{0.0f, -5.0f, ad::deg(-90)});

  mcl.update(readings);

  ad::Point est = mcl.estimate();

  chassis.setPose(est.x, est.y, 0);

  mcl.resample();
}

// Task odomTask([] {
//   //   while (true) {
//   //     odom();
//   //     delay(10); // 100Hz
//   //   }
// });

void initialize() {
  chassis.calibrate(false);
  chassis.setBrakeMode(E_MOTOR_BRAKE_COAST);
  lvgl_init();
  screeninit();
}

void disabled() {}

void competition_initialize() { chassis.calibrate(); }

void autonomous() {}

void opcontrol() {
  int leftY;
  int rightX;

  chassis.setBrakeMode(E_MOTOR_BRAKE_COAST);

  while (true) {
    // Collect and store controller input
    leftY = userInput.get_analog(E_CONTROLLER_ANALOG_LEFT_Y);
    rightX = userInput.get_analog(E_CONTROLLER_ANALOG_RIGHT_X);

    applyButtons(userInput);

    // Apply controller input for movement using split arcade controls
    chassis.arcade(leftY, rightX, true, 0.40);

    delay(10);
  }
}