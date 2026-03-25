#include "main.h"
#include "controls.hpp"
#include "graphics.hpp"
#include "liblvgl/display/lv_display.h"
#include "liblvgl/misc/lv_event.h"
#include "liblvgl/misc/lv_types.h"
#include "mcl.hpp"
#include "portDef.hpp"
#include "pros/motors.h"
#include "pros/screen.hpp"
#include <cmath>
#include <cstdio>

constexpr size_t PARTICLES = 3000;
ad::MCL<PARTICLES> mcl;
ad::Point last_odom_pos{0.0f, 0.0f};
bool mcl_initialized = false;

using namespace pros;
using namespace std;

void odom() {

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

enum auton { Left, Right, rSolo, lSolo, skills };

auton selected;

void initialize() {
  chassis.calibrate(true);
  lvgl_init();
  screeninit();
}

void disabled() {}

void competition_initialize() {
  chassis.calibrate(true);
}

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