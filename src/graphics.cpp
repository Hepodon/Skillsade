#include "graphics.hpp"
#include "autons.hpp"
#include "liblvgl/core/lv_obj.h"
#include "liblvgl/core/lv_obj_pos.h"
#include "liblvgl/core/lv_obj_scroll.h"
#include "liblvgl/core/lv_obj_style_gen.h"
#include "liblvgl/display/lv_display.h"
#include "liblvgl/draw/sw/lv_draw_sw_gradient.h"
#include "liblvgl/lv_conf_internal.h"
#include "liblvgl/lvgl.h"
#include "liblvgl/misc/lv_area.h"
#include "liblvgl/misc/lv_color.h"
#include "liblvgl/misc/lv_palette.h"
#include "liblvgl/misc/lv_timer.h"
#include "liblvgl/widgets/arc/lv_arc.h"
#include "liblvgl/widgets/chart/lv_chart.h"
#include "liblvgl/widgets/label/lv_label.h"
#include "lvgl_Customs.hpp"
#include "main.h"
#include "portDef.hpp"
#include "pros/abstract_motor.hpp"
#include "pros/motors.h"
#include "pros/screen.hpp"
#include <cstring>
#include <functional>

float t_kp;
float t_ki;
float t_kd;

float l_kp;
float l_ki;
float l_kd;

int loadtime = 500;
enum ScreenState { UI, HEADING, POSITION, TEMP, TORQUE };
ScreenState currentScreen;
lv_obj_t *autonScreen;
lv_obj_t *uiScreen;
lv_obj_t *autoPIDScreen;
lv_obj_t *turnPIDScreen;
lv_obj_t *latPIDScreen;
lv_obj_t *diagScreen;
lv_obj_t *headingScreen = nullptr;
lv_obj_t *positionScreen = nullptr;
lv_obj_t *tempScreen = nullptr;
lv_obj_t *torqueScreen = nullptr;
lv_color_t customColor = {60, 29, 40};
int motorCount = sizeof(motorPorts) / sizeof(motorPorts[0]);
std::vector<lv_obj_t *> tempArcs(motorCount, nullptr);
std::vector<lv_obj_t *> torqueArcs(motorCount, nullptr);

Chartseries heading;
customChart position;

void activateHeadingChart(lv_event_t *e);
void activatePositionChart(lv_event_t *e);
void loadTempScreen(lv_event_t *e);
void loadTorqueScreen(lv_event_t *e);
void uiScreenloader(lv_event_t *e);

int buttonCount = 6;
int screen_width = 480;

lv_obj_t *leftAutonButton;
lv_obj_t *rightAutonButton;
lv_obj_t *autoPIDButton;
lv_obj_t *turnPIDButton;
lv_obj_t *latPIDButton;
lv_obj_t *rightSoloAutonButton;
lv_obj_t *SkillsAutonButton;
lv_obj_t *backButton;
lv_obj_t *title;
lv_obj_t *headingButton;
lv_obj_t *errorButton;
lv_obj_t *torqueButton;
lv_obj_t *positionButton;
lv_obj_t *backDiagButton;
lv_obj_t *tempButton;

void update_arc_color(lv_obj_t *arc, int value, int maxValue) {
  lv_color_t color;
  color = lv_palette_main(value > maxValue * 0.85   ? LV_PALETTE_RED
                          : value > maxValue * 0.75 ? LV_PALETTE_DEEP_ORANGE
                          : value > maxValue * 0.65 ? LV_PALETTE_ORANGE
                          : value > maxValue * 0.50 ? LV_PALETTE_YELLOW
                          : value > maxValue * 0.35 ? LV_PALETTE_GREEN
                                                    : LV_PALETTE_BLUE);

  lv_obj_set_style_arc_color(arc, color, LV_PART_INDICATOR);
}
int navStartIndex = 0;
const int visibleButtons = 4;

struct NavItem {
  const char *label;
  lv_event_cb_t cb;
};

std::vector<NavItem> navItems = {
    {"Heading", activateHeadingChart},   {"Error", nullptr},
    {"Position", activatePositionChart}, {"Temp", loadTempScreen},
    {"Torque", loadTorqueScreen},
    // add more freely here
};
lv_obj_t *navContainer = nullptr;
void refreshNavBar(lv_obj_t *parent) {
  lv_obj_clean(navContainer);

  // LEFT
  createLvglButton(
      navContainer, "<",
      [](lv_event_t *e) {
        if (navStartIndex > 0) {
          navStartIndex--;
          refreshNavBar(navContainer);
        }
      },
      40, 30, LV_ALIGN_LEFT_MID, 0, 0, LV_PALETTE_PURPLE, 0);

  // BUTTONS
  for (int i = 0; i < visibleButtons; i++) {
    int idx = navStartIndex + i;
    if (idx >= navItems.size())
      break;

    createLvglButton(navContainer, navItems[idx].label, navItems[idx].cb, 90,
                     30, LV_ALIGN_LEFT_MID, 45 + (i * 90), 0, LV_PALETTE_PURPLE,
                     0);
  }

  // RIGHT
  createLvglButton(
      navContainer, ">",
      [](lv_event_t *e) {
        if (navStartIndex + visibleButtons < navItems.size()) {
          navStartIndex++;
          refreshNavBar(navContainer);
        }
      },
      40, 30, LV_ALIGN_RIGHT_MID, 0, 0, LV_PALETTE_PURPLE, 0);
}
bool tuningActive = false;

float wrapError(float target, float current) {
  float error = target - current;
  if (error > 180)
    error -= 360;
  if (error < -180)
    error += 360;
  return error;
}

float lastScore = 1e9;
int stagnantRuns = 0;
int goodRuns = 0;
int totalRuns;

int settleStart = -1;
int zeroCrossings = 0;

void turnPIDTunerTask(void *) {
  tuningActive = true;

  while (tuningActive) {
    float target = 90 + rand() % 90;

    chassis.setPose(0, 0, 0);

    float maxError = 0;
    float lastError = 0;
    float steadyStateError = 0;
    int settleTime = 0;
    zeroCrossings = 0;
    bool settled = false;
    bool first = true;
    bool lastSign = false;

    chassis.turnToHeading(target, 2000, {}, false);

    int startTime = pros::millis();

    while (true) {
      
      float current = chassis.getPose().theta;
      float error = wrapError(target, current);
      float absError = fabs(error);

      if (absError > maxError) {
        maxError = absError;
      }

      float deadband = 1.0;

      if (fabs(error) > deadband) {
        bool sign = error > 0;

        if (!first && sign != lastSign) {
          zeroCrossings++;
        }

        lastSign = sign;
        first = false;
      }

      if (absError < 1.0) {
        if (settleStart == -1) {
          settleStart = pros::millis();
        }

        if (pros::millis() - settleStart > 150) {
          settled = true;
          settleTime = pros::millis() - startTime;
        }
      } else {
        settleStart = -1;
      }

      lastError = absError;

      if (pros::millis() - startTime > 2000 || settled) {
        break;
      }

      pros::delay(10);
    }

    steadyStateError = lastError;

    if (steadyStateError > 3) {
      float factor = std::clamp(steadyStateError / 10.0f, 1.05f, 1.2f);
      angular_controller.kP *= factor;
    } else if (maxError > 25) {
      angular_controller.kP *= 0.9;
    }

    if (maxError > 2) {
      angular_controller.kD *=
          std::clamp(static_cast<float>(maxError), 1.1f, 1.5f);
    }

    if (zeroCrossings >= 2) {
      float factor =
          std::clamp(static_cast<float>(zeroCrossings) / 2, 1.1f, 1.5f);
      angular_controller.kD *= factor;
    } else if (zeroCrossings == 0 && maxError < 10) {
      angular_controller.kD *= 0.95;
    }

    if (settleTime > 1200) {
      angular_controller.kP *= 1.05;
    }

    angular_controller.kP = std::clamp(angular_controller.kP, 0.1f, 20.0f);
    angular_controller.kD = std::clamp(angular_controller.kD, 0.0f, 50.0f);

    t_kp = angular_controller.kP;
    t_kd = angular_controller.kD;

    float score = settleTime + (maxError * 5) + (steadyStateError * 10) +
                  (zeroCrossings * 100);

    if (fabs(score - lastScore) < 5.0) {
      stagnantRuns++;
    } else {
      stagnantRuns = 0;
    }

    bool meetsCriteria;

    if (steadyStateError < 1.0 && maxError < 15 && settleTime < 800 &&
        zeroCrossings <= 2) {
      meetsCriteria = true;
    } else {
      meetsCriteria = false;
    }

    if (meetsCriteria) {
      goodRuns++;
    } else {
      goodRuns = 0;
    }

    lastScore = score;

    if ((goodRuns >= 5 || stagnantRuns >= 5) && totalRuns > 10) {
      tuningActive = false;
    }
    totalRuns++;
    pros::delay(300);
  }
}

lv_obj_t *ZC = nullptr;

void updatePIDTextp(lv_timer_t *timer) {
  char buf[32];

  snprintf(buf, sizeof(buf), "kp: %.5f", t_kp);
  lv_label_set_text((lv_obj_t *)timer->user_data, buf);

  lv_label_set_text_fmt(ZC, "ZC: %d", zeroCrossings);
}

void updatePIDTexti(lv_timer_t *timer) {
  char buf[32];

  snprintf(buf, sizeof(buf), "ki: %.5f", t_ki);
  lv_label_set_text((lv_obj_t *)timer->user_data, buf);
}
void updatePIDTextd(lv_timer_t *timer) {
  char buf[32];

  snprintf(buf, sizeof(buf), "kd: %.5f", t_kd);
  lv_label_set_text((lv_obj_t *)timer->user_data, buf);
}
void createNavBar(lv_obj_t *parent) {
  lv_obj_set_style_bg_color(parent, customColor, 0);

  backDiagButton =
      createLvglButton(parent, "X", uiScreenloader, 30, 30,
                       LV_ALIGN_BOTTOM_LEFT, 5, -5, LV_PALETTE_PURPLE, 0);

  navContainer = lv_obj_create(parent);
  lv_obj_set_size(navContainer, screen_width, 40);
  lv_obj_align(navContainer, LV_ALIGN_TOP_LEFT, 0, 0);

  lv_obj_set_style_bg_opa(navContainer, 0, 0);

  lv_obj_remove_flag(navContainer, LV_OBJ_FLAG_SCROLLABLE);

  lv_obj_set_style_border_width(navContainer, 0, 0);

  refreshNavBar(parent);
}

double robot_X = 0;
double robot_Orientation = 0;
double robot_Y = 0;

double headingValue = 0;

void updateChartLVGL(lv_timer_t *timer) {
  lv_chart_set_next_value(heading.chart, heading.headingSeries, headingValue);
  lv_chart_refresh(heading.chart);
}

lv_timer_t *headingTimer = nullptr;
lv_timer_t *tempTimer = nullptr;
lv_timer_t *torqueTimer = nullptr;

void masterUpdateTask() {
  while (true) {

    if (currentScreen == HEADING || currentScreen == POSITION) {
      double head = inertial1.get_heading();
      if (head > 180)
        head -= 360;
      headingValue = head;
    } else if (currentScreen == TEMP) {
      for (int i = 0; i < motorCount; i++) {
        motorPorts[i][1] = pros::c::motor_get_temperature(motorPorts[i][0]);
      }
    } else if (currentScreen == TORQUE) {
      for (int i = 0; i < motorCount; i++) {
        motorPorts[i][2] = pros::c::motor_get_torque(motorPorts[i][0]) * 100;
      }
    } else if (currentScreen == POSITION) {
      robot_X = chassis.getPose().x;
      robot_Y = chassis.getPose().y;
      robot_Orientation = chassis.getPose(true).theta;
    }

    pros::delay(5);
  }
}

void updateTempArc(lv_timer_t *timer) {
  for (int i = 0; i < motorCount; i++) {
    int val = motorPorts[i][1];
    lv_arc_set_value(tempArcs[i], val);
    update_arc_color(tempArcs[i], val, lv_arc_get_max_value(tempArcs[i]));
  }
}
void updateTorqueArc(lv_timer_t *timer) {
  for (int i = 0; i < motorCount; i++) {
    int val = motorPorts[i][2];
    lv_arc_set_value(torqueArcs[i], val);
    update_arc_color(torqueArcs[i], val, lv_arc_get_max_value(torqueArcs[i]));
  }
}

void createTempArcs() {

  for (int i = 0; i < motorCount; i++) {

    tempArcs[i] = lv_arc_create(tempScreen);

    lv_obj_set_size(tempArcs[i], 90, 90);

    lv_arc_set_rotation(tempArcs[i], 180);

    lv_arc_set_range(tempArcs[i], 20, 62);

    lv_arc_set_bg_angles(tempArcs[i], 30, 150);

    lv_arc_set_value(tempArcs[i], i * 11);

    lv_obj_remove_style(tempArcs[i], NULL, LV_PART_KNOB);

    lv_obj_remove_flag(tempArcs[i], LV_OBJ_FLAG_CLICKABLE);

    update_arc_color(tempArcs[i], lv_arc_get_value(tempArcs[i]),
                     lv_arc_get_max_value(tempArcs[i]));

    std::string position;

    if (i < 6) {
      position = "CHASSIS";
    } else if (i == 6) {
      position = "L-MIDDLE";
    } else if (i == 7) {
      position = "R-MIDDLE";
    }

    if (i < 5) {
      lv_obj_align(tempArcs[i], LV_ALIGN_BOTTOM_LEFT, 4 + (96 * i),
                   -(242 / 2) + 15);
    } else if (i < 10) {
      lv_obj_align(tempArcs[i], LV_ALIGN_BOTTOM_LEFT, 4 + (96 * (i - 5)), -15);
    }
    lv_obj_t *label = lv_label_create(tempArcs[i]);
    lv_label_set_text_fmt(label, "Port: %d",
                          static_cast<int>(motorPorts[i][0]));
    lv_obj_center(label);
    lv_obj_t *name = lv_label_create(tempArcs[i]);
    lv_label_set_text_fmt(name, "%s", position.c_str());
    lv_obj_align(name, LV_ALIGN_CENTER, 0, 15);
  }
}

void createTorqueArcs() {

  for (int i = 0; i < motorCount; i++) {

    torqueArcs[i] = lv_arc_create(torqueScreen);

    lv_obj_set_size(torqueArcs[i], 90, 90);

    lv_arc_set_rotation(torqueArcs[i], 180);

    lv_arc_set_range(torqueArcs[i], 0, 100);

    lv_arc_set_bg_angles(torqueArcs[i], 30, 150);

    lv_obj_remove_style(torqueArcs[i], NULL, LV_PART_KNOB);

    lv_obj_remove_flag(torqueArcs[i], LV_OBJ_FLAG_CLICKABLE);

    update_arc_color(torqueArcs[i], lv_arc_get_value(torqueArcs[i]),
                     lv_arc_get_max_value(torqueArcs[i]));

    std::string position;

    if (i < 6) {
      position = "CHASSIS";
    } else if (i == 6) {
      position = "L-MIDDLE";
    } else if (i == 7) {
      position = "R-MIDDLE";
    }
    if (i < 5) {
      lv_obj_align(torqueArcs[i], LV_ALIGN_BOTTOM_LEFT, 4 + (96 * i),
                   -(242 / 2) + 15);
    } else if (i < 10) {
      lv_obj_align(torqueArcs[i], LV_ALIGN_BOTTOM_LEFT, 4 + (96 * (i - 5)),
                   -15);
    }
    lv_obj_t *label = lv_label_create(torqueArcs[i]);
    lv_label_set_text_fmt(label, "Port: %d",
                          static_cast<int>(motorPorts[i][0]));
    lv_obj_center(label);
    lv_obj_t *name = lv_label_create(torqueArcs[i]);
    lv_label_set_text_fmt(name, "%s", position.c_str());
    lv_obj_align(name, LV_ALIGN_CENTER, 0, 15);
  }
}

void loadTempScreen(lv_event_t *e) {
  createNavBar(tempScreen);

  lv_obj_set_style_bg_grad_color(tempScreen, customColor, 0);

  static bool created = false;

  if (!created) {
    createTempArcs();
    created = true;
  }

  if (tempTimer == nullptr) {
    tempTimer = lv_timer_create(updateTempArc, 100, NULL);
    lv_timer_resume(tempTimer);
  }

  currentScreen = TEMP;
  lv_screen_load(tempScreen);
}
void loadTorqueScreen(lv_event_t *e) {
  createNavBar(torqueScreen);

  lv_obj_set_style_bg_grad_color(torqueScreen, customColor, 0);

  static bool created = false;

  if (!created) {
    createTorqueArcs();
    created = true;
  }

  if (torqueTimer == nullptr) {
    torqueTimer = lv_timer_create(updateTorqueArc, 100, NULL);
  } else {
    lv_timer_resume(torqueTimer);
  }

  currentScreen = TORQUE;
  lv_screen_load(torqueScreen);
}

void createHeadingChart() {
  lv_obj_t *headingChart = createLVGLChart(heading, headingScreen);
  lv_obj_align(headingChart, LV_ALIGN_CENTER, 0, 16);
}
lv_obj_t *createPositionChart(customChart &stru, lv_obj_t *parent) {
  lv_obj_t *chart = lv_chart_create(parent);

  lv_chart_set_type(chart, LV_CHART_TYPE_SCATTER);

  lv_chart_set_range(chart, LV_CHART_AXIS_PRIMARY_X, -72, 72);
  lv_chart_set_range(chart, LV_CHART_AXIS_PRIMARY_Y, -72, 72);

  lv_chart_set_point_count(chart, 1);

  lv_chart_series_t *series = lv_chart_add_series(
      chart, lv_palette_main(LV_PALETTE_RED), LV_CHART_AXIS_PRIMARY_Y);

  lv_obj_set_size(chart, 204, 204);
  lv_obj_align(chart, LV_ALIGN_LEFT_MID, 15, 16);

  stru = {chart, series};

  return chart;
}

void updatePositionChartLVGL(lv_timer_t *timer) {
  if (position.chart == nullptr || position.series == nullptr)
    return;

  position.series->x_points[0] = robot_X;
  position.series->y_points[0] = robot_Y;

  lv_chart_refresh(position.chart);
}
void createPositionChartScreen() {
  createPositionChart(position, positionScreen);
}

lv_timer_t *positionTimer = nullptr;

void activateHeadingChart(lv_event_t *e) {
  static bool chartCreated = false;
  createNavBar(headingScreen);

  if (!chartCreated) {
    createHeadingChart();
    chartCreated = true;
  }

  if (headingTimer == nullptr) {
    headingTimer = lv_timer_create(updateChartLVGL, 100, NULL);
  } else {
    lv_timer_resume(headingTimer);
  }

  currentScreen = HEADING;
  lv_screen_load(headingScreen);
}

void activatePositionChart(lv_event_t *e) {
  createNavBar(positionScreen);

  static bool created = false;

  if (!created) {
    createPositionChartScreen();
    created = true;
  }

  if (positionTimer == nullptr) {
    positionTimer = lv_timer_create(updatePositionChartLVGL, 100, NULL);
  } else {
    lv_timer_resume(positionTimer);
  }

  lv_obj_set_style_bg_grad_color(positionScreen, customColor, 0);

  currentScreen = POSITION;
  lv_screen_load(positionScreen);
}

void uiScreenloader(lv_event_t *e) {

  if (headingTimer != nullptr) {
    lv_timer_pause(headingTimer);
  }

  currentScreen = UI;
  lv_screen_load_anim(uiScreen, LV_SCR_LOAD_ANIM_MOVE_BOTTOM, loadtime, 250,
                      false);
}

void loadDiagScreen(lv_event_t *e) {
  pros::Task masterTask(masterUpdateTask);
  createNavBar(diagScreen);
  currentScreen = UI;
  lv_screen_load_anim(diagScreen, LV_SCR_LOAD_ANIM_MOVE_TOP, loadtime, 250,
                      false);
}

void loadTurnPID(lv_event_t *e) {
  lv_obj_t *kp =
      createLVGLText(turnPIDScreen, nullptr, LV_ALIGN_CENTER, 0, -30);
  lv_obj_t *ki = createLVGLText(turnPIDScreen, nullptr, LV_ALIGN_CENTER, 0, 0);
  lv_obj_t *kd = createLVGLText(turnPIDScreen, nullptr, LV_ALIGN_CENTER, 0, 30);
  ZC = createLVGLText(turnPIDScreen, "ZC: ", LV_ALIGN_CENTER, 0, -60);
  static pros::Task tunerTask(turnPIDTunerTask);
  lv_timer_create(updatePIDTextp, 100, kp);
  lv_timer_create(updatePIDTexti, 100, ki);
  lv_timer_create(updatePIDTextd, 100, kd);

  lv_screen_load_anim(turnPIDScreen, LV_SCR_LOAD_ANIM_FADE_IN, loadtime, 100,
                      false);
}

void loadLatPID(lv_event_t *e) {
  lv_obj_t *kp = createLVGLText(latPIDScreen, nullptr, LV_ALIGN_CENTER, 0, -30);
  lv_obj_t *ki = createLVGLText(latPIDScreen, nullptr, LV_ALIGN_CENTER, 0, 0);
  lv_obj_t *kd = createLVGLText(latPIDScreen, nullptr, LV_ALIGN_CENTER, 0, 30);

  char buf[32];
  snprintf(buf, sizeof(buf), "kp: %.5f", l_kp);
  lv_label_set_text(kp, buf);
  snprintf(buf, sizeof(buf), "ki: %.5f", l_ki);
  lv_label_set_text(ki, buf);
  snprintf(buf, sizeof(buf), "kd: %.5f", l_kd);
  lv_label_set_text(kd, buf);

  lv_screen_load_anim(latPIDScreen, LV_SCR_LOAD_ANIM_FADE_IN, loadtime, 100,
                      false);
}

void loadAutoPIDScreen(lv_event_t *e) {
  t_kp = angular_controller.kP;
  t_ki = angular_controller.kI;
  t_kd = angular_controller.kD;
  l_kp = lateral_controller.kP;
  l_ki = lateral_controller.kI;
  l_kd = lateral_controller.kD;

  lv_obj_t *title =
      createLVGLText(autoPIDScreen, "Auto PID Tuner", LV_ALIGN_TOP_MID, 0, 5);

  turnPIDButton = createLvglButton(autoPIDScreen, "Turn", loadTurnPID, 85, 75,
                                   LV_ALIGN_CENTER, -50, 5);
  latPIDButton = createLvglButton(autoPIDScreen, "Lateral", loadLatPID, 85, 75,
                                  LV_ALIGN_CENTER, 50, 5);

  lv_screen_load_anim(autoPIDScreen, LV_SCR_LOAD_ANIM_MOVE_TOP, loadtime, 100,
                      false);
}

void loadAutonScreen(lv_event_t *e) {
  title = createLVGLText(autonScreen, "22204W Auton", LV_ALIGN_TOP_MID, 0, 6);

  backButton =
      createLvglButton(autonScreen, "X", uiScreenloader, 40, 40,
                       LV_ALIGN_TOP_LEFT, 14, 14, LV_PALETTE_PURPLE, 5);
  leftAutonButton =
      createLvglButton(autonScreen, "Left Auton", leftAuton, 115, 50,
                       LV_ALIGN_CENTER, -60, 0, LV_PALETTE_PURPLE);

  rightAutonButton =
      createLvglButton(autonScreen, "Right Auton", rightAuton, 115, 50,
                       LV_ALIGN_CENTER, 60, 0, LV_PALETTE_PURPLE);

  rightSoloAutonButton =
      createLvglButton(autonScreen, "Right Solo Auton", soloRightAuton, 230, 60,
                       LV_ALIGN_CENTER, 0, -60, LV_PALETTE_PURPLE);

  SkillsAutonButton =
      createLvglButton(autonScreen, "Skills Auton", skillsAuton, 230, 60,
                       LV_ALIGN_CENTER, 0, 60, LV_PALETTE_PURPLE);

  lv_obj_set_style_text_font(title, &lv_font_montserrat_16, 0);
  lv_screen_load_anim(autonScreen, LV_SCR_LOAD_ANIM_MOVE_TOP, loadtime, 250,
                      false);
}

void screeninit() {
  autonScreen = lv_obj_create(NULL);
  uiScreen = lv_obj_create(NULL);
  diagScreen = lv_obj_create(NULL);
  headingScreen = lv_obj_create(NULL);
  positionScreen = lv_obj_create(NULL);
  tempScreen = lv_obj_create(NULL);
  autoPIDScreen = lv_obj_create(NULL);
  turnPIDScreen = lv_obj_create(NULL);
  latPIDScreen = lv_obj_create(NULL);
  torqueScreen = lv_obj_create(NULL);
  currentScreen = UI;

  lv_obj_set_style_bg_color(uiScreen, customColor, 0);
  lv_obj_set_style_bg_color(autonScreen, customColor, 0);
  lv_obj_set_style_bg_color(diagScreen, customColor, 0);

  lv_obj_t *title = createLVGLText(uiScreen, "22204W", LV_ALIGN_TOP_MID, 0, 30);
  lv_obj_set_style_text_font(title, &lv_font_montserrat_24, 0);
  lv_obj_t *autonButton =
      createLvglButton(uiScreen, "Auton", loadAutonScreen, 105, 60,
                       LV_ALIGN_RIGHT_MID, -105, -10, LV_PALETTE_PURPLE);

  autoPIDButton = createLvglButton(uiScreen, "AUTO PID", loadAutoPIDScreen, 200,
                                   60, LV_ALIGN_BOTTOM_MID, 0, -8);

  lv_obj_t *diagButton =
      createLvglButton(uiScreen, "Diagnostics", loadDiagScreen, 105, 60,
                       LV_ALIGN_LEFT_MID, 85, -10, LV_PALETTE_PURPLE);
  lv_screen_load_anim(uiScreen, LV_SCR_LOAD_ANIM_OUT_TOP, loadtime * 1.25, 250,
                      false);
}
