#include "graphics.hpp"
#include "autons.hpp"
#include "liblvgl/core/lv_obj.h"
#include "liblvgl/core/lv_obj_pos.h"
#include "liblvgl/core/lv_obj_style_gen.h"
#include "liblvgl/display/lv_display.h"
#include "liblvgl/draw/sw/lv_draw_sw_gradient.h"
#include "liblvgl/lv_conf_internal.h"
#include "liblvgl/lvgl.h"
#include "liblvgl/misc/lv_area.h"
#include "liblvgl/misc/lv_color.h"
#include "liblvgl/misc/lv_palette.h"
#include "liblvgl/widgets/arc/lv_arc.h"
#include "liblvgl/widgets/chart/lv_chart.h"
#include "lvgl_Customs.hpp"
#include "main.h"
#include "portDef.hpp"
#include "pros/abstract_motor.hpp"
#include "pros/motors.h"
#include "pros/screen.hpp"
#include <cstring>
#include <functional>

int loadtime = 500;
enum ScreenState { UI, HEADING, POSITION, TEMP, TORQUE };
ScreenState currentScreen;
lv_obj_t *autonScreen;
lv_obj_t *uiScreen;
lv_obj_t *diagScreen;
lv_obj_t *headingScreen = nullptr;
lv_obj_t *positionScreen = nullptr;
lv_obj_t *tempScreen = nullptr;
lv_color_t customColor = {60, 29, 40};
int motorCount = sizeof(motorPorts) / sizeof(motorPorts[0]);
std::vector<lv_obj_t *> arcs(motorCount, nullptr);

Chartseries heading;
customChart position;
void activateHeadingChart(lv_event_t *e);
void activatePositionChart(lv_event_t *e);
void loadTempScreen(lv_event_t *e);
void uiScreenloader(lv_event_t *e);

int buttonCount = 5;
int screen_width = 480;

lv_obj_t *leftAutonButton;
lv_obj_t *rightAutonButton;
lv_obj_t *rightSoloAutonButton;
lv_obj_t *SkillsAutonButton;
lv_obj_t *backButton;
lv_obj_t *title;
lv_obj_t *headingButton;
lv_obj_t *errorButton;
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

void createNavBar(lv_obj_t *parent) {
  lv_obj_set_style_bg_color(parent, customColor, 0);

  backDiagButton = createLvglButton(
      parent, "<", uiScreenloader, (screen_width / buttonCount), 30,
      LV_ALIGN_TOP_LEFT, (0 * (screen_width / buttonCount)), 0,
      LV_PALETTE_PURPLE, 0);

  if (parent != headingScreen) {
    headingButton = createLvglButton(
        parent, "Heading", activateHeadingChart, (screen_width / buttonCount),
        30, LV_ALIGN_TOP_LEFT, (1 * (screen_width / buttonCount)), 0,
        LV_PALETTE_PURPLE, 0);
  } else {
    headingButton = createLvglButton(
        parent, "Heading", activateHeadingChart, (screen_width / buttonCount),
        30, LV_ALIGN_TOP_LEFT, (1 * (screen_width / buttonCount)), 0,
        LV_PALETTE_DEEP_PURPLE, 0);
  }

  errorButton = createLvglButton(
      parent, "Error", nullptr, (screen_width / buttonCount), 30,
      LV_ALIGN_TOP_LEFT, (2 * (screen_width / buttonCount)), 0,
      LV_PALETTE_PURPLE, 0);

  if (parent != positionScreen) {
    positionButton = createLvglButton(
        parent, "Position", activatePositionChart, (screen_width / buttonCount),
        30, LV_ALIGN_TOP_LEFT, (3 * (screen_width / buttonCount)), 0,
        LV_PALETTE_PURPLE, 0);
  } else {
    positionButton = createLvglButton(
        parent, "Position", activatePositionChart, (screen_width / buttonCount),
        30, LV_ALIGN_TOP_LEFT, (3 * (screen_width / buttonCount)), 0,
        LV_PALETTE_DEEP_PURPLE, 0);
  }
  if (parent != tempScreen) {
    tempButton = createLvglButton(
        parent, "temp", loadTempScreen, (screen_width / buttonCount), 30,
        LV_ALIGN_TOP_LEFT, (4 * (screen_width / buttonCount)), 0,
        LV_PALETTE_PURPLE, 0);
  } else {
    tempButton = createLvglButton(
        parent, "temp", loadTempScreen, (screen_width / buttonCount), 30,
        LV_ALIGN_TOP_LEFT, (4 * (screen_width / buttonCount)), 0,
        LV_PALETTE_DEEP_PURPLE, 0);
  }
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
        motorPorts[i][2] = pros::c::motor_get_torque(motorPorts[i][0]);
      }
    } else if (currentScreen == POSITION) {
      robot_X = chassis.getPose().x;
      robot_Y = chassis.getPose().y;
      robot_Orientation = chassis.getPose(true).theta;
    }

    pros::delay(5);
  }
}

pros::Task masterTask(masterUpdateTask);

void updateTempArc(lv_timer_t *timer) {
  for (int i = 0; i < 9; i++) {
    int val = motorPorts[i][1];
    lv_arc_set_value(arcs[i], val);
    update_arc_color(arcs[i], val, lv_arc_get_max_value(arcs[i]));
  }
}

void createTempArcs() {

  for (int i = 0; i < motorCount; i++) {

    arcs[i] = lv_arc_create(tempScreen);

    lv_obj_set_size(arcs[i], 90, 90);

    lv_arc_set_range(arcs[i], 20, 62);

    lv_arc_set_rotation(arcs[i], 180);

    lv_arc_set_bg_angles(arcs[i], 30, 150);

    lv_arc_set_value(arcs[i], i * 11);

    lv_obj_remove_style(arcs[i], NULL, LV_PART_KNOB);

    lv_obj_remove_flag(arcs[i], LV_OBJ_FLAG_CLICKABLE);

    update_arc_color(arcs[i], lv_arc_get_value(arcs[i]),
                     lv_arc_get_max_value(arcs[i]));

    std::string position;

    if (i < 6) {
      position = "CHASSIS";
    } else if (i == 6) {
      position = "INTAKE";
    } else if (i == 7) {
      position = "HOOD";
    } else if (i == 8) {
      position = "MIDDLE";
    }

    if (i < 5) {
      lv_obj_align(arcs[i], LV_ALIGN_BOTTOM_LEFT, 4 + (96 * i),
                   -(242 / 2) + 15);
    } else if (i < 10) {
      lv_obj_align(arcs[i], LV_ALIGN_BOTTOM_LEFT, 4 + (96 * (i - 5)), -15);
    }
    lv_obj_t *label = lv_label_create(arcs[i]);
    lv_label_set_text_fmt(label, "Port: %d",
                          static_cast<int>(motorPorts[i][0]));
    lv_obj_center(label);
    lv_obj_t *name = lv_label_create(arcs[i]);
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
  } else {
    lv_timer_resume(tempTimer);
  }

  currentScreen = TEMP;
  lv_screen_load(tempScreen);
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
  createNavBar(diagScreen);
  currentScreen = UI;
  lv_screen_load_anim(diagScreen, LV_SCR_LOAD_ANIM_MOVE_TOP, loadtime, 250,
                      false);
}

void loadAutonScreen(lv_event_t *e) {
  title = createLVGLText(autonScreen, "22204W Auton", LV_ALIGN_TOP_MID, 0, 6);

  backButton =
      createLvglButton(autonScreen, "<", uiScreenloader, 40, 40,
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
  currentScreen = UI;

  lv_obj_set_style_bg_color(uiScreen, customColor, 0);
  lv_obj_set_style_bg_color(autonScreen, customColor, 0);
  lv_obj_set_style_bg_color(diagScreen, customColor, 0);

  lv_obj_t *title = createLVGLText(uiScreen, "22204W", LV_ALIGN_TOP_MID, 0, 30);
  lv_obj_set_style_text_font(title, &lv_font_montserrat_24, 0);
  lv_obj_t *autonButton =
      createLvglButton(uiScreen, "Auton", loadAutonScreen, 105, 60,
                       LV_ALIGN_RIGHT_MID, -105, -10, LV_PALETTE_PURPLE);

  lv_obj_t *diagButton =
      createLvglButton(uiScreen, "Diagnostics", loadDiagScreen, 105, 60,
                       LV_ALIGN_LEFT_MID, 85, -10, LV_PALETTE_PURPLE);
  lv_screen_load_anim(uiScreen, LV_SCR_LOAD_ANIM_OUT_TOP, loadtime * 1.25, 250,
                      false);
}
