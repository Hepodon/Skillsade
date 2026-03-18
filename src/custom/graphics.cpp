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
#include "liblvgl/widgets/chart/lv_chart.h"
#include "lvgl_Customs.hpp"
#include "portDef.hpp"
#include "pros/screen.hpp"
#include <cstring>
#include <functional>

lv_obj_t *autonScreen;
lv_obj_t *uiScreen;
lv_obj_t *diagScreen;

Chartseries heading;

std::vector<pros::Task *> taskList;

void chartTaskManager(pros::Task *activeTask) {
  for (auto task : taskList) {
    if (task == activeTask && task != nullptr) {
      task->resume();
    } else if (task != nullptr) {
      task->suspend();
    }
  }
}

double headingValue = 0;

void updateChartLVGL(lv_timer_t *timer) {
  lv_chart_set_next_value(heading.chart, heading.headingSeries, headingValue);
  lv_chart_refresh(heading.chart);
}

pros::Task *headingTask = nullptr;
lv_obj_t *headingScreen = nullptr;

lv_timer_t *headingTimer = nullptr;

void updateHeading(void *param) {
  while (true) {
    double head = inertial1.get_heading();
    if (head > 180)
      head -= 360;

    headingValue = head;

    pros::delay(100);
  }
}
void createHeadingChart() {
  lv_obj_t *headingChart = createLVGLChart(heading, headingScreen);
}

lv_obj_t *headingButton;
lv_obj_t *errorButton;
lv_obj_t *positionButton;
lv_obj_t *backDiagButton;

void activateHeadingChart(lv_event_t *e) {
  static bool chartCreated = false;

  if (!chartCreated) {
    createHeadingChart();
    chartCreated = true;
  }

  if (headingTask == nullptr) {
    headingTask = new pros::Task(updateHeading, NULL);
    headingTask->suspend();
    taskList.push_back(headingTask);
  }

  if (headingTimer == nullptr) {
    headingTimer = lv_timer_create(updateChartLVGL, 100, NULL);
  } else {
    lv_timer_resume(headingTimer);
  }

  lv_screen_load(headingScreen);
  chartTaskManager(headingTask);
}

int buttonCount = 4;
int screen_width = 480;

void uiScreenloader(lv_event_t *e) {
  chartTaskManager(nullptr);

  if (headingTimer != nullptr) {
    lv_timer_pause(headingTimer);
  }

  lv_screen_load_anim(uiScreen, LV_SCR_LOAD_ANIM_MOVE_BOTTOM, 1000, 250, false);
}
void loadDiagScreen(lv_event_t *e) {
  static bool diagInitialized = false;

  if (!diagInitialized) {
    diagInitialized = true;

    backDiagButton = createLvglButton(
        diagScreen, "<", uiScreenloader, (screen_width / buttonCount), 60,
        LV_ALIGN_TOP_LEFT, (0 * (screen_width / buttonCount)), 0,
        LV_PALETTE_PURPLE, 0);

    headingButton = createLvglButton(
        diagScreen, "Heading", activateHeadingChart,
        (screen_width / buttonCount), 60, LV_ALIGN_TOP_LEFT,
        (1 * (screen_width / buttonCount)), 0, LV_PALETTE_PURPLE, 0);

    errorButton = createLvglButton(
        diagScreen, "Error", nullptr, (screen_width / buttonCount), 60,
        LV_ALIGN_TOP_LEFT, (2 * (screen_width / buttonCount)), 0,
        LV_PALETTE_PURPLE, 0);

    positionButton = createLvglButton(
        diagScreen, "Position", nullptr, (screen_width / buttonCount), 60,
        LV_ALIGN_TOP_LEFT, (3 * (screen_width / buttonCount)), 0,
        LV_PALETTE_PURPLE, 0);
  }
  lv_screen_load_anim(diagScreen, LV_SCR_LOAD_ANIM_MOVE_TOP, 1000, 250, false);
}

lv_obj_t *leftAutonButton;
lv_obj_t *rightAutonButton;
lv_obj_t *rightSoloAutonButton;
lv_obj_t *SkillsAutonButton;
lv_obj_t *backButton;
lv_obj_t *title;

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
  lv_screen_load_anim(autonScreen, LV_SCR_LOAD_ANIM_MOVE_TOP, 1000, 250, false);
}

void screeninit() {
  autonScreen = lv_obj_create(NULL);
  uiScreen = lv_obj_create(NULL);
  diagScreen = lv_obj_create(NULL);
  headingScreen = lv_obj_create(NULL);

  lv_color_t customColor = {60, 29, 40};

  // lv_obj_set_style_bg_grad_color(uiScreen, customColor, 0);

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
  lv_screen_load_anim(uiScreen, LV_SCR_LOAD_ANIM_OUT_TOP, 1750, 250, false);
}
