#include "graphics.hpp"
#include "autons.hpp"
#include "liblvgl/core/lv_obj.h"
#include "liblvgl/core/lv_obj_style_gen.h"
#include "liblvgl/display/lv_display.h"
#include "liblvgl/lv_conf_internal.h"
#include "liblvgl/lvgl.h"
#include "liblvgl/misc/lv_area.h"
#include "liblvgl/widgets/chart/lv_chart.h"
#include "portDef.hpp"
#include "pros/screen.hpp"
#include <cstring>

lv_obj_t *autonScreen;
lv_obj_t *uiScreen;
lv_obj_t *diagScreen;

lv_obj_t *createLvglButton(lv_obj_t *parent, const char *text,
                           lv_event_cb_t event_cb, int width, int height,
                           lv_align_t align, int x_ofs, int y_ofs,
                           lv_palette_t color = LV_PALETTE_BLUE) {
  lv_obj_t *button = lv_button_create(parent);

  lv_obj_set_size(button, width, height);
  lv_obj_align(button, align, x_ofs, y_ofs);
  lv_obj_set_style_radius(button, 20, 0);
  lv_obj_set_style_bg_color(button, lv_palette_main(color), 0);
  lv_obj_set_style_bg_color(button, lv_palette_darken(color, 3),
                            LV_STATE_PRESSED);
  if (text != nullptr) {
    lv_obj_t *label = lv_label_create(button);
    lv_label_set_text(label, text);
    lv_obj_center(label);
  }

  lv_obj_add_event_cb(button, event_cb, LV_EVENT_RELEASED, NULL);

  return button;
}

lv_obj_t *createLVGLText(lv_obj_t *parent, const char *text, lv_align_t align,
                         int x_ofs, int y_ofs) {
  lv_obj_t *textBox = lv_label_create(parent);

  lv_obj_align(textBox, align, x_ofs, y_ofs);
  lv_label_set_text(textBox, text);

  return textBox;
}

struct Chartseries {
  lv_obj_t *chart;
  lv_chart_series_t *headingSeries;
};

Chartseries hey;

lv_obj_t *createLVGLChart(Chartseries &stru) {
  lv_obj_t *chart = lv_chart_create(uiScreen);
  lv_chart_series_t *headingSeries;

  lv_chart_set_type(chart, LV_CHART_TYPE_LINE);

  lv_chart_set_range(chart, LV_CHART_AXIS_PRIMARY_Y, -180, 180);
  lv_chart_set_div_line_count(chart, 20, 180);

  lv_chart_set_update_mode(chart, LV_CHART_UPDATE_MODE_SHIFT);

  lv_chart_set_point_count(chart, 100);

  headingSeries = lv_chart_add_series(chart, lv_palette_main(LV_PALETTE_RED),
                                      LV_CHART_AXIS_PRIMARY_Y);

  stru = {chart, headingSeries};

  return chart;
}

void updateChart() {
  lv_chart_set_next_value(hey.chart, hey.headingSeries,
                          inertial1.get_heading());
  lv_chart_refresh(hey.chart);
  pros::delay(100);
}

void loadAutonScreen(lv_event_t *e) { lv_screen_load(autonScreen); }

void loadDiagScreen(lv_event_t *e) { lv_screen_load(diagScreen); }

void screeninit() {
  lv_obj_t *imuChart = createLVGLChart(hey);
  autonScreen = lv_obj_create(NULL);
  uiScreen = lv_obj_create(NULL);
  diagScreen = lv_obj_create(NULL);

  lv_obj_t *title = createLVGLText(uiScreen, "22204W", LV_ALIGN_TOP_MID, 0, 15);
  lv_obj_set_style_text_font(title, &lv_font_montserrat_16, 0);

  lv_obj_t *autonButton =
      createLvglButton(uiScreen, "Auton", loadAutonScreen, 105, 60,
                       LV_ALIGN_RIGHT_MID, -85, -10);

  lv_obj_t *diagButton =
      createLvglButton(uiScreen, "Diagnostics", loadDiagScreen, 105, 60,
                       LV_ALIGN_LEFT_MID, 85, -10);
}
