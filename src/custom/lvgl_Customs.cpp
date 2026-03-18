#include "lvgl_Customs.hpp"
#include "autons.hpp"
#include "graphics.hpp"
#include "liblvgl/core/lv_obj.h"
#include "liblvgl/core/lv_obj_pos.h"
#include "liblvgl/core/lv_obj_style_gen.h"
#include "liblvgl/display/lv_display.h"
#include "liblvgl/lv_conf_internal.h"
#include "liblvgl/lvgl.h"
#include "liblvgl/misc/lv_area.h"
#include "liblvgl/misc/lv_palette.h"
#include "liblvgl/widgets/chart/lv_chart.h"
#include "portDef.hpp"
#include "pros/screen.hpp"
#include <cstring>

lv_obj_t *createLvglButton(lv_obj_t *parent, const char *text,
                           lv_event_cb_t event_cb, int width, int height,
                           lv_align_t align, int x_ofs, int y_ofs,
                           lv_palette_t color, int radius) {
  lv_obj_t *button = lv_button_create(parent);

  lv_obj_set_size(button, width, height);
  lv_obj_align(button, align, x_ofs, y_ofs);
  lv_obj_set_style_radius(button, radius, 0);
  lv_obj_set_style_bg_color(button, lv_palette_main(color), 0);
  lv_obj_set_style_bg_color(button, lv_palette_darken(color, 3),
                            LV_STATE_PRESSED);
  if (text != nullptr) {
    lv_obj_t *label = lv_label_create(button);
    lv_label_set_text(label, text);
    lv_obj_center(label);
  }

  lv_obj_set_style_border_color(button, lv_color_black(), 0);
  lv_obj_set_style_border_width(button, 2, 0);
  lv_obj_set_style_bg_opa(button, LV_OPA_50, 0);

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

lv_obj_t *createLVGLChart(Chartseries &stru, lv_obj_t *parent) {
  lv_obj_t *chart = lv_chart_create(parent);
  lv_chart_series_t *headingSeries;

  lv_chart_set_type(chart, LV_CHART_TYPE_LINE);

  lv_chart_set_range(chart, LV_CHART_AXIS_PRIMARY_Y, -180, 180);

  lv_chart_set_update_mode(chart, LV_CHART_UPDATE_MODE_SHIFT);

  lv_chart_set_point_count(chart, 100);

  headingSeries = lv_chart_add_series(chart, lv_palette_main(LV_PALETTE_RED),
                                      LV_CHART_AXIS_PRIMARY_Y);

  lv_obj_set_size(chart, 400, 200);
  lv_obj_align(chart, LV_ALIGN_CENTER, 0, 0);
  stru = {chart, headingSeries};

  return chart;
}
