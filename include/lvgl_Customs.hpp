#pragma once
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
#include <functional>

struct Chartseries {
  lv_obj_t *chart;
  lv_chart_series_t *headingSeries;
};
struct customChart {
  lv_obj_t *chart;
  lv_chart_series_t *series;
};


extern lv_obj_t *createLvglButton(lv_obj_t *parent, const char *text,
                                  lv_event_cb_t event_cb, int width, int height,
                                  lv_align_t align, int x_ofs, int y_ofs,
                                  lv_palette_t color = LV_PALETTE_DEEP_PURPLE,
                                  int radius = 20);
extern lv_obj_t *createLVGLText(lv_obj_t *parent, const char *text,
                                lv_align_t align, int x_ofs, int y_ofs);

extern lv_obj_t *createLVGLChart(Chartseries &stru, lv_obj_t *parent);
