#pragma once
#include "autons.hpp"
#include "liblvgl/display/lv_display.h"
#include "liblvgl/lvgl.h"
#include "portDef.hpp"
#include <cstring>

extern void screeninit();
extern void updateChart();
extern void screenshot(pros::Task task);