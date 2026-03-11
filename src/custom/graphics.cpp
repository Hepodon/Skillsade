#include "autons.hpp"
#include "liblvgl/core/lv_obj.h"
#include "liblvgl/core/lv_obj_style_gen.h"
#include "liblvgl/display/lv_display.h"
#include "liblvgl/lv_conf_internal.h"
#include "liblvgl/lvgl.h"
#include "liblvgl/misc/lv_area.h"
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

void loadAutonScreen(lv_event_t *e) { lv_screen_load(autonScreen); }

void loadDiagScreen(lv_event_t *e) { lv_screen_load(diagScreen); }

void screeninit() {
  autonScreen = lv_obj_create(NULL);
  uiScreen = lv_obj_create(NULL);
  diagScreen = lv_obj_create(NULL);

  lv_obj_t *title = createLVGLText(uiScreen, "22204W", LV_ALIGN_TOP_MID, 0, 15);
  lv_obj_set_style_text_font(title, &lv_font_montserrat_16, 0);

  lv_obj_t *autonButton = createLvglButton(
      uiScreen, "Auton", loadAutonScreen, 75, 60, LV_ALIGN_RIGHT_MID, -85, -10);

  lv_obj_t *diagButton =
      createLvglButton(uiScreen, "Diagnostics", loadDiagScreen, 75, 60,
                       LV_ALIGN_LEFT_MID, 85, -10);
}

void screeninitBACK(lv_event_t *e) {
  autonScreen = lv_obj_create(NULL);
  uiScreen = lv_obj_create(NULL);
  diagScreen = lv_obj_create(NULL);

  lv_obj_t *title = createLVGLText(uiScreen, "22204W", LV_ALIGN_TOP_MID, 0, 15);
  lv_obj_set_style_text_font(title, &lv_font_montserrat_16, 0);

  lv_obj_t *autonButton = createLvglButton(
      uiScreen, "Auton", loadAutonScreen, 75, 60, LV_ALIGN_RIGHT_MID, -85, -10);

  lv_obj_t *diagButton =
      createLvglButton(uiScreen, "Diagnostics", loadDiagScreen, 75, 60,
                       LV_ALIGN_LEFT_MID, 85, -10);
}
