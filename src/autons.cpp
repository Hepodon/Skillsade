#include "main.h"
#include "portDef.hpp"
#include "pros/rtos.hpp"
#include "score.hpp"

void rightAuton(lv_event_t *e) {
  userInput.clear_line(1);
  pros::delay(100);
  userInput.print(0, 0, "Right Auton");
}

void leftAuton(lv_event_t *e) {
  userInput.clear_line(1);
  pros::delay(100);
  userInput.print(0, 0, "Left Auton");
}

void soloRightAuton(lv_event_t *e) {
  userInput.clear_line(1);
  pros::delay(100);
  userInput.print(0, 0, "Solo Auton");
}

void skillsAuton(lv_event_t *e) {
  userInput.clear_line(1);
  pros::delay(100);
  userInput.print(0, 0, "Skills Auton");
}