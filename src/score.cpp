#include "score.hpp"
#include "main.h"
#include "portDef.hpp"

score_State::toggleState status = score_State::pause;

score_State::score_State() {}

// Stop motors
void score_State::cancel() {
  middle.brake();
  status = pause;
}
// Intake and store blocks
void score_State::store() {
  if (status != intaking) {
    status = intaking;
    tripstate.extend();
    tripstate2.retract();
    middle.move(127);
  } else {
    cancel();
  }
}

// Score blocks on long goals
void score_State::loadTop() {
  if (status != upper) {
    status = upper;
    tripstate.extend();
    tripstate2.extend();
    middle.move(127);
  } else {
    cancel();
  }
}

// Score blocks on central upper goal
void score_State::loadMiddle() {
  if (status != central_upper) {
    status = central_upper;
    tripstate.retract();
    tripstate2.retract();
    middle.move(127);
  } else {
    cancel();
  }
}

// score blocks on central lower goal
void score_State::loadBottom() {
  if (status != central_lower) {
    middle.move(-127);
    while (userInput.get_digital(DIGITAL_B)) {
      pros::delay(5);
    }
  } else {
    cancel();
  }
}