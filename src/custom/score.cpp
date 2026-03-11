#include "score.hpp"
#include "portDef.hpp"

score_State::toggleState status = score_State::pause;

score_State::score_State() {}

// Stop motors
void score_State::cancel() {
  topOut.move(0);
  sorter.move(0);
  intake.move(0);
  status = pause;
}
// Intake and store blocks
void score_State::store() {
  if (status != intaking) {
    status = intaking;
    topOut.move(127);
    sorter.move(-127);
    intake.move(-127);
  } else {
    cancel();
  }
}

// Score blocks on long goals
void score_State::loadTop() {
  if (status != upper) {
    status = upper;
    topOut.move(-127);
    sorter.move(-127);
    intake.move(-127);
  } else {
    cancel();
  }
}

// Score blocks on central upper goal
void score_State::loadMiddle() {
  if (status != central_upper) {
    status = central_upper;
    topOut.move(127);
    sorter.move(127);
    intake.move(-127);
  } else {
    cancel();
  }
}

// Score blocks on central upper goal
void score_State::loadMiddleSLOW() {
  if (status != central_upper_SLOW) {
    topOut.move(60);
    sorter.move(60);
    intake.move(-60);
  } else {
    cancel();
  }
}

// score blocks on central lower goal
void score_State::loadBottom() {
  if (status != central_lower) {
    topOut.move(127);
    sorter.move(127);
    intake.move(127);
  } else {
    cancel();
  }
}