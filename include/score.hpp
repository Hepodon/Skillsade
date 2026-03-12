#pragma once

#include "pros/misc.h"
class score_State {

public:
  enum teamColor { red, blue };

  enum toggleState { upper, central_lower, central_upper, pause, intaking };
  // Different scoring and intake states
  score_State();

  toggleState status = pause;

  // Intake and store blocks
  void store();

  // Score blocks on long goals
  void loadTop();

  // Score blocks on central upper goal
  void loadMiddle();

  // Score blocks on central upper goal
  void loadBottomHold(pros::controller_digital_e_t button);

  // score blocks on central lower goal
  void loadBottom();
  // Stop motors
  void cancel();
};