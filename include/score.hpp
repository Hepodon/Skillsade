#pragma once

class score_State {

public:
  enum teamColor { red, blue };

  enum toggleState {
    upper,
    central_lower,
    central_upper,
    central_upper_SLOW,
    pause,
    intaking
  };
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
  void loadMiddleSLOW();

  // score blocks on central lower goal
  void loadBottom();
  // Stop motors
  void cancel();
};