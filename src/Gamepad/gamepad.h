#ifndef RAI_gamepad_h
#define RAI_gamepad_h

#include <Core/array.h>
#include <Core/thread.h>

struct GamepadInterface : Thread {
  struct jsJoystick *joystick;
  Var<arr> gamepadState;
  Var<bool> quitSignal;
  GamepadInterface();
  ~GamepadInterface();
  void open();
  void step();
  void close();
};

/// The buttons on gamepad have the following values.
/// Note that you can use the binary operators & and | to check for multiple
/// button presses.
enum BUTTON {
  BTN_NONE = 0,
  BTN_A = 1,
  BTN_B = 2,
  BTN_X = 4,
  BTN_Y = 8,
  BTN_LB = 16,
  BTN_RB = 32,
  BTN_LT = 64,
  BTN_RT = 128,
  BTN_BACK = 256,
  BTN_START = 512,
  BTN_LSTICK = 1024,
  BTN_RSTICK = 2048,
};

inline bool stopButtons(const arr& gamepadState){
  if(!gamepadState.N) return false;
  uint mode = uint(gamepadState(0));
  if(mode&BTN_LB || mode&BTN_RB || mode&BTN_LT || mode&BTN_RT) return true;
  return false;
}

#ifdef  RAI_IMPLEMENTATION
#  include "gamepad.cpp"
#endif

#endif
