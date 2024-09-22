#ifndef RAI_gamepad_h
#define RAI_gamepad_h

#include <Core/array.h>
#include <Core/thread.h>


struct GamepadInterface : Thread {
  int count;
  struct jsJoystick* joysticks[2];
  Var<arr> gamepadState[2];
  Var<bool> quitSignal;
  GamepadInterface();
  ~GamepadInterface();
  void open();
  void step();
  void close();
  int getButtonPressed(unsigned int controller_id=0);
  // Logging
  int writeData = 0;
  double ctrlTime = 0.;
  ofstream dataFile;
};

/// The buttons on gamepad have the following values.
/// Note that you can use the binary operators & and | to check for multiple
/// button presses.
enum BUTTON {
  BTN_NONE = 0,
  BTN_X = 1,
  BTN_A = 2,
  BTN_B = 4,
  BTN_Y = 8,
  BTN_L = 16,
  BTN_R = 32,
  BTN_SELECT = 256,
  BTN_START = 512,
};

inline bool stopButtons(const arr& gamepadState){
  if(!gamepadState.N) return false;
  uint mode = uint(gamepadState(0));
  if(mode&BTN_START) return true;
  return false;
}

#endif
