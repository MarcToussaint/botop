#include "gamepad.h"

#ifdef RAI_PLIB

#include <plib/js.h>
#undef min
#undef max

GamepadInterface::GamepadInterface()
    : Thread("GamepadInterface", .01) {
    threadLoop(true);
}

GamepadInterface::~GamepadInterface(){
    threadClose();
}

void GamepadInterface::open(){
  jsInit();
  for(uint i=0;;i++){
    joystick = new jsJoystick(i);
    if(joystick->notWorking()) break;
    uint n=joystick->getNumAxes();

    std::cout << "name     = "   << joystick->getName()
              << "\n#axis    = " << joystick->getNumAxes()
              // << " \n#buttons = " <<gamepad->getNumButtons()
              << "\nerror?   = " <<joystick->notWorking()
              << std::endl;

    if(n>=4 && n<=8) break;
    //iterate and try a new one
    delete joystick;
  }
  step(); //clear the buffers...
}

void GamepadInterface::close(){
  delete joystick;
  gamepadState.set()->clear();
}

void GamepadInterface::step(){
  if(joystick->notWorking()) return;
  uint n=joystick->getNumAxes();
  floatA A(n);
  int B;
  joystick->rawRead(&B, A.p);
  gamepadState.writeAccess();
  gamepadState().resize(n+1);
  gamepadState()(0)=B;
  for(uint i=0; i<n; i++) gamepadState()(i+1)=(double)A(i)/(1<<15);
  if(step_count>20 && stopButtons(gamepadState())){
    LOG(1) <<"*** STOP button pressed";
//    moduleShutdown()->incrementStatus();
    quitSignal.set()=true;
  }
  gamepadState.deAccess();

}

#else //dummy implementations
GamepadInterface::GamepadInterface()
    : Thread("GamepadInterfaceINACTIVE") {}
GamepadInterface::~GamepadInterface(){ }
void GamepadInterface::open(){ gamepadState.set()->resize(10); gamepadState.set()->setZero(); RAI_MSG("WARNING: dummy gamepad implementation"); }
void GamepadInterface::step(){ }
void GamepadInterface::close(){ }
#endif


