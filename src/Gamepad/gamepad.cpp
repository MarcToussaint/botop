#include "gamepad.h"


#define RAI_PLIB
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
  count = 0;
  for(uint i=0;;i++) {
    struct jsJoystick* joystick = new jsJoystick(i);
    if(joystick->notWorking()) break;
    uint n=joystick->getNumAxes();

    std::cout << "name     = "   << joystick->getName()
              << "\n#axis    = " << joystick->getNumAxes()
              << "\n#buttons = " << joystick->getNumButtons()
              << "\nerror?   = " << joystick->notWorking()
              << "______________" << std::endl;

    if(n==2) {
      joysticks[count] = joystick;
      count++;
      if (count >= 2) break;
    } else {
      delete joystick;
    }
  }
  std::cout << "Found " << count << " gamepad(s)!" << std::endl;
  step(); //clear the buffers...
}

void GamepadInterface::close(){
  for (int i = 0; i < count; i++) {
    delete joysticks[i];
    gamepadState[i].set()->clear();
  }
}

void GamepadInterface::step(){
  arr pad_logs;
  pad_logs.resize(count).setZero();
  ctrlTime += .01;
  for (int i = 0; i < count; i++) {
    if(joysticks[i]->notWorking()) return;
    uint n=joysticks[i]->getNumAxes();
    floatA A(n);
    int B;
    joysticks[i]->rawRead(&B, A.p);
    gamepadState[i].writeAccess();
    gamepadState[i]().resize(n+1);
    gamepadState[i]()(0)=B;
    for(uint j=0; j<n; j++) gamepadState[i]()(j+1)=(double)A(j)/(1<<15);
    if(step_count>20 && stopButtons(gamepadState[i]())){
      LOG(1) <<"*** STOP button pressed";
  //    moduleShutdown()->incrementStatus();
      quitSignal.set()=true;
    }
    gamepadState[i].deAccess();
    pad_logs.elem(i) = getButtonPressed(i);
  }
  if(writeData>0){
    if(!dataFile.is_open()) dataFile.open(STRING("z.gamepad.dat"));
    dataFile <<ctrlTime <<' ';
    pad_logs.modRaw().write(dataFile);
    dataFile <<endl;
  }
}

int GamepadInterface::getButtonPressed(unsigned int controller_id){
  gamepadState[controller_id].readAccess();
  int value = -1;
  if(gamepadState[controller_id]().N) {
    value = gamepadState[controller_id].get()().elem(0);
  }
  gamepadState[controller_id].deAccess();
  return value;
}

#else //dummy implementations
GamepadInterface::GamepadInterface()
    : Thread("GamepadInterfaceINACTIVE") {}
void GamepadInterface::open(){ gamepadState.set()->resize(10); gamepadState.set()->setZero(); RAI_MSG("WARNING: dummy gamepad implementation"); }
void GamepadInterface::step(){ }
void GamepadInterface::close(){ }
#endif


