#include <Control/TaskControlThread.h>
#include <LGPop/lgpop.h>

#include <Gui/viewer.h>

//===========================================================================

void online(){
  LGPop OP(LGPop::SimulationMode);

  //-- that's the "working" config,
  rai::KinematicWorld K(OP.rawModel);
  StringA joints = K.getJointNames();

//  OP.runRobotControllers();
  OP.runTaskController(1);
  OP.runCamera(0);
  OP.runPerception(1);

  rai::wait();
}

//===========================================================================

int main(int argc, char * argv[]){

  online();

  return 0;
}
