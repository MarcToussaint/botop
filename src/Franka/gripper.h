#pragma once

#include <Core/util.h>
#include <Core/thread.h>

#include "controlEmulator.h"

namespace franka{
  class Gripper;
}

//The control message send to the robot
struct GripperCmdMsg {
    enum Command { _open, _close, _home };
    Command cmd;
    double force=20;  //which is 1kg
    double width=.05; //which is 5cm
    double speed=.1;
};

struct FrankaGripper : rai::GripperAbstraction, Thread{
  Var<GripperCmdMsg> cmd;
  double maxWidth;

  FrankaGripper(uint whichRobot=0);
  ~FrankaGripper(){ threadClose(); }

  void homing();

  void open(double width=.075, //which is 7.5cm
            double speed=.2);

  void close(double force=20,  //which is 1kg
             double width=.05, //which is 5cm
             double speed=.1);

  double pos();

  bool isGrasped();

  void step();

private:
  shared_ptr<franka::Gripper> gripper;
};
