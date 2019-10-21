#pragma once

#include <Core/util.h>
#include <Core/thread.h>

namespace franka{
  class Gripper;
}

struct FrankaGripper : Thread{
  ptr<franka::Gripper> gripper;
  double maxWidth;

  FrankaGripper(uint whichRobot=0);

  void homing();

  bool open(double width=.075, //which is 7.5cm
            double speed=.2);

  bool close(double force=20,  //which is 1kg
             double width=.05, //which is 5cm
             double speed=.1);

  double pos();

  bool isGrasped();
};
