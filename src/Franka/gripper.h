#pragma once

#include <Core/util.h>

namespace franka{
  class Gripper;
}

struct FrankaGripper {
  ptr<franka::Gripper> gripper;
  double maxWidth;

  FrankaGripper();

  void homing();

  bool move(double width=.075, //which is 7.5cm
            double speed=.2);

  bool grasp(double force=10,  //which is 1kg
             double width=.05, //which is 5cm
             double speed=.1);

  double pos();

  bool isGrasped();
};
