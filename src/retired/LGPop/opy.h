#pragma once

#include "lgpop.h"

struct RyLGPop{
  std::shared_ptr<LGPop> self;

  RyLGPop(LGPop::OpMode opMode=LGPop::RealMode){
    self = make_shared<LGPop>(opMode);
  }

  void runRobotControllers(LGPop::OpMode opMode=LGPop::RealMode);
  void runTaskController(int verbose=0);
  void runCamera(int verbose=0);
  void runPerception(int verbose=0);
  void runCalibration();

  void reportCycleTimes();

  void updateArmPoseCalibInModel();

  void sim_addRandomBox(const char* name="randomBox");
};
