#pragma once

#include "lgpop.h"

struct RyLGPop{
  std::shared_ptr<LGPop> self;

  RyLGPop(bool _simulationMode=true){
    self = make_shared<LGPop>(_simulationMode);
  }

  void runRobotControllers(bool simuMode=false);
  void runTaskController(int verbose=0);
  void runCamera(int verbose=0);
  void runPerception(int verbose=0);
  void runCalibration();

  void reportCycleTimes();

  void updateArmPoseCalibInModel();

  void sim_addRandomBox(const char* name="randomBox");
};
