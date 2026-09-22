#pragma once

#include <Core/array.h>
#include <Core/thread.h>
#include <Control/CtrlMsgs.h>

namespace rai {

struct FrankaThread : RobotAbstraction, Thread{
  FrankaThread(Var<CtrlCmdMsg>& cmd, Var<CtrlStateMsg>& state, uint _robotID, const char* _ipAddress, uint _qIndex=0);
  ~FrankaThread();

private:
  bool stop=false; //send end to libfranka
  bool requiresInitialization=true;  //waits in constructor until first contact/initialization
  int robotID=0;
  arr Kp_freq, Kd_ratio; //read from rai.cfg
  arr friction;

  str ipAddress;

  uint qIndex=0;

  uint steps=0;
  ofstream dataFile;
  double ctrlTime=0.;

  void step();
};

} //namespace
