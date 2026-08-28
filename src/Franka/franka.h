#pragma once

#include <Core/array.h>
#include <Core/thread.h>
#include <Control/ctrlMsg.h>
#include <Control/CtrlMsgs.h>


struct FrankaThread : rai::RobotAbstraction, Thread{
  FrankaThread(Var<rai::CtrlCmdMsg>& cmd, Var<rai::CtrlStateMsg>& state, uint robotID, const char* _ipAddress, const uintA& _qIndices={0, 1, 2, 3, 4, 5, 6})
      : RobotAbstraction(cmd, state), Thread("FrankaThread"){ init(robotID, _ipAddress, _qIndices); }
  ~FrankaThread();

private:
  bool stop=false; //send end to libfranka
  bool requiresInitialization=true;  //waits in constructor until first contact/initialization
  int robotID=0;
  arr Kp_freq, Kd_ratio; //read from rai.cfg
  arr friction;

  str ipAddress;

  uintA qIndices;
  uint qIndices_max=0;

  uint steps=0;
  ofstream dataFile;
  double ctrlTime=0.;

  void init(uint _robotID, const char* _ipAddress, const uintA& _qIndices);
  void step();
};
