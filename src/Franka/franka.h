#pragma once

#include <Core/array.h>
#include <Core/thread.h>
#include <Control/ctrlMsg.h>
#include <Control/CtrlMsgs.h>


struct FrankaThread : rai::RobotAbstraction, Thread{
  FrankaThread(uint robotID=0, const uintA& _qIndices={0, 1, 2, 3, 4, 5, 6}) : Thread("FrankaThread"){ init(robotID, _qIndices); }
  FrankaThread(uint robotID, const uintA& _qIndices, const Var<rai::CtrlCmdMsg>& _cmd, const Var<rai::CtrlStateMsg>& _state) : RobotAbstraction(_cmd, _state), Thread("FrankaThread"){ init(robotID, _qIndices); }
  ~FrankaThread();

private:
  bool stop=false; //send end to libfranka
  bool requiresInitialization=true;  //waits in constructor until first contact/initialization
  int robotID=0;
  arr Kp_freq, Kd_ratio; //read from rai.cfg
  arr friction;

  const char* ipAddress;

  uintA qIndices;
  uint qIndices_max=0;

  uint steps=0;
  ofstream dataFile;
  double ctrlTime=0.;

  void init(uint _robotID, const uintA& _qIndices);
  void step();
};
