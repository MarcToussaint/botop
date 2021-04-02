#pragma once

#include <Core/array.h>
#include <Core/thread.h>
#include <Control/ctrlMsg.h>
#include <Control/CtrlMsgs.h>

//TODO: remove this old one:
struct FrankaThread : Thread{
  Var<CtrlMsg> ctrl;
  Var<CtrlMsg> ctrl_state;
  bool stop=false, firstTime=true;
  arr Kp_freq, Kd_ratio;
  const char* ipAddress;
  uintA qIndices;
  uint qIndices_max=0;
  uint steps=0;

  FrankaThread(Var<CtrlMsg>& _ctrl, Var<CtrlMsg>& _state, uint whichRobot=0, const uintA& _qIndices={});
  ~FrankaThread();

private:
  void step();
};


struct FrankaThreadNew : Thread{
  // input/output messages
  Var<rai::CtrlCmdMsg> ctrlCmd;
  Var<rai::CtrlStateMsg> ctrlState;

  bool writeData=false;

  FrankaThreadNew(Var<rai::CtrlCmdMsg>& _ctrl, Var<rai::CtrlStateMsg>& _state, uint whichRobot=0, const uintA& _qIndices={0, 1, 2, 3, 4, 5, 6});
  ~FrankaThreadNew();

private:
  bool stop=false, requiresInitialization=true; //stop -> send end to libfranka; requiresInitialization -> waits in constructor until first contact/initialization
  arr Kp_freq, Kd_ratio; //read from rai.cfg
  const char* ipAddress;

  uintA qIndices;
  uint qIndices_max=0;

  uint steps=0;
  ofstream dataFile;

  void step();
};
