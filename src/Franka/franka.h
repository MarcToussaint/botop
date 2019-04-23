#pragma once

#include <Core/array.h>
#include <Core/thread.h>
#include <Control/ctrlMsg.h>
#include <NewControl/ctrlMsgs.h>

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
  Var<CtrlCmdMsg> ctrl;
  Var<CtrlStateMsg> ctrl_state;
  bool stop=false, firstTime=true;
  arr Kp_freq, Kd_ratio;
  const char* ipAddress;

  uintA qIndices;
  uint qIndices_max=0;

  uint steps=0;

  FrankaThreadNew(Var<CtrlCmdMsg>& _ctrl, Var<CtrlStateMsg>& _state, uint whichRobot=0, const uintA& _qIndices={0, 1, 2, 3, 4, 5, 6});
  ~FrankaThreadNew();

private:
  void step();
};
