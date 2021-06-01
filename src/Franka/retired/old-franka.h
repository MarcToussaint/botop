#pragma once

#include <Core/array.h>
#include <Core/thread.h>
#include <Control/ctrlMsg.h>
#include <Control/CtrlMsgs.h>

//TODO: remove this old one:
struct FrankaThread :  Thread{
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
