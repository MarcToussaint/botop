#pragma once

#include <Core/array.h>
#include <Core/thread.h>
#include <Control/ctrlMsg.h>

struct FrankaThread : Thread{
  Var<CtrlMsg> ctrl;
  Var<CtrlMsg> state;
  bool stop=false;

  FrankaThread(Var<CtrlMsg>& _ctrl, Var<CtrlMsg>& _state);
  ~FrankaThread();

private:
  void step();
};
