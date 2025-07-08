#pragma once

#include <Core/array.h>
#include <Core/thread.h>
#include <Control/ctrlMsg.h>
#include <Control/CtrlMsgs.h>

struct RangerController;

struct RangerThread : rai::RobotAbstraction, Thread {
  RangerThread(uint robotID, const uintA& _qIndices, const Var<rai::CtrlCmdMsg>& _cmd, const Var<rai::CtrlStateMsg>& _state);
  ~RangerThread();

private:
  int robotID=0;

  // Read from rai.cfg
  arr Kp, Ki, Kd;
  rai::String address;

  uintA qIndices;
  uint qIndices_max=0;

  uint steps=0;
  ofstream dataFile;
  double ctrlTime=0.;

  std::shared_ptr<RangerController> robot;

  void init(uint _robotID, const uintA& _qIndices);
  void open();
  void step();
  void close();
};
