#pragma once

#include <Core/array.h>
#include <Core/thread.h>
#include <Control/ctrlMsg.h>
#include <Control/CtrlMsgs.h>

struct OmnibaseController;

struct OmnibaseThread : rai::RobotAbstraction, Thread {
  OmnibaseThread(uint robotID, const uintA& _qIndices, const Var<rai::CtrlCmdMsg>& _cmd, const Var<rai::CtrlStateMsg>& _state);
  ~OmnibaseThread();

private:
  int robotID=0;
  double outer_Kp;
  int Kp, Ki, Kd, KdDelay, KiLimit, Friction; //read from rai.cfg
  double gear_ratio, center2wheel, wheel_radius //read from rai.cfg
  StringA addresses;

  uintA qIndices;
  uint qIndices_max=0;

  uint steps=0;
  ofstream dataFile;
  double ctrlTime=0.;

  std::shared_ptr<OmnibaseController> robot;


  void init(uint _robotID, const uintA& _qIndices);
  void open();
  void step();
  void close();
};
