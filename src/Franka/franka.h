#pragma once

#include <Core/array.h>
#include <Core/thread.h>
#include <Control/ctrlMsg.h>
#include <Control/CtrlMsgs.h>


struct FrankaThreadNew : rai::RobotAbstraction, Thread{
  FrankaThreadNew(uint whichRobot=0, const uintA& _qIndices={0, 1, 2, 3, 4, 5, 6});
  ~FrankaThreadNew();

private:
  bool stop=false, requiresInitialization=true; //stop -> send end to libfranka; requiresInitialization -> waits in constructor until first contact/initialization
  arr Kp_freq, Kd_ratio; //read from rai.cfg
  const char* ipAddress;

  uintA qIndices;
  uint qIndices_max=0;

  uint steps=0;
  ofstream dataFile;
  double ctrlTime=0.;

  void step();
};
