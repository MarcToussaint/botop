#pragma once

#include <Core/array.h>
#include <Core/thread.h>
#include <Control/ctrlMsg.h>
#include <Control/CtrlMsgs.h>


struct AllegroThread : rai::RobotAbstraction, Thread{
  AllegroThread(Var<rai::CtrlCmdMsg>& cmd, Var<rai::CtrlStateMsg>& state);
  ~AllegroThread();

private:
  str canName;
  arr Kp, Kd; //read from rai.cfg
  double t_prev;
  arr q_real, q_real_prev, q_filtered, q_filtered_prev, q_vel;

  uint steps=0;
  double ctrlTime=0.;

  void step();

  bool OpenCAN(const char*);
  void CloseCAN();
  void ioThreadProc();
  void publish_state_and_compute_torques(double* _q, double* _tau_des);

};
