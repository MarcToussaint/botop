#pragma once

#include <Core/thread.h>
#include <NewControl/ctrlMsgs.h>
#include <Kin/kin.h>

struct ControlEmulator : Thread{
  Var<rai::KinematicWorld> sim_config;
  Var<CtrlCmdMsg> ctrl_ref;
  Var<CtrlStateMsg> ctrl_state;

  double tau;
  arr q,qdot;
  uintA q_indices;

  ControlEmulator(Var<rai::KinematicWorld>& _sim_config,
                  Var<CtrlCmdMsg>& _ctrl_ref,
                  Var<CtrlStateMsg>& _ctrl_state,
                  const StringA& joints={},
                  double _tau=.001);
  ~ControlEmulator();

private:
  void step();
};



struct GripperEmulator : Thread{
  double q;

  GripperEmulator() : Thread("GripperEmulator"), q(.02) {}

  void calibrate() {}

  bool open(double width=.075, //which is 7.5cm
            double speed=.2) { q=width;  return true; }

  bool close(double force=10,  //which is 1kg
             double width=.05, //which is 5cm
             double speed=.1) { q=width; return true; }

  double pos(){ return q; }
};
