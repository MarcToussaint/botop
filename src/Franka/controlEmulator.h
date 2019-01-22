#pragma once

#include <Core/thread.h>
#include <Control/ctrlMsg.h>
#include <Kin/kin.h>

struct ControlEmulator : Thread{
  Var<CtrlMsg> ctrl;
  Var<CtrlMsg> state;

  double tau;
  arr q,qdot;
  uintA q_indices;

  ControlEmulator(Var<CtrlMsg>& _ctrl,
                  Var<CtrlMsg>& _state,
                  const rai::KinematicWorld& robotModel,
                  const StringA& joints={},
                  double _tau=.001);
  ~ControlEmulator();

private:
  void step();
};



struct GripperEmulator {
    double q;

    GripperEmulator() : q(.02) {}

    void calibrate() {}

  bool open(double width=.075, //which is 7.5cm
            double speed=.2) { q=width;  return true; }

  bool close(double force=10,  //which is 1kg
             double width=.05, //which is 5cm
             double speed=.1) { q=width; return true; }

  double pos(){ return q; }
};
