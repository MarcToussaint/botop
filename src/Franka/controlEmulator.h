#pragma once

#include <Core/thread.h>
#include <Control/CtrlMsgs.h>
#include <Kin/kin.h>

namespace rai{
struct GripperAbstraction {
  virtual void open(double width=1.,
                    double speed=1.) = 0;
  virtual void close(double force=.0, //relative to [min,max]
                     double width=.2, //relative to [min,max]
                     double speed=.2) = 0; //relative to [min,max]
  virtual double pos() = 0;
  virtual bool isDone() = 0;
};
}

struct ControlEmulator : rai::RobotAbstraction, Thread {
  ControlEmulator(const rai::Configuration& _sim_config,
                  const Var<rai::CtrlCmdMsg>& _cmd, const Var<rai::CtrlStateMsg>& _state,
                  const StringA& joints={},
                  double _tau=.001,
                  double hyperSpeed=1.
                  );

  ~ControlEmulator();

private:
  double tau;
  double ctrlTime = 0.;
  arr q_real, qDot_real;
  uintA q_indices;
  ofstream dataFile;
  rai::Configuration emuConfig;
  FrameL collisionPairs;

  void step();
};

struct GripperEmulator : rai::GripperAbstraction, Thread{
  double q;

  GripperEmulator() : Thread("GripperEmulator"), q(.02) {}

  void calibrate() {}

  void open(double width=.075, //which is 7.5cm
            double speed=.2) { q=width; }

  void close(double force=10,  //which is 1kg
             double width=.05, //which is 5cm
             double speed=.1) { q=width; }

  double pos(){ return q; }
  bool isDone(){ return true; }
};
