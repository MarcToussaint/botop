#pragma once

#include <Core/thread.h>
#include <Control/CtrlMsgs.h>
#include <Kin/simulation.h>

struct BotSimulation : rai::RobotAbstraction, Thread {
  BotSimulation(const rai::Configuration& _sim_config,
                const Var<rai::CtrlCmdMsg>& _cmd={}, const Var<rai::CtrlStateMsg>& _state={},
                const StringA& joints={},
                double _tau=.001,
                double hyperSpeed=1.);

  ~BotSimulation();

private:
  rai::Configuration emuConfig;
  double tau;
  double ctrlTime = 0.;
  arr q_real, qDot_real;
  uintA q_indices;
  ofstream dataFile;
  FrameL collisionPairs;

  //two options: trivial double integrator model, or physical simulation
  std::shared_ptr<rai::Simulation> sim;
  arr noise;
  double noise_th, noise_sig;

  void step();
};

struct GripperSimulation : rai::GripperAbstraction, Thread{
  double q;

  GripperSimulation() : Thread("GripperSimulation"), q(.02) {}

  void calibrate() {}

  void open(double width=.075, //which is 7.5cm
            double speed=.2) { q=width; }

  void close(double force=10,  //which is 1kg
             double width=.05, //which is 5cm
             double speed=.1) { q=width; }

  double pos(){ return q; }
  bool isDone(){ return true; }
};
