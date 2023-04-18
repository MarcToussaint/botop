#pragma once

#include <Core/thread.h>
#include <Control/CtrlMsgs.h>
#include <Kin/simulation.h>

struct BotThreadedSim : rai::RobotAbstraction, Thread {
  BotThreadedSim(const rai::Configuration& _sim_config,
                const Var<rai::CtrlCmdMsg>& _cmd={}, const Var<rai::CtrlStateMsg>& _state={},
                const StringA& joints={},
                double _tau=.001,
                double hyperSpeed=1.);

  ~BotThreadedSim();

  void pullDynamicStates(rai::Configuration& C);

private:
  rai::Configuration simConfig;
  double tau;
  double ctrlTime = 0.;
  arr q_real, qDot_real;
  uintA q_indices;
  ofstream dataFile;
  FrameL collisionPairs;

  //two options: trivial double integrator model, or physical simulation
protected:
  std::shared_ptr<rai::Simulation> sim;
  arr noise;
  double noise_th, noise_sig;

  void step();

  friend struct GripperSim;
};

struct GripperSim : rai::GripperAbstraction, Thread{
  std::shared_ptr<BotThreadedSim> sim;
  double q;
  bool isClosing=false, isOpening=false;

  GripperSim(const std::shared_ptr<BotThreadedSim>& _sim) : Thread("GripperSimulation"), sim(_sim), q(.02) {}

  //gripper virtual methods
  void calibrate() {}

  void open(double width=.075, //which is 7.5cm
            double speed=.2);

  void close(double force=10,  //which is 1kg
             double width=.05, //which is 5cm
             double speed=.1);
  void closeGrasp(const char* objName, double force=.0, double width=.2, double speed=.2);

  double pos(){ return q; }
  bool isDone();

};
