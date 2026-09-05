#pragma once

#include <Core/thread.h>
#include <Control/CtrlMsgs.h>
#include <Kin/simulation.h>
#include <Kin/frame.h>

struct BotThreadedSim : rai::RobotAbstraction, rai::Thread {
  BotThreadedSim(const rai::Configuration& _sim_config,
                 rai::Var<rai::CtrlCmdMsg>& cmd, rai::Var<rai::CtrlStateMsg>& state,
                 const StringA& joints={},
                 double _tau=-1,
                 double hyperSpeed=-1.);

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

  void step();

  void attach(str from, str to){
    CHECK(sim, "");
    sim->attach(simConfig.getFrame(from), simConfig.getFrame(to));
  }
  void detach(str from, str to){
    CHECK(sim, "");
    sim->detach(simConfig.getFrame(from), simConfig.getFrame(to));
  }

  friend struct BotOp;
  friend struct GripperSim;
  friend struct CameraSimThread;
};

struct GripperSim : rai::GripperAbstraction, rai::Thread{
  std::shared_ptr<BotThreadedSim> simthread;
  rai::String gripperName;
  double q;
  bool isClosing=false, isOpening=false;

  GripperSim(const std::shared_ptr<BotThreadedSim>& _simthread, const char* _gripperName)
    : rai::Thread("GripperSimulation"), simthread(_simthread), gripperName(_gripperName), q(.02) {}

  //gripper virtual methods
  void calibrate() {}

  void open(double width=.075, //which is 7.5cm
            double speed=.2);

  void close(double force=10,  //which is 1kg
             double width=.05, //which is 5cm
             double speed=.1);
  void closeGrasp(const char* objName, double force=.0, double width=.2, double speed=.2);

  double pos();
  bool isDone();

};

struct CameraSimThread : rai::CameraAbstraction, rai::Thread {
  std::shared_ptr<BotThreadedSim> simthread;

  CameraSimThread(const std::shared_ptr<BotThreadedSim>& _sim, rai::Frame *f_cam)
      : Thread(f_cam->name, .05), simthread(_sim) {
    {
      auto mux = simthread->stepMutex(RAI_HERE);
      camera_name = f_cam->name;
      simthread->sim->setCamera(f_cam);
    }
    LOG(0) <<"launching camera " <<camera_name;
    threadLoop();
    image.waitForRevisionGreaterThan(0);
  }
  ~CameraSimThread(){
    LOG(0) <<"shutting down camera " <<camera_name;
    threadClose();
  }

  void step() {
    auto mux = simthread->stepMutex(RAI_HERE);
    auto f_cam = simthread->sim->C.getFrame(camera_name);
    simthread->sim->setCamera(f_cam);
    simthread->sim->getImageAndDepth(image.set(), depth.set());
  }

  virtual arr getFxycxy(){
    auto mux = simthread->stepMutex(RAI_HERE);
    auto f_cam = simthread->sim->C.getFrame(camera_name);
    simthread->sim->setCamera(f_cam);
    return simthread->sim->cameraview().currentCamera->cam.getFxycxy();
  }
  virtual rai::Transformation getPose(){
    auto mux = simthread->stepMutex(RAI_HERE);
    auto f_cam = simthread->sim->C.getFrame(camera_name);
    simthread->sim->setCamera(f_cam);
    return simthread->sim->cameraview().currentCamera->frame.ensure_X();
  }
};
