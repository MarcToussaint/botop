#pragma once

#include <Kin/kin.h>
#include <Control/CtrlMsgs.h>

//fwd declarations
namespace rai{
  struct GripperAbstraction;
  struct OptiTrack;
  struct ViveController;
  struct Sound;
}
struct BotThreadedSim;

//===========================================================================

struct BotOp{
  Var<rai::CtrlCmdMsg> cmd;
  Var<rai::CtrlStateMsg> state;
  //since each of the following interfaces is already pimpl, we don't have to hide them again
  std::shared_ptr<rai::RobotAbstraction> robotL;
  std::shared_ptr<rai::RobotAbstraction> robotR;
  std::shared_ptr<rai::GripperAbstraction> gripperL;
  std::shared_ptr<rai::GripperAbstraction> gripperR;
  std::shared_ptr<rai::ReferenceFeed> ref;
  std::shared_ptr<rai::OptiTrack> optitrack;
  std::shared_ptr<rai::ViveController> vivecontroller;
  std::shared_ptr<rai::Sound> audio;
  std::shared_ptr<BotThreadedSim> simthread;
  rai::Array<std::shared_ptr<rai::CameraAbstraction>> cameras;

  arr qHome;
  int keypressed=0;

  BotOp(rai::Configuration& C, bool useRealRobot);
  ~BotOp();

  //-- state info
  arr get_q();
  arr get_qDot();
  double get_t();
  void getState(arr& q_real, arr& qDot_real, double& ctrlTime);
  void getReference(arr& q_ref, arr& qDot_ref, arr& qDDot_ref, const arr& q_real, const arr& qDot_real, double ctrlTime);
  const arr& get_qHome(){ return qHome; }
  double getTimeToEnd(); //negative, if motion spline is done
  arr getEndPoint(); //negative, if motion spline is done
  arr get_tauExternal();
  int getKeyPressed(){ return keypressed; }

  //-- motion commands
  void move(const arr& path, const arr& times, bool overwrite=false, double overwriteCtrlTime=-1.);
  void move_oldCubic(const arr& path, const arr& times, bool overwrite=false, double overwriteCtrlTime=-1.);
  void moveAutoTimed(const arr& path, double maxVel=1., double maxAcc=1.); //double timeCost);
  void moveTo(const arr& q_target, double timeCost=1., bool overwrite=false);
  void setControllerWriteData(int _writeData);
  void setCompliance(const arr& J, double compliance=.5);

  //-- gripper commands - directly calling the gripper abstraction
  void gripperMove(rai::ArgWord leftRight, double width=.075, double speed=.2);
  void gripperClose(rai::ArgWord leftRight, double force=10, double width=.05, double speed=.1);
  void gripperCloseGrasp(rai::ArgWord leftRight, const char* objName, double force=10, double width=.05, double speed=.1);
  double getGripperPos(rai::ArgWord leftRight);
  bool gripperDone(rai::ArgWord leftRight);

  //-- camera commands
  void getImageAndDepth(byteA& image, floatA& depth, const char* sensor);
  void getImageDepthPcl(byteA& image, floatA& depth, arr& points, const char* sensor, bool globalCoordinates=false);
  arr  getCameraFxycxy(const char* sensor);

  //-- sync the user's C with the robot, update the display, return pressed key
  int sync(rai::Configuration& C, double waitTime=.1, rai::String viewMsg={});
  int wait(rai::Configuration& C, bool forKeyPressed=true, bool forTimeToEnd=true, bool forGripper=false);

  //-- motion macros
  void home(rai::Configuration& C);
  void stop(rai::Configuration& C);
  void hold(bool floating=true, bool damping=true);

  //-- audio
  void sound(int noteRelToC=0, float a=.5, float decay=0.0007);

private:
  std::shared_ptr<rai::CameraAbstraction>& getCamera(const char* sensor);
  template<class T> BotOp& setReference();
  std::shared_ptr<rai::BSplineCtrlReference> getSplineRef();
  double startRealTime;
};

//===========================================================================

struct ZeroReference : rai::ReferenceFeed {
  Var<arr> position_ref; ///< if set, defines a non-zero velocity reference
  Var<arr> velocity_ref; ///< if set, defines a non-zero velocity reference

  ZeroReference& setVelocityReference(const arr& _velocity_ref){ velocity_ref.set() = _velocity_ref; return *this; }
  ZeroReference& setPositionReference(const arr& _position_ref){ position_ref.set() = _position_ref; return *this; }

  /// callback called by a robot control loop
  virtual void getReference(arr& q_ref, arr& qDot_ref, arr& qDDot_ref, const arr& q_real, const arr& qDot_real, double ctrlTime);
};

//===========================================================================

template<class T> BotOp& BotOp::setReference(){
  //comment the next line to only get gravity compensation instead of 'zero reference following' (which includes damping)
  ref = make_shared<T>();
  cmd.set()->ref = ref;
//  ref->setPositionReference(q_now);
//ref->setVelocityReference({.0,.0,.2,0,0,0,0});
  return *this;
}

//===========================================================================

