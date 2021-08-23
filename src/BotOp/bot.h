#pragma once

#include <Kin/kin.h>
#include <KOMO/komo.h>
#include <Control/CtrlMsgs.h>
#include <Franka/franka.h>
#include <Franka/gripper.h>
#include <Franka/controlEmulator.h>

//===========================================================================

struct BotOp{
  Var<rai::CtrlCmdMsg> cmd;
  Var<rai::CtrlStateMsg> state;
  std::unique_ptr<rai::RobotAbstraction> robotL;
  std::unique_ptr<rai::RobotAbstraction> robotR;
  std::unique_ptr<rai::GripperAbstraction> gripperL;
  std::unique_ptr<rai::GripperAbstraction> gripperR;
  std::shared_ptr<rai::ReferenceFeed> ref;
  arr qHome;
  int keypressed=0;

  BotOp(rai::Configuration& C, bool useRealRobot);
  ~BotOp();

  //-- state info
  arr get_q();
  arr get_qDot();
  double get_t();
  const arr& get_qHome(){ return qHome; }
  double getTimeToEnd(); //negative, if motion spline is done

  //-- motion commands
  void move(const arr& path, const arr& times);
  void moveAutoTimed(const arr& path, double timeCost);
  void moveOverride(const arr& path, const arr& times);
  double moveLeap(const arr& q_target, double timeCost=1.);
  void setControllerWriteData(int _writeData){ if(robotL) robotL->writeData=_writeData;  if(robotR) robotR->writeData=_writeData;  }

  //-- gripper commands - directly calling the gripper abstraction
  void gripperOpen(double width=.075, double speed=.2){ if(!gripperL) LOG(-1) <<"gripper disabled"; else gripperL->open(width, speed); }
  void gripperClose(double force=10, double width=.05, double speed=.1){ if(!gripperL) LOG(-1) <<"gripper disabled"; else gripperL->close(force, width, speed); }
  double gripperPos(){ if(!gripperL){ LOG(-1) <<"gripper disabled"; return 0.; } else return gripperL->pos(); }
  bool isDone() {if(!gripperL){ LOG(-1) <<"gripper disabled"; return false; } else return gripperL->isDone(); }

  //-- sync the user's C with the robot, update the display, return false if motion spline is done
  bool step(rai::Configuration& C, double waitTime=.1);

  //-- motion macros
  void home(rai::Configuration& C);
  void hold(bool floating=true, bool damping=true);

private:
  template<class T> BotOp& setReference();
  std::shared_ptr<rai::SplineCtrlReference> getSplineRef();
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

