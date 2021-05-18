#pragma once

#include <Kin/kin.h>
#include <KOMO/komo.h>
#include <Control/CtrlMsgs.h>
#include <Franka/franka.h>
#include <Franka/gripper.h>
#include <Franka/controlEmulator.h>

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

struct BotOp{
  std::unique_ptr<rai::RobotAbstraction> robot;
  std::unique_ptr<rai::GripperAbstraction> gripper;
  std::shared_ptr<rai::ReferenceFeed> ref;
  arr qHome;

  BotOp(rai::Configuration& C, bool sim);
  ~BotOp();

  template<class T> BotOp& setReference();

  bool step(rai::Configuration& C, double waitTime=.1);

  void move(const arr& path, double duration){
    arr times;
    if(path.d0>1){
      times = range(0., duration, path.d0-1);
      times += times(1);
    }else{
      times = {duration};
    }
    auto sp = std::dynamic_pointer_cast<rai::SplineCtrlReference>(ref);
    if(!sp){
      setReference<rai::SplineCtrlReference>();
      sp = std::dynamic_pointer_cast<rai::SplineCtrlReference>(ref);
      CHECK(sp, "this is not a spline reference!")
    }
    double ctrlTime = robot->state.get()->time;
    sp->append(path, times, ctrlTime, true);
  }

  void hold(bool floating=true){
    auto ref = std::dynamic_pointer_cast<rai::ZeroReference>(ref);
    if(!ref){
      setReference<ZeroReference>();
      ref = std::dynamic_pointer_cast<rai::ZeroReference>(ref);
      CHECK(ref, "this is not a spline reference!")
    }
    if(floating){
      ref->setVelocityReference({});
      ref->setPositionReference({});
      ref->
    }

  }


};

//===========================================================================

template<class T> BotOp& BotOp::setReference(){
  //comment the next line to only get gravity compensation instead of 'zero reference following' (which includes damping)
  ref = make_shared<T>();
  robot->cmd.set()->ref = ref;
//  ref->setPositionReference(q_now);
//ref->setVelocityReference({.0,.0,.2,0,0,0,0});
  return *this;
}

//===========================================================================

arr getLoopPath(rai::Configuration& C);
