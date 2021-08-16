#include "bot.h"

#include <Franka/help.h>
#include <Kin/F_qFeatures.h>
#include <Kin/viewer.h>
#include <KOMO/pathTools.h>

//===========================================================================

BotOp::BotOp(rai::Configuration& C, bool useRealRobot){
  bool useGripper = rai::getParameter<bool>("botUseGripper", true);
  rai::String useArm = rai::getParameter<rai::String>("botUseArm", "left");

  C.ensure_indexedJoints();
  qHome = C.getJointState();
  state.set()->initZero(qHome.N);
  if(useRealRobot){
    if(useArm=="left"){
      robotL = make_unique<FrankaThreadNew>(0, franka_getJointIndices(C,'l'), cmd, state);
      if(useGripper) gripperL = make_unique<FrankaGripper>(0);
    }else if(useArm=="right"){
      robotR = make_unique<FrankaThreadNew>(1, franka_getJointIndices(C,'r'), cmd, state);
      if(useGripper) gripperR = make_unique<FrankaGripper>(1);
    }else if(useArm=="both"){
      robotL = make_unique<FrankaThreadNew>(0, franka_getJointIndices(C,'l'), cmd, state);
      robotR = make_unique<FrankaThreadNew>(1, franka_getJointIndices(C,'r'), cmd, state);
      if(useGripper){
        gripperL = make_unique<FrankaGripper>(0);
        gripperR = make_unique<FrankaGripper>(1);
      }
    }else{
      HALT("you need a botUseArm configuration (right, left, both)");
    }
  }else{
    robotL = make_unique<ControlEmulator>(C); //, StringA(), .001, 10.);
    if(useGripper) gripperL = make_unique<GripperEmulator>();
  }
  C.setJointState(get_q());
  C.watch(false, STRING("time: 0"));
}

BotOp::~BotOp(){
  robotL.release();
  robotR.release();
}

double BotOp::get_t(){
  return state.get()->time;
}

arr BotOp::get_q() {
  return state.get()->q;
}

arr BotOp::get_qDot() {
  return state.get()->qDot;
}

double BotOp::getTimeToEnd(){
  auto sp = std::dynamic_pointer_cast<rai::SplineCtrlReference>(ref);
  if(!sp){
    LOG(-1) <<"can't get timeToEnd for non-spline mode";
    return 0.;
  }
  double ctrlTime = get_t();
  return sp->getEndTime() - ctrlTime;
}

bool BotOp::step(rai::Configuration& C, double waitTime){
  C.setJointState(state.get()->q);
//  C.gl()->raiseWindow();
  double ctrlTime = state.get()->time;
  keypressed = C.watch(false,STRING("time: "<<ctrlTime <<"\n[q or ESC to ABORT]"));
  if(keypressed==13) return false;
  if(keypressed=='q' || keypressed==27) return false;
  auto sp = std::dynamic_pointer_cast<rai::SplineCtrlReference>(ref);
  if(sp && ctrlTime>sp->getEndTime()) return false;
  if(waitTime) rai::wait(waitTime);
  return true;
}

std::shared_ptr<rai::SplineCtrlReference> BotOp::getSplineRef(){
    auto sp = std::dynamic_pointer_cast<rai::SplineCtrlReference>(ref);
    if(!sp){
      setReference<rai::SplineCtrlReference>();
      sp = std::dynamic_pointer_cast<rai::SplineCtrlReference>(ref);
      CHECK(sp, "this is not a spline reference!")
    }
    return sp;
}

void BotOp::moveAutoTimed(const arr& path, double timeCost){
  double T = path.d0;
  CHECK_GE(T, 16, "this only works for smooth paths!")
  double accSOS = sumOfSqr(getAccelerations_centralDifference(path, 1.));
  double tau = sqrt( accSOS / (timeCost * T));
  arr times(T);
  for(uint t=0;t<T;t++) times(t) = tau*(t+1);
  double ctrlTime = state.get()->time;
  getSplineRef()->append(path, times, ctrlTime, true);
}

void BotOp::move(const arr& path, const arr& times){
  arr _times;
  if(path.d0==times.N){
    _times = times;  // all is good, nothing to do
  }else{
    CHECK_EQ(times.N,1, "");
    CHECK_GE(path.d0, 2, "");
    double duration = times.scalar();
    _times = range(0., duration, path.d0-1);
    _times += _times(1);
  }
  double ctrlTime = state.get()->time;
  getSplineRef()->append(path, _times, ctrlTime, true);
}

void BotOp::moveOverride(const arr& path, const arr& times){
  double ctrlTime = state.get()->time;
  getSplineRef()->overrideSmooth(path, times, ctrlTime);
}

double BotOp::moveLeap(const arr& q_target, double timeCost){
  arr q = get_q();
  arr qDot = get_qDot();
  double dist = length(q-q_target);
  double vel = scalarProduct(qDot, q_target-q)/dist;
  double T = (sqrt(6.*timeCost*dist+vel*vel) - vel)/timeCost;
//  move(~q_target, T);

  if(dist<1e-4 || T<.2) T=.2;
  auto sp = std::dynamic_pointer_cast<rai::SplineCtrlReference>(ref);
  if(sp){
      double ctrlTime = state.get()->time;
      sp->overrideSmooth(~q_target, {T}, ctrlTime);
  }
  else move(~q_target, {T});
  return T;
}

void BotOp::home(rai::Configuration& C){
    C.gl()->raiseWindow();
    arr q=get_q();
    if(maxDiff(q,qHome)>1e-3){
        moveLeap(qHome, 1.);
        while(step(C));
    }else{
        move(~qHome, {.1});
    }
}

void BotOp::hold(bool floating, bool damping){
  auto zref = std::dynamic_pointer_cast<ZeroReference>(ref);
  if(!zref){
    setReference<ZeroReference>();
    zref = std::dynamic_pointer_cast<ZeroReference>(ref);
    CHECK(zref, "this is not a spline reference!")
  }
  if(floating){
    zref->setPositionReference({});
    if(damping){
      zref->setVelocityReference({0.});
    }else{
      zref->setVelocityReference({});
    }
  }else{
    arr q = get_q();
    zref->setPositionReference(q);
    zref->setVelocityReference({0.});
  }
}

//===========================================================================

void ZeroReference::getReference(arr& q_ref, arr& qDot_ref, arr& qDDot_ref, const arr& q_real, const arr& qDot_real, double ctrlTime){
  {
    arr pos = position_ref.get()();
    if(pos.N) q_ref = pos;
    else q_ref = q_real;  //->no position gains at all
  }
  {
    arr vel = velocity_ref.get()();
    if(vel.N==1 && vel.scalar()==0.) qDot_ref.resize(qDot_real.N).setZero(); //[0] -> zero vel reference -> damping
    else if(vel.N) qDot_ref = vel;
    else qDot_ref = qDot_real;  //[] -> no damping at all!
  }
  qDDot_ref.resize(q_ref.N).setZero();
}

