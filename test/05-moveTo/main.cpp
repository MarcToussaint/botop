//#include <NewControl/TaskControlThread.h>
//#include <LGPop/lgpop.h>
#include <Kin/kinViewer.h>

#include <Gui/viewer.h>

#include <Franka/controlEmulator.h>

#include <Franka/franka.h>
#include <Franka/gripper.h>
#include <Franka/help.h>

#include <Algo/spline.h>

struct TrivialZeroReference : rai::ReferenceFeed {
  void getReference(arr& qRef, arr& qDotRef, arr& qDDotRef, const arr& q_real, const arr& qDot_real, double time){
    qRef = q_real;
    qDotRef.resize(qRef.N).setZero();
    qDDotRef.resize(qRef.N).setZero();
  }
};

struct SplineCtrlReference : rai::ReferenceFeed {
  Var<rai::Spline> spline;

  void initialize(const arr& q_real, const arr& qDot_real) {
    spline.set()->set(2, ~q_real, {rai::realTime()});
  }

  void getReference(arr& qRef, arr& qDotRef, arr& qDDotRef, const arr& q_real, const arr& qDot_real, double time){
    auto splineGet = spline.get();
    CHECK(splineGet->times.N, "spline was not initialized");
    qRef = splineGet->eval(time);
    if(!!qDotRef) qDotRef = splineGet->eval(time, 1);
    if(!!qDDotRef) qDDotRef = splineGet->eval(time, 2);
  }

  void set(const arr& x, const arr& t, bool append=true){
    auto splineSet = spline.set();
    double now = rai::realTime();
    if(now > splineSet->end()){ //previous spline is done... create new one
      splineSet->set(2, x, t+now);
    }else{ //previous spline still active... append
      splineSet->append(x, t);
    }
  }

  void moveTo(const arr& x, double t, bool append=true){
    arr x0 = spline.get()->points[-1];
    set(cat(x0, x).reshape(2,-1), {0., t});
  }

};

//===========================================================================


void testMoveTo() {
  //-- setup a configuration
  rai::Configuration C;
  C.addFile(rai::raiPath("../model/pandaSingle.g"));
  C.getFrame("R_panda_finger_joint1")->setJoint(rai::JT_rigid);
  C.getFrame("R_panda_finger_joint2")->setJoint(rai::JT_rigid);
  C.addFrame("target") -> setPosition(C["endeffR"]->getPosition() + arr{0,.0,-.5});
  C.watch(true);

  Var<rai::CtrlCmdMsg> ctrlRef;
  Var<rai::CtrlStateMsg> ctrlState;
  {
    auto set = ctrlState.set();
    set->q = C.getJointState();
    set->tauExternal.resize(C.getJointStateDimension());
    set->tauExternal.setZero();
  }

  ControlEmulator robot(C, ctrlRef, ctrlState, {}, .001);
//  FrankaThreadNew robot(ctrlRef, ctrlState, 0, franka_getJointIndices(C.get()(),'R'));
  robot.writeData = true;

  ctrlState.waitForRevisionGreaterThan(10);
  arr q0 = ctrlState.get()->q;
  arr qT = q0;
  qT(1) += .5;
  auto sp = make_shared<SplineCtrlReference>();

  sp->initialize(q0, {});
  sp->set(cat(q0, qT, qT, q0).reshape(4,-1), arr{0., 2., 2., 4.});

  ctrlRef.set()->ref = sp;

  //ctrlRef.set()->ref = make_shared<TrivialZeroReference>();

  for(;;){
    if(C.watch(false,STRING("time: "<<rai::realTime()))=='q') break;
    C.setJointState(ctrlState.get()->q);
    rai::wait(.1);
  }

  sp->moveTo(qT, 1.);
  sp->moveTo(q0, 1.);
  sp->moveTo(qT, 1.);
//  sp->append(arr{qT}.reshape(1,-1), {1.});

  for(;;){
    if(C.watch(false,STRING("time: "<<rai::realTime()))=='w') break;
    C.setJointState(ctrlState.get()->q);
    rai::wait(.1);
  }


  //  KinViewer viewer(mine.ctrl_config, 0.05);

  //  rai::wait();
}

//===========================================================================

int main(int argc, char * argv[]){
  testMoveTo();

  return 0;
}
