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
  void getReference(arr& q_ref, arr& qDot_ref, arr& qDDot_ref, const arr& q_real, const arr& qDot_real, double time){
    q_ref = q_real;
    qDot_ref.resize(q_ref.N).setZero();
    qDDot_ref.resize(q_ref.N).setZero();
  }
};

struct SplineCtrlReference : rai::ReferenceFeed {
  Var<rai::Spline> spline;

  void initialize(const arr& q_real, const arr& qDot_real) {
    spline.set()->set(2, ~q_real, {rai::realTime()});
  }

  void getReference(arr& q_ref, arr& qDot_ref, arr& qDDot_ref, const arr& q_real, const arr& qDot_real, double time){
    if(!spline.get()->points.N) initialize(q_real, qDot_real);
    spline.get() -> eval(q_ref, qDot_ref, qDDot_ref, time);
  }

  void waitForInitialized(){
    while(!spline.get()->times.N) spline.waitForNextRevision();
  }

  void append(const arr& x, const arr& t, bool prependLast=true){
    waitForInitialized();
    double now = rai::realTime();
    arr _x(x), _t(t);
    auto splineSet = spline.set();
    if(prependLast){
      _x.prepend(splineSet->points[-1]);
      _t.prepend(0.);
    }
    if(now > splineSet->end()){ //previous spline is done... create new one
      splineSet->set(2, _x, _t+now);
    }else{ //previous spline still active... append
      splineSet->append(_x, _t);
    }
  }

  void override(const arr& x, const arr& t){
    CHECK(t.first()>.1, "that's too harsh!");
    waitForInitialized();
    double now = rai::realTime();
    arr x_now, xDot_now;
    arr _x(x), _t(t);
    auto splineSet = spline.set();
    splineSet->eval(x_now, xDot_now, NoArr, now);
    _x.prepend(x_now);
    _t.prepend(0.);
    splineSet->set(2, _x, _t+now, xDot_now);
  }

  void moveTo(const arr& x, double t, bool append=true){
    if(append) this->append(~x, {t}, true);
    else  override(~x, {t});
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

  //TODO -> shouldn't be here...
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

  //-- create 2 simple reference configurations
  ctrlState.waitForRevisionGreaterThan(10);
  arr q0 = ctrlState.get()->q;
  arr qT = q0;
  qT(1) += .5;

  auto sp = make_shared<SplineCtrlReference>();
  ctrlRef.set()->ref = sp;

  //1st motion:
  sp->append(cat(qT, qT, q0).reshape(3,-1), arr{2., 2., 4.});

  for(;;){
    if(C.watch(false,STRING("time: "<<rai::realTime()))=='q') break;
    C.setJointState(ctrlState.get()->q);
    rai::wait(.1);
  }

  //2nd motion:
  sp->moveTo(qT, 1., false);
  cout <<"OVERRIDE AT t=" <<rai::realTime() <<endl;
  sp->moveTo(q0, 1.);

  for(;;){
    if(C.watch(false,STRING("time: "<<rai::realTime()))=='w') break;
    C.setJointState(ctrlState.get()->q);
    rai::wait(.1);
  }

}

//===========================================================================

int main(int argc, char * argv[]){
  testMoveTo();

  return 0;
}
