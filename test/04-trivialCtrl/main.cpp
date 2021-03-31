//#include <NewControl/TaskControlThread.h>
//#include <LGPop/lgpop.h>
#include <Kin/kinViewer.h>

#include <Gui/viewer.h>

#include <Franka/controlEmulator.h>

#include <Franka/franka.h>
#include <Franka/gripper.h>
#include <Franka/help.h>

#include <Control/control.h>
#include <Control/CtrlThread.h>
#include <Control/splineRunner.h>

void testUnrollControl(uint T, rai::Configuration& C, CtrlSet& CS, CtrlSolver& ctrl){
  arr q=C.getJointState();
  arr qDot = zeros(q.N);
  for(uint t=0;t<T;t++){
    ctrl.set(CS);
    ctrl.update(q,qDot,C);
    arr q_new = ctrl.solve();
    qDot = (q_new-q)/ctrl.tau;
    q = q_new;
    C.setJointState(q);
    C.stepSwift();

    ctrl.report();
    C.watch(false, STRING("t:" <<t));
    rai::wait(.01);
//    if(pos->status>AS_running) break;
    if(!CS.canBeInitiated(ctrl.komo.pathConfig)){
      cout <<"*** IS INFEASIBLE ***" <<endl;
      rai::wait();
    }
    if(CS.isConverged(ctrl.komo.pathConfig)){
      cout <<"*** IS CONVERGED ***" <<endl;
      rai::wait();
      break;
    }
  }
}

struct TrivialZeroControl : ControlLoop {
  virtual void stepReference(arr& qRef, arr& qDotRef, arr& qDDotRef, const arr& q_real, const arr& qDot_real){
    qRef = q_real;
    qDotRef.resize(qRef.N).setZero();
    qDDotRef.resize(qRef.N).setZero();
  }
};

struct SplineControlLoop : ControlLoop {
  Var<rai::SplineRunner> sp;
  double startTime, lastTime;
  SplineControlLoop(const arr& x, const arr& t, const arr& x0) {
    sp.set()->set(x, t, x0, false);
  }

  virtual void initialize(const arr& q_real, const arr& qDot_real) {
    startTime = rai::realTime();
    lastTime = startTime;
  }

  virtual void stepReference(arr& qRef, arr& qDotRef, arr& qDDotRef, const arr& q_real, const arr& qDot_real){
    double now = rai::realTime();

    qRef = sp.set()->run(now-lastTime, qDotRef);
    qDDotRef.resize(qRef.N).setZero();
    lastTime = now;
  }
};

struct ClassicCtrlSetController : ControlLoop {
  CtrlSet CS;
  CtrlSolver ctrl;

  ClassicCtrlSetController(const rai::Configuration& C, double tau=.01, uint k_order=2)
    : ctrl(C, tau, k_order){
    //control costs
    CS.add_qControlObjective(2, 1e-2*sqrt(tau), ctrl.komo.world);
    CS.add_qControlObjective(1, 1e-1*sqrt(tau), ctrl.komo.world);

    //position carrot (is transient!)
    auto pos = CS.addObjective(make_feature(FS_positionDiff, {"endeffR", "target"}, ctrl.komo.world, {1e0}), OT_eq, .02);

    //collision constraint
    //CS.addObjective(make_feature<F_AccumulatedCollisions>({"ALL"}, C, {1e2}), OT_eq);
  }

  virtual void stepReference(arr& qRef, arr& qDotRef, arr& qDDotRef, const arr& q_real, const arr& qDot_real){
    ctrl.set(CS);
    ctrl.komo.world.setJointState(q_real);
    ctrl.komo.world.watch();
    ctrl.update(q_real, qDot_real, ctrl.komo.world);
    qRef = ctrl.solve();
    qDotRef.resize(qRef.N).setZero();
    qDotRef = .5*(qRef - q_real)/ctrl.tau;
    qDDotRef.resize(qRef.N).setZero();
//    cout <<"C q_real: " <<q_real.modRaw() <<" q_ref: " <<qRef.modRaw() <<endl;
  }
};





void testNew() {
  Var<rai::Configuration> C;
  {
    auto Cset = C.set();
    Cset->addFile(rai::raiPath("../model/pandaSingle.g"));
    Cset->getFrame("R_panda_finger_joint1")->setJoint(rai::JT_rigid);
    Cset->getFrame("R_panda_finger_joint2")->setJoint(rai::JT_rigid);
    Cset->addFrame("target") -> setPosition(Cset->getFrame("endeffR")->getPosition() + arr{0,.0,-.5});
    Cset->watch(true);
  }

//  {
//    ClassicCtrlSetController tmp(C.get());
//    testUnrollControl(1000, C.set(), tmp.CS, tmp.ctrl);
//  }

  Var<CtrlCmdMsg> ctrlRef;
  Var<CtrlStateMsg> ctrlState;
  {
    auto set = ctrlState.set();
    set->q = C.get()->getJointState();
    set->tauExternal.resize(C.get()->getJointStateDimension());
    set->tauExternal.setZero();
  }


  ControlEmulator robot(C, ctrlRef, ctrlState, {}, .001);
//  FrankaThreadNew robotR(ctrlRef, ctrlState, 0, franka_getJointIndices(K.get()(),'R'));

  arr q0 = C.get()->getJointState();
  arr qT = q0;
  qT(1) += 1.;
  auto sp = make_shared<SplineControlLoop>(cat(q0, qT).reshape(2,-1), arr{0., 2.}, q0);

  ControlThread mine(C, ctrlRef, ctrlState, sp);
//  ControlThread mine(C, ctrlRef, ctrlState, make_shared<ClassicCtrlSetController>(C.get()));
//  ControlThread mine(C, ctrlRef, ctrlState, make_shared<TrivialZeroControl>());

  KinViewer viewer(C, 0.05);

  rai::wait();
}



int main(int argc, char * argv[]){
  testNew();

  return 0;
}
