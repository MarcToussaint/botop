//#include <NewControl/TaskControlThread.h>
//#include <LGPop/lgpop.h>
#include <Kin/kinViewer.h>

#include <Gui/viewer.h>

#include <Franka/controlEmulator.h>

#include <Franka/franka.h>
#include <Franka/gripper.h>
#include <Franka/help.h>

#include <Control/CtrlMsgs.h>

#if 0

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

struct  SplineCtrlReference {
  Var<rai::Spline> spline;
  double startTime, lastTime;
  SplineCtrlReference(const arr& x, const arr& t) {
    spline.set()->set(2, x, t);
  }

  void initialize(const arr& q_real, const arr& qDot_real) {
    startTime = rai::realTime();
  }

  void eval(arr& qRef, arr& qDotRef, arr& qDDotRef, double time){
    auto splineGet = spline.get();
    qRef = splineGet->eval(time-startTime);
    if(!!qDotRef) qDotRef = splineGet->eval(time-startTime, 1);
    if(!!qDDotRef) qDDotRef = splineGet->eval(time-startTime, 2);
  }

  rai::ReferenceFunction getRef(){
    return std::bind(&SplineCtrlReference::eval, this,
                     std::placeholders::_1,
                     std::placeholders::_2,
                     std::placeholders::_3,
                     std::placeholders::_4);
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

  virtual void step(rai::CtrlCmdMsg& ctrlCmdMsg, const arr& q_real, const arr& qDot_real, double time){
    ctrlCmdMsg.controlType=rai::ControlType::configRefs;
    ctrlCmdMsg.ref = std::bind(&SplineControlLoop::eval, this,
                               std::placeholders::_1,
                               std::placeholders::_2,
                               std::placeholders::_3,
                               std::placeholders::_4);
  }

  virtual void eval(arr& qRef, arr& qDotRef, arr& qDDotRef, double time){
    qRef = sp.set()->run(time-lastTime, qDotRef);
    qDDotRef.resize(qRef.N).setZero();
    lastTime = time;
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

#endif


struct ZeroReference : rai::ReferenceFeed {
  virtual void getReference(arr& q_ref, arr& qDot_ref, arr& qDDot_ref, const arr& q_real, const arr& qDot_real, double time){
     q_ref = q_real;
     qDot_ref.resize(q_ref.N).setZero();
     qDDot_ref.resize(q_ref.N).setZero();
  }
};


void testNew() {
  //-- setup a configuration
  rai::Configuration C;
  C.addFile(rai::raiPath("../model/pandaSingle.g"));
  C.getFrame("R_panda_finger_joint1")->setJoint(rai::JT_rigid);
  C.getFrame("R_panda_finger_joint2")->setJoint(rai::JT_rigid);
  C.addFrame("target") -> setPosition(C["endeffR"]->getPosition() + arr{0,.0,-.5});
  C.watch(true);

  C.ensure_indexedJoints();
  FrankaThreadNew robot(0, franka_getJointIndices(C,'R'));
  robot.writeData = true;

  robot.cmd.set()->ref = make_shared<ZeroReference>();

  for(;;){
    if(C.watch(false,STRING("time: "<<rai::realTime()))=='q') break;
    C.setJointState(robot.state.get()->q);
    rai::wait(.1);
  }

  cout <<"bye bye" <<endl;
}



int main(int argc, char * argv[]){
  rai::initCmdLine(argc, argv);

  testNew();

  return 0;
}
