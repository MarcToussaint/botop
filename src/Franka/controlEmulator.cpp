#include "controlEmulator.h"
#include <Kin/kin.h>
#include <Kin/frame.h>

void naturalGains(double& Kp, double& Kd, double decayTime, double dampingRatio);

ControlEmulator::ControlEmulator(Var<rai::KinematicWorld>& _sim_config,
                                 Var<CtrlCmdMsg>& _ctrl_ref,
                                 Var<CtrlStateMsg>& _ctrl_state,
                                 const StringA& joints,
                                 double _tau)
  : Thread("FrankaThread", _tau),
    sim_config(_sim_config),
    ctrl_ref(_ctrl_ref),
    ctrl_state(_ctrl_state),
    tau(_tau){
  //        rai::KinematicWorld K(rai::raiPath("../rai-robotModels/panda/panda.g"));
  //        K["panda_finger_joint1"]->joint->makeRigid();

  {
    auto K = sim_config.set();
    q = K->getJointState();
    qdot.resize(q.N).setZero();
    if(joints.N){
      q_indices.resize(joints.N);
      uint i=0;
      for(auto& s:joints){
        rai::Frame *f = K->getFrameByName(s);
        CHECK(f, "frame '" <<s <<"' does not exist");
        CHECK(f->joint, "frame '" <<s <<"' is not a joint");
        CHECK(f->joint->qDim()==1, "joint '" <<s <<"' is not 1D");
        q_indices(i++) = f->joint->qIndex;
      }
      CHECK_EQ(i, joints.N, "");
    }else{
      q_indices.setStraightPerm(q.N);
    }
  }
  ctrl_state.set()->q = q;
  ctrl_state.set()->qDot = qdot;
  threadLoop();
  ctrl_state.waitForNextRevision(); //this is enough to ensure the ctrl loop is running
}

ControlEmulator::~ControlEmulator(){
  threadClose();
}

void ControlEmulator::step(){
  //-- publish state
  {
    arr tauExternal;
    tauExternal = zeros(q.N);
    auto stateset = ctrl_state.set();
    stateset->q.resize(q.N).setZero();
    stateset->qDot.resize(qdot.N).setZero();
    stateset->tauExternal.resize(q.N).setZero();
    for(uint i:q_indices){
      stateset->q(i) = q(i);
      stateset->qDot(i) = qdot(i);
      stateset->tauExternal(i) = tauExternal(i);
    }
  }

  //-- publish to sim_config
  {
    sim_config.set()->setJointState(q, qdot);
  }


  //-- get current ctrl
  arr q_ref, qdot_ref, qdd_des, P_compliance;
  {
    auto ref = ctrl_ref.get();
    q_ref = ref->qRef;
    qdot_ref = ref->qDotRef;
    P_compliance = ref->P_compliance;
    qdd_des = zeros(ref->qRef.N);
  }

  //check for correct ctrl otherwise do something...
  if(q_ref.N!=q.N || qdot_ref.N!=q.N){
    LOG(-1) <<"inconsistent ctrl message";
    return;
  }
  if(P_compliance.N){
    if(!(P_compliance.nd==2 && P_compliance.d0==q.N && P_compliance.d1==q.N)){
      LOG(-1) <<"inconsistent ctrl P_compliance message";
      P_compliance.clear();
    }
  }

  //-- compute desired torques
  double k_p, k_d;
  naturalGains(k_p, k_d, .2, 1.);
  if(q_ref.N==q.N){
    qdd_des += k_p * (q_ref - q) + k_d * (qdot_ref - qdot);
  }

  //-- directly integrate
  q += .5 * tau * qdot;
  qdot += tau * qdd_des;
  q += .5 * tau * qdot;
}
