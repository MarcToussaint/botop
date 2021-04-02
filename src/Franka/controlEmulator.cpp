#include "controlEmulator.h"
#include <Kin/kin.h>
#include <Kin/frame.h>

void naturalGains(double& Kp, double& Kd, double decayTime, double dampingRatio);
void naturalGains(double& Kp, double& Kd, double decayTime, double dampingRatio) {
  CHECK(decayTime>0. && dampingRatio>=0., "this does not define proper gains!");
  double lambda = decayTime*dampingRatio/(-log(.1));
//  double lambda = decayTime/(-log(.1)); //assume the damping ratio always 1. -- just so that setting ratio to zero still gives a reasonable value
  double freq = 1./lambda;
  Kp = freq*freq;
  Kd = 2.*dampingRatio*freq;
}

ControlEmulator::ControlEmulator(const rai::Configuration& C,
                                 Var<rai::CtrlCmdMsg>& _ctrl_ref,
                                 Var<rai::CtrlStateMsg>& _ctrl_state,
                                 const StringA& joints,
                                 double _tau)
  : Thread("FrankaThread_Emulated", _tau),
    ctrl_ref(_ctrl_ref),
    ctrl_state(_ctrl_state),
    tau(_tau){
  //        rai::Configuration K(rai::raiPath("../rai-robotModels/panda/panda.g"));
  //        K["panda_finger_joint1"]->joint->makeRigid();

  {
    q = C.getJointState();
    qdot.resize(q.N).setZero();
    if(joints.N){
      q_indices.resize(joints.N);
      uint i=0;
      for(auto& s:joints){
        rai::Frame *f = C.getFrame(s);
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
//  {
//    sim_config.set()->setJointState(q);
//  }

  //-- get current ctrl
  arr q_ref, qdot_ref, qDDotRef, KpRef, KdRef, P_compliance; // TODO Kp, Kd, u_b and also read out the correct indices
  rai::ControlType controlType;
  {
    auto ctrl_refGet = ctrl_ref.get();

    controlType = ctrl_refGet->controlType;

    if(!ctrl_refGet->ref){
      q_ref = q;
      qdot_ref.resize(q.N).setZero();
    }else{
      //get the reference from the callback (e.g., sampling a spline reference)
      ctrl_refGet->ref->getReference(q_ref, qdot_ref, qDDotRef, q, qdot, rai::realTime());
    }

    KpRef = ctrl_refGet->Kp;
    KdRef = ctrl_refGet->Kd;
    P_compliance = ctrl_refGet->P_compliance;
  }


  arr qdd_des = zeros(7);

  //-- construct torques from control message depending on the control type

  if(controlType == rai::ControlType::configRefs) { // plain qRef, qDotRef references
    if(q_ref.N == 0) return;

    double k_p, k_d;
    naturalGains(k_p, k_d, .02, 1.);
    qdd_des = k_p * (q_ref - q) + k_d * (qdot_ref - qdot);
//    q = q_ref;
//    qdot = qdot_ref;

  } else if(controlType == rai::ControlType::projectedAcc) { // projected Kp, Kd and u_b term for projected operational space control
    qdd_des = qDDotRef - KpRef*q - KdRef*qdot;
  }

  //-- directly integrate
  q += .5 * tau * qdot;
  qdot += tau * qdd_des;
  q += .5 * tau * qdot;

  //-- data log?
  if(writeData){
    if(!dataFile.is_open()) dataFile.open("z.panda.dat");
    dataFile <<rai::realTime() <<' ';
    q.writeRaw(dataFile);
    q_ref.writeRaw(dataFile);
    dataFile <<endl;
  }
}
