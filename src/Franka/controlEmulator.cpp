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
                                 const StringA& joints,
                                 double _tau)
  : Thread("FrankaThread_Emulated", _tau),
    tau(_tau){
  //        rai::Configuration K(rai::raiPath("../rai-robotModels/panda/panda.g"));
  //        K["panda_finger_joint1"]->joint->makeRigid();

  {
    q_real = C.getJointState();
    qDot_real.resize(q_real.N).setZero();
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
      q_indices.setStraightPerm(q_real.N);
    }
  }
  state.set()->q = q_real;
  state.set()->qDot = qDot_real;
  threadLoop();
  state.waitForNextRevision(); //this is enough to ensure the ctrl loop is running
}

ControlEmulator::~ControlEmulator(){
  threadClose();
}

void ControlEmulator::step(){
  //-- publish state
  {
    arr tauExternal;
    tauExternal = zeros(q_real.N);
    auto stateSet = state.set();
    stateSet->q.resize(q_real.N).setZero();
    stateSet->qDot.resize(qDot_real.N).setZero();
    stateSet->tauExternal.resize(q_real.N).setZero();
    for(uint i:q_indices){
      stateSet->q(i) = q_real(i);
      stateSet->qDot(i) = qDot_real(i);
      stateSet->tauExternal(i) = tauExternal(i);
    }
  }

  //-- publish to sim_config
//  {
//    sim_config.set()->setJointState(q);
//  }

  //-- get current ctrl
  arr cmd_q_ref, cmd_qDot_ref, cmd_qDDot_ref, KpRef, KdRef, P_compliance; // TODO Kp, Kd, u_b and also read out the correct indices
  rai::ControlType controlType;
  {
    auto cmdGet = cmd.get();

    controlType = cmdGet->controlType;

    if(!cmdGet->ref){
      cmd_q_ref = q_real;
      cmd_qDot_ref.resize(q_real.N).setZero();
    }else{
      //get the reference from the callback (e.g., sampling a spline reference)
      cmdGet->ref->getReference(cmd_q_ref, cmd_qDot_ref, cmd_qDDot_ref, q_real, qDot_real, rai::realTime());
    }

    KpRef = cmdGet->Kp;
    KdRef = cmdGet->Kd;
    P_compliance = cmdGet->P_compliance;
  }


  arr qDDot_des = zeros(7);

  //-- construct torques from control message depending on the control type

  if(controlType == rai::ControlType::configRefs) { // plain qRef, qDotRef references
    if(cmd_q_ref.N == 0) return;

    double k_p, k_d;
    naturalGains(k_p, k_d, .02, 1.);
    qDDot_des = k_p * (cmd_q_ref - q_real) + k_d * (cmd_qDot_ref - qDot_real);
//    q = q_ref;
//    qdot = qdot_ref;

  } else if(controlType == rai::ControlType::projectedAcc) { // projected Kp, Kd and u_b term for projected operational space control
    qDDot_des = cmd_qDDot_ref - KpRef*q_real - KdRef*qDot_real;
  }

  //-- directly integrate
  q_real += .5 * tau * qDot_real;
  qDot_real += tau * qDDot_des;
  q_real += .5 * tau * qDot_real;

  //-- data log?
  if(writeData){
    if(!dataFile.is_open()) dataFile.open("z.panda.dat");
    dataFile <<rai::realTime() <<' ';
    q_real.writeRaw(dataFile);
    cmd_q_ref.writeRaw(dataFile);
    dataFile <<endl;
  }
}
