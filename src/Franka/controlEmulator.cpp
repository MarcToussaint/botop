#include "controlEmulator.h"
#include <Kin/kin.h>
#include <Kin/frame.h>
#include <Kin/F_collisions.h>
#include <Kin/viewer.h>

void naturalGains(double& Kp, double& Kd, double decayTime, double dampingRatio);

ControlEmulator::ControlEmulator(const rai::Configuration& C,
                                 const Var<rai::CtrlCmdMsg>& _cmd, const Var<rai::CtrlStateMsg>& _state,
                                 const StringA& joints,
                                 double _tau, double hyperSpeed)
  : RobotAbstraction(_cmd, _state),
    Thread("FrankaThread_Emulated", _tau/hyperSpeed),
    tau(_tau),
    emuConfig(C),
    noise_sig(.001){
  //        rai::Configuration K(rai::raiPath("../rai-robotModels/panda/panda.g"));
  //        K["panda_finger_joint1"]->joint->makeRigid();

  noise_th = rai::getParameter<double>("botemu/noise_th", -1.);

  {
    q_real = C.getJointState();
    qDot_real.resize(q_real.N).setZero();
    collisionPairs = emuConfig.getCollisionAllPairs();
    //    cout <<" CollisionPairs:" <<endl;
    //    for(uint i=0;i<collisionPairs.d0;i++) cout <<collisionPairs(i,0)->name <<'-' <<collisionPairs(i,1)->name <<endl;

    if(joints.N){
      q_indices.resize(joints.N);
      uint i=0;
      for(auto& s:joints){
        rai::Frame *f = C.getFrame(s);
        CHECK(f, "frame '" <<s <<"' does not exist");
        CHECK(f->joint, "frame '" <<s <<"' is not a joint");
        CHECK(f->joint->dim==1, "joint '" <<s <<"' is not 1D");
        q_indices(i++) = f->joint->qIndex;
      }
      CHECK_EQ(i, joints.N, "");
    }else{
      q_indices.setStraightPerm(q_real.N);
    }
  }
  state.set()->q = q_real;
  state.set()->qDot = qDot_real;
  //emuConfig.watch(false, STRING("EMULATION - initialization"));
  //emuConfig.gl()->update(0, true);
  threadLoop();
  state.waitForNextRevision(); //this is enough to ensure the ctrl loop is running
}

ControlEmulator::~ControlEmulator(){
  emuConfig.glClose();
  threadClose();
}

void ControlEmulator::step(){
  //-- get real time
  ctrlTime += tau;
//  ctrlTime = rai::realTime();

  //-- publish state
  {
    arr tauExternal;
    tauExternal = zeros(q_real.N);
    auto stateSet = state.set();
    stateSet->ctrlTime=ctrlTime;
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
      cmd_qDDot_ref.resize(q_real.N).setZero();
    }else{
      //get the reference from the callback (e.g., sampling a spline reference)
      cmdGet->ref->getReference(cmd_q_ref, cmd_qDot_ref, cmd_qDDot_ref, q_real, qDot_real, ctrlTime);
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
    naturalGains(k_p, k_d, .05, 1.);
    qDDot_des = k_p * (cmd_q_ref - q_real) + k_d * (cmd_qDot_ref - qDot_real);
    qDDot_des += cmd_qDDot_ref;
//    q = q_ref;
//    qdot = qdot_ref;

  } else if(controlType == rai::ControlType::projectedAcc) { // projected Kp, Kd and u_b term for projected operational space control
    qDDot_des = cmd_qDDot_ref - KpRef*q_real - KdRef*qDot_real;
  }

  //-- add noise (to controls, vel, pos?)
  if(noise_th>0.){
    if(!noise.N) noise = zeros(q_real.N);
    rndGauss(noise, noise_sig, true);
    noise *= noise_th;
    qDot_real += noise;
  }

  //-- add other crazy perturbations?
  if((step_count%1000)<100) q_real(0) = .1;

  //-- directly integrate
  q_real += .5 * tau * qDot_real;
  qDot_real += tau * qDDot_des;
  q_real += .5 * tau * qDot_real;

  //-- display? 20fps
  if(false && !(step_count%int(1./(20.*tau)))){
    emuConfig.setJointState(q_real);
    emuConfig.watch(false, STRING("EMULATION - time " <<ctrlTime));
  }

  //-- check for collisions!
#if 0
  emuConfig.setJointState(q_real);
  emuConfig.ensure_proxies();
  double p = emuConfig.getTotalPenetration();
  if(p>0.){
    emuConfig.reportProxies(std::cout, 0.01, false);
    emuConfig.watch(false, "EMU CONFIG IN COLLISION");
    rai::wait(); metronome.reset(tau);
  }
#endif

  //-- data log?
  if(writeData>0 && !(step_count%1)){
    if(!dataFile.is_open()) dataFile.open("z.panda.dat");
    dataFile <<ctrlTime <<' '; //single number
    q_real.writeRaw(dataFile); //7
    cmd_q_ref.writeRaw(dataFile); //7
    if(writeData>1){
      qDot_real.writeRaw(dataFile); //7
      cmd_qDot_ref.writeRaw(dataFile); //7
      qDDot_des.writeRaw(dataFile); //7
      cmd_qDDot_ref.writeRaw(dataFile);
    }
    if(writeData>2){
    }
    dataFile <<endl;
  }
}
