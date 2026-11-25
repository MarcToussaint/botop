#include "Z1Thread.h"

#if 1 //def RAI_Z1

#include <unitree_arm_sdk/control/unitreeArm.h>

//from z1_sdk/examples/lowcmd_development.cpp

Z1Thread::Z1Thread(rai::Var<rai::CtrlCmdMsg>& cmd, rai::Var<rai::CtrlStateMsg>& state, const strA& ids)
    : rai::RobotAbstraction(cmd, state),
    Thread("Z1Thread", .002), //HARD CODED step frequency of 500Hz
    ipAddresses(ids) {

  Kp = rai::getParameter<arr>("Z1/Kp", arr{});
  Kd = rai::getParameter<arr>("Z1/Kd", arr{});

  LOG(0) <<"launching Z1 at " <<ipAddresses;

  threadOpen(true);
  threadLoop();
}

void Z1Thread::open(){
  bool hasGripper = true;
  driver = make_shared<UNITREE_ARM::unitreeArm>(hasGripper);
  UNITREE_ARM::unitreeArm& arm(*driver.get());

  arm.sendRecvThread->start();
  // arm.backToStart();
  arm.setFsm(UNITREE_ARM::ArmFSMState::PASSIVE);
  arm.setFsm(UNITREE_ARM::ArmFSMState::LOWCMD);

  std::vector<double> KP, KW;
  KP = arm._ctrlComp->lowcmd->kp;
  KW = arm._ctrlComp->lowcmd->kd;
  // for(double& x:KP) x *= 0.2;
  // for(double& x:KW) x *= 0.1;
  arm._ctrlComp->lowcmd->setControlGain(KP, KW);
  arm._ctrlComp->lowcmd->setGripperGain(KP[KP.size()-1], KW[KW.size()-1]);
  arm.sendRecvThread->shutdown();

  //get initial state
  arr q_init = conv_eigen2arr(arm.lowstate->getQ());
  q_init.append(arm.lowstate->getGripperQ());
  {
    auto stateSet = state.set();
    stateSet->q = q_init;
    stateSet->qDot.resize(q_init.N).setZero();
    stateSet->tauExternalIntegral.resize(q_init.N).setZero();
    stateSet->tauExternalCount=0;
  }
  //set initial position reference
  {
    auto cmd_set = cmd.set();
    cmd_set->setConst(q_init, false, true);
  }
}

void Z1Thread::close(){
  UNITREE_ARM::unitreeArm& arm(*driver.get());

  arm.sendRecvThread->start();
  arm.setFsm(UNITREE_ARM::ArmFSMState::JOINTCTRL);
  arm.backToStart();
  arm.setFsm(UNITREE_ARM::ArmFSMState::PASSIVE);
  arm.sendRecvThread->shutdown();
  driver.reset();
}

void Z1Thread::step(){
  UNITREE_ARM::unitreeArm& arm(*driver.get());
  //-- get real state
  arr q_real = conv_eigen2arr(arm.lowstate->getQ());
  arr qDot_real = conv_eigen2arr(arm.lowstate->getQd());
  arr tauExternal = conv_eigen2arr(arm.lowstate->getTau());
  q_real.append(arm.lowstate->getGripperQ());
  qDot_real.append(arm.lowstate->getGripperQd());
  tauExternal.append(arm.lowstate->getGripperTau());

  //-- publish state & INCREMENT CTRL TIME
  {
    auto stateSet = state.set();
    if(!stateSet->stall) stateSet->ctrlTime += metronome.ticInterval;
    else stateSet->stall--;
    ctrlTime = stateSet->ctrlTime;
    stateSet->q = q_real;
    stateSet->qDot = qDot_real;
    stateSet->tauExternalIntegral += tauExternal;
    stateSet->tauExternalCount++;
  }

  //-- get current ctrl reference
  arr q_ref, qDot_ref, qDDot_ref;
  {
    auto cmdGet = cmd.get();

    //get commanded reference from the reference callback (e.g., sampling a spline reference)
    if(cmdGet->ref){
      cmdGet->ref->getReference(q_ref, qDot_ref, qDDot_ref, q_real, qDot_real, ctrlTime);
    }else{
      q_ref = q_real;
    }
  }

  //-- check reference error
  bool isStalled = false;
  if(q_ref.N){
    double err = length(q_ref - q_real);
    if(err>.1){ //stall!
      state.set()->stall = 2; //no progress in reference time! for at least 2 iterations (to ensure continuous stall with multiple threads)
      isStalled=true;
      cout <<"STALLING - err: " <<err <<' ' <<q_ref - q_real <<endl;
    }
  }

  //-- data log?
  if(writeData>0 && !(step_count%5)){
    if(!dataFile.is_open()) dataFile.open(STRING("z.Z1.dat"));
    dataFile <<ctrlTime <<' ' <<q_real.modRaw() <<' ' <<q_ref.modRaw() <<endl;
  }

  if(q_ref.N){
    arm.q = conv_arr2eigen(q_ref({0,6}));
    arm.qd = conv_arr2eigen(qDot_ref({0,6}));
    arm.tau = arm._ctrlComp->armModel->inverseDynamics(arm.q, arm.qd, Vec6::Zero(), Vec6::Zero());
    arm.gripperQ = q_ref(6);
    arm.gripperW = qDot_ref(6);
    arm.gripperTau = 0.;

    arm.setArmCmd(arm.q, arm.qd, arm.tau);
    arm.setGripperCmd(arm.gripperQ, arm.gripperW, arm.gripperTau);
    arm.sendRecv();
  }else{
	HALT("no reference!");
  }
}

#else

Z1Thread::Z1Thread(rai::Var<rai::CtrlCmdMsg>& cmd, rai::Var<rai::CtrlStateMsg>& state, const strA& ids)
    : rai::RobotAbstraction(cmd, state),
    Thread("Z1Thread", .002), //HARD CODED step frequency of 100Hz
    ipAddresses(ids) { NICO }
void Z1Thread::open(){ NICO }
void Z1Thread::step(){ NICO }
void Z1Thread::close(){ NICO }

#endif
