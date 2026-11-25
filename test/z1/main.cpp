#define RAI_EIGEN
#include <Core/util.h>
#include <Core/thread.h>
#include <Control/CtrlMsgs.h>
#include <Kin/kin.h>
#include <BotOp/bot.h>
#include <Z1/Z1Thread.h>

// #include <UnitreeZ1/UnitreeZ1.h>

#if 1 //RAI_UNITREE_Z1


#include <iostream>

#include "unitree_arm_sdk/control/unitreeArm.h"

int direct(){
  //verbatim from z1_sdk/examples/lowcmd_development.cpp

  std::cout << std::fixed << std::setprecision(3);
  bool hasGripper = true;
  UNITREE_ARM::unitreeArm arm(hasGripper);
  // arm._ctrlComp->dt = 0.005;

  arm.sendRecvThread->start();
  // arm.backToStart();
  arm.setFsm(UNITREE_ARM::ArmFSMState::PASSIVE);
  arm.setFsm(UNITREE_ARM::ArmFSMState::LOWCMD);

  std::vector<double> KP, KW;
  KP = arm._ctrlComp->lowcmd->kp;
  KW = arm._ctrlComp->lowcmd->kd;
  for(double& x:KP) x *= 0.2;
  for(double& x:KW) x *= 0.1;
  arm._ctrlComp->lowcmd->setControlGain(KP, KW);
  arm._ctrlComp->lowcmd->setGripperGain(KP[KP.size()-1], KW[KW.size()-1]);
  arm.sendRecvThread->shutdown();

  Vec6 initQ = arm.lowstate->getQ();
  cout <<"initQ: " <<conv_eigen2arr(initQ) <<endl;

  double duration = 2000;
  Vec6 targetQ;
  targetQ << 0, 1.5, -1, -0.54, 0, 0;
  UNITREE_ARM::Timer timer(arm._ctrlComp->dt);
  for(int i(0); i<duration; i++){

    std::cout <<i <<' ' <<conv_eigen2arr(arm.lowstate->getQ()) <<std::endl;
    // arm.q = initQ * (1-i/duration) + targetQ * (i/duration);
    // arm.qd = (targetQ - initQ) / (duration * arm._ctrlComp->dt);
      arm.q = initQ;
      arm.qd = Vec6::Zero();
    arm.tau = arm._ctrlComp->armModel->inverseDynamics(arm.q, arm.qd, Vec6::Zero(), Vec6::Zero());
    arm.tau = Vec6::Zero();
    arm.gripperQ = -(i/duration);

    // arm.setArmCmd(arm.q, arm.qd, arm.tau);
    // arm.setGripperCmd(arm.gripperQ, arm.gripperW, arm.gripperTau);
    // arm.sendRecv();
    timer.sleep();
  }
  //for(uint i=0;i<1000;i++) timer.sleep();

  arm.sendRecvThread->start();
  arm.setFsm(UNITREE_ARM::ArmFSMState::JOINTCTRL);
  arm.backToStart();
  arm.setFsm(UNITREE_ARM::ArmFSMState::PASSIVE);
  arm.sendRecvThread->shutdown();
  return 0;
}

#endif


void thread(){
  rai::Var<rai::CtrlCmdMsg> cmd;
  rai::Var<rai::CtrlStateMsg> state;

  rai::Configuration C;
  C.addFile("scene.yml");

  Z1Thread z1(cmd, state, {"192.168.1.5"});

  for(;;){
    rai::wait(.02);
    arr q = state.get()->q;
    cout <<"q: " <<q <<endl;
    C.setJointState(q);
    int key = C.view(false);
    if(key=='q') break;
  }
}

void botop(){
  rai::Configuration C;
  C.addFile("scene.yml");

  {
    BotOp bot(C, false, false);
    bot.launch_z1({""});
    bot.setControllerWriteData(1);

    bot.home(C);

#if 0
    // bot.hold(false, true);
    bot.wait(C, true, false);
#else
    uint T=2;
    arr q0 = bot.get_qHome();
    arr path(T, q0.N);
    for(uint t=0;t<T;t++){
      path[t] = q0;
      // path(t,{0,6}) += 0.3*randn(6);
    }
    path(0,0) -= .5;
    path[-1] = q0;
    bot.move(path, {.5*T});
    bot.wait(C);
#endif
  }

  gnuplot("plot 'z.Z1.dat' us 1:2 t 'REAL', '' us 1:9 t 'REF'", true);
}

void park(){
  rai::Configuration C;
  C.addFile("scene.yml");
  BotOp bot(C, true, true);
  bot.moveTo(zeros(7));
  bot.wait(C);
}

int main(int argc, char** argv){
  rai::initCmdLine(argc, argv);

  // direct();
  // thread();
  botop();
  // park();
  return 0;
}
