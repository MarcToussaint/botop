#include <Core/util.h>
#include <Core/thread.h>
#include <Control/CtrlMsgs.h>
#include <Kin/kin.h>
#include <BotOp/bot.h>

#include <Trossen/TrossenThread.h>

#ifdef RAI_TROSSEN

//COPY AND PASTE from trossen_arm/demos/cpp/gravity_compensation

#include <iostream>

#include "libtrossen_arm/trossen_arm.hpp"

int direct(){
  // Initialize the driver
  trossen_arm::TrossenArmDriver driver;

  // Configure the driver
  driver.configure(
      trossen_arm::Model::wxai_v0,
      trossen_arm::StandardEndEffector::wxai_v0_follower,
      "192.168.1.5",
      true
      );

  // Start gravity compensation
  driver.set_all_modes(trossen_arm::Mode::external_effort);
  driver.set_all_external_efforts({0, 0, 0, 0, 0, 0, 0}, 0.0f, false);

  rai::wait();
  driver.cleanup(false);

  return 0;
}

#endif


void thread(){
  rai::Var<rai::CtrlCmdMsg> cmd;
  rai::Var<rai::CtrlStateMsg> state;

  rai::Configuration C;
  C.addFile("scene.yml");

  TrossenThread trossen(cmd, state, {"192.168.1.5"});

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
  arr q0 = C.getJointState();

  {

    BotOp bot(C, true, true);

    bot.home(C);

#if 0
    bot.hold(false, true);
    bot.wait(C, true, false);
#else
    uint T=10;
    arr path(T, q0.N);
    for(uint t=0;t<T;t++){
      path[t] = q0;
      path(t,{0,6}) += 0.3*randn(6);
    }
    path[-1] = q0;
    bot.move(path, {.5*T});
    bot.wait(C);
#endif
  }

  gnuplot("plot 'trossen.dat' us 1:6 t 'REF', '' us 1:13 t 'REAL'", true);
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
  // botop();
  park();
  return 0;
}
