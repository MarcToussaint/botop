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
      trossen_arm::StandardEndEffector::wxai_v0_leader,
      "192.168.1.3",
      true
      );

  // Start gravity compensation
  driver.set_all_modes(trossen_arm::Mode::external_effort);
  driver.set_all_external_efforts({0, 0, 0, 0, 0, 0, 0}, 0.0f, false);

  rai::wait();

  return 0;
}

#endif


void thread(){
  Var<rai::CtrlCmdMsg> cmd;
  Var<rai::CtrlStateMsg> state;

  rai::Configuration C;
  C.addFile("scene.yml");

  TrossenThread trossen(cmd, state);

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

  q0 = {0.124552, 0.630388, 0.830282, -0.140574, -0.621233, 0.422866, 0.02};

  {

    BotOp bot(C, false);

    bot.launch_trossen();

    bot.wait(C, true, false);

    uint T=10;
    arr path(T, q0.N);
    for(uint t=0;t<T;t++){
      path[t] = q0;
      path(t,{0,6}) += 0.3*randn(6);
    }
    path[-1] = q0;
    bot.move(path, {.5*T});
    bot.wait(C);
  }

  gnuplot("plot 'trossen.dat' us 1:4 t 'REF', '' us 1:11 t 'REAL'", true);
}


int main(int argc, char** argv){
  rai::initCmdLine(argc, argv);

    // direct();
    // thread();
    botop();
    return 0;
}
