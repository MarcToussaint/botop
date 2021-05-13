#include <Franka/franka.h>

#include <Kin/kin.h>

int main(int argc, char** argv) {
  /* minimal demo to hold robot in current state - with set gains */

  Var<CtrlMsg> ctrl;
  Var<CtrlMsg> state;

  FrankaThread F(ctrl, state, 1);

  //control robot to current state -- this is automatically done in the fanka launch
  //  ctrl.set()->q = state.get()->q;

  rai::KinematicWorld K("../../model/pandaSingle.g");

  for(;;){
    rai::wait(.02);
    arr q = state.get()->q;
    q.append(.1);
    K.setJointState(q);
    if(K.watch()=='q') break;
  }

  return 0;
}
