#include <Franka/controlEmulator.h>
#include <Franka/franka.h>
#include <Franka/help.h>

#include <Algo/SplineCtrlFeed.h>

#include <Kin/frame.h>

//===========================================================================

void testMoveTo() {
  //-- setup a configuration
  rai::Configuration C;
  C.addFile(rai::raiPath("../rai-robotModels/scenarios/pandaSingle.g"));
  C.watch(true);

  //-- start a robot thread
  C.ensure_indexedJoints();
  ControlEmulator robot(C, {});
//  FrankaThreadNew robot(0, franka_getJointIndices(C,'R'));
  robot.writeData = true;

  //-- create 2 simple reference configurations
  arr q0 = robot.state.get()->q;
  arr qT = q0;
  qT(1) -= .5;

  //-- define the reference feed to be a spline
  auto sp = make_shared<rai::SplineCtrlReference>();
  robot.cmd.set()->ref = sp;

  //1st motion:
  double ctrlTime = robot.state.get()->time;
  sp->append(cat(qT, qT, q0).reshape(3,-1), arr{2., 2., 4.}, ctrlTime, true);

  for(;;){
    if(C.watch(false,STRING("time: "<<robot.state.get()->time))=='q') break;
    C.setJointState(robot.state.get()->q);
    rai::wait(.1);
  }

  //2nd motion:
  ctrlTime = robot.state.get()->time;
  sp->moveTo(qT, 1., ctrlTime, false);
  cout <<"OVERRIDE AT t=" <<ctrlTime <<endl;
  sp->moveTo(q0, 1., ctrlTime, true);

  for(;;){
    if(C.watch(false,STRING("time: "<<robot.state.get()->time))=='q') break;
    C.setJointState(robot.state.get()->q);
    rai::wait(.1);
  }

}

//===========================================================================

int main(int argc, char * argv[]){
  rai::initCmdLine(argc, argv);

  testMoveTo();

  return 0;
}
