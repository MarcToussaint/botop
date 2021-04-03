#include <Franka/controlEmulator.h>
#include <Franka/franka.h>

#include <Control/SplineCtrlFeed.h>

#include <Kin/frame.h>

//===========================================================================

void testMoveTo() {
  //-- setup a configuration
  rai::Configuration C;
  C.addFile(rai::raiPath("../model/pandaSingle.g"));
  C.getFrame("R_panda_finger_joint1")->setJoint(rai::JT_rigid);
  C.getFrame("R_panda_finger_joint2")->setJoint(rai::JT_rigid);
  C.addFrame("target") -> setPosition(C["endeffR"]->getPosition() + arr{0,.0,-.5});
  C.watch(true);


  //-- start a robot thread
  ControlEmulator robot(C, {});
//  FrankaThreadNew robot(ctrlRef, ctrlState, 0, franka_getJointIndices(C.get()(),'R'));
  robot.writeData = true;

  //-- create 2 simple reference configurations
  robot.state.waitForRevisionGreaterThan(10);
  arr q0 = robot.state.get()->q;
  arr qT = q0;
  qT(1) += .5;

  //-- define the reference feed to be a spline
  auto sp = make_shared<rai::SplineCtrlReference>();
  robot.cmd.set()->ref = sp;

  //1st motion:
  sp->append(cat(qT, qT, q0).reshape(3,-1), arr{2., 2., 4.});

  for(;;){
    if(C.watch(false,STRING("time: "<<rai::realTime()))=='q') break;
    C.setJointState(robot.state.get()->q);
    rai::wait(.1);
  }

  //2nd motion:
  sp->moveTo(qT, 1., false);
  cout <<"OVERRIDE AT t=" <<rai::realTime() <<endl;
  sp->moveTo(q0, 1.);

  for(;;){
    if(C.watch(false,STRING("time: "<<rai::realTime()))=='w') break;
    C.setJointState(robot.state.get()->q);
    rai::wait(.1);
  }

}

//===========================================================================

int main(int argc, char * argv[]){
  testMoveTo();

  return 0;
}
