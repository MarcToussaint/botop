//#include <NewControl/TaskControlThread.h>
//#include <LGPop/lgpop.h>
#include <Kin/kinViewer.h>

#include <Gui/viewer.h>

#include <Franka/controlEmulator.h>

#include <Franka/franka.h>
#include <Franka/gripper.h>
#include <Franka/help.h>

#include <Control/CtrlMsgs.h>



struct ZeroReference : rai::ReferenceFeed {
  virtual void getReference(arr& q_ref, arr& qDot_ref, arr& qDDot_ref, const arr& q_real, const arr& qDot_real, double time){
     q_ref = q_real;
     qDot_ref.resize(q_ref.N).setZero();
     qDDot_ref.resize(q_ref.N).setZero();
  }
};


void testNew() {
  //-- setup a configuration
  rai::Configuration C;
  C.addFile(rai::raiPath("../model/pandaSingle.g"));
  C.getFrame("R_panda_finger_joint1")->setJoint(rai::JT_rigid);
  C.getFrame("R_panda_finger_joint2")->setJoint(rai::JT_rigid);
  C.addFrame("target") -> setPosition(C["endeffR"]->getPosition() + arr{0,.0,-.5});
  C.watch(true);

  C.ensure_indexedJoints();
  FrankaThreadNew robot(0, franka_getJointIndices(C,'R'));
  robot.writeData = true;

  //comment the next line to only get gravity compensation instead of 'zero reference following' (which includes damping)
  robot.cmd.set()->ref = make_shared<ZeroReference>();

  for(;;){
    if(C.watch(false,STRING("time: "<<rai::realTime()))=='q') break;
    C.setJointState(robot.state.get()->q);
    rai::wait(.1);
  }

  cout <<"bye bye" <<endl;
}



int main(int argc, char * argv[]){
  rai::initCmdLine(argc, argv);

  testNew();

  return 0;
}
