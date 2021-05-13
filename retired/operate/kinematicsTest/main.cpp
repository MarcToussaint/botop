#include <Kin/kin.h>

#include <KOMO/komo.h>
#include <KOMO/komo-ext.h>

#include <Operate/path.h>
#include <Operate/robotio.h>
#include <Operate/simulationThread.h>
#include <Operate/op.h>


//===========================================================================

void test_pickAndPlace(){
  rai::KinematicWorld K;
  K.addFile("../../model/pandaStation-noTilt.g");

  //-- robot simulation
  SimulationThread R(K);

  //==============
  //
  // add the hook now
  //

//  R.addFile("../../model/hook.g", "table", rai::Transformation({.5,.7,.01}, 0));
  R.addFile("../../model/hook.g", "table", rai::Transformation({.3,.3,.01}, 0));


  //cheating! in real I could not copy the real world!
  R.K.waitForNextRevision(1);
  K = R.K.get();



  //==============
  //
  // what follows is a little script to make the robot do things
  //

  const char* endeff="endeffL";
  const char* object="stickTip";

  arr x0 = K.getFrameState();
  arr q0 = K.getJointState();
  StringA joints = K.getJointNames();
  K.watch(true);

  //compute a grasp
  chooseBoxGrasp(K, endeff, object);
  arr grasp = K.getJointState();
  K.watch(true);

  //compute a path from x0 to grasp
  K.setFrameState(x0);
  auto path = computePath(K, grasp, joints, endeff, .0, .8);

  //open the gripper
  R.execGripper("pandaL", .1);
  R.waitForCompletion();

  //execute the path
  R.executeMotion(joints, path.first, path.second);
  R.waitForCompletion();

  //close gripper
  R.execGripper("pandaL", .0);
  R.waitForCompletion();

  //attach
  R.attach(endeff, object);

  arr q_now = R.getJointPositions(joints);
  K.setJointState(q_now);
  K.watch(true);

  path = computePath(K, q0, joints, endeff, .2, .8);
  R.executeMotion(joints, path.first, path.second);
  R.waitForCompletion();

  rai::wait();
}

//===========================================================================

int main(int argc,char** argv){
  rai::initCmdLine(argc,argv);

  test_pickAndPlace();

  return 0;
}


