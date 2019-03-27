#include <Gui/opengl.h>
#include <LGP/optLGP.h>

#include <Exec/execution.h>

/*

  Goal: We want a full pick-and-place cycle test:

  * the object poses are initially set to unknown/random
  * the object poses are retrieved from the sim
  * We plan a full pick-and-place of one object using LGP (1st use of KOMO)
  * We retrieve the key frame poses
  * Use 1. keyframe as regularization for a high-precision grasp pose IK (2nd use of KOMO)
  * For this grasp pose, generate a nice motion with final downward motion (3nd use of KOMO)
  * Execute the motion in sim
  * Close the gripper
  * Use the 2. keyframe as regularization for a high-prediction placement IK
  * Generate a nice motion lift-and-down from current
  * Execute motion
  * open gripper

  We first manage all this in simulation (close/open gripper needs to send 'link' message)

  We then use the sim as middleware to send the exact same to the real robot

*/

//======================================================================

int main(int argc,char **argv){
  rai::initCmdLine(argc, argv);
  rnd.clockSeed();

  LGPExecution E(false, true);

  //-- compute the rough full skeleton plan
  Skeleton S = {
//    { {"grasp", "endeff", "stick"}, 1., 1.},
//    { {"touch", "stick", "redBall"}, 2., 2.},
//    { {"place", "stick", "table1"}, 2., 2.},
    { {"grasp", "endeff", "stick"}, 1., 1.},
    { {"grasp", "stickTip", "redBall"}, 2., 2.},
  };
  ptr<KOMO> plan = E.planSkeleton(S);

  //-- extract the relevant key pose
  arr q_grasp_rough = E.getJointConfiguration(plan, 1.);

  //-- compute the precise key pose
  arr q_grasp_precise = E.computePreciseGrasp(q_grasp_rough, "stick");

  //-- compute first motion, execute, close gripper
  E.syncModelJointStateWithRealOrSimulation();
  auto q_grasp_motion = E.computePreciseMotion(q_grasp_precise, false, true);
  E.executeMotion(E.armJoints, q_grasp_motion.first, q_grasp_motion.second);
  E.closeGripper();

  //-- wait, send 'attach'
  E.waitForCompletion();
  E.attach("endeff", "stick");

  //-- compute second motion, execute, open gripper
  arr q_place_rough = E.getJointConfiguration(plan, 2.);
  rai::Transformation X = E.getObjectPose(plan, 2., "stick");
  arr q_place_precise = E.computePrecisePlace(q_place_rough, "stick", "table1", X);
  E.syncModelJointStateWithRealOrSimulation();
  auto q_place_motion = E.computePreciseMotion(q_place_precise, true, true);
  E.executeMotion(E.armJoints, q_place_motion.first, q_place_motion.second);
  E.openGripper();

  //-- wait, send 'attach'
  E.waitForCompletion();
  E.attach("table1", "stick");

  //-- compute homing motion, execute
  E.syncModelJointStateWithRealOrSimulation();
  auto q_homing = E.computePreciseMotion(E.q_home, true, false);
  E.executeMotion(E.armJoints, q_homing.first, q_homing.second);

  //-- wait
  E.waitForCompletion();
  rai::wait();
  return 0;
}

