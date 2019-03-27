#include <Gui/opengl.h>
#include <LGP/optLGP.h>

#include <Exec/execution.h>

//======================================================================

int main(int argc,char **argv){
  rai::initCmdLine(argc, argv);
  rnd.clockSeed();

  LGPExecution E(true, false);

//  for(uint k=0;k<20;k++){
//    E.syncModelObjectPosesWithRealOrSimulation();
//    rai::wait(.1);
//  }

#if 0 //basic home-zero-home
  auto path = E.computePreciseMotion(zeros(7), false, false);
  E.executeMotion(E.armJoints, path.first, path.second);
  E.closeGripper();

  path = E.computePreciseMotion(E.q_home, false, false);
  E.executeMotion(E.armJoints, path.first, path.second);
  E.openGripper();

  return 0;
#endif

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

  //-- send 'attach'
  E.attach("endeff", "stick");

#if 0
  //-- compute second motion, execute, open gripper
  arr q_place_rough = E.getJointConfiguration(plan, 2.);
  rai::Transformation X = E.getObjectPose(plan, 2., "stick");
  arr q_place_precise = E.computePrecisePlace(q_place_rough, "stick", "table1", X);
  E.syncModelJointStateWithRealOrSimulation();
  auto q_place_motion = E.computePreciseMotion(q_place_precise, true, true);
  E.executeMotion(E.armJoints, q_place_motion.first, q_place_motion.second);
  E.openGripper();

  //-- wait, send 'attach'
  E.attach("table1", "stick");
#endif

  //-- compute homing motion, execute
  E.syncModelJointStateWithRealOrSimulation();
  auto q_homing = E.computePreciseMotion(E.q_home, true, false);
  E.executeMotion(E.armJoints, q_homing.first, q_homing.second);
  E.openGripper();

  //-- wait
  rai::wait();
  return 0;
}

