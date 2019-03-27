#include <Gui/opengl.h>
#include <LGP/optLGP.h>
#include <Kin/TM_qItself.h>
#include <trajectory_msgs/JointTrajectory.h>

#include <RosCom/roscom.h>
#include <RosCom/spinner.h>

#include <ros/ros.h>
#include <RosCom/rai_msgs/SendJointTrajectory.h>
#include <RosCom/rai_msgs/StringString.h>
#include <Core/util.h>

//===================================a===================================

trajectory_msgs::JointTrajectory computeMinimalPlan(){
  rai::KinematicWorld K("../model/LGP-kuka.g");
  K.optimizeTree();
  K.watch(true);

  /**Skeleton S = {
    { {"grasp", "endeff", "stick"}, 1., 1.},
    { {"grasp", "stickTip", "redBall"}, 2., 2.},
  };

  double maxPhase = 1.;
  for(const SkeletonEntry& s:S){
    if(s.phase0 > maxPhase) maxPhase = s.phase0;
    if(s.phase1 > maxPhase) maxPhase = s.phase1;
  }
  maxPhase += .5;**/

  KOMO komo;
  komo.setModel(K, false);
  komo.setPathOpt(1., 10, 2.);
  //komo.setModel(K, true);
  //komo.setPathOpt(maxPhase, 10., 2.);

  komo.setCollisions(false);
  komo.deactivateCollisions("table1", "iiwa_link_0_0");
  komo.activateCollisions("table1", "stick");
  komo.activateCollisions("table1", "stickTip");

  arr q_init = K.getJointState();
  arr target = q_init;
  target(-4) += .5;
//  target.setZero();
  cout <<"init: " <<q_init <<"\n goal: " <<target <<endl;
  komo.setTask(1., 1., new TM_qItself(), OT_sos, target, 1e2);

  komo.setSlow(.9, 1., 1e2);
  //komo.setSkeleton(S);


  komo.reset();
  //  komo.reportProblem();
  komo.animateOptimization = 0;
  komo.run();
  //   komo.checkGradients();

  cout <<komo.getReport(false) <<endl;
  komo.reportEffectiveJoints();
//  komo.reportProxies();
  while(komo.displayTrajectory(.1));


  //-- read out results

  StringA joints = K.getJointNames();
  arr times = komo.getPath_times();
  arr path = komo.getPath(joints);
  cout <<"times: " <<times <<endl;
  cout <<"path: " <<path <<endl;

  //-- construct message

  trajectory_msgs::JointTrajectory msg;

  //msg.header.stamp = ros::Time::now();

  for(rai::String& str:joints){
    msg.joint_names.push_back(str.p);
  }

  for(uint t=0;t<komo.T;t++){
    trajectory_msgs::JointTrajectoryPoint point;
    point.positions = conv_arr2stdvec(path[t]);
    point.velocities = conv_arr2stdvec(zeros(path.d1));
    point.accelerations = conv_arr2stdvec(zeros(path.d1));
    point.effort = conv_arr2stdvec(zeros(path.d1));
    point.time_from_start = ros::Duration(times(t));
    msg.points.push_back(point);
  }

  return msg;
}

void sendToROS(trajectory_msgs::JointTrajectory trajMsg){
//  RosCom ROS("miniSendToROS");

  rai_msgs::SendJointTrajectory com;
  com.request.trajectory = trajMsg;
  if(ros::service::call("/robot_control/SendJointTrajectory", com)){
    cout <<"SUCCESS=" <<(com.response.success?"TRUE":"FALSE") <<endl;
  }else cout <<"getState failed" <<endl;


  //Var<trajectory_msgs::JointTrajectory> ref("JointTrajectory");

  //auto pubRef = ROS.publish(ref);

//  ref.set() = trajeMsg;

    //ref.writeAccess();
    //ref() = trajMsg;
    //ref.deAccess();


}

//======================================================================

int main(int argc,char **argv){
  rai::initCmdLine(argc, argv);
  ros::init(argc, argv, "mini");

  auto msg = computeMinimalPlan();
  sendToROS(msg);

  return 0;
}

//===========================================================================

/**int main(int argc, char** argv) {
  setLogLevels(0,0);
  rai::initCmdLine(argc, argv);
  ros::init(rai::argc, rai::argv, "RAPshell");

  if(argc<2) return query("getState");
  if(!strcmp(argv[1],"st")) return query("getState");
  if(!strcmp(argv[1],"sy")) return query("getSymbols");

  //-- send a fact
  rai::String fact;
  fact <<"( ";
  for(int i=1;i<argc;i++) fact <<argv[i] <<' ';
  fact <<")";
  return query(fact);

  return 0;
}**/
