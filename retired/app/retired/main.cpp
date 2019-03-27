#include <KOMO/komo.h>
#include <Control/taskControl.h>
//#include <RosCom/subscribeRosKinect.h>
//#include <RosCom/subscribeRosKinect2PCL.h>
#include <Gui/viewer.h>
#include <Perception/percept.h>
#include <Kin/kinViewer.h>
//#include <memory>
//#include <RosCom/roscom.h>
#include <Kin/frame.h>
#include <Kin/taskMap_InsideBox.h>

#include "sim.h"
#include "komo_fine.h"
#include "filter.h"
#include <Msg/MotionReference.h>

#include "robot_msgs/SendJointTrajectory.h"


#include <ros/ros.h>
#include <RosCom/spinner.h>

//===============================================================================

Msg_MotionReference planPath(const rai::String& cmd, const rai::Transformation& where=NoTransformation, bool fromCurrent=true){
  Access<rai::KinematicWorld> K("filterWorld");
  Access<rai::KinematicWorld> Ktail("kinTail");
  if(!fromCurrent){
    StringA joints = K.get()->getJointNames();
    arr q = Ktail.get()->getJointState(joints);
    K.set() -> setJointState(q, joints);
  }
  KOMO_fineManip komo(K.get());

  komo.setPathOpt(1., 20, 3.);

  if(cmd=="grasp") komo.setFineGrasp(1., "endeff", "box0", "wsg_50_base_joint_gripper_left");
  if(cmd=="place"){
    komo.setFineLift(0., "endeff");
    komo.setFinePlace(1., "endeff", "box0", "table1", "wsg_50_base_joint_gripper_left");
  }
  if(cmd=="placeFixed"){
    komo.setFineLift(0., "endeff");
    CHECK(!where.isZero(), "");
    rai::Transformation rel = where / komo.world["table1"]->X;
    komo.setFinePlaceFixed(1., "endeff", "box0", "table1", rel, "wsg_50_base_joint_gripper_left");
  }
  if(cmd=="home")  komo.setFineHoming(1., "wsg_50_base_joint_gripper_left");

  komo.reset();
  komo.run();
  komo.getReport(true);

  while(komo.displayTrajectory(.05, true));

  Ktail.set() = *komo.configurations.elem(-1);

  StringA joints = Access<StringA>("jointNames").get();
  arr x = komo.getPath(joints);
  Msg_MotionReference ref;
  ref.path = x;
  ref.tau = { komo.tau };
  ref.append = true;
  return ref;
}

//===============================================================================

void callTrajectoryService(const Msg_MotionReference& msg){
#ifdef RAI_ROS
  static robot_msgs::SendJointTrajectory srv;

  srv.request.trajectory.points.resize(msg.path.d0);
  double time=0.;
  for(uint t=0;t<msg.path.d0;t++){
      if(msg.tau.N==1) time += msg.tau.scalar();
      else time += msg.tau(t);

      trajectory_msgs::JointTrajectoryPoint& p = srv.request.trajectory.points[t];
      p.positions = msg.path[t].copy();
      p.velocities = zeros(msg.path.d1);
      p.accelerations = zeros(msg.path.d1);
      p.effort = zeros(msg.path.d1);
      p.time_from_start = ros::Duration(2.*time);
    }


  srv.request.trajectory.joint_names.resize(msg.path.d1);
  StringA joints = Access<StringA>("jointNames").get();
  CHECK_EQ(joints.N, msg.path.d1, "");
  for(uint i=0;i<msg.path.d1;i++){
      srv.request.trajectory.joint_names[i] = joints(i).p;
    }


  if(ros::service::call("/robot_control/SendJointTrajectory", srv)){
      ROS_INFO("Success?: %ld", (long int)srv.response.success);
    }else{
      ROS_ERROR("Failed to call service add_two_ints");
    }
#endif
}

//===============================================================================

Msg_MotionReference planPath_IK(const rai::String& cmd, const rai::Transformation& where=NoTransformation, bool fromCurrent=true){
  Access<rai::KinematicWorld> K("filterWorld");
  Access<rai::KinematicWorld> Ktail("kinTail");
  KOMO_fineManip komo(K.get());
  komo.setPathOpt(1., 20, 3.);

  if(cmd=="grasp"){
    KOMO_fineManip komo1(K.get());
    komo1.setIKOpt();
    komo1.setIKGrasp("endeff", "box0");
    komo1.reset();
    komo1.run();
    cout <<komo1.getReport(false);
//    while(komo1.displayTrajectory(.05, true));

    komo.setGoTo(komo1.configurations.last()->q, "endeff", .2, .8);
  }
  if(cmd=="place"){
    komo.setFineLift(0., "endeff");
    komo.setFinePlace(1., "endeff", "box0", "table1", "wsg_50_base_joint_gripper_left");
  }
  if(cmd=="placeFixed"){
    KOMO_fineManip komo1(K.get());
    komo1.setIKOpt();
    komo1.setIKPlaceFixed("box0", where);
    komo1.reset();
    komo1.run();
    cout <<komo1.getReport(false);
//    while(komo1.displayTrajectory(.05, true));

    komo.setGoTo(komo1.configurations.last()->q, "endeff", .2, .8);
  }
  if(cmd=="home")  komo.setFineHoming(1., "wsg_50_base_joint_gripper_left");

  komo.reset();
  komo.run();
  komo.getReport(true);

//  while(komo.displayTrajectory(.05, true));

  Ktail.set() = *komo.configurations.elem(-1);

  StringA joints = Access<StringA>("jointNames").get();
  arr x = komo.getPath(joints);
  Msg_MotionReference ref;
  ref.path = x;
  ref.tau = { komo.tau };
  ref.append = true;
  return ref;
}

//===============================================================================

void go(int probId=1) {
  //-- load the problem description into the 'world' variable
  Access<rai::KinematicWorld>("world").set()->init(STRING("problem-0"<<probId<<".g"));

  //-- load the robot joints into a global variable
  StringA joints = Access<rai::KinematicWorld>("world").get()->getJointNames();
  joints.removeValue("slider1Joint", false);
  joints.removeValue("wsg_50_base_joint_gripper_left");
  Access<StringA>("jointNames").set() = joints;

  //-- start a simulation
  KinSim sim;
  sim.threadLoop();

  //-- start the filter
  FilterSimple filter;
  filter.threadLoop();

  //wait for all objects to be localized
  filter.problemPerceived.waitForValueEq(true);

  //wait for robot pose msg
  Access<arr>("currentQ").waitForRevisionGreaterThan(10);
  Access<double> ttg("timeToGo");

  Msg_MotionReference ref = planPath_IK("home");
  Access<Msg_MotionReference>("MotionReference").set() = ref;
  callTrajectoryService(ref);
  rai::wait(1.);
  for(;;){ ttg.waitForNextRevision(); if(ttg.get()<=0.) break;  }

  for(uint l=0;;l++){
    ref = planPath_IK("grasp");
    Access<Msg_MotionReference>("MotionReference").set() = ref;
    callTrajectoryService(ref);
    rai::wait(1.);
    for(;;){ ttg.waitForNextRevision(); if(ttg.get()<=0.) break;  }
    Access<double>("gripperReference").set() = .004;
    Access<StringA>("switches").set() = {"attach", "endeff", "box0"};
    rai::wait(.3);

//    ref = planPath("home");
//    Access<Msg_MotionReference>("MotionReference").set() = ref;
//    for(;;){ ttg.waitForNextRevision(); if(ttg.get()<=0.) break;  }


    if(l%2){
      ref = planPath_IK("placeFixed", { {.5, -.5, 1.12}, Quaternion_Id } );
    }else{
      ref = planPath_IK("placeFixed", { {.5, .5, 1.12}, Quaternion_Id } );
    }
    Access<Msg_MotionReference>("MotionReference").set() = ref;
    callTrajectoryService(ref);
    rai::wait(1.);
    for(;;){ ttg.waitForNextRevision(); if(ttg.get()<=0.) break;  }
    Access<double>("gripperReference").set() = .05;
    Access<StringA>("switches").set() = {"attach", "table1", "box0"};
    rai::wait(.3);

//    ref = planPath("home");
//    Access<Msg_MotionReference>("MotionReference").set() = ref;
//    for(;;){ ttg.waitForNextRevision(); if(ttg.get()<=0.) break;  }
  }


}

//===============================================================================

int main(int argc, char** argv){
  rai::initCmdLine(argc, argv);

  RosCom_Spinner rs;
  rs.threadLoop();

  go();

  return 0;
}

