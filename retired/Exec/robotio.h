#pragma once

#include "robotio.h"
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseStamped.h>
#include <RosCom/roscom.h>
#include <RosCom/rai_msgs/WSG_50_command.h>

struct RobotIO{
  RosCom ROS;
  bool pubSubToROS;
  
  StringA jointNames;
  
  Var<sensor_msgs::JointState> jointState;
  Var<tf::tfMessage> tfMessages;
  Var<rai_msgs::WSG_50_command> gripperCommand;
  std::map<String, Var<geometry_msgs::PoseStamped>*> objectStates;

  
  std::shared_ptr<Subscriber<sensor_msgs::JointState>> sub_jointState; //subscriber
  std::shared_ptr<Subscriber<tf::tfMessage>> sub_tfMessages; //subscriber
  std::shared_ptr<Publisher<rai_msgs::WSG_50_command>> pub_gripperCommand; //subscriber
  std::map<String, std::shared_ptr<Subscriber<geometry_msgs::PoseStamped>>> sub_objectStates; //subscribers

  uint gripperCommandCounter=0;
  
  RobotIO(bool pubSubToROS);
  
  //-- execution
  bool executeMotion(const StringA& joints, const arr& path, const arr& times, double timeScale=1.);
  void execGripper(double position, double force=40.);

  arr getJointPositions(const StringA& joints);

  //see example code in rai/RosCom/perc/subscribeOptitrack.h
  arr getObjectPoses(const StringA& objs);

  StringA getJointNames();

  StringA getObjectNames();
};

