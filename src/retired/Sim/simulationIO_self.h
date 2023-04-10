#pragma once

#include "simulationIO.h"

struct SimulationIO_self{
  RosCom ROS;  // communication with ROS
  Simulation SIM;
  double dt;

  //inputs (reactive variables / messages)
  Var<rai_msgs::MotionReference> ref; // communication variable with ROS
  Var<rai_msgs::StringA> command;

  //outputs
  Var<rai_msgs::arr> currentQ;
  Var<rai_msgs::arr> objectPoses;
  Var<rai_msgs::StringA> objectNames;
  Var<std_msgs::Float64> timeToGo;

  std::shared_ptr<Subscriber<rai_msgs::MotionReference>> sub_ref; //subscriber
  std::shared_ptr<Subscriber<rai_msgs::StringA>> sub_command; //subscriber
  std::shared_ptr<Publisher<rai_msgs::arr>> pub_currentQ;
  std::shared_ptr<Publisher<rai_msgs::arr>> pub_objectPoses;
  std::shared_ptr<Publisher<rai_msgs::StringA>> pub_objectNames;
  std::shared_ptr<Publisher<std_msgs::Float64>> pub_timeToGo;

  SimulationIO_self(bool pubSubToROS, const char *modelFile, double dt);
};
