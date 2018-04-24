#pragma once

#include <RosCom/roscom.h>
#include <Algo/spline.h>
#include <Gui/opengl.h>
#include <rai_msgs/MotionReference.h>
#include <std_msgs/Float64.h>

struct Simulator{
private:
  RosCom ROS;  // communication with ROS
  rai::KinematicWorld K, K_disp, K_ref; // kinematic configurations (K_disp only to display)
  OpenGL gl; // display

  //inputs (reactive variables / messages)
  Var<rai_msgs::MotionReference> ref; // communication variable with ROS
  Var<std_msgs::String> command;

  //outputs
  Var<rai_msgs::arr> currentQ;
  Var<std_msgs::Float64> timeToGo;
//  Var<PerceptSimpleL> percepts_input;


  std::shared_ptr<Subscriber<rai_msgs::MotionReference>> sub_ref; //subscriber
  std::shared_ptr<Subscriber<std_msgs::String>> sub_command; //subscriber
  std::shared_ptr<Publisher<rai_msgs::arr>> pub_currentQ;
  std::shared_ptr<Publisher<std_msgs::Float64>> pub_timeToGo;

  rai::Spline reference; // reference spline constructed from ref
  arr refPoints, refTimes; // the knot points and times of the spline
  double phase=0.; // current phase in the spline
  double dt; // time stepping interval
  uint stepCount=0; // number of simulation steps

public:
  Simulator(const char* modelFile, double dt=.01);
  void step();
  void loop();
};
