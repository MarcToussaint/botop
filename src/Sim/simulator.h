#pragma once

#include <RosCom/roscom.h>
#include <Algo/spline.h>
#include <Gui/opengl.h>
#include <rai_msgs/MotionReference.h>

struct Simulator{
private:
  RosCom ROS;  // communication with ROS
  rai::KinematicWorld K, K_disp, K_ref; // kinematic configurations (K_disp only to display)
  OpenGL gl; // display
  Var<rai_msgs::MotionReference> ref; // communication variable with ROS
  std::shared_ptr<Subscriber<rai_msgs::MotionReference>> subRef; //subscriber
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
