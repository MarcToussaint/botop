#include <franka/gripper.h>
#include "gripper.h"

FrankaGripper::FrankaGripper(){
  gripper = make_shared<franka::Gripper>("172.16.0.2");
  franka::GripperState gripper_state = gripper->readOnce();
  maxWidth = gripper_state.max_width;
  LOG(0) <<"gripper max width:" <<maxWidth;
  if(maxWidth<.05) homing();
}

void FrankaGripper::homing(){
  gripper->homing();
  franka::GripperState gripper_state = gripper->readOnce();
  maxWidth = gripper_state.max_width;
  LOG(0) <<"gripper max width:" <<maxWidth;
}

bool FrankaGripper::open(double width, double speed){
  if(width>maxWidth){
    LOG(-1) <<"width " <<width <<" is too large (max:" <<maxWidth <<')';
    width = maxWidth - .001;
  }
  bool ret = gripper->move(width, speed);
  if(!ret) LOG(-1) <<"gripper move failed (" <<width <<' ' <<speed <<')' <<endl;
  return ret;
}

bool FrankaGripper::close(double force, double width, double speed){
  if(width>maxWidth){
    LOG(-1) <<"width " <<width <<" is too large (max:" <<maxWidth <<')';
    width = maxWidth - .001;
  }
  bool ret = gripper->grasp(width, speed, force);
//  if(!ret) LOG(-1) <<"grasp failed" <<endl;
  return ret;
}

double FrankaGripper::pos(){
  franka::GripperState gripper_state = gripper->readOnce();
  return gripper_state.width;
}

bool FrankaGripper::isGrasped(){
  franka::GripperState gripper_state = gripper->readOnce();
  return gripper_state.is_grasped;
}
