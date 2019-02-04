#include <franka/gripper.h>
#include "gripper.h"

const char *gripperIpAddresses[2] = {"172.16.0.2", "172.17.0.2"};

FrankaGripper::FrankaGripper(uint whichRobot)
  : Thread(STRING("FrankaGripper_"<<whichRobot)){
  //-- choose robot/ipAddress
  CHECK_LE(whichRobot, 1, "");
  gripper = make_shared<franka::Gripper>(gripperIpAddresses[whichRobot]);
  franka::GripperState gripper_state = gripper->readOnce();
  maxWidth = gripper_state.max_width;
  LOG(0) <<"gripper max width:" <<maxWidth;
  if(maxWidth<.05){
    LOG(0) <<" -- no reasonable value -> homing gripper";
    homing();
  }
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
