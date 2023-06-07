#include "FrankaGripper.h"

#ifdef RAI_FRANKA

#include <franka/gripper.h>

const char *gripperIpAddresses[2] = {"172.16.0.2", "172.17.0.2"};

FrankaGripper::FrankaGripper(uint whichRobot)
  : Thread(STRING("FrankaGripper_"<<whichRobot))
  , cmd(this, false){
  //-- choose robot/ipAddress
  CHECK_LE(whichRobot, 1, "");
  LOG(0) <<"launching FrankaGripper " <<whichRobot <<" at " <<gripperIpAddresses[whichRobot];
  frankaGripper = make_shared<franka::Gripper>(gripperIpAddresses[whichRobot]);
  franka::GripperState gripper_state = frankaGripper->readOnce();
  maxWidth = gripper_state.max_width;
  LOG(0) <<"gripper max width:" <<maxWidth;
  if(maxWidth<.05){
    LOG(0) <<" -- no reasonable value -> homing gripper";
    homing();
  }

  threadOpen();
}

FrankaGripper::~FrankaGripper(){
  threadClose();
  LOG(0) <<"shutting down FrankaGripper";
}

void FrankaGripper::homing(){
  {
    auto cmdSet = cmd.set();
    cmdSet->cmd = GripperCmdMsg::Command::_home;
  }
  threadStep();
}

void FrankaGripper::open(double width, double speed){
  {
    auto cmdSet = cmd.set();
    cmdSet->cmd = GripperCmdMsg::Command::_open;
    cmdSet->width = width;
    cmdSet->speed = speed;
  }
  threadStep();
}

void FrankaGripper::close(double force, double width, double speed){
  {
    auto cmdSet = cmd.set();
    cmdSet->cmd = GripperCmdMsg::Command::_close;
    cmdSet->force = force;
    cmdSet->width = width;
    cmdSet->speed = speed;
  }
  threadStep();
}

double FrankaGripper::pos(){
  franka::GripperState gripper_state = frankaGripper->readOnce();
  return gripper_state.width;
}

bool FrankaGripper::isGrasped(){
  franka::GripperState gripper_state = frankaGripper->readOnce();
  return gripper_state.is_grasped;
}

void FrankaGripper::step() {
  GripperCmdMsg msg = cmd.set();

  if(msg.cmd==msg._done) return;

  if(msg.width>maxWidth){
      LOG(-1) <<"width " <<msg.width <<" is too large (max:" <<maxWidth <<')';
      msg.width = maxWidth - .001;
  }

  switch(msg.cmd){
    case msg._close:
      frankaGripper->grasp(msg.width, msg.speed, msg.force, 0.08, 0.08);
      break;
    case msg._open:
      frankaGripper->move(msg.width, msg.speed);
      break;
    case msg._home:
      frankaGripper->homing();
      break;
    case msg._done:
      break;
  }

  //return value means something else
  //if(!ret) LOG(-1) <<"gripper command " <<msg.cmd <<" failed (" <<msg.width <<' ' <<msg.speed <<')' <<endl;
  //LOG(0) <<"gripper command " <<msg.cmd <<" SEND (" <<msg.width <<' ' <<msg.speed <<' ' <<msg.force  <<')' <<endl;

  msg.cmd = msg._done;
}

#else //RAI_FRANKA

FrankaGripper::FrankaGripper(uint whichRobot)
  : Thread(STRING("FrankaGripper_"<<whichRobot))
  , cmd(this, false){ NICO }
FrankaGripper::~FrankaGripper(){ threadClose(); }
void FrankaGripper::homing(){ NICO }
void FrankaGripper::open(double width, double speed){ NICO }
void FrankaGripper::close(double force, double width, double speed){ NICO }
double FrankaGripper::pos(){ NICO }
bool FrankaGripper::isGrasped(){ NICO }
void FrankaGripper::step() { NICO }

#endif
