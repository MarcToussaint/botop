#include "RobotiqGripper.h"

//const char *gripperIpAddresses[2] = {"172.16.0.2", "172.17.0.2"};


RobotiqGripper::RobotiqGripper(uint whichRobot)
: Thread(STRING("RobotiqGripper_"<<whichRobot))
, cmd(this, false){
  //-- choose robot/ipAddress
  CHECK_LE(whichRobot, 1, "");
//  robotiqGripper = make_shared<robotiq::Gripper>(gripperIpAddresses[whichRobot]);

    auto cmdSet = cmd.set();
    cmdSet->cmd = GripperCmdMsg::Command::_done;

  threadOpen();


  io_service io;
  serialPort = std::make_shared<serial_port>(io);

  serialPort->open("/dev/ttyUSB0");

  serialPort->set_option(serial_port_base::baud_rate(115200));
  serialPort->set_option(serial_port_base::stop_bits(serial_port_base::stop_bits::one));
  //serialPort->set_option(serial_port_base::ce(8));
  serialPort->set_option(serial_port_base::parity(serial_port_base::parity::none));

  // send initialization command
  serialPort->write_some(buffer(
          "\x09\x10\x03\xE8\x00\x03\x06\x00\x00\x00\x00\x00\x00\x73\x30",
                                15));
  std::cout<<"hex value for init is: "<< "\x09\x10\x03\xE8\x00\x03\x06\x00\x00\x00\x00\x00\x00\x73\x30"<< std::endl;


}


void RobotiqGripper::homing(){
  {
    auto cmdSet = cmd.set();
    cmdSet->cmd = GripperCmdMsg::Command::_home;
  }
  threadStep();
}

void RobotiqGripper::open(double width, double speed){
  {
    auto cmdSet = cmd.set();
    cmdSet->cmd = GripperCmdMsg::Command::_open;
    cmdSet->width = width;
    cmdSet->speed = speed;
  }
  threadStep();
}


void RobotiqGripper::close(double force, double width, double speed){
  {
    auto cmdSet = cmd.set();
    cmdSet->cmd = GripperCmdMsg::Command::_close;
    cmdSet->force = force;
    cmdSet->width = width;
    cmdSet->speed = speed;
  }
  threadStep();
}

double RobotiqGripper::pos(){
  //TODO
  std::cout<<"Hello"<<endl;
  return 0;
}

bool RobotiqGripper::isGrasped(){
  //TODO
  return true;
}

void RobotiqGripper::step() {

  GripperCmdMsg msg = cmd.set();

  if(msg.cmd==msg._done) return;

  if(msg.width>MAX_WIDTH){
    LOG(-1) <<"width " <<msg.width <<" is too large (max:" <<MAX_WIDTH <<')';
  }

  bool ret=true;
  int val_width, val_force;
  std::stringstream stream;


  switch(msg.cmd){
    case msg._close:
        serialPort->write_some(buffer(
                "\x09\x10\x03\xE8\x00\x03\x06\x09\x00\x00\xFF\xFF\xFF\x42\x29",
                15));
          break;
    case msg._open:
      //convert values to ints between 0 and 256 (one byte)
      val_width = int(256* msg.width/MAX_WIDTH);

      std::cout<<"Int value for width is: "<<val_width<<std::endl;

      //convert to hex string
      stream << std::hex << val_width;
      std::cout<<"hex value for width is: "<< stream.str()<< std::endl;

      serialPort->write_some(buffer(
              "\x09\x10\x03\xE8\x00\x03\x06\x09\x00\x00\x00\xFF\xFF\x72\x19",
              15));
      //ret = frankaGripper->move(msg.width, msg.speed);
      break;
    case msg._home:
      //ret = frankaGripper->homing();
    case msg._done:
      break;
  }
  msg.cmd = msg._done;
  return;

  //return value means something else
  //if(!ret) LOG(-1) <<"gripper command " <<msg.cmd <<" failed (" <<msg.width <<' ' <<msg.speed <<')' <<endl;
  //LOG(0) <<"gripper command " <<msg.cmd <<" SEND (" <<msg.width <<' ' <<msg.speed <<' ' <<msg.force  <<')' <<endl;
}

void RobotiqGripper::waitForIdle() {
  return Thread::waitForIdle();
}
