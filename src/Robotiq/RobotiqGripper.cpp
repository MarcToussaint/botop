#include "RobotiqGripper.h"

/* documentation:
https://assets.robotiq.com/website-assets/support_documents/document/2F-85_2F-140_General_PDF_20210623.pdf
*/

RobotiqGripper::RobotiqGripper(uint whichRobot) {
  //-- choose robot/ipAddress
  CHECK_LE(whichRobot, 1, "");
//  robotiqGripper = make_shared<robotiq::Gripper>(gripperIpAddresses[whichRobot]);

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

void RobotiqGripper::open(double width, double speed){
  //convert values to ints between 0 and 256 (one byte)
  int val_width = int(256* width/MAX_WIDTH);

  std::cout<<"Int value for width is: "<<val_width<<std::endl;

  //convert to hex string
  std::stringstream stream;
  stream <<std::hex <<val_width;
  std::cout<<"hex value for width is: "<< stream.str()<< std::endl;

  serialPort->write_some(buffer(
                           "\x09\x10\x03\xE8\x00\x03\x06\x09\x00\x00\x00\xFF\xFF\x72\x19",
                           15));
}


void RobotiqGripper::close(double force, double width, double speed){
  serialPort->write_some(buffer(
                           "\x09\x10\x03\xE8\x00\x03\x06\x09\x00\x00\xFF\xFF\xFF\x42\x29",
                           15));
}

double RobotiqGripper::pos(){
  LOG(0) <<"NIY";
  return 0;
}

void RobotiqGripper::waitForIdle() {
  LOG(0) <<"NIY";
}
