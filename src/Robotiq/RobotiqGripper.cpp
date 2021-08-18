#include "RobotiqGripper.h"

#include <cstdint>
/* documentation:
https://assets.robotiq.com/website-assets/support_documents/document/2F-85_2F-140_General_PDF_20210623.pdf
*/

const double MAX_WIDTH = 0.85; // in mm

const double MIN_FORCE = 20;
const double MAX_FORCE = 235; // in Newton

const double MIN_SPEED = 0.02;
const double MAX_SPEED = 0.15; // in  mm/s


/**
  METHOD TAKEN FROM git/robotiq/robotiq_ft_sensor/src/rq_sensor_com.cpp !!!

 * \fn static UINT_16 rq_com_compute_crc(UINT_8 * adr, INT_32 length )
 * \param adr, Address of the first byte
 * \param length Length of the buffer on which the crc is computed
 * \return Value of the crc
 */
static uint16_t rq_com_compute_crc(uint8_t const * adr, int32_t length ) {
  uint16_t CRC_calc = 0xFFFF;
  int32_t j=0;
  int32_t k=0;

  //precondition, null pointer
  if (adr == NULL) return 0;

  //While there are bytes left in the message
  while (j < length) {
    if (j==0) CRC_calc ^= *adr & 0xFF; //If it's the first byte
    else CRC_calc ^= *adr; //Else we'll use an XOR on the word
    k=0;

    //While the byte is not completed
    while (k < 8) {
      //If the last bit is a 1
      if (CRC_calc & 0x0001)  CRC_calc =  (CRC_calc >> 1)^ 0xA001;	//Shifts 1 bit to the right and XOR with the polynomial factor
      else CRC_calc >>= 1;			//Shifts 1 bit to the right
      k++;
    }

    //Increments address and address counter
    adr++;
    j++;
  }

  return CRC_calc;
}

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

RobotiqGripper::~RobotiqGripper(){
  serialPort->close();
}

void RobotiqGripper::open(double width, double speed){
  //convert values to ints between 0 and 256 (one byte)
  int val_width = int(256* width/MAX_WIDTH);

  std::cout<<"Int value for width is: "<<val_width<<std::endl;

  //convert to hex string
  std::stringstream stream;
  stream <<std::hex <<val_width;
  std::cout<<"hex value for width is: "<< stream.str()<< std::endl;

  byte cmd[16] = "\x09\x10\x03\xE8\x00\x03\x06\x09\x00\x00\x00\xFF\xFF\x72\x19"; //template command (p.60 of documentation)

  serialPort->write_some(buffer(
                           cmd,
                           15));
}


void RobotiqGripper::close(double force, double width, double speed){
  byte cmd[16] = "\x09\x10\x03\xE8\x00\x03\x06\x09\x00\x00\xFF\xFF\xFF\x42\x29"; //template command (p.57 of documentation)

//  cout <<"\nBEFORE: ";
//  for(uint i=0;i<sizeof(cmd);i++) printf("x%02hhx", cmd[i]);
//  cout <<endl;

  //width
  if(width>MAX_WIDTH){
    LOG(0) <<"width " <<width <<" exceeds max width " <<MAX_WIDTH <<" -- clipping";
    width=MAX_WIDTH;
  }
//  cmd[10]=(byte)(255.*width/MAX_WIDTH);

  //speed
  if(speed>MAX_SPEED){
    LOG(0) <<"speed " <<speed <<" exceeds max speed " <<MAX_SPEED <<" -- clipping";
    speed=MAX_SPEED;
  }
  cmd[11]=(byte)(255.*speed/MAX_SPEED);

  //force
  if(force>MAX_FORCE){
    LOG(0) <<"force " <<force <<" exceeds max force " <<MAX_FORCE <<" -- clipping";
    force=MAX_FORCE;
  }
  cmd[12]=(byte)(255.*force/MAX_FORCE);

  //recompute & overwrite CRC
  uint16_t r = rq_com_compute_crc(cmd, 13);
  cmd[13] = ((byte*)&r)[0];
  cmd[14] = ((byte*)&r)[1];

//  cout <<"\nAFTER:  ";
//  for(uint i=0;i<sizeof(cmd);i++) printf("x%02hhx", cmd[i]);
//  cout <<endl;

  serialPort->write_some(buffer(cmd, 15));
}

double RobotiqGripper::pos(){
  LOG(0) <<"NIY";
  return 0;
}

void RobotiqGripper::waitForIdle() {
  LOG(0) <<"NIY";
}
