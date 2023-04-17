#include "RobotiqGripper.h"

#ifdef RAI_ROBOTIQ

#include <cstdint>
/* documentation:
https://assets.robotiq.com/website-assets/support_documents/document/2F-85_2F-140_General_PDF_20210623.pdf
*/

// all values taken from docs of gripper
const double MAX_WIDTH = 0.85; // in mm

const double MIN_FORCE = 20;
const double MAX_FORCE = 235; // in Newton

const double MIN_SPEED = 0.02;
const double MAX_SPEED = 0.15; // in  mm/s

static int msg_len=0;
static uint8_t msg[301];

const char *robotiqSerialPorts[2] = {"/dev/ttyUSB0", "/dev/ttyUSB1"};

void writeHex(uint8_t *msg, int len){
  cout <<"\nSERIAL MSG: ";
  for(int i=0;i<len;i++) printf("x%02hhx", msg[i]);
  cout <<endl;
}

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

  boost::asio::io_service io;
  serialPort = std::make_shared<boost::asio::serial_port>(io);

  serialPort->open(robotiqSerialPorts[whichRobot]);

  serialPort->set_option(boost::asio::serial_port_base::baud_rate(115200));
  serialPort->set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
  //serialPort->set_option(serial_port_base::ce(8));
  serialPort->set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));

  // send initialization command09 10 03 E8 00 03 01 30 -- NOT NECESSARY! closes gripper
  if(rai::getParameter<bool>("bot/initRobotiq", true)){
     serialPort->write_some(boost::asio::buffer(
                              "\x09\x10\x03\xE8\x00\x03\x06\x00\x00\x00\x00\x00\x00\x73\x30",
                              15));
     msg_len = serialPort->read_some(boost::asio::buffer(msg, 300));
     //  writeHex(msg, msg_len);
     //  response: 09 10 03 E8 00 03 01 30
  }
}

RobotiqGripper::~RobotiqGripper(){
//  serialPort->cancel();
//  serialPort->close();
  serialPort.reset();
}

void RobotiqGripper::goTo(double force, double width, double speed){
  uint8_t cmd[16] = "\x09\x10\x03\xE8\x00\x03\x06\x09\x00\x00\xFF\xFF\xFF\x42\x29"; //template command (p.57 of documentation)

//  cout <<"\nBEFORE: ";
//  for(uint i=0;i<sizeof(cmd);i++) printf("x%02hhx", cmd[i]);
//  cout <<endl;

  //width
  rai::clip(force, 0., 1.);
  rai::clip(width, 0., 1.);
  rai::clip(speed, 0., 1.);
  cmd[10]=(byte)(255.*(1.-width));
  cmd[11]=(byte)(255.*speed);
  cmd[12]=(byte)(255.*force);

  //recompute & overwrite CRC
  uint16_t r = rq_com_compute_crc(cmd, 13);
  cmd[13] = ((byte*)&r)[0];
  cmd[14] = ((byte*)&r)[1];

//  cout <<"\nAFTER:  ";
//  for(uint i=0;i<sizeof(cmd);i++) printf("x%02hhx", cmd[i]);
//  cout <<endl;

  serialPort->write_some(boost::asio::buffer(cmd, 15));
  msg_len = serialPort->read_some(boost::asio::buffer(msg, 300));
//  writeHex(msg, msg_len);
//  response: 09 10 03 E8 00 03 01 30
}

double RobotiqGripper::pos(){
  getStatus();
  if(msg_len==11){
    return (255-int(msg[7]))/255.;
  }
  return 0;
}

bool RobotiqGripper::isDone() {
  getStatus();
  if(msg_len==11 && msg[3]&0xc0) return true; //zero gOBJ means 'fingers are in motion'
  return false;
}

void RobotiqGripper::getStatus(){
  uint8_t cmd[9] = "\x09\x03\x07\xD0\x00\x03\x04\x0E";
  serialPort->write_some(boost::asio::buffer(cmd, 8));
  msg_len = serialPort->read_some(boost::asio::buffer(msg, 300));
//  writeHex(msg, msg_len);
//  if(msg_len==11){ //that's the right response!
//    cout <<"status:" <<(int)msg[3] <<" goto bit: " <<int(msg[3]&0x8) <<" status bits: " <<int(msg[3]&0xc0) <<endl;
//    cout <<"position:" <<(int)msg[7] <<endl;
//    cout <<"current:" <<(int)msg[8] <<endl;
//  }
}

#else

RobotiqGripper::RobotiqGripper(uint whichRobot){ NICO }
RobotiqGripper::~RobotiqGripper(){ NICO }
void RobotiqGripper::goTo(double force, double width, double speed){ NICO }
double RobotiqGripper::pos(){ NICO }
bool RobotiqGripper::isDone(){ NICO }
void RobotiqGripper::getStatus(){ NICO }

#endif
