#pragma once

#include <Franka/controlEmulator.h>
#include <boost/asio/serial_port.hpp>
#include <boost/asio.hpp>

// all values taken from docs of gripper

using namespace boost::asio;

struct RobotiqGripper : rai::GripperAbstraction {
  double maxWidth;

  RobotiqGripper(uint whichRobot=0);
  ~RobotiqGripper();

  void open(double width=.85, //which is 7.5cm
            double speed=.2);

  void close(double force=20,  //which is 1kg
             double width=.05, //which is 5cm
             double speed=.1);

  double pos();

  void waitForIdle();

private:
  std::shared_ptr<serial_port> serialPort;
};
