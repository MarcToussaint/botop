#pragma once

#include <Franka/controlEmulator.h>
#include <boost/asio/serial_port.hpp>
#include <boost/asio.hpp>

struct RobotiqGripper : rai::GripperAbstraction {
  double maxWidth;

  RobotiqGripper(uint whichRobot=0);
  ~RobotiqGripper();

  void goTo(double force, double width, double speed);

  void open(double width, double speed){ goTo(0., width, speed); }
  void close(double force, double width, double speed){ goTo(force, width, speed); }

  double pos();

  bool isDone();

private:
  std::shared_ptr<boost::asio::serial_port> serialPort;

  void getStatus();
};
