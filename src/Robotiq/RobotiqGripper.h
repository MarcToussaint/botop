#pragma once

#include <Franka/controlEmulator.h>
#include <boost/asio/serial_port.hpp>
#include <boost/asio.hpp>

// all values taken from docs of gripper
const double MAX_WIDTH = 0.85; // in mm

const double MIN_FORCE = 20;
const double MAX_FORCE = 235; // in Newton

const double MIN_SPEED = 0.02;
const double MAX_SPEED = 0.15; // in  mm/s

using namespace boost::asio;

struct RobotiqGripper : rai::GripperAbstraction {
    double maxWidth;

    RobotiqGripper(uint whichRobot=0);

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
