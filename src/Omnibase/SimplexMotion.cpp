#include "SimplexMotion.h"
#include "SimplexMotion-com.h"

#include <iostream>

using std::cerr;
#define CHECK_RANGE(var, lo, up) if(var<lo || var>up){ cerr <<"RANGE ERROR: " <<lo <<'<' <<var <<'<' <<up; return; }

//===========================================================================

SimplexMotion::SimplexMotion(const char* devPath, unsigned short vendor_id, unsigned short product_id) {
  com = new SimplexMotion_Communication(devPath, vendor_id, product_id);
}

SimplexMotion::~SimplexMotion() {
  delete com;

}

const char* SimplexMotion::getModelName(){
  return com->readString(REG_MODEL_NAME, 20);
}

const char* SimplexMotion::getSerialNumber(){
  return com->readString(REG_SERIAL_NUMBER, 20);
}

int SimplexMotion::getAddress(){
  return com->readRegister(REG_ADDRESS, com->uns16);
}

double SimplexMotion::getVoltage(){
  return com->readRegister(REG_SUPPLY, com->uns16) * .01;
}

double SimplexMotion::getMotorTemperature(){
  return com->readRegister(REG_TEMP_MOTOR, com->uns16) * .01;
}

double SimplexMotion::getMotorPosition(){
  return com->readRegister(REG_MOTOR_POSITION, com->int32) * RAI_2PI/4096.;
}

double SimplexMotion::getMotorSpeed(){
  return com->readRegister(REG_MOTOR_SPEED, com->int16) * RAI_2PI/256.;
}

double SimplexMotion::getMotorTorque(){
  return com->readRegister(REG_MOTOR_TORQUE, com->int16) * .001;
}

void SimplexMotion::setSpeedFiltering(int filter){
  CHECK_RANGE(filter, 0, 15);
  com->writeRegister(REG_SPEED_FILTER, com->uns16, filter);
}

void SimplexMotion::setPID(int Kp, int Ki, int Kd, int KiLimit, int KdDelay, int Friction){
  CHECK_RANGE(Kp, 0, 2000);
  CHECK_RANGE(Ki, 0, 2000);
  CHECK_RANGE(Kd, 0, 2000);
  CHECK_RANGE(KiLimit, 10, 500);
  CHECK_RANGE(KdDelay, 0, 8);
  CHECK_RANGE(Friction, 0, 200);
  com->writeRegister(REG_REG_KP, com->int16, Kp);
  com->writeRegister(REG_REG_KI, com->int16, Ki);
  com->writeRegister(REG_REG_KD, com->int16, Kd);
  com->writeRegister(REG_REG_LIMIT, com->int16, KiLimit);
  com->writeRegister(REG_REG_DELAY, com->int16, KdDelay);
  com->writeRegister(REG_REG_FRICTION, com->int16, Friction);
}

//void SimplexMotion::setRamps(double maxSpeed, double maxAcc){
//  com->writeRegister(REG_RAMP_SPEED_MAX, com->int16, 256.*maxSpeed/RAI_2PI);
//  void runPosition(double position){ setTarget(4096.*position/RAI_2PI); setMode(21); } //using ramps & PIDactivates PositionRamp control mode

//}

void SimplexMotion::setMode(int mode){
  CHECK_RANGE(mode, 0, 201);
  com->writeRegister(REG_MODE, com->uns16, mode);
}

void SimplexMotion::setTarget(int target){
  com->writeRegister(REG_TARGET_INPUT, com->int32, target);
}
