#pragma once

#define RAI_2PI 6.283195307179587

//===========================================================================

//user interface
struct SimplexMotion{
  struct SimplexMotion_Communication* com=0;
  double maxTorque = 2.;

  SimplexMotion(const char* devPath = "/dev/hidraw0", unsigned short vendor_id = 0x04d8, unsigned short product_id = 0xf79a);
  ~SimplexMotion();

  const char* getModelName();
  const char* getSerialNumber();
  int getAddress();

  double getVoltage();
  double getMotorTemperature();
  double getMotorPosition();
  double getMotorSpeed();
  double getMotorTorque();

  void setSpeedFiltering(int filter);
  void setMaxTorque(double maxTorqueInNm =1.);

  //using the ramp & PID regulator that maps pos/vel to torques
  void setPID(int Kp, int Ki, int Kd, int KiLimit, int KdDelay, int Friction);
  void setRamps(double maxSpeed, double maxAcc);
  void setIntertia(int mass);
  double getPIDoutput(); //torque reference

  void setMode(int mode);
  void setTarget(int target);

  void runOff(){ setMode(0); }
  void runReset(){ setMode(1); }
  void runStop(){ setMode(5); }
  void runCoggingCalibration(){ setMode(110); }
  void runSpeed(double speed){ setTarget(256.*speed/RAI_2PI); setMode(33); } //using ramp & PID; activates SpeedRamp control mode
  void runPosition(double position){ setTarget(4096.*position/RAI_2PI); setMode(21); } //using ramps & PIDactivates PositionRamp control mode
  void runTorque(double torque){ setTarget(torque/maxTorque*32767); setMode(40); }

};

//===========================================================================
