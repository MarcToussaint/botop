
#include "omnibase.h"
#include "SimplexMotion.h"

#define RAI_RANGER
#ifdef RAI_RANGER


#include <iostream>
#include <cstring>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <sys/socket.h>
#include <unistd.h>
#include <net/if.h>
#include <fcntl.h>
#include <arpa/inet.h>
#include <errno.h>
#include <sys/ioctl.h>  // Include for ioctl and SIOCGIFINDEX

struct RangerController{
  int socket_fd;
  int ranger_mode;
  arr qLast, sLast; //q: joint state; s: motor state
  struct ifreq ifr;
  struct sockaddr_can addr;
  uint32_t id_control_mode;
  struct can_frame frame_control_mode;

  RangerController(const String& address, int Kp, int Ki, int Kd)
  {
    socket_fd = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (socket_fd < 0) {
        std::cerr << "Error creating socket: " << strerror(errno) << std::endl;
        return 1;
    }

    std::strncpy(ifr.ifr_name, address, IFNAMSIZ - 1);
    if (ioctl(socket_fd, SIOCGIFINDEX, &ifr) < 0) {
        std::cerr << "Error getting interface index: " << strerror(errno) << std::endl;
        close(socket_fd);
        return 1;
    }

    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(socket_fd, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        std::cerr << "Error binding socket: " << strerror(errno) << std::endl;
        close(socket_fd);
        return 1;
    }

    // Send control mode message
    ranger_mode = 0;
    id_control_mode = 0x421; 
    frame_control_mode.can_id = id_control_mode;
    frame_control_mode.can_dlc = 8; // Data length code (number of bytes)
    frame_control_mode.data[0] = 0x01; // Byte 0 for control mode
    std::memset(frame_control_mode.data + 1, 0, 7); // Remaining bytes = 0

    // Send the control mode message
    if (write(socket_fd, &frame_control_mode, sizeof(frame_control_mode)) != sizeof(frame_control_mode)) {
        std::cerr << "Error sending control mode message: " << strerror(errno) << std::endl;
        close(socket_fd);
        return 1;
    }
    std::cout << "Control mode message sent: ID = " << std::hex << id_control_mode << std::endl;
  }

  ~RangerController(){
    close(socket_fd);
  }

  arr getJacobian(){
    //return Jacobian based on qLast

    arr J = {
      .5,          .5,            -1.,
      .5*sqrt(3.),  -.5*sqrt(3.),   0.,
      1/(3.*center2wheel),      1/(3.*center2wheel),      1./(3.*center2wheel)
    };
    J *= wheel_radius/gear_ratio;
    J.reshape(3,3);

    double phi = qLast(2);
    arr rot = eye(3);
    rot(0,0) = rot(1,1) = cos(phi);
    rot(0,1) = -sin(phi);
    rot(1,0) = sin(phi);
    J = rot * J;

    return J;
  }

  void getState(arr& q, arr& qDot){
    //-- get motor state
    arr s(motors.N);
    arr sDot(motors.N);
    for(uint i=0;i<motors.N;i++){
      s(i) = motors(i)->getMotorPosition();
      sDot(i) = motors(i)->getMotorSpeed();
    }

    //-- convert to joint state
    arr sDelta = s - sLast;
    // DO MAGIC HERE: convert the sDelta (change in motor positions) to a qDelta (change in joint state)
    arr Jacobian = getJacobian();
    arr qDelta = Jacobian * sDelta;

    q = qLast + qDelta;
    qDot = Jacobian * sDot;
    qLast = q;
    sLast = s;
  }

  void setVelocities(const arr& v)
  {
    // v[0] x vel
    // v[1] y vel
    // v[2] phi vel
    // v[3] mode

    // Safety
    v[0] = min(2000, v[0]);
    v[1] = min(2000, v[1]);
    v[2] = min(PI_M, v[2]);

    // Step 2: Define motion control command
    uint32_t id_motion = 0x111; 
    int16_t linear_speed = v[0]; // Linear speed in mm/s
    int16_t spin_speed = v[2];       // Spin speed in 0.001 rad/s
    uint8_t reserved = 0;         // Reserved byte

    // Create the data payload
    struct can_frame frame_motion;
    frame_motion.can_id = id_motion;
    frame_motion.can_dlc = 8; // Data length code (number of bytes)

    // Convert linear speed and spin speed to bytes
    frame_motion.data[0] = (linear_speed >> 8) & 0xFF; // High byte of linear speed
    frame_motion.data[1] = linear_speed & 0xFF;        // Low byte of linear speed
    frame_motion.data[2] = (spin_speed >> 8) & 0xFF;   // High byte of spin speed
    frame_motion.data[3] = spin_speed & 0xFF;          // Low byte of spin speed
    frame_motion.data[4] = reserved;                    // Reserved byte
    std::memset(frame_motion.data + 5, 0, 3);          // Remaining reserved bytes

    // Send the motion command message
    if (write(socket_fd, &frame_motion, sizeof(frame_motion)) != sizeof(frame_motion)) {
        std::cerr << "Error sending motion command message: " << strerror(errno) << std::endl;
        close(socket_fd);
        return 1;
    }
    std::cout << "Motion command message sent: ID = " << std::hex << id_motion << std::endl;
  }
};

RangerThread::RangerThread(uint robotID, const uintA &_qIndices, const Var<rai::CtrlCmdMsg> &_cmd, const Var<rai::CtrlStateMsg> &_state)
    : RobotAbstraction(_cmd, _state), Thread("RangerThread", .015){
    init(robotID, _qIndices);
    writeData = 10;
}

RangerThread::~RangerThread(){
    LOG(0) <<"shutting down Ranger " <<robotID;
    threadClose();
}

void RangerThread::init(uint _robotID, const uintA& _qIndices) {
  robotID=_robotID;
  qIndices=_qIndices;

  CHECK_EQ(qIndices.N, 3, "");
  qIndices_max = rai::max(qIndices);

  //-- basic Kp Kd settings for reference control mode
  Kp = rai::getParameter<int>("Ranger/Kp", 20);
  Ki = rai::getParameter<int>("Ranger/Ki", 0);
  Kd = rai::getParameter<int>("Ranger/Kd", 10);

  LOG(0) << "Ranger/PID: Kp:" << Kp << " Ki:" << Ki << " Kd:" << Kd;

  //-- get robot address
  address = rai::getParameter<String>("Ranger/address", "can0");

  //-- start thread and wait for first state signal
  LOG(0) <<"launching Ranger " <<robotID <<" at " <<address;

  threadLoop();
  while(Thread::step_count<2) rai::wait(.01);
}

void RangerThread::open(){
  // connect to robot
  robot = make_shared<RangerController>(address, Kp, Ki, Kd);

  //-- initialize state and ctrl with first state
  arr q_real, qDot_real;
  robot->getState(q_real, qDot_real);

  auto stateSet = state.set();

  //ensure state variables have sufficient size
  while(stateSet->q.N<=qIndices_max) stateSet->q.append(0.);
  while(stateSet->qDot.N<=qIndices_max) stateSet->qDot.append(0.);
  while(stateSet->tauExternalIntegral.N<=qIndices_max) stateSet->tauExternalIntegral.append(0.);

  for(uint i=0; i<3; i++){
    stateSet->q.elem(qIndices(i)) = q_real(i);
    stateSet->qDot.elem(qIndices(i)) = qDot_real(i);
    stateSet->tauExternalIntegral.elem(qIndices(i)) = 0.;
    stateSet->tauExternalCount=0;
  }
}

void RangerThread::step(){
  steps++;

  //-- get current state from robot
  arr q_real, qDot_real;
  robot->getState(q_real, qDot_real);

  //-- publish state & INCREMENT CTRL TIME
  arr state_q_real, state_qDot_real;
  {
    auto stateSet = state.set();
    if(robotID==0){ // if this is the lead robot, increment ctrlTime if no stall
      if(!stateSet->stall) stateSet->ctrlTime += metronome.ticInterval;
      else stateSet->stall--;
    }
    ctrlTime = stateSet->ctrlTime;
    for(uint i=0;i<3;i++){
      stateSet->q.elem(qIndices(i)) = q_real.elem(i);
      stateSet->qDot.elem(qIndices(i)) = qDot_real.elem(i);
    }
    state_q_real = stateSet->q;
    state_qDot_real = stateSet->qDot;
  }

  //-- get current ctrl command
  arr q_ref, qDot_ref, qDDot_ref, Kp_ref, Kd_ref, P_compliance;
  {
    auto cmdGet = cmd.get();

    //get commanded reference from the reference callback (e.g., sampling a spline reference)
    arr cmd_q_ref, cmd_qDot_ref, cmd_qDDot_ref;
    if(cmdGet->ref){
      cmdGet->ref->getReference(cmd_q_ref, cmd_qDot_ref, cmd_qDDot_ref, state_q_real, state_qDot_real, ctrlTime);
      CHECK(!cmd_q_ref.N || cmd_q_ref.N > qIndices_max, "");
      CHECK(!cmd_qDot_ref.N || cmd_qDot_ref.N > qIndices_max, "");
      CHECK(!cmd_qDDot_ref.N || cmd_qDDot_ref.N > qIndices_max, "");
    }

    //pick qIndices for this particular robot
    if(cmd_q_ref.N){
      q_ref.resize(3);
      for(uint i=0; i<3; i++) q_ref.elem(i) = cmd_q_ref.elem(qIndices(i));
    }
    if(cmd_qDot_ref.N){
      qDot_ref.resize(3);
      for(uint i=0; i<3; i++) qDot_ref.elem(i) = cmd_qDot_ref.elem(qIndices(i));
    }
    if(cmd_qDDot_ref.N){
      qDDot_ref.resize(3);
      for(uint i=0; i<3; i++) qDDot_ref.elem(i) = cmd_qDDot_ref.elem(qIndices(i));
    }
    if(cmdGet->Kp.d0 >= 3 && cmdGet->Kp.d1 >=3 && cmdGet->Kp.d0 == cmdGet->Kp.d1){
      Kp_ref.resize(3, 3);
      for(uint i=0; i<3; i++) for(uint j=0; j<3; j++) Kp_ref(i, j) = cmdGet->Kp(qIndices(i), qIndices(j));
    }
    if(cmdGet->Kd.d0 >= 3 && cmdGet->Kd.d1 >=3 && cmdGet->Kd.d0 == cmdGet->Kd.d1){
      Kd_ref.resize(3, 3);
      for(uint i=0; i<3; i++) for(uint j=0; j<3; j++) Kd_ref(i, j) = cmdGet->Kd(qIndices(i), qIndices(j));
    }
    if(cmdGet->P_compliance.N) {
      P_compliance.resize(3,3);
      for(uint i=0; i<3; i++) for(uint j=0; j<3; j++) P_compliance(i,j) = cmdGet->P_compliance(qIndices(i), qIndices(j));
    }

  }

  //-- cap the reference difference
  if(q_ref.N==3){
    double err = length(q_ref - q_real);
    if(P_compliance.N){
      arr del = q_ref - q_real;
      err = ::sqrt(scalarProduct(del, P_compliance*del));
    }
    if(err>1.05){ //stall!
      state.set()->stall = 2; //no progress in reference time! for at least 2 iterations (to ensure continuous stall with multiple threads)
      cout <<"STALLING - step:" <<steps <<" err: " <<err <<endl;
    }
  }

  //-- compute torques from control message depending on the control type
  arr v;

  //-- initialize zero torques
  v.resize(3).setZero();
  arr J = robot->getJacobian();
  arr Jinv = pseudoInverse(J, NoArr, 1e-6);

  if(q_ref.N==3){
    arr delta_motor = Jinv * (q_ref - q_real);
    v += outer_Kp * delta_motor;
  }

  if(qDot_ref.N==3){
    v += Jinv * qDot_ref;
  }

  //-- data log
  if(writeData>0 && !(steps%1)){
      if(!dataFile.is_open()) dataFile.open(STRING("z.ranger"<<robotID <<".dat"));
      dataFile <<ctrlTime <<' '; //single number
      q_real.modRaw().write(dataFile); //3
      q_ref.modRaw().write(dataFile); //3
      if(writeData>1){
      qDot_real.modRaw().write(dataFile); //3
      qDot_ref.modRaw().write(dataFile); //3
      v.modRaw().write(dataFile); //3
      qDDot_ref.modRaw().write(dataFile);
      }
      dataFile <<endl;
  }
  robot->setVelocities(v);
}

void RangerThread::close(){
  robot.reset();
}

#else // RAI_RANGER

RangerThread::~RangerThread(){ NICO }
void RangerThread::init(uint _robotID, const uintA& _qIndices) { NICO }
void RangerThread::step(){ NICO }
void RangerThread::open(){ NICO }
void RangerThread::close(){ NICO }

#endif
