
#include "ranger.h"

#define RAI_RANGER
#ifdef RAI_RANGER

// #define FAKE
#define JOINT_DIM 1

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
#include <chrono> // For time tracking


const char* ranger_address = "can0";

struct RangerController{
  int socket_fd;
  int ranger_mode;
  arr qLast, sLast; //q: joint state; s: motor state
  struct ifreq ifr;
  struct sockaddr_can addr;
  uint32_t id_control_mode;
  struct can_frame frame_control_mode;
  double position;
  std::chrono::time_point<std::chrono::steady_clock> last_time;


  RangerController(const char* address, int Kp, int Ki, int Kd)
  {
    position = 0.0; // Position in meters
    last_time = std::chrono::steady_clock::now(); // For time tracking
    #ifdef FAKE
    LOG(0) << "Conneted to ranger ;)";
    #else
    socket_fd = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (socket_fd < 0) {
      LOG(-1) << "Error creating socket: " << strerror(errno) << std::endl;
    }

    std::strncpy(ifr.ifr_name, address, IFNAMSIZ - 1);
    if (ioctl(socket_fd, SIOCGIFINDEX, &ifr) < 0) {
      LOG(-1) << "Error getting interface index: " << strerror(errno) << std::endl;
      close(socket_fd);
    }

    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(socket_fd, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
      LOG(-1) << "Error binding socket: " << strerror(errno) << std::endl;
      close(socket_fd);
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
      LOG(-1) << "Error sending control mode message: " << strerror(errno) << std::endl;
      close(socket_fd);
    }
    std::cout << "Control mode message sent: ID = " << std::hex << id_control_mode << std::endl;
    #endif
    qLast = zeros(JOINT_DIM);
    sLast = zeros(1);
  }

  ~RangerController(){
    #ifdef FAKE
    #else
    close(socket_fd);
    #endif
  }

  arr wheelVelToJointVel(arr s){
    //return Jacobian based on qLast
    double wheel_radius = .2;
    arr q_vel(JOINT_DIM);
    q_vel = s;
    return q_vel;
  }

  arr jointVelToWheelVel(arr q){
    //return Jacobian based on qLast
    double wheel_radius = .2;
    arr s_vel(JOINT_DIM);
    s_vel = q;
    return s_vel;
  }

  double getLinearVelocity(){
    while (true) {
      struct can_frame recv_frame;
      int nbytes = read(socket_fd, &recv_frame, sizeof(recv_frame));
      if (nbytes < 0) {
          LOG(-1) << "Error reading message: " << strerror(errno) << std::endl;
          break;
      }

      if (nbytes < sizeof(struct can_frame)) {
          LOG(-1) << "Incomplete CAN frame received" << std::endl;
          continue;
      }

      if (recv_frame.can_id == 0x221) {
        // Extract the linear velocity (bytes 0 and 1)
        int16_t linear_velocity = (recv_frame.data[0] << 8) | recv_frame.data[1];
        // Convert to m/s (unit in feedback is 0.001 m/s)
        double linear_velocity_mps = linear_velocity / 1000.0;

        std::cout << "Current Linear Velocity: " << linear_velocity_mps << " m/s" << std::endl;

        return linear_velocity_mps;
      }
    }
    return 0.0;
  }

  void getState(arr& q, arr& qDot){
    //-- get motor state
    arr s(1);
    arr sDot(1);

    double linear_vel = getLinearVelocity();

    auto current_time = std::chrono::steady_clock::now();
    std::chrono::duration<double> elapsed_time = current_time - last_time;
    last_time = current_time;

    // Integrate velocity to estimate position
    position += linear_vel * elapsed_time.count();

    s = arr{position}; // Wheel positions
    sDot = linear_vel; // Wheel velocities

    arr sDelta = s - sLast;

    //-- convert to joint state
    // DO MAGIC HERE: convert the sDelta (change in motor positions) to a qDelta (change in joint state)
    arr qDelta = wheelVelToJointVel(sDelta);

    q = qLast + qDelta;
    qDot = wheelVelToJointVel(sDot);
    qLast = q;
    sLast = s;
  }

  void setVelocities(const arr& qDotTarget)
  {
    #ifdef FAKE
    #else
    // Safety
    // TODO: Check if velocities are too high

    // Step 2: Define motion control command
    uint32_t id_motion = 0x111; 
    int16_t linear_speed = qDotTarget.elem(0) * 1000; // Linear speed in mm/s
    int16_t spin_speed = 0;       // Spin speed in 0.001 rad/s
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
    frame_motion.data[4] = reserved;                   // Reserved byte
    std::memset(frame_motion.data + 5, 0, 3);          // Remaining reserved bytes

    // Send the motion command message
    if (write(socket_fd, &frame_motion, sizeof(frame_motion)) != sizeof(frame_motion)) {
      LOG(-1) << "Error sending motion command message: " << strerror(errno) << std::endl;
      close(socket_fd);
    }
    std::cout << "Motion command message sent: ID = " << std::hex << id_motion << std::endl;
    #endif
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

  CHECK_EQ(qIndices.N, JOINT_DIM, "");
  qIndices_max = rai::max(qIndices);

  //-- basic Kp Kd settings for reference control mode
  Kp = rai::getParameter<int>("Ranger/Kp", 20);
  Ki = rai::getParameter<int>("Ranger/Ki", 0);
  Kd = rai::getParameter<int>("Ranger/Kd", 10);

  LOG(0) << "Ranger/PID: Kp:" << Kp << " Ki:" << Ki << " Kd:" << Kd;

  //-- get robot address
  // address = rai::getParameter<rai::String>("Ranger/address", "can0");
  address = ranger_address;

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

  for(uint i=0; i<JOINT_DIM; i++){
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
    for(uint i=0;i<JOINT_DIM;i++){
      stateSet->q.elem(qIndices(i)) = q_real.elem(i);
      stateSet->qDot.elem(qIndices(i)) = qDot_real.elem(i);
    }
    state_q_real = stateSet->q;
    state_qDot_real = stateSet->qDot;
  }

  //-- get current ctrl command
  arr q_ref, qDot_ref, qDDot_ref, P_compliance;
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
      q_ref.resize(JOINT_DIM);
      for(uint i=0; i<JOINT_DIM; i++) q_ref.elem(i) = cmd_q_ref.elem(qIndices(i));
    }
    if(cmd_qDot_ref.N){
      qDot_ref.resize(JOINT_DIM);
      for(uint i=0; i<JOINT_DIM; i++) qDot_ref.elem(i) = cmd_qDot_ref.elem(qIndices(i));
    }
    if(cmd_qDDot_ref.N){
      qDDot_ref.resize(JOINT_DIM);
      for(uint i=0; i<JOINT_DIM; i++) qDDot_ref.elem(i) = cmd_qDDot_ref.elem(qIndices(i));
    }
    if(cmdGet->P_compliance.N) {
      P_compliance.resize(JOINT_DIM,JOINT_DIM);
      for(uint i=0; i<JOINT_DIM; i++) for(uint j=0; j<JOINT_DIM; j++) P_compliance(i,j) = cmdGet->P_compliance(qIndices(i), qIndices(j));
    }
  }
  LOG(0) << "q_real" << q_real;
  LOG(0) << "qDot_real" << qDot_real;
  LOG(0) << "q_ref" << q_ref;
  LOG(0) << "qDot_ref" << qDot_ref;

  //-- cap the reference difference
  if(q_ref.N==JOINT_DIM){
    double err = length(q_ref - q_real);
    if(P_compliance.N){
      arr del = q_ref - q_real;
      err = ::sqrt(scalarProduct(del, P_compliance*del));
    }
    if(err>.3){ //stall!
      state.set()->stall = 2; //no progress in reference time! for at least 2 iterations (to ensure continuous stall with multiple threads)
      cout <<"STALLING - step:" <<steps <<" err: " <<err <<endl;
    }
  }

  //-- compute torques from control message depending on the control type
  arr sDotTarget;
  sDotTarget.resize(3).setZero();
  
  if(q_ref.N==JOINT_DIM && qDot_ref.N==JOINT_DIM)
  {
    arr p_term = Kp * (q_ref - q_real);
    arr d_term = Kd * qDot_ref;
    arr qDotTarget = p_term + d_term;
    sDotTarget = robot->jointVelToWheelVel(qDotTarget);
  }

  //-- data log
  if(writeData>0 && !(steps%1)){
      if(!dataFile.is_open()) dataFile.open(STRING("z.ranger"<<robotID <<".dat"));
      dataFile <<ctrlTime <<' ';
      q_real.modRaw().write(dataFile);
      q_ref.modRaw().write(dataFile);
      if(writeData>1){
      qDot_real.modRaw().write(dataFile);
      qDot_ref.modRaw().write(dataFile);
      sDotTarget.modRaw().write(dataFile);
      qDDot_ref.modRaw().write(dataFile);
      }
      dataFile <<endl;
  }

  robot->setVelocities(sDotTarget);
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
