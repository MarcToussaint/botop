
#include "ranger.h"

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
#define PI 3.1415926
#define DEBUG 0

// TODO: setLights func in botop (also like flashing lights or so)

struct RangerController
{
  int socket_fd;
  int mode; // 0: Oblique, 1: Spin, 2: Ackermann
  arr qLast, sLast; //q: joint state; s: motor state
  double real_steering_rad;
  double real_steering_rad_s;
  double target_steering_rad_s;

  RangerController(const char* address)
  {
    mode = 0;
    qLast = zeros(3);
    real_steering_rad = 0.0; // CAREFULL!! Only updates when you call "get_qDot_oblique".
    real_steering_rad_s = 0.0; // CAREFULL!! Only updates when you call "get_qDot_oblique".
    target_steering_rad_s = 0.0; // CAREFULL!! Only updates when you call "setVelocities".
    // sLast = zeros(8); // (FL, FR, BL, BR) Angle, (FL, FR, BL, BR) Wheel
    sLast = zeros(4); // FL, FR, BL, BR

    //--------- SETTING UP CAN INTERFACE ---------//
    struct ifreq ifr;
    struct sockaddr_can addr;
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

    // CAN stuff
    struct can_frame frame_control_mode;
    uint32_t id_control_mode = 0x421;
    frame_control_mode.can_id = id_control_mode;
    frame_control_mode.can_dlc = 8; // Data length code (number of bytes)
    frame_control_mode.data[0] = 0x01; // Byte 0 for control mode
    std::memset(frame_control_mode.data + 1, 0, 7); // Remaining bytes = 0

    send_package(frame_control_mode);

    // Control mode stuff
    set_control_mode(0x01);
    // 0x00 front and rear Ackerman mode
    // 0x01 oblique motion mode
    // 0x02 spin mode
    // 0x03 Parking mode
    
    std::cout << "Control mode message sent: ID = " << std::hex << id_control_mode << std::endl;
    
    sLast = get_s();
  }

  ~RangerController(){
    close(socket_fd);
  }

  void set_control_mode(canid_t type) {
    struct can_frame frame_control_mode;
    uint32_t id_control_mode = 0x141; 
    frame_control_mode.can_id = id_control_mode;
    frame_control_mode.can_dlc = 1; // Data length code (number of bytes)
    frame_control_mode.data[0] = type;
    std::memset(frame_control_mode.data + 1, 0, 7); // Remaining bytes = 0

    send_package(frame_control_mode);
  }

  arr get_s()
  {
    struct can_frame recv_frame;

    // Front Wheels
    recv_frame = recv_package(0x311);

    int32_t front_left_wheel_odom_mm = (recv_frame.data[0] << 24) | (recv_frame.data[1] << 16) | (recv_frame.data[2] << 8) | recv_frame.data[3];
    int32_t front_right_wheel_odom_mm = (recv_frame.data[4] << 24) | (recv_frame.data[5] << 16) | (recv_frame.data[6] << 8) | recv_frame.data[7];
    
    double front_left_wheel_odom = static_cast<double>(front_left_wheel_odom_mm) / 1000;
    double front_right_wheel_odom = static_cast<double>(front_right_wheel_odom_mm) / 1000;

    // Rear Wheels
    recv_frame = recv_package(0x312);

    int32_t rear_left_wheel_odom_mm = (recv_frame.data[0] << 24) | (recv_frame.data[1] << 16) | (recv_frame.data[2] << 8) | recv_frame.data[3];
    int32_t rear_right_wheel_odom_mm = (recv_frame.data[4] << 24) | (recv_frame.data[5] << 16) | (recv_frame.data[6] << 8) | recv_frame.data[7];
    
    double rear_left_wheel_odom = static_cast<double>(rear_left_wheel_odom_mm) / 1000;
    double rear_right_wheel_odom = static_cast<double>(rear_right_wheel_odom_mm) / 1000;

    #if DEBUG
      std::cout << "------------- Odometry -------------" << std::endl;
      std::cout << "Front Left Wheel Odometer: " << front_left_wheel_odom << " m" << std::endl;
      std::cout << "Front Right Wheel Odometer: " << front_right_wheel_odom << " m" << std::endl;
      std::cout << "Rear Left Wheel Odometer: " << rear_left_wheel_odom << " m" << std::endl;
      std::cout << "Rear Right Wheel Odometer: " << rear_right_wheel_odom << " m" << std::endl;
    #endif

    return arr{front_left_wheel_odom, front_right_wheel_odom, rear_left_wheel_odom, rear_right_wheel_odom};
  }

  arr get_qDot_oblique()
  {
    struct can_frame recv_frame = recv_package(0x221);
    
    // Extract actual speed and steering angle
    int16_t actual_speed = (recv_frame.data[0] << 8) | recv_frame.data[1];
    int16_t actual_steering_angle = (recv_frame.data[6] << 8) | recv_frame.data[7];

    double actual_speed_mps = actual_speed / 1000.0; // Convert mm/s to m/s
    real_steering_rad = actual_steering_angle / 1000.0; // Convert mrad to rad
    real_steering_rad_s = real_steering_rad;
    #if DEBUG
      std::cout << "Actual Steering Angle before: " << real_steering_rad << " rad" << std::endl;
    #endif

    real_steering_rad *= -1;
    if (actual_speed_mps < 0) { // Lower side
      real_steering_rad += PI;
    }

    // Compute X-Y velocity components
    double angle = real_steering_rad - qLast.elem(2);
    double velocity_x = actual_speed_mps *  cos(angle);
    double velocity_y = actual_speed_mps * -sin(angle);

    #if DEBUG
      std::cout << "Actual Linear Speed: " << actual_speed_mps << " m/s" << std::endl;
      std::cout << "Actual Steering Angle: " << real_steering_rad << " rad" << std::endl;
      std::cout << "qLast.z: " << qLast.elem(2) << " rad" << std::endl;
      std::cout << "Velocity X: " << velocity_x << " m/s, Velocity Y: " << velocity_y << " m/s" << std::endl;
      std::cout << "---------------------------------" << std::endl;
    #endif

    return arr{velocity_x, velocity_y, 0.};
  }

  arr get_qDot_spin()
  {
    struct can_frame recv_frame = recv_package(0x221);

    // TODO: Translate wheel speed to actual q speed
    int16_t actual_spin_speed = (recv_frame.data[2] << 8) | recv_frame.data[3];
    double actual_spin_speed_mps = actual_spin_speed / 1000.0; // Convert mm/s to m/s

    #if DEBUG
      std::cout << "Actual Angular Speed: " << actual_spin_speed_mps << " m/s" << std::endl;
    #endif
    return arr{0., 0., actual_spin_speed_mps};
  }

  struct can_frame recv_package(canid_t type)
  {
    struct can_frame recv_frame;
    while (true)
    {
      int nbytes = read(socket_fd, &recv_frame, sizeof(recv_frame));
      if (nbytes < 0) {
        LOG(-1) << "Error reading CAN frame: " << strerror(errno) << std::endl;
      }
      if (recv_frame.can_id == type) {
        break;
      }
    }
    return recv_frame;
  }

  void getState(arr& q, arr& qDot)
  {
    arr s = get_s();
    arr sDelta = s - sLast;

    arr qDelta = zeros(3);
    qDot = zeros(3);
    switch (mode)
    {
      case 1: {
        // Spin mode
        double mean_delta_s = (-1.*sDelta.elem(0) +
                                   sDelta.elem(1) +
                               -1.*sDelta.elem(2) +
                                   sDelta.elem(3)) * .25; // m
        #if DEBUG
          std::cout << "Mean delta s: " << mean_delta_s << " mDelta" << std::endl;
        #endif
        qDot = get_qDot_spin();
        double base_radious = .31;
        double angle_traveled = atan(mean_delta_s/base_radious);
        qDelta = arr{0., 0., angle_traveled};
        break;
      }
     
      case 0: {
        // Oblique mode
        qDot = get_qDot_oblique();
        double mean_delta_s = (sDelta.elem(0) +
                               sDelta.elem(1) +
                               sDelta.elem(2) +
                               sDelta.elem(3)) * .25; // m
        mean_delta_s = abs(mean_delta_s);
        double steering_angle = real_steering_rad - qLast.elem(2);
        double xDelta =  cos(steering_angle) * mean_delta_s;
        double yDelta = -sin(steering_angle) * mean_delta_s;
        qDelta = arr{xDelta, yDelta, 0.};
        break;
      }

      case 2: {
        // Ackerman mode
        break;
      }

      default: {
        break;
      }
    }

    q = qLast + qDelta;
    qLast = q;
    sLast = s;
  }

  void setVelocitiesOblique(double dir_angle, double dir_magnitude)
  {
    struct can_frame frame_motion;
    frame_motion.can_dlc = 8; // Data length code (number of bytes)
    frame_motion.can_id = 0x111;

    if (dir_angle > PI*.5 && dir_angle < 1.5*PI) { // Lower half
      if (dir_angle > PI) { // Lower left
        dir_angle -= PI;
        dir_angle *=-1;
        // std::cout << "Lower left" << std::endl;
      } else { // Lower right
        dir_angle = PI - dir_angle;
        // std::cout << "Lower right" << std::endl;
      }
      dir_magnitude *= -1;
    } else { // Upper half
      if (dir_angle >= 1.5*PI) { // Upper left
        dir_angle = 2*PI - dir_angle;
        // std::cout << "Upper left" << std::endl;
      } else { // Upper right
        dir_angle *=-1;
        // std::cout << "Upper right" << std::endl;
      }
    }

    target_steering_rad_s = dir_angle;

    #if DEBUG
      std::cout << "---------- Oblique mode velocities ----------" << std::endl;
      std::cout << "New Mag: " << dir_magnitude << " m/s" << std::endl;
      std::cout << "New Angle: " << dir_angle << " rad" << std::endl;
    #endif

    int16_t linear_speed = dir_magnitude * 1000; // mm/s
    int16_t spin_speed = 0;                      // No rotation
    int16_t steering_angle = dir_angle * 1000;   // 0.001 rad

    frame_motion.data[0] = (linear_speed >> 8) & 0xFF;
    frame_motion.data[1] = linear_speed & 0xFF;
    frame_motion.data[2] = (spin_speed >> 8) & 0xFF;
    frame_motion.data[3] = spin_speed & 0xFF;
    frame_motion.data[4] = 0x00; // Reserved
    frame_motion.data[5] = 0x00; // Reserved
    frame_motion.data[6] = (steering_angle >> 8) & 0xFF;
    frame_motion.data[7] = steering_angle & 0xFF;

    send_package(frame_motion);
  }

  void setSteeringAngle(double dir_angle)
  {
    struct can_frame frame_motion;
    frame_motion.can_dlc = 8; // Data length code (number of bytes)
    frame_motion.can_id = 0x111;

    int16_t linear_speed = 0;
    int16_t spin_speed = 0;
    int16_t steering_angle = dir_angle * 1000;  // 0.001 rad

    frame_motion.data[0] = (linear_speed >> 8) & 0xFF;
    frame_motion.data[1] = linear_speed & 0xFF;
    frame_motion.data[2] = (spin_speed >> 8) & 0xFF;
    frame_motion.data[3] = spin_speed & 0xFF;
    frame_motion.data[4] = 0x00; // Reserved
    frame_motion.data[5] = 0x00; // Reserved
    frame_motion.data[6] = (steering_angle >> 8) & 0xFF;
    frame_motion.data[7] = steering_angle & 0xFF;

    send_package(frame_motion);
  }

  void setVelocitiesSpin(double spin_speed)
  {
    struct can_frame frame_motion;
    frame_motion.can_dlc = 8; // Data length code (number of bytes)
    frame_motion.can_id = 0x111;

    #if DEBUG
      std::cout << "---------- Spin mode velocities ----------" << std::endl;
      std::cout << "New Speed: " << spin_speed << " rad/s" << std::endl;
    #endif

    int16_t linear_speed = 0;
    int16_t spin_speed_ = spin_speed * 1000;
    int16_t steering_angle = 0;

    frame_motion.data[0] = (linear_speed >> 8) & 0xFF;
    frame_motion.data[1] = linear_speed & 0xFF;
    frame_motion.data[2] = (spin_speed_ >> 8) & 0xFF;
    frame_motion.data[3] = spin_speed_ & 0xFF;
    frame_motion.data[4] = 0x00; // Reserved
    frame_motion.data[5] = 0x00; // Reserved
    frame_motion.data[6] = (steering_angle >> 8) & 0xFF;
    frame_motion.data[7] = steering_angle & 0xFF;

    send_package(frame_motion);
  }

  void send_package(struct can_frame frame_motion)
  {
    if (write(socket_fd, &frame_motion, sizeof(frame_motion)) != sizeof(frame_motion)) {
      LOG(-1) << "Error sending motion command message: " << strerror(errno) << std::endl;
      close(socket_fd);
    }
  }

  void setVelocities(const arr& qDotTarget)
  {
    int prev_mode = mode;
    // Check which mode should be active
    double thresh = 2e-2; // 2cm error
    bool is_spin_mode = abs(qDotTarget.elem(0)) <= thresh &&
                        abs(qDotTarget.elem(1)) <= thresh &&
                        abs(qDotTarget.elem(2)) >= thresh;

    bool is_oblique_mode = (abs(qDotTarget.elem(0)) >= thresh ||
                            abs(qDotTarget.elem(1)) >= thresh) &&
                            abs(qDotTarget.elem(2)) <= thresh;

    bool is_still = abs(qDotTarget.elem(0)) <= thresh &&
                    abs(qDotTarget.elem(1)) <= thresh &&
                    abs(qDotTarget.elem(2)) <= thresh;

    bool is_ackerman_mode = false;

    if (is_spin_mode) {
      mode = 1;
    } else if (is_oblique_mode) {
      mode = 0;
    } else if (is_ackerman_mode) {
      mode = 2;
    } else if (is_still) {
      return;
    } else {
      LOG(-1) << "Impossible qDotTarget! " << qDotTarget << " >:(" << std::endl;
      return;
    }

    // Safety
    // TODO: Check if velocities are too high
    switch (mode)
    {
      case 1: {
        // Spin mode
        if (mode != prev_mode) {
          set_control_mode(0x02);
        }
        double spin_speed = qDotTarget.elem(2);
        setVelocitiesSpin(spin_speed);
        break;
      }
     
      case 0: {
        // Oblique mode
        if (mode != prev_mode) {
          set_control_mode(0x01);
        }

        double dir_magnitude = ::sqrt(::pow(qDotTarget.elem(0), 2) + ::pow(qDotTarget.elem(1), 2));
        double target_steering_rad = ::acos(qDotTarget.elem(0) / dir_magnitude);
        if (qDotTarget.elem(1) > 0) {
          target_steering_rad = PI*2-target_steering_rad;
        }

        // Adjust for current rotation
        target_steering_rad += qLast.elem(2);

        if (target_steering_rad < 0) {
          target_steering_rad += PI*2;
        } else if (target_steering_rad >= PI*2) {
          target_steering_rad -= PI*2;
        }

        #if DEBUG
          std::cout << "qLast.z: " << qLast.elem(2) << " rad" << std::endl;
          std::cout << "Mag: " << dir_magnitude << " m/s" << std::endl;
          std::cout << "Angle: " << target_steering_rad << " rad" << std::endl;
        #endif

        setVelocitiesOblique(target_steering_rad, dir_magnitude);
        break;
      }

      case 2: {
        break;
      }

      default: {
        break;
      }
    }
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
  Kp = rai::getParameter<arr>("Ranger/Kp", arr{1., 1., 1.});
  Ki = rai::getParameter<arr>("Ranger/Ki", arr{0., 0., 0.});
  Kd = rai::getParameter<arr>("Ranger/Kd", arr{0., 0., 0.});

  LOG(0) << "Ranger/PID: Kp:" << Kp << " Ki:" << Ki << " Kd:" << Kd;

  //-- get robot address
  address = rai::getParameter<rai::String>("Ranger/address", "can0");

  //-- start thread and wait for first state signal
  LOG(0) <<"launching Ranger " <<robotID <<" at " <<address;

  threadLoop();
  while(Thread::step_count<2) rai::wait(.01);
}

void RangerThread::open(){
  // connect to robot
  robot = make_shared<RangerController>(address);

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
    if(cmdGet->P_compliance.N) {
      P_compliance.resize(3,3);
      for(uint i=0; i<3; i++) for(uint j=0; j<3; j++) P_compliance(i,j) = cmdGet->P_compliance(qIndices(i), qIndices(j));
    }
  }
  #if DEBUG
    LOG(0) << "q_real" << q_real;
    LOG(0) << "qDot_real" << qDot_real;
    LOG(0) << "q_ref" << q_ref;
    LOG(0) << "qDot_ref" << qDot_ref;
  #endif

  //-- cap the reference difference
  if(q_ref.N==3){
    double err = length(q_ref - q_real);
    if(P_compliance.N){
      arr del = q_ref - q_real;
      err = ::sqrt(scalarProduct(del, P_compliance*del));
    }
    double steering_error = abs(robot->real_steering_rad_s-robot->target_steering_rad_s);
    if(err>.3 || steering_error > .1){ //stall!
      state.set()->stall = 2; //no progress in reference time! for at least 2 iterations (to ensure continuous stall with multiple threads)
      cout <<"STALLING - step:" <<steps <<" err: " <<err <<endl;
      if (steering_error > .1) {
        robot->setSteeringAngle(robot->target_steering_rad_s);
        #if DEBUG
          std::cout << "Stalling due to steering error:" << std::endl;
          std::cout << "- Real steering rad: " << robot->real_steering_rad_s << std::endl;
          std::cout << "- Target steering rad: " << robot->target_steering_rad_s << std::endl;
        #endif
      }
    }
  }

  //-- compute torques from control message depending on the control type
  arr qDotTarget;
  qDotTarget.resize(3).setZero();
  
  if(q_ref.N==3 && qDot_ref.N==3)
  {
    arr p_term = q_ref - q_real;
    p_term *= Kp;
    arr d_term(qDot_ref);
    d_term *= Kd;
    qDotTarget = p_term + d_term;
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
      qDDot_ref.modRaw().write(dataFile);
      }
      dataFile <<endl;
  }

  robot->setVelocities(qDotTarget);
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
