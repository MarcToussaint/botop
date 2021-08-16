#include "franka.h"

#include <franka/model.h>
#include <franka/robot.h>

void naturalGains(double& Kp, double& Kd, double decayTime, double dampingRatio);

const char *frankaIpAddresses[2] = {"172.16.0.2", "172.17.0.2"};

FrankaThreadNew::~FrankaThreadNew(){
  stop = true;
  rai::wait(.1);
  threadClose();
}

long c = 0;

void FrankaThreadNew::init(uint whichRobot, const uintA& _qIndices) {
  qIndices=_qIndices;

  CHECK_EQ(qIndices.N, 7, "");
  qIndices_max = qIndices.max();

  //-- basic Kp Kd settings for reference control mode
  Kp_freq = rai::getParameter<arr>("Franka/Kp_freq", ARR(20., 20., 20., 20., 10., 15., 10.)); //18., 18., 18., 13., 8., 8., 6.));
  Kd_ratio = rai::getParameter<arr>("Franka/Kd_ratio", ARR(.6, .6, .4, .4, .1, .5, .1)); //.8, .8, .7, .7, .1, .1, .1));
  LOG(0) << "FRANKA: Kp_freq=" << Kp_freq << " Kd_ratio=" << Kd_ratio;

  //-- choose robot/ipAddress
  CHECK_LE(whichRobot, 1, "");
  ipAddress = frankaIpAddresses[whichRobot];

  //-- start thread and wait for first state signal
  threadStep();  //this is not looping! The step method passes a callback to robot.control, which is blocking! (that's why we use a thread) until stop becomes true
  while(requiresInitialization) rai::wait(.01);
}

void FrankaThreadNew::step(){
  // connect to robot
  franka::Robot robot(ipAddress);

  // load the kinematics and dynamics model
  franka::Model model = robot.loadModel();

  arr lastTorque = zeros(7);

  // set collision behavior
  robot.setCollisionBehavior({{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                             {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                             {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                             {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}});

  // initialize state and ctrl with first state
  {
    franka::RobotState initial_state = robot.readOnce();
    arr q_real, qDot_real;
    q_real.setCarray(initial_state.q.begin(), initial_state.q.size());
    qDot_real.setCarray(initial_state.dq.begin(), initial_state.dq.size());

    auto stateSet = state.set();
    auto cmdSet = cmd.set();

    // TODO really modify the complete state?
    while(stateSet->q.N<=qIndices_max) stateSet->q.append(0.);
    while(stateSet->qDot.N<=qIndices_max) stateSet->qDot.append(0.);
    while(stateSet->tauExternal.N<=qIndices_max) stateSet->tauExternal.append(0.);

    for(uint i=0; i < 7; i++){
      stateSet->q(qIndices(i)) = q_real(i);
      stateSet->qDot(qIndices(i)) = qDot_real(i);
      stateSet->tauExternal(qIndices(i)) = 0.;
    }
  }


  // define callback for the torque control loop
  std::function<franka::Torques(const franka::RobotState&, franka::Duration)>
      torque_control_callback = [&](const franka::RobotState& robot_state,
                                   franka::Duration /*duration*/) -> franka::Torques {

    steps++;

    if(stop) return franka::MotionFinished(franka::Torques( std::array<double, 7>{0., 0., 0., 0., 0., 0., 0.}));

    //-- get current state from libfranka
    arr q_real(robot_state.q.begin(), robot_state.q.size(), false);
    arr qDot_real(robot_state.dq.begin(), robot_state.dq.size(), false);
    arr torquesExternal_real(robot_state.tau_ext_hat_filtered.begin(), robot_state.tau_ext_hat_filtered.size(), false);
    arr torques_real(robot_state.tau_J.begin(), 7, false);

    //-- get real time
    ctrlTime += .001; //HARD CODED: 1kHz
    //ctrlTime = rai::realTime();

    //-- publish state
    arr state_q_real, state_qDot_real;
    {
      auto stateSet = state.set();
      stateSet->time = ctrlTime;
      for(uint i=0;i<7;i++){
        stateSet->q(qIndices(i)) = q_real(i);
        stateSet->qDot(qIndices(i)) = qDot_real(i);
        stateSet->tauExternal(qIndices(i)) = torquesExternal_real(i);
      }
      state_q_real = stateSet->q;
      state_qDot_real = stateSet->qDot;
    }

    //-- get current ctrl command
    arr q_ref, qDot_ref, qDDot_ref, KpRef, KdRef, P_compliance; // TODO Kp, Kd and also read out the correct indices
    rai::ControlType controlType;
    {
      auto cmdGet = cmd.get();

      controlType = cmdGet->controlType;

      //get the reference from the callback (e.g., sampling a spline reference)

      arr cmd_q_ref, cmd_qDot_ref, cmd_qDDot_ref;
      if(cmdGet->ref){
        cmdGet->ref->getReference(cmd_q_ref, cmd_qDot_ref, cmd_qDDot_ref, state_q_real, state_qDot_real, ctrlTime);
        CHECK(cmd_q_ref.N > qIndices_max, "");
        CHECK(cmd_qDot_ref.N > qIndices_max, "");
        CHECK(cmd_qDDot_ref.N > qIndices_max, "");
      }

      if(cmd_q_ref.N) q_ref.resize(7).setZero();
      if(cmd_qDot_ref.N) qDot_ref.resize(7).setZero();
      if(cmd_qDDot_ref.N) qDDot_ref.resize(7).setZero();

      for(uint i=0; i<7; i++) {
        if(cmd_q_ref.N) q_ref(i) = cmd_q_ref(qIndices(i));
        if(cmd_qDot_ref.N) qDot_ref(i) = cmd_qDot_ref(qIndices(i));
        if(cmd_qDDot_ref.N) qDDot_ref(i) = cmd_qDDot_ref(qIndices(i));
      }

      if(cmdGet->Kp.d0 >= 7 && cmdGet->Kp.d1 >=7 && cmdGet->Kp.d0 == cmdGet->Kp.d1){
        KpRef.resize(7, 7);
        for(uint i=0; i<7; i++) for(uint j=0; j<7; j++) KpRef(i, j) = cmdGet->Kp(qIndices(i), qIndices(j));
      }

      if(cmdGet->Kd.d0 >= 7 && cmdGet->Kd.d1 >=7 && cmdGet->Kd.d0 == cmdGet->Kd.d1){
        for(uint i=0; i<7; i++) for(uint j=0; j<7; j++) KdRef(i, j) = cmdGet->Kd(qIndices(i), qIndices(j));
      }

      if(cmdGet->P_compliance.N) {
        HALT("NOT IMPLEMENTED YET (at least properly)")
        P_compliance = cmdGet->P_compliance;
      }

    }

    requiresInitialization=false;

    //-- cap the reference difference
    if(q_ref.N==7){
        double err = length(q_ref - q_real);
        if(err>.05){ //if(err>.02){ //stall!
            ctrlTime -= .001; //no progress in reference time!
            cout <<"STALLING - step:" <<steps <<endl;
        }
    }

    arr u = zeros(7); // torques send to the robot

    //-- grab dynamics
    arr M_org(model.mass(robot_state).begin(), 49, false);
    M_org.reshape(7,7);
    arr C_org(model.coriolis(robot_state).begin(), 7, false);
    arr G_org(model.gravity(robot_state).begin(), 7, false);

    //-- construct torques from control message depending on the control type
    if(controlType == rai::ControlType::configRefs) { // plain qRef, qDotRef references
      //check for correct ctrl otherwise do something...
      if(q_ref.N!=7){
        if(!(steps%10)){
          cerr <<"FRANKA: inconsistent ctrl q_ref message - step: " <<steps <<endl;
        }
        return std::array<double, 7>({0., 0., 0., 0., 0., 0., 0.});
      }
      //check for correct compliance objective
      if(P_compliance.N){
        if(!(P_compliance.nd==2 && P_compliance.d0==7 && P_compliance.d1==7)){
          cerr << "FRANKA: inconsistent ctrl P_compliance message" << endl;
          P_compliance.clear();
        }
      }

      //-- compute desired torques
      arr Kp(7), Kd(7);
      CHECK_EQ(Kp.N, 7,"");
      CHECK_EQ(Kd.N, 7,"");
      CHECK_EQ(Kp_freq.N, 7,"");
      CHECK_EQ(Kd_ratio.N, 7,"");
      for(uint i=0;i<7;i++){
        double freq = Kp_freq(i);
        Kp(i) = freq*freq;
        Kd(i) = 2.*Kd_ratio(i)*freq;
      }

      if(P_compliance.N){
        Kp = P_compliance * (Kp % P_compliance);
      }else{
        Kp = diag(Kp);
      }

      //-- feedback term
      u.resize(7).setZero();
      if(q_ref.N==7){
        u += Kp * (q_ref - q_real);
        u += Kd % (qDot_ref - qDot_real);
      }

      //-- feedforward term
      if(absMax(qDDot_ref)>0.){
        arr M = M_org; // + diag(ARR(0.4, 0.3, 0.3, 0.4, 0.4, 0.4, 0.2));
        u += M*qDDot_ref;
      }

      if(P_compliance.N) u = P_compliance * u;

    } else if(controlType == rai::ControlType::projectedAcc) { // projected Kp, Kd and u_b term for projected operational space control
      CHECK_EQ(KpRef.nd, 2, "")
      CHECK_EQ(KpRef.d0, 7, "")
      CHECK_EQ(KpRef.d1, 7, "")
      CHECK_EQ(KdRef.nd, 2, "")
      CHECK_EQ(KdRef.d0, 7, "")
      CHECK_EQ(KdRef.d1, 7, "")
      CHECK_EQ(qDDot_ref.N, 7, "")

      arr M;
      M.setCarray(model.mass(robot_state).begin(), 49);
      M.reshape(7,7);

#if 0
      c++;
      if(c%10 == 0) {
        cout << M << endl << endl;
      }
#endif


#if 0
      M(4,4) = 0.2;
      M(5,5) = 0.2;../04-trivialCtrl/retired.cpp
      M(6,6) = 0.1;
#else
      arr MDiag = diag(ARR(0.4, 0.3, 0.3, 0.4, 0.4, 0.4, 0.2));
      M = M + MDiag;
#endif

      KpRef = M*KpRef;
      KdRef = M*KdRef;
      qDDot_ref = M*qDDot_ref;

      u = qDDot_ref - KpRef*q_real - KdRef*qDot_real;

      //u(5) *= 2.0;

      //cout << u << endl;

      //u *= 0.0; // useful for testing new stuff without braking the robot
    }

    //-- filter torques
    if(u.N==lastTorque.N){
      double alpha=.5;
      u = alpha*u + (1.-alpha)*lastTorque;
      lastTorque = u;
    }

    //-- data log?
    if(writeData>0 && !(steps%1)){
      if(!dataFile.is_open()) dataFile.open("z.panda.dat");
      dataFile <<ctrlTime <<' '; //single number
      q_real.writeRaw(dataFile); //7
      q_ref.writeRaw(dataFile); //7
      if(writeData>1){
          qDot_real.writeRaw(dataFile); //7
          u.writeRaw(dataFile); //7
          torques_real.writeRaw(dataFile); //7
          G_org.writeRaw(dataFile); //7-vector gravity
          C_org.writeRaw(dataFile); //7-vector coriolis
          M_org.write(dataFile, " ", " ", "  "); //7x7 inertia matrix
//      qDDot_ref.writeRaw(dataFile);
      }
      dataFile <<endl;
    }

    std::array<double, 7> u_array = {0., 0., 0., 0., 0., 0., 0.};
    std::copy(u.begin(), u.end(), u_array.begin());

    if(stop) return franka::MotionFinished(franka::Torques(u_array));
    return u_array;
  };

  // start real-time control loop
  robot.control(torque_control_callback, true, 2000.);
}
