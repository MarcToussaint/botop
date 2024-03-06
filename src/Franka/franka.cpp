#include "franka.h"

#ifdef RAI_FRANKA

#include <franka/model.h>
#include <franka/robot.h>
#include <franka/exception.h>

void naturalGains(double& Kp, double& Kd, double decayTime, double dampingRatio);

const char *frankaIpAddresses[2] = {"172.16.0.2", "172.17.0.2"};

FrankaThread::~FrankaThread(){
  LOG(0) <<"shutting down Franka " <<robotID;
  stop = true;
  waitForIdle();
  threadClose();
}

long c = 0;

void FrankaThread::init(uint _robotID, const uintA& _qIndices) {
  robotID=_robotID;
  qIndices=_qIndices;

  CHECK_EQ(qIndices.N, 7, "");
  qIndices_max = rai::max(qIndices);

  //-- basic Kp Kd settings for reference control mode
  Kp_freq = rai::getParameter<arr>("Franka/Kp_freq", arr{20., 20., 20., 20., 10., 15., 10.}); //18., 18., 18., 13., 8., 8., 6.));
  Kd_ratio = rai::getParameter<arr>("Franka/Kd_ratio", arr{.6, .6, .4, .4, .1, .5, .1}); //.8, .8, .7, .7, .1, .1, .1));
  friction = rai::getParameter<arr>("Franka/friction", zeros(7));  //Franka/friction: arr{0.8, 1.0, 0.8, 1.0, 0.9, 0.5, 0.4}
  //friction = rai::getParameter<arr>("Franka/friction", arr{0.8, 1.0, 0.8, 1.0, 0.9, 0.5, 0.4});
  LOG(0) << "FRANKA: Kp_freq:" << Kp_freq << " Kd_ratio:" << Kd_ratio <<" friction:" <<friction;

  /* hand tuning result of friction calib:
     Franka/friction: [0.8, 1.0, 0.8, 1.0, 0.9, 0.5, 0.4]
     Franka/Kd_ratio: [0.6, 0.6, 0.3, 0.3, 0.3, 0.3, 0.4]
  */

  //-- choose robot/ipAddress
  CHECK_LE(robotID, 1, "");
  ipAddress = frankaIpAddresses[robotID];

  //-- start thread and wait for first state signal
  LOG(0) <<"launching Franka " <<robotID <<" at " <<ipAddress;
  threadStep();  //this is not looping! The step method passes a callback to robot.control, which is blocking! (that's why we use a thread) until stop becomes true

  for(uint i=0;i<200;i++){
    if(!requiresInitialization) break;
    rai::wait(.01);
  }
  if(requiresInitialization){
    threadCancel();
    THROW("lauching Franka timeout!");
  }
}

void FrankaThread::step(){
  // connect to robot
  franka::Robot robot(ipAddress);

  // load the kinematics and dynamics model
  franka::Model model = robot.loadModel();

  arr lastTorque = zeros(7);
  arr qDotFilter = zeros(7);
  double qDotFilterAlpha = .7;

  // set collision behavior
  robot.setCollisionBehavior({{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                             {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                             {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                             {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}});

  //-- initialize state and ctrl with first state
  {
    franka::RobotState initial_state = robot.readOnce();
    arr q_real, qDot_real;
    q_real.setCarray(initial_state.q.begin(), initial_state.q.size());
    qDot_real.setCarray(initial_state.dq.begin(), initial_state.dq.size());

    auto stateSet = state.set();
    auto cmdSet = cmd.set();

    //ensure state variables have sufficient size
    while(stateSet->q.N<=qIndices_max) stateSet->q.append(0.);
    while(stateSet->qDot.N<=qIndices_max) stateSet->qDot.append(0.);
    while(stateSet->tauExternalIntegral.N<=qIndices_max) stateSet->tauExternalIntegral.append(0.);

    for(uint i=0; i<7; i++){
      stateSet->q.elem(qIndices(i)) = q_real(i);
      stateSet->qDot.elem(qIndices(i)) = qDot_real(i);
      stateSet->tauExternalIntegral.elem(qIndices(i)) = 0.;
      stateSet->tauExternalCount=0;
    }
  }


  //-- define the callback for the torque control loop
  std::function<franka::Torques(const franka::RobotState&, franka::Duration)>
      torque_control_callback = [&](const franka::RobotState& robot_state,
                                    franka::Duration /*duration*/) -> franka::Torques {

    steps++;

//    if(stop) return franka::MotionFinished(franka::Torques( std::array<double, 7>{0., 0., 0., 0., 0., 0., 0.}));

    //-- get current state from libfranka
    arr q_real(robot_state.q.begin(), robot_state.q.size(), false);
    arr qDot_real(robot_state.dq.begin(), robot_state.dq.size(), false);
    arr torquesExternal_real(robot_state.tau_ext_hat_filtered.begin(), robot_state.tau_ext_hat_filtered.size(), false);
    arr torques_real(robot_state.tau_J.begin(), robot_state.tau_J.size(), false);

    qDotFilter = qDotFilterAlpha * qDotFilter + (1.-qDotFilterAlpha) * qDot_real;
    qDot_real = qDotFilter;

    //-- get real time
    //ctrlTime += .001; //HARD CODED: 1kHz
    //ctrlTime = rai::realTime();

    //-- publish state & INCREMENT CTRL TIME
    arr state_q_real, state_qDot_real;
    {
      auto stateSet = state.set();
      if(robotID==0){ // if this is the lead robot, increment ctrlTime if no stall
        if(!stateSet->stall) stateSet->ctrlTime += .001; //HARD CODED: 1kHz
        else stateSet->stall--;
      }
      ctrlTime = stateSet->ctrlTime;
      for(uint i=0;i<7;i++){
        stateSet->q.elem(qIndices(i)) = q_real.elem(i);
        stateSet->qDot.elem(qIndices(i)) = qDot_real.elem(i);
        stateSet->tauExternalIntegral.elem(qIndices(i)) += torquesExternal_real.elem(i);
      }
      stateSet->tauExternalCount++;
      state_q_real = stateSet->q;
      state_qDot_real = stateSet->qDot;
    }

    //-- get current ctrl command
    arr q_ref, qDot_ref, qDDot_ref, Kp_ref, Kd_ref, P_compliance; // TODO Kp, Kd and also read out the correct indices
    rai::ControlType controlType;
    {
      auto cmdGet = cmd.get();

      controlType = cmdGet->controlType;

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
        q_ref.resize(7);
        for(uint i=0; i<7; i++) q_ref.elem(i) = cmd_q_ref.elem(qIndices(i));
      }
      if(cmd_qDot_ref.N){
        qDot_ref.resize(7);
        for(uint i=0; i<7; i++) qDot_ref.elem(i) = cmd_qDot_ref.elem(qIndices(i));
      }
      if(cmd_qDDot_ref.N){
        qDDot_ref.resize(7);
        for(uint i=0; i<7; i++) qDDot_ref.elem(i) = cmd_qDDot_ref.elem(qIndices(i));
      }
      if(cmdGet->Kp.d0 >= 7 && cmdGet->Kp.d1 >=7 && cmdGet->Kp.d0 == cmdGet->Kp.d1){
        Kp_ref.resize(7, 7);
        for(uint i=0; i<7; i++) for(uint j=0; j<7; j++) Kp_ref(i, j) = cmdGet->Kp(qIndices(i), qIndices(j));
      }
      if(cmdGet->Kd.d0 >= 7 && cmdGet->Kd.d1 >=7 && cmdGet->Kd.d0 == cmdGet->Kd.d1){
        Kd_ref.resize(7, 7);
        for(uint i=0; i<7; i++) for(uint j=0; j<7; j++) Kd_ref(i, j) = cmdGet->Kd(qIndices(i), qIndices(j));
      }
      if(cmdGet->P_compliance.N) {
        P_compliance.resize(7,7);
        for(uint i=0; i<7; i++) for(uint j=0; j<7; j++) P_compliance(i,j) = cmdGet->P_compliance(qIndices(i), qIndices(j));
      }

    }

    requiresInitialization=false;

    //-- cap the reference difference
    if(q_ref.N==7){
      double err = length(q_ref - q_real);
      if(P_compliance.N){
        arr del = q_ref - q_real;
        err = ::sqrt(scalarProduct(del, P_compliance*del));
      }
      if(err>.05){ //if(err>.02){ //stall!
        state.set()->stall = 2; //no progress in reference time! for at least 2 iterations (to ensure continuous stall with multiple threads)
        cout <<"STALLING - step:" <<steps <<" err: " <<err <<endl;
      }
    }

    //-- grab dynamics
    arr M_org(model.mass(robot_state).begin(), 49, false);
    M_org.reshape(7,7);
    arr C_org(model.coriolis(robot_state).begin(), 7, false);
    arr G_org(model.gravity(robot_state).begin(), 7, false);

    //-- compute torques from control message depending on the control type
    arr u;

    if(controlType == rai::ControlType::configRefs) { //default: PD for given references
      //check for compliance objective
      if(P_compliance.N){
        if(!(P_compliance.nd==2 && P_compliance.d0==7 && P_compliance.d1==7)){
          cerr << "FRANKA: inconsistent ctrl P_compliance message" << endl;
          P_compliance.clear();
        }
      }

      //-- compute desired torques
      arr Kp(7), Kd(7);
      CHECK_EQ(Kp_freq.N, 7,"");
      CHECK_EQ(Kd_ratio.N, 7,"");
      for(uint i=0;i<7;i++){
        double freq = Kp_freq.elem(i);
        Kp.elem(i) = freq*freq;
        Kd.elem(i) = 2.*Kd_ratio.elem(i)*freq;
      }

      if(P_compliance.N){
        Kp = P_compliance * (Kp % P_compliance);
      }else{
        Kp = diag(Kp);
      }

      //-- initialize zero torques
      u.resize(7).setZero();

      //-- add feedback term
      if(q_ref.N==7){
        u += Kp * (q_ref - q_real);
      }
      if(qDot_ref.N==7){
        u += Kd % (qDot_ref - qDot_real);
      }

      //-- add feedforward term
      if(qDDot_ref.N==7 && absMax(qDDot_ref)>0.){
        arr M = M_org; // + diag(arr{0.4, 0.3, 0.3, 0.4, 0.4, 0.4, 0.2));
        u += M*qDDot_ref;
      }

      //-- add friction term
      if(friction.N==7 && qDot_ref.N==7){
        double velThresh=1e-3;
        for(uint i=0;i<7;i++){
          double coeff = qDot_ref.elem(i)/velThresh;
          if(coeff>1.) coeff=1.;
          if(coeff<-1.) coeff=-1.;
          u.elem(i) += coeff*friction.elem(i);
//          if(qDot_ref.elem(i)>1e-4) u.elem(i) += friction.elem(i);
//          if(qDot_ref.elem(i)<-1e-4) u.elem(i) -= friction.elem(i);
        }
      }

      //-- project with compliance
      if(P_compliance.N) u = P_compliance * u;

    } else if(controlType == rai::ControlType::projectedAcc) { // projected Kp, Kd and u_b term for projected operational space control
      CHECK_EQ(Kp_ref.nd, 2, "")
      CHECK_EQ(Kp_ref.d0, 7, "")
      CHECK_EQ(Kp_ref.d1, 7, "")
      CHECK_EQ(Kd_ref.nd, 2, "")
      CHECK_EQ(Kd_ref.d0, 7, "")
      CHECK_EQ(Kd_ref.d1, 7, "")
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
      arr MDiag = diag(arr{0.4, 0.3, 0.3, 0.4, 0.4, 0.4, 0.2});
      M = M + MDiag;
#endif

      Kp_ref = M*Kp_ref;
      Kd_ref = M*Kd_ref;
      qDDot_ref = M*qDDot_ref;

      u = qDDot_ref - Kp_ref*q_real - Kd_ref*qDot_real;

      //u(5) *= 2.0;

      //cout << u << endl;

      //u *= 0.0; // useful for testing new stuff without braking the robot
    }

    //-- filter torques
    if(u.N==lastTorque.N){
//      double alpha=.5;
//      u = alpha*u + (1.-alpha)*lastTorque;
      lastTorque = u;
    }

    //-- data log?
    if(writeData>0 && !(steps%10)){
      if(!dataFile.is_open()) dataFile.open(STRING("z.panda"<<robotID <<".dat"));
      dataFile <<ctrlTime <<' '; //single number
      q_real.modRaw().write(dataFile); //7
      q_ref.modRaw().write(dataFile); //7
      if(writeData>1){
        qDot_real.modRaw().write(dataFile); //7
        qDot_ref.modRaw().write(dataFile); //7
        u.modRaw().write(dataFile); //7
        torques_real.modRaw().write(dataFile); //7
        G_org.modRaw().write(dataFile); //7-vector gravity
        C_org.modRaw().write(dataFile); //7-vector coriolis
        qDDot_ref.modRaw().write(dataFile);
      }
      if(writeData>2){
        M_org.write(dataFile, " ", " ", "  "); //7x7 inertia matrix
      }
      dataFile <<endl;
    }

    //-- send torques
    std::array<double, 7> u_array = {0., 0., 0., 0., 0., 0., 0.};
    for(uint i=0;i<7;i++) u_array[i]= u.elem(i);
    if(stop){
      return franka::MotionFinished(franka::Torques(u_array));
    }
    return franka::Torques(u_array);
  };

  //start real-time control loop
  //cout <<"HERE" <<endl;

  try {
    robot.control(torque_control_callback, true, 2000.);
  } catch (franka::Exception const& e) {
    std::cout << e.what() << std::endl;
  }
  LOG(0) <<"EXIT FRANKA CONTROL LOOP";
}

#else //RAI_FRANKA

FrankaThread::~FrankaThread(){ NICO }
void FrankaThread::init(uint _robotID, const uintA& _qIndices) { NICO }
void FrankaThread::step(){ NICO }

#endif
