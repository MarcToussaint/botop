#include "omnibase.h"
#include "SimplexMotion.h"

#ifdef RAI_OMNIBASE

#define USE_FAKE

struct OmnibaseController{
#ifdef USE_FAKE
  arr fake_q, fake_qDot;
#else
  rai::Array<std::shared_ptr<SimplexMotion>> motors; //three motors
  arr qLast, sLast; //q: joint state; s: motor state
#endif

  OmnibaseController(rai::String address){
#ifdef USE_FAKE
    fake_q.resize(3).setZero();
    fake_qDot.resize(3).setZero();
#else
    //-- launch 3 motors
    motors.resize(3);
    for(uint i=0;i<motors.N;i++){
      motors(i) = make_shared<SimplexMotion>(address, 0x04d8, 0xf79a);
      motors(i)->runReset(); //resets position counter
      motors(i)->runTorque(0.); //starts torque mode
      address(address.N-1)++;
    }
    qLast = zeros(3);
    sLast = zeros(motors.N);
#endif
  }

  ~OmnibaseController(){
#ifdef USE_FAKE
#else
    for(uint i=0;i<motors.N;i++) motors(i)->runStop();
    rai::wait(.1);
    for(uint i=0;i<motors.N;i++) motors(i)->runOff();
    rai::wait(.1);
    for(uint i=0;i<motors.N;i++) motors(i).reset();
#endif
  }

  arr getJacobian(){
    //return Jacobian based on qLast
    arr J(3,3);
    NIY;
    return J;
  }

  void getState(arr& q, arr& qDot){
#ifdef USE_FAKE //fake implementation: directly return fake state
    q = fake_q;
    qDot = fake_qDot;
#else
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
#endif
  }

  void setTorques(const arr& u){
#ifdef USE_FAKE //fake implementation: replace real physics dynamics by trivial double integrator
    double invMass = 1.;
    double tau=.01;
    fake_qDot += .5*tau*invMass*u;
    fake_q += tau*fake_qDot;
    fake_qDot += .5*tau*invMass*u;
    LOG(0) <<"stepping..." <<u <<fake_q <<fake_qDot;
#else
    //-- convert joint torques to motor torques
    arr Jacobian = getJacobian();
    arr u_s = ~Jacobian * u; //Jacobian transpose law

    //-- send torques to motors
    CHECK_EQ(u_s.N, motors.N, "control signal has wrong dimensionality");
    for(uint i=0;i<motors.N;i++){
      motors(i)->setTarget(u_s(i)/motors(i)->maxTorque*32767);
    }
#endif
  }
};

OmnibaseThread::~OmnibaseThread(){
  LOG(0) <<"shutting down Omnibase " <<robotID;
  threadClose();
}

void OmnibaseThread::init(uint _robotID, const uintA& _qIndices) {
  robotID=_robotID;
  qIndices=_qIndices;

  CHECK_EQ(qIndices.N, 3, "");
  qIndices_max = rai::max(qIndices);

  //-- basic Kp Kd settings for reference control mode
  Kp_freq = rai::getParameter<arr>("Omnibase/Kp_freq", arr{10., 10., 10.});
  Kd_ratio = rai::getParameter<arr>("Omnibase/Kd_ratio", arr{.5, .5, .5});
  LOG(0) << "Omnibase: Kp_freq:" << Kp_freq << " Kd_ratio:" << Kd_ratio;

  //-- get robot address
  address = rai::getParameter<rai::String>("Omnibase/address", "172.16.0.2");

  //-- start thread and wait for first state signal
  LOG(0) <<"launching Omnibase " <<robotID <<" at " <<address;

  threadLoop();
  while(Thread::step_count<2) rai::wait(.01);
}

void OmnibaseThread::open(){
  // connect to robot
   robot = make_shared<OmnibaseController>(address);

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

void OmnibaseThread::step(){
  steps++;

  //-- get current state from robot
  arr q_real, qDot_real;
  robot->getState(q_real, qDot_real);
//  arr torquesExternal_real(robot_state.tau_ext_hat_filtered.begin(), robot_state.tau_ext_hat_filtered.size(), false);
//  arr torques_real(robot_state.tau_J.begin(), robot_state.tau_J.size(), false);

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
    if(err>.05){ //stall!
      state.set()->stall = 2; //no progress in reference time! for at least 2 iterations (to ensure continuous stall with multiple threads)
      cout <<"STALLING - step:" <<steps <<" err: " <<err <<endl;
    }
  }

  //-- compute torques from control message depending on the control type
  arr u;

  //-- compute desired torques
  arr Kp(3), Kd(3);
  CHECK_EQ(Kp_freq.N, 3,"");
  CHECK_EQ(Kd_ratio.N, 3,"");
  for(uint i=0;i<3;i++){
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
  u.resize(3).setZero();

  //-- add feedback term
  if(q_ref.N==3){
    u += Kp * (q_ref - q_real);
  }
  if(qDot_ref.N==3){
    u += Kd % (qDot_ref - qDot_real);
  }

  //-- project with compliance
  if(P_compliance.N) u = P_compliance * u;

  //-- data log?
  if(writeData>0 && !(steps%1)){
    if(!dataFile.is_open()) dataFile.open(STRING("z.omnibase"<<robotID <<".dat"));
    dataFile <<ctrlTime <<' '; //single number
    q_real.modRaw().write(dataFile); //3
    q_ref.modRaw().write(dataFile); //3
    if(writeData>1){
      qDot_real.modRaw().write(dataFile); //3
      qDot_ref.modRaw().write(dataFile); //3
      u.modRaw().write(dataFile); //3
      qDDot_ref.modRaw().write(dataFile);
    }
    dataFile <<endl;
  }

  //-- send torques
  robot->setTorques(u);
}

void OmnibaseThread::close(){
  robot.reset();
}

#else //RAI_Omnibase

OmnibaseThread::~OmnibaseThread(){ NICO }
void OmnibaseThread::init(uint _robotID, const uintA& _qIndices) { NICO }
void OmnibaseThread::step(){ NICO }
void OmnibaseThread::open(){ NICO }
void OmnibaseThread::close(){ NICO }

#endif
