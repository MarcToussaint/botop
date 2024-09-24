#include "simulation.h"
#include <Kin/kin.h>
#include <Kin/frame.h>
#include <Kin/F_collisions.h>
#include <Kin/viewer.h>

void naturalGains(double& Kp, double& Kd, double decayTime, double dampingRatio);

BotThreadedSim::BotThreadedSim(const rai::Configuration& C,
                               const Var<rai::CtrlCmdMsg>& _cmd, const Var<rai::CtrlStateMsg>& _state,
                               const StringA& joints,
                               double _tau, double hyperSpeed)
  : RobotAbstraction(_cmd, _state),
    Thread("FrankaThread_Emulated"),
    simConfig(C),
    tau(_tau){

  //create a rai Simulator!
  int verbose = rai::getParameter<int>("botsim/verbose", 1);
  if(tau<0.) tau = rai::getParameter<double>("botsim/tau", .01);
  if(hyperSpeed<0.) hyperSpeed = rai::getParameter<double>("botsim/hyperSpeed", 1.);
  Thread::metronome.reset(tau/hyperSpeed);
  rai::String engine = rai::getParameter<rai::String>("botsim/engine", "physx");
  sim=make_shared<rai::Simulation>(simConfig, rai::Enum<rai::Simulation::Engine>(engine), verbose);

  {
    q_real = C.getJointState();
    qDot_real.resize(q_real.N).setZero();
    //    collisionPairs = simConfig.getCollidablePairs();
    //    cout <<" CollisionPairs:" <<endl;
    //    for(uint i=0;i<collisionPairs.d0;i++) cout <<collisionPairs(i,0)->name <<'-' <<collisionPairs(i,1)->name <<endl;

    if(joints.N){
      q_indices.resize(joints.N);
      uint i=0;
      for(auto& s:joints){
        rai::Frame *f = C.getFrame(s);
        CHECK(f, "frame '" <<s <<"' does not exist");
        CHECK(f->joint, "frame '" <<s <<"' is not a joint");
        CHECK(f->joint->dim==1, "joint '" <<s <<"' is not 1D");
        q_indices(i++) = f->joint->qIndex;
      }
      CHECK_EQ(i, joints.N, "");
    }else{
      q_indices.setStraightPerm(q_real.N);
    }
  }
  state.set()->q = q_real;
  state.set()->qDot = qDot_real;
  //emuConfig.watch(false, STRING("EMULATION - initialization"));
  //emuConfig.gl()->update(0, true);
  threadLoop();
  state.waitForNextRevision(); //this is enough to ensure the ctrl loop is running
}

BotThreadedSim::~BotThreadedSim(){
  LOG(0) <<"shutting down SimThread";
  threadClose();
  sim.reset();
  simConfig.view_close();
}

void BotThreadedSim::pullDynamicStates(rai::Configuration& C){
  auto mux = stepMutex(RAI_HERE);
  for(rai::Frame *f:C.frames){
    if(f->inertia && f->inertia->type==rai::BT_dynamic){
      f->set_X() = simConfig.frames(f->ID)->ensure_X(); //THIS IS DEBATABLE! In the real world, one could not just sync with the true state of all dynamic objects... so simulation should also not..?
    }
    if(f->joint && !f->joint->active && f->joint->dim==1){ //gripper?
      CHECK_EQ(f->joint->qIndex, simConfig.frames(f->ID)->joint->qIndex, "");
      f->joint->setDofs(simConfig.qInactive, f->joint->qIndex);
    }
  }
}

void BotThreadedSim::step(){
  //-- get real time
  ctrlTime += tau;
  //  ctrlTime = rai::realTime();

  //-- publish state
  {
    arr tauExternal;
    tauExternal = zeros(q_real.N);
    auto stateSet = state.set();
    stateSet->ctrlTime=ctrlTime;
    stateSet->q.resize(q_real.N).setZero();
    stateSet->qDot.resize(qDot_real.N).setZero();
    stateSet->tauExternalIntegral.resize(q_real.N).setZero();
    for(uint i:q_indices){
      stateSet->q(i) = q_real(i);
      stateSet->qDot(i) = qDot_real(i);
      stateSet->tauExternalIntegral(i) = tauExternal(i);
    }
  }

  //-- publish to sim_config
//  {
//    sim_config.set()->setJointState(q);
//  }

  //-- get current ctrl
  arr cmd_q_ref, cmd_qDot_ref, cmd_qDDot_ref, KpRef, KdRef, P_compliance; // TODO Kp, Kd, u_b and also read out the correct indices
  {
    auto cmdGet = cmd.get();

    if(!cmdGet->ref){
      cmd_q_ref = q_real;
      cmd_qDot_ref.resize(q_real.N).setZero();
      cmd_qDDot_ref.resize(q_real.N).setZero();
    }else{
      //get the reference from the callback (e.g., sampling a spline reference)
      cmdGet->ref->getReference(cmd_q_ref, cmd_qDot_ref, cmd_qDDot_ref, q_real, qDot_real, ctrlTime);
    }

    KpRef = cmdGet->Kp;
    KdRef = cmdGet->Kd;
    P_compliance = cmdGet->P_compliance;
  }

  if(cmd_q_ref.N && cmd_qDot_ref.N){
    sim->step((cmd_q_ref, cmd_qDot_ref), tau, sim->_posVel);
  }else{
    sim->step({}, tau, sim->_none);
  }
  q_real = simConfig.getJointState();
  if(cmd_qDot_ref.N==qDot_real.N) qDot_real = cmd_qDot_ref;

  //-- add other crazy perturbations?
//  if((step_count%1000)<100) q_real(0) = .1;

  //-- display? 20fps
  if(false && !(step_count%int(1./(20.*tau)))){
    simConfig.setJointState(q_real);
    simConfig.view(false, STRING("EMULATION - time " <<ctrlTime));
  }

  //if(!(step_count%100)) cout <<"simulation thread load:" <<timer.report() <<endl;

  //-- check for collisions!
#if 0
  emuConfig.setJointState(q_real);
  emuConfig.ensure_proxies();
  double p = emuConfig.getTotalPenetration();
  if(p>0.){
    emuConfig.reportProxies(std::cout, 0.01, false);
    emuConfig.watch(false, "EMU CONFIG IN COLLISION");
    rai::wait(); metronome.reset(tau);
  }
#endif

  //-- data log?
  if(writeData>0 && !(step_count%1)){
    if(!dataFile.is_open()) dataFile.open("z.panda.dat");
    dataFile <<ctrlTime <<' '; //single number
    q_real.modRaw().write(dataFile); //7
    cmd_q_ref.modRaw().write(dataFile); //7
    if(writeData>1){
      qDot_real.modRaw().write(dataFile); //7
      cmd_qDot_ref.modRaw().write(dataFile); //7
      //qDDot_des.modRaw().write(dataFile); //7
      //cmd_qDDot_ref.modRaw().write(dataFile);
    }
    if(writeData>2){
    }
    dataFile <<endl;
  }
}

void GripperSim::open(double width, double speed) {
  auto mux = simthread->stepMutex(RAI_HERE);
  simthread->sim->moveGripper(gripperName, width, speed);
  q=width;
  isClosing=false; isOpening=true;
}

void GripperSim::close(double force, double width, double speed) {
  auto mux = simthread->stepMutex(RAI_HERE);
  simthread->sim->closeGripper(gripperName, width, speed);
  q=width;
  isOpening=false; isClosing=true;
}

void GripperSim::closeGrasp(const char* objName, double force, double width, double speed){
  auto mux = simthread->stepMutex(RAI_HERE);
  simthread->sim->closeGripperGrasp(gripperName, objName);
//  simthread->sim->attach(gripperName, objName);
  q=width;
  isOpening=false; isClosing=true;
}

double GripperSim::pos(){
  auto mux = simthread->stepMutex(RAI_HERE);
  return simthread->sim->getGripperWidth(gripperName);
}

bool GripperSim::isDone(){
  auto mux = simthread->stepMutex(RAI_HERE);
  return simthread->sim->gripperIsDone(gripperName);
}
