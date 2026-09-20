#include "TrossenThread.h"

#ifdef RAI_TROSSEN

//COPY AND PASTE from trossen_arm/demos/cpp/gravity_compensation

#include "libtrossen_arm/trossen_arm.hpp"

TrossenThread::TrossenThread(rai::Var<rai::CtrlCmdMsg>& cmd, rai::Var<rai::CtrlStateMsg>& state, const strA& ids)
    : rai::RobotAbstraction(cmd, state),
    Thread("TrossenThread", .002), //HARD CODED step frequency of 100Hz
    ipAddresses(ids) {

  Kp = rai::getParameter<arr>("Trossen/Kp", arr{});
  Kd = rai::getParameter<arr>("Trossen/Kd", arr{}); //FOR TROSSEN, this corresponds to the Kp of the velocity PID

  LOG(0) <<"launching Trossen at " <<ipAddresses;

  threadOpen(true);
  threadLoop();
}

void print_motor_parameters(const std::vector<std::map<trossen_arm::Mode, trossen_arm::MotorParameter>>& motor_parameters)
{
  for (size_t i = 0; i < motor_parameters.size(); ++i) {
    const std::map<trossen_arm::Mode, trossen_arm::MotorParameter>& motor_parameter =
        motor_parameters.at(i);
    std::cout << "  Joint " << i << ":" << std::endl;
    for (const auto& [mode, parameter] : motor_parameter) {
      std::cout << "    Mode " << static_cast<int>(mode) << ":" << std::endl;
      std::cout << "      Position loop:";
      std::cout << " kp: " << parameter.position.kp;
      std::cout << ", ki: " << parameter.position.ki;
      std::cout << ", kd: " << parameter.position.kd;
      std::cout << ", imax: " << parameter.position.imax << std::endl;
      std::cout << "      Velocity loop:";
      std::cout << " kp: " << parameter.velocity.kp;
      std::cout << ", ki: " << parameter.velocity.ki;
      std::cout << ", kd: " << parameter.velocity.kd;
      std::cout << ", imax: " << parameter.velocity.imax << std::endl;
    }
  }
}

void TrossenThread::open(){
  driver = make_shared<trossen_arm::TrossenArmDriver>();

  CHECK_EQ(ipAddresses.N, 1, "only 1 trossen for now");

  driver->configure(
      trossen_arm::Model::wxai_v0,
      trossen_arm::StandardEndEffector::wxai_v0_follower,
      ipAddresses(0).p,
      true
      );

  //== change Kp, Kd?
  driver->set_motor_parameters(trossen_arm::StandardMotorParameters::wxai_v0_latest);

  auto motor_parameters = driver->get_motor_parameters();

  arr Kp_default(7), Kd_default(7);
  for(uint i=0;i<Kp.N;i++){
    Kp_default(i) = motor_parameters.at(i).at(trossen_arm::Mode::position).position.kp;
    Kd_default(i) = motor_parameters.at(i).at(trossen_arm::Mode::position).velocity.kp;
  }
  cout <<"Trossen/Kp: " <<Kp_default <<endl;
  cout <<"Trossen/Kd: " <<Kd_default <<endl;
  if(Kp.N==7 && Kd.N==7){
    for(uint i=0;i<Kp.N;i++){
      Kp_default(i) = motor_parameters.at(i).at(trossen_arm::Mode::position).position.kp = Kp(i);
      Kd_default(i) = motor_parameters.at(i).at(trossen_arm::Mode::position).velocity.kp = Kd(i);
    }
    cout <<"Trossen/Kp: " <<Kp <<endl;
    cout <<"Trossen/Kd: " <<Kd <<endl;
    driver->set_motor_parameters(motor_parameters);
  }

  //get initial state
  arr q_init = as_arr(driver->get_all_positions(), false);
  {
    auto stateSet = state.set();
    stateSet->q = q_init;
    stateSet->qDot.resize(q_init.N).setZero();
    stateSet->tauExternalIntegral.resize(q_init.N).setZero();
    stateSet->tauExternalCount=0;
  }
  //set initial position reference
  {
    auto cmd_set = cmd.set();
    cmd_set->setConst(q_init, false, true);
  }

  // start effort control mode
#if 0 //own PD
  driver->set_all_modes(trossen_arm::Mode::external_effort);
  driver->set_all_external_efforts({0, 0, 0, 0, 0, 0, 0}, 0.0f, false);
#else
  mode = position_mode;
  driver->set_all_modes(trossen_arm::Mode::position);
#endif
}

void TrossenThread::close(){
  driver->set_all_modes(trossen_arm::Mode::idle);
  rai::wait(.1);
  driver->cleanup(false);
  driver.reset();
}

void TrossenThread::step(){
  //-- get real state
  arr q_real = as_arr(driver->get_all_positions(), false);
  arr qDot_real = as_arr(driver->get_all_velocities(), false);
  arr tauExternal = as_arr(driver->get_all_external_efforts(), false);

  //-- publish state & INCREMENT CTRL TIME
  {
    auto stateSet = state.set();
    if(!stateSet->stall) stateSet->ctrlTime += metronome.ticInterval;
    else stateSet->stall--;
    ctrlTime = stateSet->ctrlTime;
    stateSet->q = q_real;
    stateSet->qDot = qDot_real;
    stateSet->tauExternalIntegral += tauExternal;
    stateSet->tauExternalCount++;
  }

  //-- get current ctrl reference
  arr q_ref, qDot_ref, qDDot_ref;
  {
    auto cmdGet = cmd.get();

    //get commanded reference from the reference callback (e.g., sampling a spline reference)
    if(cmdGet->ref){
      cmdGet->ref->getReference(q_ref, qDot_ref, qDDot_ref, q_real, qDot_real, ctrlTime);
    }else{
      q_ref = q_real;
    }
  }

  //-- check reference error
  bool isStalled = false;
  if(q_ref.N){
    double err = length(q_ref - q_real);
    if(err>.05){ //stall!
      state.set()->stall = 2; //no progress in reference time! for at least 2 iterations (to ensure continuous stall with multiple threads)
      isStalled=true;
      cout <<"STALLING - err: " <<err <<' ' <<q_ref - q_real <<endl;
    }
  }

  //-- data log?
  if(writeData>0 && !(step_count%5)){
    if(!dataFile.is_open()) dataFile.open(STRING("z.trossen.dat"));
    dataFile <<ctrlTime <<' ' <<q_real.modRaw() <<' ' <<q_ref.modRaw() <<endl;
  }

#if 0 //own PD
  arr u;
  u.resize(q_real.N).setZero();
  if(q_ref.N){
    u += Kp % (q_ref - q_real);
    u += Kd % (qDot_ref - qDot_real);
  }

  driver->set_all_external_efforts(as_vector(u), 0.0f, false);
#else
  if(q_ref.N){
    if(mode!=position_mode){
      driver->set_all_modes(trossen_arm::Mode::position);
      mode=position_mode;
    }
    if(!isStalled){
      driver->set_all_positions(as_vector(q_ref), 0.0f, false, as_vector(qDot_ref));
    }
  }else{
    if(mode!=torque_mode){
      driver->set_all_modes(trossen_arm::Mode::external_effort);
      mode=torque_mode;
    }
    arr u = zeros(q_real.N);
    driver->set_all_external_efforts(as_vector(u), 0.0f, false);
  }
#endif
}

#else

TrossenThread::TrossenThread(rai::Var<rai::CtrlCmdMsg>& cmd, rai::Var<rai::CtrlStateMsg>& state, const strA& ids)
    : rai::RobotAbstraction(cmd, state),
    Thread("TrossenThread", .002), //HARD CODED step frequency of 100Hz
    ipAddresses(ids) { NICO }
void TrossenThread::open(){ NICO }
void TrossenThread::step(){ NICO }
void TrossenThread::close(){ NICO }

#endif
