#include "TrossenThread.h"

#ifdef RAI_TROSSEN

//COPY AND PASTE from trossen_arm/demos/cpp/gravity_compensation

#include "libtrossen_arm/trossen_arm.hpp"

TrossenThread::TrossenThread(Var<rai::CtrlCmdMsg>& cmd, Var<rai::CtrlStateMsg>& state, const char* ipAddress)
    : rai::RobotAbstraction(cmd, state),
    Thread("TrossenThread", .002), //HARD CODED step frequency of 100Hz
    ipAddress(ipAddress), fil("trossen.dat") {

  Kp = rai::getParameter<arr>("Trossen/Kp");
  Kd = rai::getParameter<arr>("Trossen/Kd"); //FOR TROSSEN, this corresponds to the Kp of the velocity PID

  LOG(0) <<"launching Trossen at " <<ipAddress;

  threadOpen(true);
  threadLoop();
}

void print_motor_parameters(const std::vector<std::map<trossen_arm::Mode, trossen_arm::MotorParameter>>& motor_parameters)
{
  std::cout << "Motor parameters:" << std::endl;
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

  driver->configure(
      trossen_arm::Model::wxai_v0,
      trossen_arm::StandardEndEffector::wxai_v0_leader,
      ipAddress.p,
      true
      );

  auto motor_parameters = driver->get_motor_parameters();
  print_motor_parameters(motor_parameters);

#if 0 //totally bad yet!!
  for(uint i=0;i<Kp.N;i++){
    motor_parameters.at(i).at(trossen_arm::Mode::position).position.kp = Kp(i);
    motor_parameters.at(i).at(trossen_arm::Mode::position).velocity.kp = Kd(i);
  }
  driver->set_motor_parameters(motor_parameters);
#else
  driver->set_motor_parameters(trossen_arm::StandardMotorParameters::wxai_v0_latest);
#endif

  { //get initial state
    arr q_init = as_arr(driver->get_all_positions(), false);
    auto stateSet = state.set();
    stateSet->q = q_init;
    stateSet->qDot.resize(q_init.N).setZero();
    stateSet->tauExternalIntegral.resize(q_init.N).setZero();
    stateSet->tauExternalCount=0;
  }

  // start effort control mode
#if 0 //own PD
  driver->set_all_modes(trossen_arm::Mode::external_effort);
  driver->set_all_external_efforts({0, 0, 0, 0, 0, 0, 0}, 0.0f, false);
#else
  //TODO, set motor params according to Kp Kd - for now just defaults
  driver->set_all_modes(trossen_arm::Mode::position);
#endif
}

void TrossenThread::close(){
  driver->set_all_modes(trossen_arm::Mode::idle);
  rai::wait(.1);
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

  //write into log file, need to be made optional
  fil <<ctrlTime <<' ' <<q_ref.modRaw() <<' ' <<q_real.modRaw() <<endl;

  //-- check reference error
  bool isStalled = false;
  if(q_ref.N){
    double err = length(q_ref - q_real);
    if(err>.05){ //stall!
      state.set()->stall = 2; //no progress in reference time! for at least 2 iterations (to ensure continuous stall with multiple threads)
      isStalled=true;
      cout <<"STALLING - err: " <<err <<endl;
    }
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
    if(!isStalled){
      driver->set_all_positions(as_vector(q_ref), 0.0f, false, as_vector(qDot_ref));
    }
  }
#endif
}

#else
TODO!!
#endif
