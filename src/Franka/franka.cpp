#include "franka.h"

#include <franka/model.h>
#include <franka/robot.h>

void naturalGains(double& Kp, double& Kd, double decayTime, double dampingRatio);

const char *frankaIpAddresses[2] = {"172.16.0.2", "172.17.0.2"};

FrankaThread::FrankaThread(Var<CtrlMsg>& _ctrl, Var<CtrlMsg>& _state, uint whichRobot, const uintA& _qIndices)
  : Thread("FrankaThread"),
    ctrl(_ctrl),
    state(_state),
    qIndices(_qIndices){
  if(qIndices.N){
    CHECK_EQ(qIndices.N, 7, "");
    qIndices_max = qIndices.max();
  }

  //-- basic Kp Kd settings
  Kp_freq = rai::getParameter<arr>("Franka/Kp_freq", ARR(15., 15., 15., 10., 5., 5., 3.));
  Kd_ratio = rai::getParameter<arr>("Franka/Kd_ratio", ARR(.8, .8, .7, .7, .1, .1, .1));
  LOG(0) <<"FRANKA: Kp_freq=" <<Kp_freq <<" Kd_ratio=" <<Kd_ratio;

  //-- choose robot/ipAddress
  CHECK_LE(whichRobot, 1, "");
  ipAddress = frankaIpAddresses[whichRobot];

  //-- start thread and wait for first state signal
  threadStep();  //this is not looping! The step method passes a callback to robot.control, which is blocking until stop becomes true
  while(firstTime) rai::wait(.01);
}

FrankaThread::~FrankaThread(){
  stop = true;
  rai::wait(.1);
  threadClose();
}

void FrankaThread::step(){
  // connect to robot
  franka::Robot robot(ipAddress);

  // load the kinematics and dynamics model
  franka::Model model = robot.loadModel();

  // set collision behavior
  robot.setCollisionBehavior({{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
  {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
  {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
  {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}});

  // initialize state and ctrl with first state
  {
    franka::RobotState initial_state = robot.readOnce();
    arr q, qdot;
    q.setCarray(initial_state.q.begin(), initial_state.q.size());
    qdot.setCarray(initial_state.dq.begin(), initial_state.dq.size());

    auto stateset = state.set();
    auto ref = ctrl.set();
    if(!qIndices.N){
      stateset->q = q;
      stateset->qdot = qdot;
      ref->q = q;
    }else{
      while(stateset->q.N<=qIndices_max) stateset->q.append(0.);
      while(stateset->qdot.N<=qIndices_max) stateset->qdot.append(0.);
      while(ref->q.N<=qIndices_max) ref->q.append(0.);
      for(uint i=0;i<7;i++){
        stateset->q(qIndices(i)) = q(i);
        stateset->qdot(qIndices(i)) = qdot(i);
        ref->q(qIndices(i)) = q(i);
      }
    }
  }



  // define callback for the torque control loop
  std::function<franka::Torques(const franka::RobotState&, franka::Duration)>
      torque_control_callback = [&](const franka::RobotState& robot_state,
                                   franka::Duration /*duration*/) -> franka::Torques {

    //-- get current state
    arr q, qdot;
    q.setCarray(robot_state.q.begin(), robot_state.q.size());
    qdot.setCarray(robot_state.dq.begin(), robot_state.dq.size());
    //publish state
    {
      auto stateset = state.set();
      if(!qIndices.N){
        stateset->q = q;
        stateset->qdot = qdot;
      }else{
        for(uint i=0;i<7;i++){
          stateset->q(qIndices(i)) = q(i);
          stateset->qdot(qIndices(i)) = qdot(i);
        }
      }
    }

    //-- get current ctrl
    arr q_ref, qdd_des, P_compliance;
    {
      auto ref = ctrl.get();
      if(!qIndices.N){
        q_ref = ref->q;
      }else{
        if(q_ref.N!=7) q_ref.resize(7);
        for(uint i=0;i<7;i++){
          q_ref(i) = ref->q(qIndices(i));
        }
      }
      P_compliance = ref->P_compliance;
      qdd_des = zeros(7);
    }

    firstTime=false;

    //check for correct ctrl otherwise do something...
    if(q_ref.N!=7){
      cerr <<"FRANKA: inconsistent ctrl q_ref message" <<endl;
      return std::array<double, 7>({0., 0., 0., 0., 0., 0., 0.});
    }
    if(P_compliance.N){
      if(!(P_compliance.nd==2 && P_compliance.d0==7 && P_compliance.d1==7)){
        cerr <<"FRANKA: inconsistent ctrl P_compliance message" <<endl;
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

    if(q_ref.N==7){
      qdd_des += Kp % (q_ref - q) - Kd % qdot;
    }

    if(P_compliance.N) qdd_des = P_compliance * qdd_des;

#if 0
    arr M;
    M.setCarray(model.mass(robot_state).begin(), 49);
    M.reshape(7,7);

    arr u = M * qdd_des;
#else
    arr u = qdd_des;
#endif

    if(P_compliance.N) u = P_compliance * u;

    std::array<double, 7> u_array = {0., 0., 0., 0., 0., 0., 0.};
    std::copy(u.begin(), u.end(), u_array.begin());

    if(stop) return franka::MotionFinished(franka::Torques(u_array));
    return u_array;
  };

  // start real-time control loop
  robot.control(torque_control_callback);
}
