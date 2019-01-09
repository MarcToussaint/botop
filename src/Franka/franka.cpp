#include "franka.h"

#include <franka/model.h>
#include <franka/robot.h>

void naturalGains(double& Kp, double& Kd, double decayTime, double dampingRatio);

FrankaThread::FrankaThread(Var<CtrlMsg>& _ctrl, Var<CtrlMsg>& _state)
  : Thread("FrankaThread"),
    ctrl(_ctrl),
    state(_state) {
  threadStep();  //this is not looping! The step method passes a callback to robot.control, which is blocking until stop becomes true
  state.waitForNextRevision(); //this is enough to ensure the ctrl loop is running
  ctrl.set()->q = state.get()->q;
}

FrankaThread::~FrankaThread(){
  stop = true;
  rai::wait(.1);
  threadClose();
}

void FrankaThread::step(){
  // connect to robot
  franka::Robot robot("172.16.0.2");

  //setDefaultBehavior:
  robot.setCollisionBehavior(
  {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}}, {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
  {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0}}, {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0}},
  {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}}, {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
  {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0}}, {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0}});
  robot.setJointImpedance({{3000, 3000, 3000, 2500, 2500, 2000, 2000}});
  robot.setCartesianImpedance({{3000, 3000, 3000, 300, 300, 300}});

  // load the kinematics and dynamics model
  franka::Model model = robot.loadModel();
  //    franka::RobotState initial_state = robot.readOnce();

  // set collision behavior
  robot.setCollisionBehavior({{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
  {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
  {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
  {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}});


  // define callback for the torque control loop
  std::function<franka::Torques(const franka::RobotState&, franka::Duration)>
      impedance_control_callback = [&](const franka::RobotState& robot_state,
                                   franka::Duration /*duration*/) -> franka::Torques {

    //-- get current state
    arr q, qdot;
    q.setCarray(robot_state.q.begin(), robot_state.q.size());
    qdot.setCarray(robot_state.dq.begin(), robot_state.dq.size());
    //publish state
    {
      auto stateset = state.set();
      stateset->q = q;
      stateset->qdot = qdot;
    }

    //-- get current ctrl
    arr q_ref, qdd_des, P_compliance;
    {
      auto ref = ctrl.get();
      q_ref = ref->q;
      P_compliance = ref->P_compliance;
      qdd_des = zeros(7);
    }

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
    double k_p=50., k_d=3.;
    naturalGains(k_p, k_d, .2, .8);

    if(q_ref.N==7){
      qdd_des += k_p * (q_ref - q) - k_d * qdot;
    }

    if(P_compliance.N) qdd_des = P_compliance * qdd_des;

    arr M;
    M.setCarray(model.mass(robot_state).begin(), 49);
    M.reshape(7,7);

    arr u = M * qdd_des;

    if(P_compliance.N) u = P_compliance * u;

    std::array<double, 7> u_array = {0., 0., 0., 0., 0., 0., 0.};
    std::copy(u.begin(), u.end(), u_array.begin());

    if(stop) return franka::MotionFinished(franka::Torques(u_array));
    return u_array;
  };

  // start real-time control loop
  robot.control(impedance_control_callback);
}
