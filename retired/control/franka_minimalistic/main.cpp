//#include <franka/duration.h>
//#include <franka/exception.h>
#include <franka/model.h>
#include <franka/robot.h>

#include "examples_common.h"

#include <Core/thread.h>

struct FrankaControlMessage{
  arr qref;

};
struct FrankaFeedbackMessage{
  arr qreal, qdotreal;
};

struct FrankaThread : Thread{
  Var<FrankaControlMessage> ctrl;
  Var<FrankaFeedbackMessage> state;
  bool stop=false;

  FrankaThread(Var<FrankaControlMessage>& _ctrl, Var<FrankaFeedbackMessage>& _state)
    : Thread("FrankaThread"),
      ctrl(_ctrl),
      state(_state) {
    threadStep();  //this is not looping! The step method passes a callback to robot.control, which is blocking until stop becomes true
  }
  ~FrankaThread(){
    threadClose();
  }

  void open(){}
  void step(){
    // connect to robot
    franka::Robot robot("172.16.0.2");
    setDefaultBehavior(robot);
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


      arr q, qdot;

      q.setCarray(robot_state.q.begin(), robot_state.q.size());
      qdot.setCarray(robot_state.dq.begin(), robot_state.dq.size());
      arr q_ref = ctrl.get()->qref;
      arr qdd_des = zeros(7);

      {
        auto stateset = state.set();
        stateset->qreal = q;
        stateset->qdotreal = qdot;
      }

      double k_p=20., k_d=5.;

      if(q_ref.N==7){
        qdd_des += k_p * (q_ref - q) - k_d * qdot;
      }

      arr M;
      M.setCarray(model.mass(robot_state).begin(), 49);
      M.reshape(7,7);

      arr u = M * qdd_des;

      std::array<double, 7> u_array = {0., 0., 0., 0., 0., 0., 0.};
      std::copy(u.begin(), u.end(), u_array.begin());

      if(stop) return franka::MotionFinished(franka::Torques(u_array));
      return u_array;
    };

    // start real-time control loop
    robot.control(impedance_control_callback);
  }
  void close(){}
};

int main(int argc, char** argv) {

  Var<FrankaControlMessage> ctrl;
  Var<FrankaFeedbackMessage> state;

  FrankaThread F(ctrl, state);

  rai::wait(.5);
  state.waitForNextRevision();
  ctrl.set()->qref = state.get()->qreal;

  rai::wait();
  F.stop=true;

  return 0;
}
