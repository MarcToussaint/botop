#include <Franka/franka.h>
#include <Franka/help.h>

// define your own reference feed

struct ZeroReference : rai::ReferenceFeed {
  Var<arr> position_ref; ///< if set, defines a non-zero velocity reference
  Var<arr> velocity_ref; ///< if set, defines a non-zero velocity reference

  ZeroReference& setVelocityReference(const arr& _velocity_ref){ velocity_ref.set() = _velocity_ref; return *this; }
  ZeroReference& setPositionReference(const arr& _position_ref){ position_ref.set() = _position_ref; return *this; }

  /// callback called by a robot control loop
  virtual void getReference(arr& q_ref, arr& qDot_ref, arr& qDDot_ref, const arr& q_real, const arr& qDot_real, double ctrlTime){
    {
      arr pos = position_ref.get()();
      if(pos.N) q_ref = pos;
      else q_ref = q_real;
    }
    {
      arr vel = velocity_ref.get()();
      if(vel.N) qDot_ref = vel;
      else qDot_ref.resize(qDot_real.N).setZero();
    }
    qDDot_ref.resize(q_ref.N).setZero();
  }
};


void test() {
  //-- setup a configuration
  rai::Configuration C;
  C.addFile(rai::raiPath("../rai-robotModels/scenarios/pandaSingle.g"));
  C.watch(true);

  //-- start the franka thread
  C.ensure_indexedJoints();
  FrankaThreadNew robot(0, franka_getJointIndices(C,'R'));

  //comment the next line to only get gravity compensation instead of 'zero reference following' (which includes damping)
  auto ref = make_shared<ZeroReference>();
  robot.cmd.set()->ref = ref;
//  ref->setPositionReference(q_now);
  //ref->setVelocityReference({.0,.0,.2,0,0,0,0});

  for(;;){
    if(C.watch(false,STRING("time: "<<rai::realTime()))=='q') break;
    C.setJointState(robot.state.get()->q);
    rai::wait(.1);
  }

}

int main(int argc, char * argv[]){
  rai::initCmdLine(argc, argv);

  test();

  cout <<"bye bye" <<endl;

  return 0;
}
