#include <Franka/franka.h>
#include <Franka/help.h>


const char *USAGE =
    "\nTest of low-level (without bot interface) FrankaThreadNew with trivial reference interface"
    "\n";


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
      else q_ref = q_real;  //->no position gains at all
    }
    {
      arr vel = velocity_ref.get()();
      if(vel.N==1 && vel.scalar()==0.) qDot_ref.resize(qDot_real.N).setZero(); //[0] -> zero vel reference -> damping
      else if(vel.N) qDot_ref = vel;
      else qDot_ref = qDot_real;  //[] -> no damping at all!
    }
    qDDot_ref.resize(q_ref.N).setZero();
  }
};


void test() {
  //-- setup a configuration
  rai::Configuration C;
  C.addFile(rai::raiPath("../rai-robotModels/scenarios/pandaSingle.g"));
  C.view(true);

  //-- start the franka thread
  C.ensure_indexedJoints();
  FrankaThread robot(0, franka_getJointIndices(C,'l'));

  //comment the next line to only get gravity compensation instead of 'zero reference following' (which includes damping)
  auto ref = make_shared<ZeroReference>();
  robot.cmd.set()->ref = ref;
  //ref->setPositionReference(robot.state.get()->q);
  //ref->setVelocityReference({.0,.0,.2,0,0,0,0});

  for(;;){
    if(C.view(false,STRING("time: "<<rai::realTime()))=='q') break;
    C.setJointState(robot.state.get()->q);
    rai::wait(.1);
  }
}


int main(int argc,char **argv){
  rai::initCmdLine(argc, argv);

  cout <<USAGE <<endl;

  test();

  LOG(0) <<" === bye bye ===\n used parameters:\n" <<rai::params() <<'\n';

  return 0;
}
