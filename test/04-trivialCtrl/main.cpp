//#include <NewControl/TaskControlThread.h>
//#include <LGPop/lgpop.h>
#include <Kin/kinViewer.h>

#include <Gui/viewer.h>

#include <Franka/controlEmulator.h>

#include <Franka/franka.h>
#include <Franka/gripper.h>
#include <Franka/help.h>


struct MyThread : Thread {
  Var<rai::Configuration> ctrl_config;
  Var<CtrlCmdMsg> ctrl_ref;
  Var<CtrlStateMsg> ctrl_state;

  arr q_real, qdot_real, tauExternal; //< real state
  arr q0; //< homing pose
  //arr Hmetric;

  bool requiresInitialSync;
  int verbose;

  MyThread(const Var<rai::Configuration>& _ctrl_config,
                    const Var<CtrlCmdMsg>& _ctrl_ref,
                    const Var<CtrlStateMsg>& _ctrl_state)
      : Thread("TaskControlThread", .01),
      ctrl_config(this, _ctrl_config),
      ctrl_ref(this, _ctrl_ref),
      ctrl_state(this, _ctrl_state),
      requiresInitialSync(true),
      verbose(0)
  {

    double hyper = rai::getParameter<double>("hyperSpeed", -1.);
    if(hyper>0.) this->metronome.reset(.01/hyper);

    //memorize the "NULL position", which is the initial model position
    q0 = ctrl_config.get()->getJointState();
    q_real = q0;
    qdot_real = zeros(q0.N);

    //Hmetric = rai::getParameter<double>("Hrate", .1)*ctrl_config.get()->getHmetric();

    threadLoop();
  };
  ~MyThread(){
      threadClose();
  }

  void step();
};


void MyThread::step() {
  //-- initial initialization
  if(requiresInitialSync){
    if(ctrl_state.getRevision()>1) {
      {
        auto state = ctrl_state.get();
        q_real = state->q;
        qdot_real = state->qDot;
      }
      ctrl_config.set()->setJointState(q_real);
      requiresInitialSync = false;
    } else{
      LOG(0) << "waiting for ctrl_state messages...";
      return;
    }
  }

  //-- read current state
  CtrlStateMsg ctrlStateMsg;
  {
    auto state = ctrl_state.get();
    ctrlStateMsg = ctrl_state();
    if(state->q.N){
      q_real = state->q;
      qdot_real = state->qDot;
      tauExternal = state->tauExternal;
    }
  }

  //-- update kinematic world for controller
  ctrl_config.set()->setJointState(q_real);

//  CtrlCmdMsg ctrlCmdMsg;
//  ctrlCmdMsg.qDotRef = {.01,.0,.0,.0,.0,.0,.0};
//  ctrl_ref.set() = ctrlCmdMsg;
}


void testNew() {
  Var<rai::Configuration> K;
  K.set()->addFile(rai::raiPath("../model/pandaSingle.g"));
  //K.set()()["R_panda_finger_joint1"]->joint->makeRigid();
  //K.set()()["L_panda_finger_joint1"]->joint->makeRigid();
  K.set()->optimizeTree();
  K.set()->watch(true);

  Var<CtrlCmdMsg> ctrlRef;
  Var<CtrlStateMsg> ctrlState;
  {
    auto set = ctrlState.set();
    set->q = K.get()->getJointState();
    set->tauExternal.resize(K.get()->getJointStateDimension());
    set->tauExternal.setZero();
  }

  ControlEmulator robot(K, ctrlRef, ctrlState);
//  FrankaThreadNew robotR(ctrlRef, ctrlState, 0, franka_getJointIndices(K.get()(),'R'));

  MyThread mine(K, ctrlRef, ctrlState);

  cout <<ctrlState.get()->q << endl;

  KinViewer viewer(K, 0.05);

  rai::wait();
}



int main(int argc, char * argv[]){
  testNew();

  return 0;
}
