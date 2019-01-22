#include "controlEmulator.h"
#include <Kin/kin.h>
#include <Kin/frame.h>

void naturalGains(double& Kp, double& Kd, double decayTime, double dampingRatio);

ControlEmulator::ControlEmulator(Var<CtrlMsg>& _ctrl,
                                 Var<CtrlMsg>& _state,
                                 const rai::KinematicWorld& K,
                                 const StringA& joints,
                                 double _tau)
    : Thread("FrankaThread", _tau),
      ctrl(_ctrl),
      state(_state),
      tau(_tau){
//        rai::KinematicWorld K(rai::raiPath("../rai-robotModels/panda/panda.g"));
//        K["panda_finger_joint1"]->joint->makeRigid();

    q = K.getJointState();
    qdot.resize(q.N).setZero();
    if(joints.N){
        q_indices.resize(joints.N);
        uint i=0;
        for(auto& s:joints){
            rai::Frame *f = K.getFrameByName(s);
            CHECK(f, "frame '" <<s <<"' does not exist");
            CHECK(f->joint, "frame '" <<s <<"' is not a joint");
            CHECK(f->joint->qDim()==1, "joint '" <<s <<"' is not 1D");
            q_indices(i++) = f->joint->qIndex;
        }
        CHECK_EQ(i, joints.N, "");
    }else{
        q_indices.setStraightPerm(q.N);
    }
    threadLoop();
    state.waitForNextRevision(); //this is enough to ensure the ctrl loop is running
    ctrl.set()->q = state.get()->q;
}

ControlEmulator::~ControlEmulator(){
    threadClose();
}

void ControlEmulator::step(){
    //-- publish state
    {
      auto stateset = state.set();
      stateset->q.resize(q.N).setZero();
      stateset->qdot.resize(qdot.N).setZero();
      for(uint i:q_indices){
          stateset->q(i) = q(i);
          stateset->qdot(i) = qdot(i);
      }
    }

    //-- get current ctrl
    arr q_ref, qdd_des, P_compliance;
    {
      auto ref = ctrl.get();
      q_ref = ref->q;
      P_compliance = ref->P_compliance;
      qdd_des = zeros(ref->q.N);
    }

    //check for correct ctrl otherwise do something...
    if(q_ref.N!=q.N){
        LOG(-1) <<"inconsistent ctrl message";
        return;
    }
    if(P_compliance.N){
      if(!(P_compliance.nd==2 && P_compliance.d0==q.N && P_compliance.d1==q.N)){
        LOG(-1) <<"inconsistent ctrl P_compliance message";
        P_compliance.clear();
      }
    }

    //-- compute desired torques
    double k_p, k_d;
    naturalGains(k_p, k_d, .2, .8);
    if(q_ref.N==q.N){
      qdd_des += k_p * (q_ref - q) - k_d * qdot;
    }

    //-- directly integrate
    q += .5 * tau * qdot;
    qdot += tau * qdd_des;
    q += .5 * tau * qdot;
}
