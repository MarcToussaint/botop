#include "bot.h"

#include <Franka/help.h>
#include <Kin/F_qFeatures.h>
#include <Kin/viewer.h>

void ZeroReference::getReference(arr& q_ref, arr& qDot_ref, arr& qDDot_ref, const arr& q_real, const arr& qDot_real, double ctrlTime){
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

BotOp::BotOp(rai::Configuration& C, bool sim){
  C.ensure_indexedJoints();
  qHome = C.getJointState();
  if(!sim){
    robot = make_unique<FrankaThreadNew>(0, franka_getJointIndices(C,'R'));
    gripper = make_unique<FrankaGripper>(0);
  }else{
    robot = make_unique<ControlEmulator>(C);
    gripper = make_unique<GripperEmulator>();
  }
  C.setJointState(robot->state.get()->q);
}

BotOp::~BotOp(){
  robot.release();
}

bool BotOp::step(rai::Configuration& C, double waitTime){
  C.setJointState(robot->state.get()->q);
  C.gl()->raiseWindow();
  int key = C.watch(false,STRING("time: "<<robot->state.get()->time <<"\n[q or ESC to ABORT]"));
  if(key==13) return false;
  if(key=='q' || key==27) return false;
  if(waitTime) rai::wait(waitTime);
  return true;
}

arr getLoopPath(rai::Configuration& C){
  //add some targets in position space
  arr center = C["R_gripperCenter"]->getPosition();
  C.addFrame("target1")
      ->setShape(rai::ST_marker, {.1})
      .setPosition(center + arr{+.3,.0,+.2});
  C.addFrame("target2")
      ->setShape(rai::ST_marker, {.1})
      .setPosition(center + arr{+.3,.0,-.2});
  C.addFrame("target3")
      ->setShape(rai::ST_marker, {.1})
      .setPosition(center + arr{-.3,.0,+.2});
  C.addFrame("target4")
      ->setShape(rai::ST_marker, {.1})
      .setPosition(center + arr{-.3,.0,-.2});
  C.watch(false);
  arr q0 = C.getJointState();

  //compute a path
  KOMO komo;
  komo.setModel(C, false);
  komo.setTiming(5, 10, 2., 2);
  komo.add_qControlObjective({}, 2, 1.);

  komo.addObjective({1.}, FS_positionDiff, {"R_gripperCenter", "target1"}, OT_eq, {1e2});
  komo.addObjective({2.}, FS_positionDiff, {"R_gripperCenter", "target2"}, OT_eq, {1e2});
  komo.addObjective({3.}, FS_positionDiff, {"R_gripperCenter", "target3"}, OT_eq, {1e2});
  komo.addObjective({4.}, FS_positionDiff, {"R_gripperCenter", "target4"}, OT_eq, {1e2});
  komo.addObjective({5.}, make_shared<F_qItself>(C.getCtrlFramesAndScale(), true), {}, OT_eq, {1e2});
  komo.addObjective({5.}, make_shared<F_qItself>(C.getCtrlFramesAndScale()), {}, OT_eq, {1e2}, {}, 1);

  komo.optimize();
  komo.view(true);
  return komo.getPath_qOrg();
}
