#include <BotOp/bot.h>
#include <BotOp/SequenceController.h>
#include <KOMO/manipTools.h>

#include <Kin/F_forces.h>

//===========================================================================

struct SequenceControllerExperiment{
  rai::Configuration& C;
  arr qHome;
  unique_ptr<SequenceController> ctrl;
  unique_ptr<BotOp> bot;
  Metronome tic;
  uint stepCount = 0;

  SequenceControllerExperiment(rai::Configuration& _C, double cycleTime=.1) : C(_C), tic(cycleTime){
    qHome = C.getJointState();
  }

  bool step(ObjectiveL& phi){
    stepCount++;

    //-- start a robot thread
    if(!bot){
      bot = make_unique<BotOp>(C, rai::checkParameter<bool>("real"));
      bot->home(C);
      bot->setControllerWriteData(1);
      rai::wait(.2);
    }

    //-- Sequence of Constraints MPC
    if(!ctrl)
      ctrl = make_unique<SequenceController>(C, phi, qHome);

    //-- iterate
    tic.waitForTic();

    //-- get optitrack
    if(bot->optitrack) bot->optitrack->pull(C);

    //-- get current state (time,q,qDot)
    arr q,qDot, q_ref, qDot_ref;
    double ctrlTime = 0.;
    bot->getState(q, qDot, ctrlTime);
    bot->getReference(q_ref, qDot_ref, NoArr, q, qDot, ctrlTime);

    //-- iterate MPC
    ctrl->cycle(C, phi, q_ref, qDot_ref, q, qDot, ctrlTime);
    ctrl->report(C, phi);

    //-- send leap target
    auto sp = ctrl->getSpline(bot->get_t());
    if(sp.pts.N) bot->move(sp.pts, sp.vels, sp.times, true);

    //-- update C
    bot->step(C, .0);
    if(bot->keypressed=='q' || bot->keypressed==27) return false;

    return true;
  }

};

//===========================================================================

void testBallFollowing() {
  rai::Configuration C;
  C.addFile(rai::raiPath("../rai-robotModels/scenarios/pandasTable-calibrated.g"));

  SequenceControllerExperiment ex(C, .02);

  C.addFrame("ball", "table")
      ->setShape(rai::ST_sphere, {.03})
      .setColor({1.,0,0})
      .setRelativePosition(arr{-.4,.4,.4});

  //-- define constraints
  ObjectiveL phi;
  phi.add({1.}, FS_positionRel, C, {"ball", "l_gripper"}, OT_eq, {1e1}, {0., 0., -.1});
  phi.add({1., 2.}, FS_positionRel, C, {"ball", "l_gripper"}, OT_eq, {{2,3},{1e1,0,0,0,1e1,0}});
  phi.add({2.}, FS_positionDiff, C, {"l_gripper", "ball"}, OT_eq, {1e1});

  bool useSimulatedBall=!rai::getParameter<bool>("bot/useOptitrack", false);
  arr ballVel = {.0, .0, .0};
  arr ballCen = C["ball"]->getPosition();

  while(ex.step(phi)){
    if(useSimulatedBall){
      if(!(ex.stepCount%20)){
        ballVel(0) = .01 * rnd.gauss();
        ballVel(2) = .01 * rnd.gauss();
      }
      if(!(ex.stepCount%40)) ballVel=0.;
      arr pos = C["ball"]->getPosition();
      pos += ballVel;
      pos = ballCen + .95 * (pos - ballCen);
      C["ball"]->setPosition(pos);
    }else{
      C["ball"]->setPosition(C["HandStick"]->getPosition());
    }
  }
}

//===========================================================================

void testPnp() {
  rai::Configuration C;
  C.addFile(rai::raiPath("../rai-robotModels/scenarios/pandasTable-calibrated.g"));

  SequenceControllerExperiment ex(C);

  C.addFrame("box", "table")
      ->setJoint(rai::JT_rigid)
      .setShape(rai::ST_ssBox, {.06,.15,.09,.01})
      .setRelativePosition(arr{-.0,-.0,.095});

  C.addFrame("target", "table")
      ->setJoint(rai::JT_rigid)
      .setShape(rai::ST_ssBox, {.4,.4,.1,.01})
      .setRelativePosition(arr{-.4,.2,.0});

  //-- define constraints
  ObjectiveL phi;
  addBoxPickObjectives(phi, C, 1., rai::_xAxis, "box", "l_gripper", "l_palm", "target");

  while(ex.step(phi));
}

//===========================================================================

void testPushing() {
  rai::Configuration C;
  C.addFile("pushScenario.g");


  //-- define constraints
  ObjectiveL phi;
  //phi.add({1.}, FS_distance, C, {"stickTip", "puck"}, OT_eq, {1e1});
  phi.add({1.}, make_shared<F_PushRadiusPrior>(.09), C, {"stickTip", "puck", "target"}, OT_eq, {1e1});
  phi.add({2.}, make_shared<F_PushRadiusPrior>(.06), C, {"stickTip", "puck", "target"}, OT_eq, {1e1});
  phi.add({1., 2.}, make_shared<F_PushAligned>(), C, {"stickTip", "puck", "target"}, OT_eq, {1e1});
  //phi.add({1., 2.}, FS_positionRel, C, {"ball", "l_gripper"}, OT_eq, {{2,3},{1e1,0,0,0,1e1,0}});
  //phi.add({2.}, FS_positionDiff, C, {"l_gripper", "ball"}, OT_eq, {1e1});

//  komo.addContact_slide(s.phase0, s.phase1, s.frames(0), s.frames(1));
//  if(s.phase1>=s.phase0+.8){
//    rai::Frame* obj = komo.world.getFrame(s.frames(1));
//    if(!(obj->shape && obj->shape->type()==ST_sphere) && obj->children.N){
//      obj = obj->children.last();
//    }
//    if(obj->shape && obj->shape->type()==ST_sphere){
//      double rad = obj->shape->radius();
//      arr times = {s.phase0+.2,s.phase1-.2};
//      if(komo.k_order==1) times = {s.phase0, s.phase1};
//      komo.addObjective(times, make_shared<F_PushRadiusPrior>(rad), s.frames, OT_sos, {1e1}, NoArr, 1, +1, 0);
//    }
//  }
//  if(komo.k_order>1){
//    komo.addObjective({s.phase0, s.phase1}, FS_position, {s.frames(1)}, OT_sos, {3e0}, {}, 2); //smooth obj motion
//    komo.addObjective({s.phase1}, FS_pose, {s.frames(0)}, OT_eq, {1e0}, {}, 1);
//    komo.addObjective({s.phase1}, FS_pose, {s.frames(1)}, OT_eq, {1e0}, {}, 1);
//  }


  SequenceControllerExperiment ex(C);
  while(ex.step(phi));
}

//===========================================================================

int main(int argc, char * argv[]){
  rai::initCmdLine(argc, argv);

  //  rnd.clockSeed();
  rnd.seed(1);

  //testBallFollowing();
  //testPnp();
  testPushing();

  LOG(0) <<" === bye bye ===\n used parameters:\n" <<rai::getParameters()() <<'\n';

  return 0;
}
