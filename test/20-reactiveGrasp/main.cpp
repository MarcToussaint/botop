#include <BotOp/bot.h>
#include <BotOp/SequenceController.h>
#include <KOMO/manipTools.h>

//===========================================================================

struct SequenceControllerExperiment{
  rai::Configuration C;
  arr qHome;
  unique_ptr<SequenceController> ctrl;
  unique_ptr<BotOp> bot;
  Metronome tic;
  uint stepCount = 0;

  SequenceControllerExperiment() : tic(.1){
    C.addFile(rai::raiPath("../rai-robotModels/scenarios/pandasTable.g"));
    qHome = C.getJointState();
  }

  bool step(ObjectiveL& phi){
    stepCount++;

    //-- Sequence of Constraints MPC
    if(!ctrl)
      ctrl = make_unique<SequenceController>(C, phi, qHome);

    //-- start a robot thread
    if(!bot){
      bot = make_unique<BotOp>(C, rai::checkParameter<bool>("real"));
      bot->home(C);
      rai::wait(.2);
    }
    //bot->setControllerWriteData(2);

    //-- iterate
    tic.waitForTic();

    //-- get current state (time,q,qDot)
    arr q,qDot;
    double ctrlTime = 0.;
    bot->getState(q, qDot, ctrlTime);

    //-- iterate MPC
    ctrl->cycle(C, phi, q, qDot, ctrlTime);
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
  SequenceControllerExperiment ex;

  ex.C.addFrame("ball", "table")
      ->setShape(rai::ST_sphere, {.03})
      .setColor({1.,0,0})
      .setRelativePosition(arr{-.4,.4,.4});

  //-- define constraints
  ObjectiveL phi;
  phi.add({1.}, FS_positionRel, ex.C, {"ball", "l_gripper"}, OT_eq, {1e1}, {0., 0., -.1});
  phi.add({1., 2.}, FS_positionRel, ex.C, {"ball", "l_gripper"}, OT_eq, {{2,3},{1e1,0,0,0,1e1,0}});
  phi.add({2.}, FS_positionDiff, ex.C, {"l_gripper", "ball"}, OT_eq, {1e1});

  arr ballVel = {.0, .0, .0};
  arr ballCen = ex.C["ball"]->getPosition();
  while(ex.step(phi)){
    if(!(ex.stepCount%20)){
      ballVel(0) = .01 * rnd.gauss();
      ballVel(2) = .01 * rnd.gauss();
    }
    if(!(ex.stepCount%40)) ballVel=0.;
    arr pos = ex.C["ball"]->getPosition();
    pos += ballVel;
    pos = ballCen + .95 * (pos - ballCen);
    ex.C["ball"]->setPosition(pos);
  }
}



//===========================================================================

void testPnp() {
  SequenceControllerExperiment ex;

  ex.C.addFrame("box", "table")
      ->setJoint(rai::JT_rigid)
      .setShape(rai::ST_ssBox, {.06,.15,.09,.01})
      .setRelativePosition(arr{-.0,-.0,.095});

  ex.C.addFrame("target", "table")
      ->setJoint(rai::JT_rigid)
      .setShape(rai::ST_ssBox, {.4,.4,.1,.01})
      .setRelativePosition(arr{-.4,.2,.0});

  //-- define constraints
  ObjectiveL phi;
  addBoxPickObjectives(phi, ex.C, 1., rai::_xAxis, "box", "l_gripper", "l_palm", "target");

  while(ex.step(phi));
}

//===========================================================================

int main(int argc, char * argv[]){
  rai::initCmdLine(argc, argv);

  //  rnd.clockSeed();
  rnd.seed(1);

  testBallFollowing();
  //testPnp();

  LOG(0) <<" === bye bye ===\n used parameters:\n" <<rai::getParameters()() <<'\n';

  return 0;
}
