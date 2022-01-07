#include <BotOp/bot.h>
#include <BotOp/SequenceController.h>
#include <KOMO/manipTools.h>


//===========================================================================

void testPnp() {
  rai::Configuration C;
  C.addFile(rai::raiPath("../rai-robotModels/scenarios/pandasTable.g"));

  rai::Frame& target =
  C.addFrame("box", "table")
      ->setJoint(rai::JT_rigid)
      .setShape(rai::ST_ssBox, {.06,.15,.09,.01})
      .setRelativePosition(arr{-.0,-.0,.095});

  C.addFrame("target", "table")
      ->setJoint(rai::JT_rigid)
      .setShape(rai::ST_ssBox, {.4,.4,.1,.01})
      .setRelativePosition(arr{-.4,.2,.0});

  arr qHome = C.getJointState();

  //-- define constraints
  ObjectiveL phi;
  addBoxPickObjectives(phi, C, 1., rai::_xAxis, "box", "l_gripper", "l_palm", "target");

  //-- Sequence of Constraints MPC
  SequenceController ctrl(C, phi, qHome);

  //-- start a robot thread
  BotOp bot(C, rai::checkParameter<bool>("real"));
  bot.home(C);
  double ctrlTime = 0.;
  //bot.setControllerWriteData(2);

  //-- iterate
  Metronome tic(.1);
  for(uint t=0;t<2000;t++){
    tic.waitForTic();

    //-- get current state (time,q,qDot)
    arr q,qDot;
    bot.getState(q, qDot, ctrlTime);
    ctrl.cycle(C, q, qDot, ctrlTime);
    ctrl.report(C, phi);

    //send leap target
    auto sp = ctrl.getSpline(bot.get_t());
    if(sp.pts.N) bot.move(sp.pts, sp.vels, sp.times, true);

    //update C
    bot.step(C, .0);
    if(bot.keypressed=='q' || bot.keypressed==27) return;
  }
}

//===========================================================================

int main(int argc, char * argv[]){
  rai::initCmdLine(argc, argv);

//  rnd.clockSeed();
  rnd.seed(1);

  testPnp();

  LOG(0) <<" === bye bye ===\n used parameters:\n" <<rai::getParameters()() <<'\n';

  return 0;
}
