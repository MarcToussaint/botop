#include <Kin/kin.h>
#include <Kin/frame.h>
#include <KOMO/komo.h>
#include <KOMO/pathTools.h>

#include <BotOp/bot.h>
#include <BotOp/motionHelpers.h>

#include <Optim/NLP_Solver.h>
//===========================================================================

void collectData(){
  //-- setup a configuration
  rai::Configuration C;
  C.addFile("station.g");
  C.view(true);

  rai::Frame* cam = C["cameraWrist"];
  rai::Frame* mount = C["l_panda_joint7"];

  BotOp bot(C, false);

  rai::Graph data;

  uint nDots=4;
  for(uint d=0;d<nDots;d++){
    for(uint k=0;k<5; k++){
      rai::Frame* dot = C[STRING("dot" <<d)];

      KOMO komo;
      komo.setConfig(C, false);
      komo.setTiming(1., 1, 1., 0);
      komo.addControlObjective({}, 0, 1e-1);

      arr dotPos = .2*(rand(3)-.5);
      dotPos(2) -= .35;
      komo.addObjective({}, FS_positionRel, {dot->name, cam->name}, OT_eq, {1e0}, dotPos);

      double wristAngle = rnd.uni(-2.3, 2.3);
      cout <<dotPos <<' ' <<wristAngle <<endl;
      komo.addObjective({}, FS_qItself, {mount->name}, OT_eq, {1e0}, {wristAngle});

      auto ret = NLP_Solver().setProblem(komo.nlp()).solve();
      //komo.view(true);

      bot.move(komo.getPath_qOrg(), {1.});
      for(;bot.getTimeToEnd()>0.;) bot.sync(C);

      rai::Graph& dat = data.addSubgraph(dot->name+k);
      byteA img;
      floatA depth;
      bot.getImageAndDepth(img, depth, cam->name);
      dat.add("mountPose", mount->getPose());
      dat.add("camPose", cam->getPose());
      dat.add("dotPosition", cam->getPosition());
      dat.add("img", img);
      dat.add("depth", depth);
    }
  }

  data.write(FILE("dat.g"), "\n", 0, -1, false, true);
}

//===========================================================================

int main(int argc, char * argv[]){
  rai::initCmdLine(argc, argv);

  collectData();

//  computeCalibration();

//  demoCalibration();

  LOG(0) <<" === bye bye ===\n used parameters:\n" <<rai::params() <<'\n';

  return 0;
}
