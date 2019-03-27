#include <Gui/opengl.h>
#include <LGP/optLGP.h>

//======================================================================

void optSkeleton(const Skeleton& S, const rai::KinematicWorld& K){
  double maxPhase = 1.;
  for(const SkeletonEntry& s:S){
    if(s.phase0 > maxPhase) maxPhase = s.phase0;
    if(s.phase1 > maxPhase) maxPhase = s.phase1;
  }
  maxPhase += .5;

  KOMO komo;
  komo.setModel(K, true);
  komo.setPathOpt(maxPhase, 10., 2.);
  komo.setCollisions(false);
  komo.deactivateCollisions("table1", "iiwa_link_0_0");
  komo.activateCollisions("table1", "stick");
  komo.activateCollisions("table1", "stickTip");

  komo.setSkeleton(S);

  komo.reset();
  //  komo.reportProblem();
  komo.animateOptimization = 0;
  komo.run();
  //   komo.checkGradients();

  cout <<komo.getReport(true) <<endl;
  komo.reportEffectiveJoints();
//  komo.reportProxies();
  while(komo.displayTrajectory(.1));
}

//======================================================================

void test(){
  rai::KinematicWorld K("../model/LGP-kuka.g");
  K.optimizeTree();

  Skeleton S = {
    { {"grasp", "endeff", "stick"}, 1., 1.},
    { {"grasp", "stickTip", "redBall"}, 2., 2.},
  };

  optSkeleton(S, K);
}

//======================================================================

int main(int argc,char **argv){
  rai::initCmdLine(argc, argv);
  rnd.clockSeed();

  test();

  return 0;
}
