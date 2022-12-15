#include <BotOp/bot.h>
#include <BotOp/SecMPC.h>
#include <KOMO/manipTools.h>
#include <KOMO/pathTools.h>
#include <Kin/viewer.h>

#include <Kin/F_forces.h>

//===========================================================================

struct SecMPC_Experiments{
  rai::Configuration& C;
  unique_ptr<SecMPC> mpc;
  unique_ptr<BotOp> bot;
  Metronome tic;
  uint stepCount = 0;

  KOMO *__komo=0;
  double timeCost=1e0, ctrlCost=1e0;

  SecMPC_Experiments(rai::Configuration& _C, ObjectiveL& phi, double cycleTime=.1)
    : C(_C),
      tic(cycleTime)
      {
  }

  SecMPC_Experiments(rai::Configuration& _C, KOMO& komo, double cycleTime=.1, double timeCost=1e1, double ctrlCost=1e-2)
    : C(_C),
      tic(cycleTime),
      __komo(&komo),
      timeCost(timeCost), ctrlCost(ctrlCost){
  }

  bool step(ObjectiveL& phi){
    stepCount++;

    //-- start a robot thread
    if(!bot){
      bot = make_unique<BotOp>(C, rai::checkParameter<bool>("real"));
      bot->home(C);
      bot->setControllerWriteData(1);
      if(bot->optitrack) bot->optitrack->pull(C);
      rai::wait(.2);
    }

    if(!mpc){
      //needs to be done AFTER bot initialization (optitrack..)
      if(__komo){
        mpc = make_unique<SecMPC>(C, *__komo, C.getJointState(), timeCost, ctrlCost);
      }else{
        mpc = make_unique<SecMPC>(C, phi, C.getJointState(), timeCost, ctrlCost);
      }
    }

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
    mpc->cycle(C, phi, q_ref, qDot_ref, q, qDot, ctrlTime);
    mpc->report(C, phi);

    //-- send spline update
    auto sp = mpc->getSpline(bot->get_t());
    if(sp.pts.N) bot->move(sp.pts, sp.vels, sp.times, true);

    //-- update C
    bot->step(C, .0);
    if(bot->keypressed=='q' || bot->keypressed==27) return false;

    return true;
  }
};

//===========================================================================

void randomWalkPosition(rai::Frame* f, arr& centerPos, arr& velocity, double rate=.001){
  arr pos = f->getPosition();
  if(!centerPos.N) centerPos = pos;
  if(!velocity.N) velocity = zeros(pos.N);
  rndGauss(velocity, rate, true);
  pos += velocity;
  pos = centerPos + .9 * (pos - centerPos);
  f->setPosition(pos);
}

//===========================================================================

void testBallFollowing() {
  rai::Configuration C;
  C.addFile(rai::raiPath("../rai-robotModels/scenarios/pandasTable-calibrated.g"));

  C.addFrame("ball", "table")
      ->setShape(rai::ST_sphere, {.03})
      .setColor({1.,0,0})
      .setRelativePosition(arr{-.4,.4,.4});

  KOMO komo;
  komo.setModel(C, false);
  komo.setTiming(1., 1, 5., 1);
  komo.add_qControlObjective({}, 1, 1e-1);
  komo.addQuaternionNorms();

  komo.addObjective({1.}, FS_positionDiff, {"l_gripper", "ball"}, OT_eq, {1e1});

#if 0 //only for development
  komo.optimize();
  komo.getReport(true);
  komo.view(true, "optimized motion");
  while(komo.view_play(true));
#endif

  SecMPC_Experiments ex(C, komo, .02, 1e0, 1e0);

  bool useSimulatedBall=!rai::getParameter<bool>("bot/useOptitrack", false);
  arr ballVel, ballCen;

  while(ex.step(komo.objectives)){
    if(useSimulatedBall){
      randomWalkPosition(C["ball"], ballCen, ballVel, .001);
    }else{
      C["ball"]->setPosition(C["HandStick"]->getPosition());
    }
  }
}

//===========================================================================

void testBallReaching() {
  rai::Configuration C;
  C.addFile(rai::raiPath("../rai-robotModels/scenarios/pandasTable-calibrated.g"));

  C.addFrame("ball", "table")
      ->setShape(rai::ST_sphere, {.03})
      .setColor({1.,0,0})
      .setRelativePosition(arr{-.4,.4,.4});

  //-- define constraints
  KOMO komo;
  komo.setModel(C, false);
  komo.setTiming(2., 1, 5., 1);
  komo.add_qControlObjective({}, 1, 1e-1);
  komo.addQuaternionNorms();
  komo.addObjective({1.}, FS_positionRel, {"ball", "l_gripper"}, OT_eq, {6e1}, {0., 0., -.1});
  komo.addObjective({1., 2.}, FS_positionRel, {"ball", "l_gripper"}, OT_eq, {{2,3},{1e1,0,0,0,1e1,0}});
  komo.addObjective({2.}, FS_positionDiff, {"l_gripper", "ball"}, OT_eq, {6e1});

  SecMPC_Experiments ex(C, komo, .1, 1e0, 1e0);
  ex.step(komo.objectives);
  ex.mpc->timingMPC.backtrackingTable=uintA{0, 0};

  bool useSimulatedBall=!rai::getParameter<bool>("bot/useOptitrack", false);
  arr ballVel = {.0, .0, .0};
  arr ballCen = C["ball"]->getPosition();

  while(ex.step(komo.objectives)){
    if(useSimulatedBall){
      if(!(ex.stepCount%20)){
        ballVel(0) = .01 * rnd.gauss();
        ballVel(2) = .01 * rnd.gauss();
      }
      if(!(ex.stepCount%40)) ballVel=0.;
      randomWalkPosition(C["ball"], ballCen, ballVel);
    }else{
      C["ball"]->setPosition(C["HandStick"]->getPosition());
    }
  }
}

//===========================================================================

void testPnp() {
  rai::Configuration C;
  C.addFile("graspScenario.g");
  arr qHome = C.getJointState();

  KOMO komo;
  komo.setModel(C, true);
  komo.setTiming(3, 1, 3., 1);
  komo.add_qControlObjective({}, 1, 1e-1);

  // homing
  if(qHome.N) komo.addObjective({}, FS_qItself, {}, OT_sos, {.1}, qHome);

  komo.addObjective({}, FS_accumulatedCollisions, {}, OT_eq, {1e1});

  const char* gripperName="l_gripper";
  const char* palmName="l_palm";
  const char* boxName="box";
  const char* targetName="target";
  //const char* arm1Name="l_panda_coll7";
  //const char* arm2Name="l_panda_coll6";
  arr boxSize={.06,.15,.09};
  rai::Enum<rai::ArgWord> pickDirection = rai::_xAxis;
  rai::Enum<rai::ArgWord> placeDirection = rai::_zAxis;

  //-- pick
  addBoxPickObjectives(komo, 1., pickDirection, boxName, boxSize, gripperName, palmName, targetName, true);
  komo.addModeSwitch({2.,3.}, rai::SY_stable, {gripperName, boxName}, true);
  addBoxPickObjectives(komo, 2., pickDirection, boxName, boxSize, gripperName, palmName, targetName);

  //-- place
  komo.addModeSwitch({3.,-1.}, rai::SY_stable, {"table", boxName}, false);
  addBoxPlaceObjectives(komo, 3., placeDirection, boxName, boxSize, targetName, gripperName, palmName);

#if 1 //only for development
  komo.optimize();
  cout <<komo.getReport(true);
  komo.view(true, "optimized motion");
  while(komo.view_play(true));
#endif

  SecMPC_Experiments ex(C, komo, .1, 1e0, 1e0);
  while(ex.step(komo.objectives));
}

//===========================================================================

void testPushing() {
  rai::Configuration C;
  C.addFile("pushScenario.g");


  //-- define constraints
  ObjectiveL phi;
  phi.add({1.}, make_shared<F_PushRadiusPrior>(.13), C, {"stickTip", "puck", "target"}, OT_eq, {1e1}, {0., 0., .1});
  phi.add({2.}, make_shared<F_PushRadiusPrior>(.10), C, {"stickTip", "puck", "target"}, OT_eq, {1e1});
  phi.add({3.}, make_shared<F_PushRadiusPrior>(.02), C, {"stickTip", "puck", "target"}, OT_eq, {1e1});
  phi.add({1., 3.}, make_shared<F_PushAligned>(), C, {"stickTip", "puck", "target"}, OT_eq, {{1,3},{0,0,1e1}});
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

  bool useOptitrack=rai::getParameter<bool>("bot/useOptitrack", false);

  SecMPC_Experiments ex(C, phi);
  ex.step(phi);
  ex.mpc->timingMPC.backtrackingTable=uintA{0, 0, 0, 0, 0};

  while(ex.step(phi)){
    if(useOptitrack){
      C["puck"]->setPosition(C["b1"]->getPosition());
    }
  }
}

//===========================================================================

void testDroneRace(){
  rai::Configuration C;
  C.addFile("droneRace.g");

  //-- define constraints
  KOMO komo;
  komo.setModel(C, false);
  komo.setTiming(7., 1, 5., 1);
  komo.add_qControlObjective({}, 1, 1e-1);
  komo.addQuaternionNorms();
  komo.addObjective({1.}, FS_positionDiff, {"drone", "target0"}, OT_eq, {1e1});
  komo.addObjective({2.}, FS_positionDiff, {"drone", "target1"}, OT_eq, {1e1});
  komo.addObjective({3.}, FS_positionDiff, {"drone", "target2"}, OT_eq, {1e1});
  komo.addObjective({4.}, FS_positionDiff, {"drone", "target3"}, OT_eq, {1e1});
  komo.addObjective({5.}, FS_positionDiff, {"drone", "target0"}, OT_eq, {1e1});
  komo.addObjective({6.}, FS_positionDiff, {"drone", "target1"}, OT_eq, {1e1});
  komo.addObjective({7.}, FS_position, {"drone"}, OT_eq, {1e1}, {0,-.5, 1.});


  arrA targetCen(4), targetVel(4);

#if 1
  //-- reactive control
  SecMPC_Experiments ex(C, komo, .1, 1e0, 1e0);
  ex.step(komo.objectives);
  ex.mpc->tauCutoff = .1;

  //void CubicSplineCtrlReference::getReference(arr& q_ref, arr& qDot_ref, arr& qDDot_ref, const arr& q_real, const arr& qDot_real, double ctrlTime){

  while(ex.step(komo.objectives)){
    if(ex.mpc->timingMPC.phase==5){ //hard code endless loop by phase backtracking
      ex.mpc->timingMPC.update_setPhase(1);
    }
    for(uint g=0;g<2;g++){
      rai::Frame *target = C[STRING("target"<<g)];
      randomWalkPosition(target, targetCen(g), targetVel(g), .003);
    }
  }

#else
  //-- manually just optimize once and dump spline
  //optimize keyframes
  komo.optimize();
  komo.getReport(true);
  komo.view(false, "optimized motion");
  arr keyframes = komo.getPath_qOrg();

  //optimize timing
  TimingMPC F(keyframes, 1e0, 10); //last number (ctrlCost) in range [1,10] from fast-slow
  arr x0 = C["drone"]->getPosition();
  arr v0 = zeros(3);
  F.solve(x0, v0);

  //get spline
  rai::CubicSpline S;
  F.getCubicSpline(S, x0, v0);

  //analyze only to plot the max vel/acc
  arr path = S.eval(range(0., S.times.last(), 100));
  double tau = S.times.last()/100.;
  arr ttau = consts<double>(tau, 101);
  double maxVel=1., maxAcc=1., maxJer=30.;
  arr time(path.d0);
  time.setZero();
  for(uint t=1;t<time.N;t++) time(t) = time(t-1) + ttau(t);

  arr v = max(getVel(path,ttau),1) / maxVel;
  arr a = max(getAcc(path,ttau),1) / maxAcc;
  arr j = max(getJerk(path,ttau),1) / maxJer;
  arr vi = min(getVel(path,ttau),1) / maxVel;
  arr ai = min(getAcc(path,ttau),1) / maxAcc;
  arr ji = min(getJerk(path,ttau),1) / maxJer;
  catCol(LIST(~~time, ~~v, ~~a, ~~j, ~~vi, ~~ai, ~~ji)).reshape(-1,7).writeRaw( FILE("z.dat") );
  gnuplot("plot [:][-1.1:1.1] 'z.dat' us 1:2 t 'vmax', '' us 1:3 t 'amax', '' us 1:5 t 'vmin', '' us 1:6 t 'amin'"); //, '' us 1:4 t 'j', , '' us 1:7 t 'jmin'

  //display
  rai::Mesh M;
  M.V = S.eval(range(0., S.times.last(), 100));
  M.makeLineStrip();
  C.gl()->add(M);
  C.view(true);


  //just sample & dump the spline
  for(double t=0;t<S.times.last();t += .01){
    //time 3-positions 3-velocities
    cout <<t <<S.eval(t).modRaw() <<' ' <<S.eval(t,1).modRaw() <<endl;
  }
#endif

}

//===========================================================================

int main(int argc, char *argv[]){
  rai::initCmdLine(argc, argv);

  //  rnd.clockSeed();
  rnd.seed(1);

  //testBallFollowing();
  //testBallReaching();
  testPnp();
  //testPushing();
  //testDroneRace();


  LOG(0) <<" === bye bye ===\n used parameters:\n" <<rai::getParameters()() <<'\n';

  return 0;
}
