#include <Kin/kin.h>
#include <Kin/frame.h>
#include <KOMO/komo.h>
#include <KOMO/pathTools.h>

#include <BotOp/bot.h>
#include <BotOp/motionHelpers.h>
#include <OptiTrack/optitrack.h>

//===========================================================================

void collectData(){
  //-- setup a configuration
  rai::Configuration C;
  C.addFile(rai::raiPath("../rai-robotModels/scenarios/pandasTable.g"));

  arr points=rai::getParameter<arr>("points");
  points.reshape(-1,2,3);

  //-- start a robot thread
  BotOp bot(C, rai::getParameter<bool>("real", false));
  bot.home(C);

  //-- prepare storing optitrack data ?
  ofstream fil("z.calib.dat");
  rai::Frame *optiFrameR=0, *optiFrameL = 0;
  if(bot.optitrack){
    rai::wait(1.);
    optiFrameR = C["ot_r_panda_gripper"];
    optiFrameL = C["ot_l_panda_gripper"];
  }

  arr q_last=bot.get_q();
  uint L = points.d0;
  for(uint l=0;l<=L;l++){
    arr q_target;

    //compute pose
    if(l<L){
      KOMO komo;
      komo.setConfig(C, true);
      komo.setTiming(1, 1, 3., 1);
      komo.add_qControlObjective({}, 1, 1e-1);
      komo.addObjective({}, FS_qItself, {}, OT_sos, {.1}, bot.qHome);
      komo.addObjective({}, FS_accumulatedCollisions, {}, OT_eq, {1e1});

      arr pt = points(l,0,{});
      komo.addObjective({}, FS_positionDiff, {"r_gripper", "table_base"}, OT_eq, {1e2}, pt);
      pt(0) *= -1.;
      komo.addObjective({}, FS_positionDiff, {"l_gripper", "table_base"}, OT_eq, {1e2}, pt);

      arr dir = points(l,1,{});
      if(length(dir)>.5){
        dir /= length(dir);
        komo.addObjective({}, FS_vectorY, {"r_gripper"}, OT_eq, {1e2}, dir);
        dir(0) *= -1.;
        komo.addObjective({}, FS_vectorY, {"l_gripper"}, OT_eq, {1e2}, dir);
      }

      komo.optimize();

      //is feasible?
      bool feasible=komo.sos<50. && komo.ineq<.1 && komo.eq<.1;

      if(!feasible){
        cout <<" === pose infeasible ===\n" <<points[l] <<endl;
        continue;
      }

      q_target = komo.x;
      cout <<" === pose feasible  === " <<endl;
    }else{
      q_target = bot.qHome;
    }

    //C.setJointState(q_target);  C.view(true, "pose");

    //compute path
    C.setJointState(q_last);
    arr path = getStartGoalPath(C, q_target, bot.qHome);
    if(!path.N){
      cout <<" === path infeasible === " <<endl;
      continue;
    }
    q_last = q_target;
    cout <<" === path feasible  === " <<endl;

    cout <<" === -> executing === " <<endl;
    bot.moveAutoTimed(path, .5, .5);

    if(bot.optitrack){
      Metronome tic(.01);
      bot.sync(C, -1.);
      while(bot.sync(C, -1.)){
        tic.waitForTic();
        arr q = bot.get_q();
        bot.optitrack->pull(C);
        fil <<rai::realTime() <<" q " <<q <<" poseL " <<optiFrameL->ensure_X() <<" poseR " <<optiFrameR->ensure_X() <<endl; // <<" poseTable " <<optiTable->ensure_X() <<endl;
      }
    }else{
      while(bot.sync(C));
    }
    if(bot.keypressed=='q' || bot.keypressed==27) break;
  }

  fil.close();
}

//===========================================================================

void computeCalibration(){
  rai::Configuration C;
  C.addFile(rai::raiPath("../rai-robotModels/scenarios/pandasTable.g"));
  C.addFrame("optitrack_base", "world");
  C.view();

  //-- load data from
  ifstream fil("z.calib.dat");
  arr _q, _poseL, _poseR;
  arr q(0,14), poseL(0,7), poseR(0,7), times;
  for(uint t=0;;t++){
    rai::skip(fil);
    if(!fil.good()) break;
    fil >>times.append() >>PARSE("q") >>_q >>PARSE("poseL") >>_poseL >>PARSE("poseR") >>_poseR;
    if(fabs(_poseL(0))>1e-10 && fabs(_poseR(0))>1e-10){ //ignore data is pose is zero!
      q.append(_q);
      poseR.append(_poseR);
      poseL.append(_poseL);
//      C.setJointState(_q);  C.view();  rai::wait(.01);
    }else{
      LOG(0) <<"skipping line " <<t;
    }
  }
  fil.close();

  int ot_delay = 2; //in steps of 10msec!
  q.delRows(-ot_delay, ot_delay);
  poseL.delRows(0, ot_delay);
  poseR.delRows(0, ot_delay);
  uint T = poseL.d0;

  //-- get/create relevant frame for markers and ot signals
  rai::Frame *calL = C["l_robotiq_optitrackMarker"];
  rai::Frame *calR = C["r_robotiq_optitrackMarker"];
  //rai::Frame *ot = & C.addFrame("optitrack_base", "world")->setShape(rai::ST_marker, {.1});
  rai::Frame *otL = & C.addFrame("otL", "optitrack_base")->setShape(rai::ST_marker, {.1});
  rai::Frame *otR = & C.addFrame("otR", "optitrack_base")->setShape(rai::ST_marker, {.1});

  //-- create "morpho joints" - constant joints that are optimized/calibrated
  C["optitrack_base"] ->setJoint(rai::JT_free) .addAttribute("constant", 1.);
//  C["l_panda_base"] ->setJoint(rai::JT_transXYPhi) .addAttribute("constant", 1.);
  C["r_panda_base"] ->setJoint(rai::JT_transXYPhi) .addAttribute("constant", 1.);
  C["l_robotiq_optitrackMarker"] ->setJoint(rai::JT_trans3) .addAttribute("constant", 1.);
  C["r_robotiq_optitrackMarker"] ->setJoint(rai::JT_trans3) .addAttribute("constant", 1.);

  //-- create komo
  KOMO komo;
  komo.setConfig(C, false);
  komo.setTiming(1., T, 1., 1);
  komo.setupPathConfig();

  //-- set the joint path with loaded q -- using set_x we first need to deactivate the morpho joints
  komo.pathConfig["optitrack_base"]->joint->setActive(false);
//  komo.pathConfig["l_panda_base"]->joint->setActive(false);
  komo.pathConfig["r_panda_base"]->joint->setActive(false);
  komo.pathConfig["l_robotiq_optitrackMarker"]->joint->setActive(false);
  komo.pathConfig["r_robotiq_optitrackMarker"]->joint->setActive(false);

  komo.set_x(q);
  //komo.view(true, "KOMO with initialized joint path");

  //-- set the position path for loaded optitrack signals
  for(uint t=0;t<komo.T;t++){
    komo.timeSlices(komo.k_order+t, otL->ID)->setPosition(poseL(t,{0,2}));
    komo.timeSlices(komo.k_order+t, otR->ID)->setPosition(poseR(t,{0,2}));
  }

  //-- now we deactivate all normal joints, and only activate the morpho joints
  komo.pathConfig.setActiveJoints({});
  komo.pathConfig["optitrack_base"]->joint->setActive(true);
//  komo.pathConfig["l_panda_base"]->joint->setActive(true);
  komo.pathConfig["r_panda_base"]->joint->setActive(true);
  komo.pathConfig["l_robotiq_optitrackMarker"]->joint->setActive(true);
  komo.pathConfig["r_robotiq_optitrackMarker"]->joint->setActive(true);

  //-- add objectives: just square errors in marker positions
  komo.addSquaredQuaternionNorms();
  komo.addObjective({}, FS_positionDiff, {"otL", "l_robotiq_optitrackMarker"}, OT_sos, {1e1});
  komo.addObjective({}, FS_positionDiff, {"otR", "r_robotiq_optitrackMarker"}, OT_sos, {1e1});
  //komo.addObjective({}, FS_qItself, {"l_panda_base"}, OT_eq, {1e1}, {-.4, -.3,0.5*RAI_PI});

  //-- optimize
  komo.optimize(0., rai::OptOptions()
                .set_verbose(6)
                .set_damping(1e-6)
                .set_stopTolerance(1e-6) );

  //-- report
  cout <<komo.getReport(true) <<endl;
  cout <<"\n CALIBRATION FILE:\n\n" <<endl;
  cout <<"Include 'pandasTable.g'" <<endl;
  cout <<"optitrack_base (world) { Q:" <<komo.pathConfig["optitrack_base"]->get_Q() <<" }" <<endl;
  cout <<"Edit l_panda_base { Q:<" <<komo.pathConfig["l_panda_base"]->get_Q() <<"> }" <<endl;
  cout <<"Edit r_panda_base { Q:<" <<komo.pathConfig["r_panda_base"]->get_Q() <<"> }" <<endl;
  cout <<"Edit l_robotiq_optitrackMarker { Q:<" <<komo.pathConfig["l_robotiq_optitrackMarker"]->get_Q() <<"> }" <<endl;
  cout <<"Edit r_robotiq_optitrackMarker { Q:<" <<komo.pathConfig["r_robotiq_optitrackMarker"]->get_Q() <<"> }" <<endl;
  cout <<endl;

  //-- for extra plotting, and RMSE
  {
    ofstream fil("z.dat");
    rai::arrayBrackets="  ";

    uint n=0;
    double err=0.;
    arr pos_ot, pos_bot;
    for(uint t=0;t<komo.T;t++){
      fil <<times(t) <<' ';

      pos_ot = komo.timeSlices(komo.k_order+t, otL->ID)->getPosition();
      pos_bot = komo.timeSlices(komo.k_order+t, calL->ID)->getPosition();
      err += sumOfSqr(pos_ot - pos_bot);
      n++;
      fil <<pos_ot <<' ' <<pos_bot <<' ';

      pos_ot = komo.timeSlices(komo.k_order+t, otR->ID)->getPosition();
      pos_bot = komo.timeSlices(komo.k_order+t, calR->ID)->getPosition();
      err += sumOfSqr(pos_ot - pos_bot);
      n++;
      fil <<pos_ot <<' ' <<pos_bot <<endl;
    }

    cout <<"sos: " <<err <<" RMSE: " <<std::sqrt(err/n) <<endl;
  }

  rai::wait();
}

//===========================================================================

void demoCalibration(){
  //-- setup a configuration
  rai::Configuration C;
  C.addFile(rai::raiPath("../rai-robotModels/scenarios/pandasTable-calibrated.g"));

  arr points=rai::getParameter<arr>("demoPoints");
  points.reshape(-1,3);

  //-- start a robot thread
  BotOp bot(C, rai::getParameter<bool>("real", false));
  bot.home(C);

  arr q_last=bot.get_q();
  uint L = points.d0;
  for(uint l=0;l<=L;l++){
    arr q_target;

    //compute pose
    if(l<L){
      KOMO komo;
      komo.setConfig(C, true);
      komo.setTiming(1, 1, 3., 1);
      komo.add_qControlObjective({}, 1, 1e-1);
      komo.addObjective({}, FS_qItself, {}, OT_sos, {.1}, bot.qHome);
//      komo.addObjective({}, FS_accumulatedCollisions, {}, OT_eq, {1e1});

      arr pt = points[l];
      komo.addObjective({}, FS_positionDiff, {"r_gripper", "table_base"}, OT_eq, {1e2}, pt);
      komo.addObjective({}, FS_positionDiff, {"r_gripper", "l_gripper"}, OT_eq, {1e2}, {.05, 0., 0.});

      komo.addObjective({}, FS_scalarProductXY, {"r_gripper", "l_gripper"}, OT_eq, {1e2});
      komo.addObjective({}, FS_vectorZ, {"r_gripper"}, OT_eq, {1e2}, {1.,0.,0.});
      komo.addObjective({}, FS_vectorZ, {"l_gripper"}, OT_eq, {1e2}, {-1.,0.,0.});

      komo.optimize();

      //is feasible?
      bool feasible=komo.sos<50. && komo.ineq<.1 && komo.eq<.1;

      if(!feasible){
        cout <<" === pose infeasible ===\n" <<points[l] <<endl;
        continue;
      }

      q_target = komo.x;
      cout <<" === pose feasible  === " <<endl;
    }else{
      q_target = bot.qHome;
    }

    //compute path
    C.setJointState(q_last);
    arr path = getStartGoalPath(C, q_target, bot.qHome);
    if(!path.N){
      cout <<" === path infeasible === " <<endl;
      continue;
    }
    q_last = q_target;
    cout <<" === path feasible  === " <<endl;

    cout <<" === -> executing === " <<endl;
    bot.moveAutoTimed(path, .5, .5);

    while(bot.sync(C));
    if(bot.keypressed=='q' || bot.keypressed==27) break;

    rai::wait();
  }
}

//===========================================================================

int main(int argc, char * argv[]){
  rai::initCmdLine(argc, argv);

//  collectData();

//  computeCalibration();

  demoCalibration();

  LOG(0) <<" === bye bye ===\n used parameters:\n" <<rai::params() <<'\n';

  return 0;
}
