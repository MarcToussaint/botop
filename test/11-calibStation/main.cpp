#include <Kin/kin.h>
#include <Kin/frame.h>

#include <KOMO/komo.h>

/*
 *
 * DELAY: Bug during data recording: take q from C, not from bot!
 */

//===========================================================================

void load(){
  rai::Configuration C;
  C.addFile(rai::raiPath("../rai-robotModels/scenarios/pandasTable.g"));
  C.watch();

  //-- load data from
  ifstream fil("21-09-14.dat");
  arr _q, _poseL, _poseR, _poseTable;
  arr q(0,14), poseL(0,7), poseR(0,7), poseTable(0,7), times;
  for(uint t=0;;t++){
    rai::skip(fil);
    if(!fil.good()) break;
    fil >>times.append() >>PARSE("q") >>_q >>PARSE("poseL") >>_poseL >>PARSE("poseR") >>_poseR;// >>PARSE("poseTable") >>_poseTable;
    q.append(_q);
    poseR.append(_poseR);
    poseL.append(_poseL);
//    poseTable.append(_poseTable);
    //C.setJointState(_q);  C.watch();  rai::wait(.01);
  }
  fil.close();

  int ot_delay = 1; //in steps of 10msec!
  q.delRows(-ot_delay, ot_delay);
  poseL.delRows(0, ot_delay);
  poseR.delRows(0, ot_delay);
  uint T = poseL.d0;

  //-- get/create relevant frame for markers and ot signals
  rai::Frame *calL = C["l_robotiq_optitrackMarker"];
  rai::Frame *calR = C["r_robotiq_optitrackMarker"];
  rai::Frame *ot = & C.addFrame("optitrack_base", "world")->setShape(rai::ST_marker, {.1});
  rai::Frame *otL = & C.addFrame("otL", "optitrack_base")->setShape(rai::ST_marker, {.1});
  rai::Frame *otR = & C.addFrame("otR", "optitrack_base")->setShape(rai::ST_marker, {.1});
  rai::Frame *otT = & C.addFrame("otT", "optitrack_base")->setShape(rai::ST_marker, {.1});

  //-- create "morpho joints" - constant joints that are optimized/calibrated
  C["optitrack_base"] ->setJoint(rai::JT_free) .addAttribute("constant", 1.);
//  C["l_panda_base"] ->setJoint(rai::JT_transXYPhi) .addAttribute("constant", 1.);
  C["r_panda_base"] ->setJoint(rai::JT_transXYPhi) .addAttribute("constant", 1.);
  C["l_robotiq_optitrackMarker"] ->setJoint(rai::JT_trans3) .addAttribute("constant", 1.);
  C["r_robotiq_optitrackMarker"] ->setJoint(rai::JT_trans3) .addAttribute("constant", 1.);

  //-- create komo
  KOMO komo;
  komo.setModel(C, false);
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
//    komo.timeSlices(komo.k_order+t, otT->ID)->setPosition(pose(t,{0,2}));
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
  komo.addObjective({}, FS_positionDiff, {"otL", "l_robotiq_optitrackMarker"}, OT_sos);
  komo.addObjective({}, FS_positionDiff, {"otR", "r_robotiq_optitrackMarker"}, OT_sos);
  //komo.addObjective({}, FS_positionDiff, {"otL", "l_calibMarker"}, OT_sos);
  //komo.addObjective({}, FS_qItself, {"l_panda_base"}, OT_eq, {1e1}, {-.4, -.3,0.5*RAI_PI});

  //-- optimize
  komo.opt.verbose=6;
  komo.optimize();

  //-- report
  cout <<komo.getReport(true) <<endl;
  cout <<"\nx: " <<komo.x <<endl <<endl;
  cout <<*komo.pathConfig["optitrack_base"] <<endl;
  cout <<*komo.pathConfig["l_panda_base"] <<endl;
  cout <<*komo.pathConfig["r_panda_base"] <<endl;
  cout <<*komo.pathConfig["l_robotiq_optitrackMarker"] <<endl;
  cout <<*komo.pathConfig["r_robotiq_optitrackMarker"] <<endl;

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

int main(int argc, char * argv[]){
  rai::initCmdLine(argc, argv);

  rnd.clockSeed();

  load();

  return 0;
}
