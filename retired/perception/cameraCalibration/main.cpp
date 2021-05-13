#include <opencv2/opencv.hpp>
#include <Perception/opencv.h>
#include <Kin/kin.h>


#include <KOMO/komo.h>
#include <KOMO/komo-ext.h>

#include <Kin/TM_qItself.h>
#include <Kin/TM_InsideBox.h>
#include <Kin/TM_default.h>
#include <Kin/TM_proxy.h>

#include <LGP/LGP_tree.h>


#include <Kin/cameraview.h>
#include <Gui/viewer.h>

#include "cameraCalibration.h"

#include <Kin/kin.h>

#include <KOMO/komo.h>
#include <KOMO/komo-ext.h>

#include <Operate/path.h>

#include <Perception/depth2PointCloud.h>

#include <Kin/cameraview.h>
#include <Gui/viewer.h>


#include <LGPop/lgpop.h>

//===========================================================================


void pandaModel() {
  rai::KinematicWorld K("../../model/pandaStation/pandaStation.g");
//  K.orsDrawVisualsOnly = true;
//  rai::KinematicWorld K("z.g");
//  K.stepSwift();
//  arr y;
//  K.kinematicsProxyCost(y, NoArr);
//  K.orsDrawProxies;
  arr q;
//  K.getJointState(q);
//  K.setJointState(q);
  K.watch(true);
}


void tuneHSV() {
  LGPop OP(LGPop::RealMode);

  OP.runCamera(1);


  rai::KinematicWorld K;
  K.addFile("world.g");

//  arr hsvFilter = ARR(100, 100, 70, 180, 255, 255).reshape(2, 3);
//  arr hsvFilter = ARR(60, 100, 50, 80, 255, 255).reshape(2, 3);
  arr hsvFilter = ARR(60, 101, 100, 82, 255, 255).reshape(2, 3);

  CameraCalibration cc(K, "calibrationVolumeR", "calibrationMarkerR");
  cc.hsvFilter = hsvFilter;
//  CameraCalibrationHSVGui hsvGui;


  while(true) {
//    cc.hsvFilter = hsvGui.getHSV();
    cc.extractPixelCoordinate(OP.cam_color.get(), OP.cam_depth.get(), cc.hsvFilter, false);
//    rai::wait(0.1);
    cv::waitKey(30);
  }

}



void test() {
  LGPop OP(LGPop::RealMode);

  OP.runCamera(1);

  OP.runRobotControllers(LGPop::RealMode);
  OP.runTaskController(1);

//  TaskControlInterface tci(OP.ctrl_config, OP.ctrl_tasks);
//  tci.ctrl_tasks.waitForRevisionGreaterThan(20);

  rai::wait();

  OP.moveToFreeView();

  rai::wait();

  //-- bring arms in calib pose
  {
//    arr q = {.0, .0, 1., 1.,  -0.1,  0.1, -1.3,  -1.3,  -1.6,   1.6,   1.7,   1.7,   -0., -1.6,    0.05,  0.05};
    arr q = OP.q_home;
//    auto t = tci.addCtrlTaskSineMP("calib", FS_qItself, {}, 5.0, 300.0, 30.0);
//    t->setTarget( q );
//    wait(+t);
  }


//  OP.execGripper(rai::_open, rai::_left);

  rai::wait();
}


void test2() {
  bool rightSide = false;

  LGPop OP(LGPop::SimulationMode);

  cout << OP.q_freeView << endl;

  OP.runCamera(1);

  OP.runRobotControllers(LGPop::SimulationMode);
  OP.runTaskController(1);


  rai::wait();


  rai::KinematicWorld K;
  K.addFile("world.g");


  arr q0;
  if(rightSide) {
//    q0 = {1.5, 0.0, 1., 1.,  -0.1,  0.1, -1.3,  -1.3,  -1.6,   1.6,   1.7,   1.7,   -0., -1.6,    0.05,  0.05};
    q0 = {0.0, 0.0, -0.5, 1., 0.0,  0.1, -1.7, -1.3, 0.0, 1.6, 2.0, 1.7, 0.0, -1.6, 0.05,  0.05};
  } else {
//    q0 = {0.0, -.5, 1., 1.,  -0.1,  0.1, -1.3,  -1.3,  -1.6,   1.6,   1.7,   1.7,   -0., -1.6,    0.05,  0.05};
    q0 = {0.0, 0.0, 1., -0.5, -0.1, 0.0, -1.3, -1.7, -1.6, 0.0, 1.7, 2.0, 0., 0.0, 0.05, 0.05};
  }

  {
    auto t = OP.tci->addCtrlTaskSineMP("calib", FS_qItself, {}, 5.0, 300.0, 30.0);
    t->setTarget(q0);
    wait(+t);
  }

  K.setJointState(OP.ctrl_state.get()->q);

  K.watch(true);

//  arr hsvFilter = ARR(-10, 150, 100, 10, 255, 255).reshape(2, 3);
//  arr hsvFilter = ARR(0, 75, 75, 100, 255, 255).reshape(2, 3);
//  arr hsvFilter = ARR(100, 100, 70, 180, 255, 255).reshape(2, 3);

  arr hsvFilter = ARR(60, 101, 100, 82, 255, 255).reshape(2, 3);

  char LR;
  if(rightSide) LR = 'R';
  else LR = 'L';

  CameraCalibration cc(K, STRING("calibrationVolume" << LR), STRING("calibrationMarker" << LR));
  cc.set_q0(q0);
  cc.hsvFilter = hsvFilter;

  arr targets = cc.generateTargets({4});
  for(uint i = 0; i < targets.d0; i++) {
    cc.updateWorldState(OP.ctrl_config.get());
    arr path = cc.generatePathToTarget(targets[i], rightSide);
    if(!path.N) continue;

    {
      auto t = OP.tci->addCtrlTaskSineMP("calib", FS_qItself, {}, 5.0, 300.0, 30.0);
      t->setTarget(path[path.d0-1]);
      wait(+t);
    }

    rai::wait(1.0);

    cc.updateWorldState(OP.ctrl_config.get());

    cc.captureDataPoint(OP.cam_color.get(), OP.cam_depth.get());

    rai::String fileName;
    fileName << "cameraCalibrationData_simulation";
    if(rightSide) {
      fileName << "_right";
    } else {
      fileName << "_left";
    }
    cc.saveData(fileName);
  }

  cout << endl << cc.computePInv(cc.XData, cc.xData) << endl << endl;

  cc.calibrateCamera();
  cout << cc.I << endl<< endl;
  cout << cc.R << endl<< endl;
  cout << cc.t << endl<< endl;
}



void testGrasp() {
  LGPop OP(LGPop::RealMode);


  OP.runRobotControllers(LGPop::RealMode);
  OP.runTaskController(1);
  TaskControlInterface tci(OP.ctrl_config, OP.ctrl_tasks);
  tci.ctrl_tasks.waitForRevisionGreaterThan(20);

//  rai::wait();

  OP.runCamera(0);
  OP.runPerception(1);

//  rai::wait();
  OP.perception_updateBackgroundModel(false);
//  OP.saveBackgroundModel();
  OP.loadBackgroundModel();

  rai::wait();
  OP.perception_setSyncToConfig(false);

  rai::wait();


  const char* endeff="endeffR";
  const char* object="obj_0";

  //open the gripper
  OP.execGripper(rai::_open, rai::_right);


  //-- code some motion
  {

    rai::KinematicWorld CTmp = OP.ctrl_config.get();
    FILE("test.g") << CTmp;
    rai::KinematicWorld C("test.g");

    StringA joints = C.getJointNames();

    KOMO komo;
    komo.setModel(C, true);
    komo.setPathOpt(2., 30, 3.);
    komo.setSquaredQAccVelHoming(0., -1., 1., 1e-1, 1e-2);
    komo.add_collision(true);

    double h = .5*shapeSize(C, object) - .0;
    komo.addObjective({.9, 1.8}, OT_eq, FS_positionDiff, {endeff, object}, {1e2}, {0,0,h+0.05});
    komo.addObjective({1.9, 2.0}, OT_eq, FS_positionDiff, {endeff, object}, {1e2}, {0,0,h});


    komo.addObjective({.9,2.}, OT_eq, FS_scalarProductXX, {endeff, object}, {1e1});
    komo.addObjective({.9,2.}, OT_eq, FS_scalarProductXZ, {endeff, object}, {1e1});

    komo.addObjective({.9,2.}, OT_sos, FS_vectorZ, {endeff}, {1e0}, {0.,0.,1.} );

//    komo.setSlow(2.,2., 1e2, true);

    komo.verbose=1;
    komo.optimize();


//    bool go = komo.displayPath(true, true);//;/komo.display();
    bool go = komo.displayTrajectory(-0.1, true, false);
    if(!go){
      cout <<"ABORT!" <<endl;
    }else{
      //execute the path
      {
        cout << endl << endl << endl;
        cout << "executePath" << endl;
        auto t = tci.addCtrlTask("qItself", FS_qItself, {}, make_shared<MotionProfile_Path>(komo.getPath(), 5.0));
        wait(+t);
      }
    }
  }

  OP.execGripper(rai::_close, rai::_right);

  rai::wait();


  //attach
  OP.ctrl_attach(endeff, object);

  {
    rai::KinematicWorld C = OP.ctrl_config.get();
    StringA joints = C.getJointNames();

    KOMO komo;
    komo.setModel(C, true);
    komo.setPathOpt(1., 30, 3.);
    komo.setSquaredQAccVelHoming(0., -1., 1., 1e-1, 1e-2);

    komo.addObjective({1.}, OT_eq, FS_qItself, {}, {3e0}, OP.q_home);
    komo.setSlow(1.,1., 1e2, true);

    komo.verbose=1;
    komo.optimize();

    bool go = komo.displayPath(true, true);//;/komo.display();
    if(!go){
      cout <<"ABORT!" <<endl;
    }else{
      //execute the path
      {
        cout << endl << endl << endl;
        cout << "executePath" << endl;
        auto t = tci.addCtrlTask("qItself", FS_qItself, {}, make_shared<MotionProfile_Path>(komo.getPath(), 5.0));
        wait(+t);
      }
    }
  }

  rai::wait();
  OP.execGripper(rai::_open, rai::_right);

}




void testGraspSimulation() {
  LGPop OP(LGPop::SimulationMode);


  OP.runRobotControllers();
  OP.runTaskController(1);

//  rai::wait();

  OP.runCamera(0);
  OP.runPerception(1);

//  rai::wait();
  OP.perception_updateBackgroundModel(false);
//  OP.saveBackgroundModel();
  OP.loadBackgroundModel();

  rai::wait();
  OP.perception_setSyncToConfig(false);

  rai::wait();


  const char* endeff="endeffR";
  const char* object="obj_0";

  //open the gripper
  OP.execGripper(rai::_open, rai::_right);


  //-- code some motion
  {

    rai::KinematicWorld CTmp = OP.ctrl_config.get();
    FILE("test.g") << CTmp;
    rai::KinematicWorld C("test.g");

    StringA joints = C.getJointNames();

    KOMO komo;
    komo.setModel(C, true);
    komo.setPathOpt(2., 30, 3.);
    komo.setSquaredQAccVelHoming(0., -1., 1., 1e-1, 1e-2);
    komo.add_collision(true);

    double h = .5*shapeSize(C, object) - .0;
    komo.addObjective({.9, 1.8}, OT_eq, FS_positionDiff, {endeff, object}, {1e2}, {0,0,h+0.05});
    komo.addObjective({1.9, 2.0}, OT_eq, FS_positionDiff, {endeff, object}, {1e2}, {0,0,h});


    komo.addObjective({.9,2.}, OT_eq, FS_scalarProductXX, {endeff, object}, {1e1});
    komo.addObjective({.9,2.}, OT_eq, FS_scalarProductXZ, {endeff, object}, {1e1});

    komo.addObjective({.9,2.}, OT_sos, FS_vectorZ, {endeff}, {1e0}, {0.,0.,1.} );

//    komo.setSlow(2.,2., 1e2, true);

    komo.verbose=1;
    komo.optimize();


//    bool go = komo.displayPath(true, true);//;/komo.display();
    bool go = komo.displayTrajectory(-0.1, true, false);
    if(!go){
      cout <<"ABORT!" <<endl;
    }else{
      //execute the path
      {
        cout << endl << endl << endl;
        cout << "executePath" << endl;
        auto t = OP.tci->addCtrlTask("qItself", FS_qItself, {}, make_shared<MotionProfile_Path>(komo.getPath(), 5.0));
        wait(+t);
      }
    }
  }

  OP.execGripper(rai::_close, rai::_right);

  rai::wait();


  //attach
  OP.ctrl_attach(endeff, object);

  {
    rai::KinematicWorld C = OP.ctrl_config.get();
    StringA joints = C.getJointNames();

    KOMO komo;
    komo.setModel(C, true);
    komo.setPathOpt(1., 30, 3.);
    komo.setSquaredQAccVelHoming(0., -1., 1., 1e-1, 1e-2);

    komo.addObjective({1.}, OT_eq, FS_qItself, {}, {3e0}, OP.q_home);
    komo.setSlow(1.,1., 1e2, true);

    komo.verbose=1;
    komo.optimize();

    bool go = komo.displayPath(true, true);//;/komo.display();
    if(!go){
      cout <<"ABORT!" <<endl;
    }else{
      //execute the path
      {
        cout << endl << endl << endl;
        cout << "executePath" << endl;
        auto t = OP.tci->addCtrlTask("qItself", FS_qItself, {}, make_shared<MotionProfile_Path>(komo.getPath(), 5.0));
        wait(+t);
      }
    }
  }

  rai::wait();
  OP.execGripper(rai::_open, rai::_right);

}




void testPerception() {
  LGPop OP(LGPop::RealMode);

  OP.runCamera(1);
  OP.runPerception(1);

//  rai::wait();

//  rai::wait();
  OP.perception_updateBackgroundModel(false);
//  OP.saveBackgroundModel();
  OP.loadBackgroundModel();

  rai::wait();
  OP.perception_setSyncToConfig(false);
  rai::wait();

  const char* endeff="endeffR";
  const char* object="obj_0";
}


#if 0
#include <Perception/opencvCamera.h>
#include <Perception/depth2PointCloud.h>

#include <RosCom/roscom.h>
#include <RosCom/rosCamera.h>

#include <Kin/kinViewer.h>

void testRos() {

  Var<byteA> _rgb;
  Var<floatA> _depth;
  RosCamera cam(_rgb, _depth, "cameraRosNode", "/camera/rgb/image_raw", "/camera/depth/image_rect");

  ImageViewer color(_rgb, .1);
  ImageViewerFloat depth(_depth, .1, 128.f);

  rai::wait();
}
#endif



int main(int argc,char** argv){
  rai::initCmdLine(argc,argv);

//  pandaModel();

//  tuneHSV();

//  test2();

//  testGrasp();

  testGraspSimulation();

//  testPerception();

//  testRos();


  return 0;
}


/*
 526.02 0.970575 319.069
 0 523.167 233.737
 0 0 0.997714

 0.999991 -0.00422805 -0.000319627
 -0.00422173 -0.999834 0.0177025
 -0.000394421 -0.017701 -0.999843

 -0.014799 0.332922 1.80108
*/

/*
526.254 0.424931 322.354
 0 521.872 233.959
 0 0 0.997025

 0.999973 -0.00473091 0.00556424
 -0.00483694 -0.999804 0.0191981
 0.00547232 -0.0192245 -0.9998

 -0.0143835 0.333404 1.80027
 */



/*
 *  529.928 -0.00612678 322.307
 0 526.99 235.228
 0 0 1.00021

 0.999977 -0.00471704 0.00482515
 -0.00480714 -0.999811 0.0188349
 0.0047354 -0.0188576 -0.999811

 -0.0155245 0.332387 1.80362
 */



/*
 0.00190136 -9.0447e-06 -0.608926 -0.0130327
 -1.01798e-05 -0.00191634 0.437327 0.330768
 6.71497e-06 4.17687e-05 -1.01298 1.80008
 */










/*
0.00185984 -3.59963e-06 -0.418472 -0.014174
 -1.40775e-05 -0.00186816 0.276088 0.330471
 7.62193e-06 3.40333e-05 -1.00571 1.81469

 537.653 -3.0844 227.671
 0 535.187 155.977
 0 0 1.00099

 0.999963 -0.00757667 0.00411061
 -0.00765029 -0.999805 0.0181996
 0.00397192 -0.0182304 -0.999826

 -0.0141646 0.330469 1.81481
 */






/*
 0.00186173 -4.21766e-06 -0.41852 -0.0145503
 -1.30519e-05 -0.00186546 0.274969 0.330771
 2.44997e-06 3.35638e-05 -1.00856 1.81722

 537.115 -2.5673 225.081
 0 535.974 155.004
 0 0 0.996854

 0.999975 -0.00701278 0.00131719
 -0.00703532 -0.999814 0.0179727
 0.0011909 -0.0179815 -0.999838

 -0.0145572 0.330768 1.81738
 */








//===========================================

// Right
/*
 0.00185991 -6.61481e-06 -0.417826 -0.0147512
 -1.203e-05 -0.00186436 0.273239 0.331235
 3.5685e-06 3.21909e-05 -1.00559 1.81495

 537.645 -1.58631 226.113
 0 536.291 154.336
 0 0 0.99979

 0.999977 -0.00646755 0.00191848
 -0.00649972 -0.99983 0.0172673
 0.00180647 -0.0172794 -0.999849

 -0.0147633 0.331238 1.81514
*/



// Left
/*
 0.00186501 1.1607e-06 -0.419854 -0.0172186
 -8.08122e-06 -0.00185767 0.276274 0.323834
 -1.06122e-06 2.05266e-05 -1.0062 1.82098

 536.171 -2.65896 223.888
 0 538.279 153.159
 0 0 0.996475

 0.99999 -0.00433639 -0.000579122
 -0.00432974 -0.99993 0.0110369
 -0.000626942 -0.0110343 -0.999939

 -0.0172331 0.323839 1.82114
 */
