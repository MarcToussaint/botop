#include <Kin/kin.h>

#include <KOMO/komo.h>
#include <KOMO/komo-ext.h>

#include <Operate/path.h>

#include <Perception/depth2PointCloud.h>

#include <Kin/cameraview.h>
#include <Gui/viewer.h>


#include <LGPop/lgpop.h>

//===========================================================================



void test() {
  LGPop OP(LGPop::SimulationMode);

  OP.runRobotControllers(LGPop::SimulationMode);
  OP.runTaskController(1);

  TaskControlInterface tci(OP.ctrl_config, OP.ctrl_tasks);
  tci.ctrl_tasks.waitForRevisionGreaterThan(20);

  rai::wait();

  //-- bring arms in calib pose
  {
    arr q = {.0, .0, 1., 1.,  -0.1,  0.1, -1.3,  -1.3,  -1.6,   1.6,   1.7,   1.7,   -0., -1.6,    0.05,  0.05};
    auto t = tci.addCtrlTaskSineMP("calib", FS_qItself, {}, 5.0, 300.0, 30.0);
    t->setTarget( q );
    wait(+t);
  }


//  OP.execGripper(rai::_open, rai::_left);

  rai::wait();
}



#if 0
void testPerception() {
  LGPop OP(LGPop::SimulationMode);

  OP.runRobotControllers(LGPop::SimulationMode);
  OP.runTaskController(1);


  OP.execGripper(rai::_open, rai::_left);

  rai::wait();

  OP.runCamera(0);
  OP.runPerception(1);


//  rai::wait();
  OP.perception_updateBackgroundModel(false);
//  OP.saveBackgroundModel();
  OP.loadBackgroundModel();

  rai::wait();
  OP.perception_setSyncToConfig(false);
  rai::wait();

  const char* endeff="endeffL";
  const char* object="obj_0";

  //open the gripper
//  OP.execGripper(rai::_open, rai::_left);

  //-- code some motion
  {
    rai::KinematicWorld C = OP.ctrl_config.get();
    StringA joints = C.getJointNames();
//    C.watch(true);

    KOMO komo;
    komo.setModel(C, true);
    komo.setPathOpt(1., 30, 3.);
    komo.setSquaredQAccVelHoming(0., -1., 1., 1e-1, 1e-2);

    double h = .5*shapeSize(C, object) - .05;
    komo.addObjective({1.}, OT_eq, FS_positionDiff, {endeff, object}, {1e2}, {0,0,h});

    komo.addObjective({.9,1.}, OT_eq, FS_scalarProductXY, {endeff, object}, {1e1});
    komo.addObjective({.9,1.}, OT_eq, FS_scalarProductXZ, {endeff, object}, {1e1});

    komo.addObjective({.9,1.}, OT_sos, FS_vectorZ, {endeff}, {1e0}, {0.,0.,1.} );

//    komo.addObjective(0., up, new TM_Default(TMT_posDiff, komo.world, endeff), OT_sos, {0.,0.,.05}, 1e2, 2);
//    komo.addObjective(.8, 1., new TM_Default(TMT_posDiff, komo.world, endeff), OT_sos, {0.,0.,-.05}, 1e2, 2);

    komo.setSlow(1.,1., 1e2, true);

//    komo.add_collision(true);

    komo.verbose=1;
    komo.optimize();

    bool go = komo.displayPath(true, true);//;/komo.display();
    if(!go){
      cout <<"ABORT!" <<endl;
    }else{
      //execute the path
      OP.execPath(komo.getPath(), komo.getPath_times(), joints, true);
    }
  }

  OP.execGripper(rai::_close, rai::_left);

  rai::wait();


  //attach
  OP.ctrl_attach(endeff, object);

  {
    rai::KinematicWorld C = OP.ctrl_config.get();
    StringA joints = C.getJointNames();

    KOMO komo;
    komo.setModel(C, false);
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
      OP.execPath(komo.getPath(), komo.getPath_times(), joints, true);
    }
  }

  rai::wait();


}



//===========================================================================

void test_operate(){
  LGPop OP(LGPop::SimulationMode);

  OP.runRobotControllers();
  OP.runTaskController();
  OP.runCamera(0);
  OP.runPerception(0);

  rai::wait(1.); //needed for the background estimation to adapt!!

  //cheating! in real I could not copy the real world!
  {
    auto C = OP.sim_config.set();
    rai::Frame *f = C->addFrame("stick", "table");
    f->setRelativePosition({-.2,-.2,.05});
    f->setRelativeQuaternion({1.,0,0,.2});
    f->setColor({.8, .6, .2});
    f->setShape(rai::ST_ssBox, {.2, .03, .05, .01});
    f->setJoint(rai::JT_rigid);
  }


  //==============
  //
  // what follows is a little script to make the robot do things
  //

  const char* endeff="endeffL";
  const char* object="obj_0";
  const char* realObject="stick";
  OP.pauseProcess("syncer");

  ptr<Object> objectToGrasp;
  for(;;){
    OP.objects.waitForNextRevision();
    cout <<"waiting for objects... #objects: " <<OP.objects.get()->N <<endl;
    {
      auto O = OP.objects.get();
      for(const ptr<Object>& obj:O()) cout <<*obj <<endl;

      for(const ptr<Object>& o:O()){
        if(o->age>10 && o->size>100){
          objectToGrasp = o;
          cout <<"SELECTED: " <<*o <<endl;
          break;
        }
      }
    }
    if(objectToGrasp) break;
  }

  rai::wait();
  OP.perception_setSyncToConfig(false);
  rai::wait();

  //open the gripper
  OP.execGripper(rai::_open, rai::_right);

  //-- code some motion
  {
    rai::KinematicWorld C = OP.ctrl_config.get();
    StringA joints = C.getJointNames();
//    C.watch(true);

    KOMO komo;
    komo.setModel(C, false);
    komo.setPathOpt(1., 30, 3.);
    komo.setSquaredQAccVelHoming(0., -1., 1., 1e-1, 1e-2);

    double h = .5*shapeSize(C, object) - .02; //2cm from top
    komo.addObjective({1.}, OT_eq, FS_positionDiff, {endeff, object}, {1e2}, {0,0,h});

    komo.addObjective({.9,1.}, OT_eq, FS_scalarProductXX, {endeff, object}, {1e1});
    komo.addObjective({.9,1.}, OT_eq, FS_scalarProductXZ, {endeff, object}, {1e1});

    komo.addObjective({.9,1.}, OT_sos, FS_vectorZ, {endeff}, {1e0}, {0.,0.,1.} );

//    komo.addObjective(0., up, new TM_Default(TMT_posDiff, komo.world, endeff), OT_sos, {0.,0.,.05}, 1e2, 2);
//    komo.addObjective(.8, 1., new TM_Default(TMT_posDiff, komo.world, endeff), OT_sos, {0.,0.,-.05}, 1e2, 2);

    komo.setSlow(1.,1., 1e2, true);

    komo.verbose=1;
    komo.optimize();

    bool go = komo.displayPath(true, true);//;/komo.display();
    if(!go){
      cout <<"ABORT!" <<endl;
    }else{
      //execute the path
      OP.execPath(komo.getPath(), komo.getPath_times(), joints, true);
    }
  }

  OP.execGripper(rai::_close, rai::_right);

  //attach
  OP.ctrl_attach(endeff, object);
  OP.sim_attach(endeff, realObject);

  {
    rai::KinematicWorld C = OP.ctrl_config.get();
    StringA joints = C.getJointNames();

    KOMO komo;
    komo.setModel(C, false);
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
      OP.execPath(komo.getPath(), komo.getPath_times(), joints, true);
    }
  }

  rai::wait();
}

//===========================================================================

void opHighLevel(){
  LGPop OP(LGPop::SimulationMode);

  OP.runRobotControllers();
  OP.runTaskController();
  OP.runCamera(1);
  OP.runPerception(1);

  rai::wait();

  //cheating! in real I could not copy the real world!
  OP.sim_addRandomBox("box1");
//  OP.sim_addRandomBox();
//  OP.sim_addRandomBox();

  rai::wait();

  //==============
  //
  // what follows is a little script to make the robot do things
  //

  const char* endeff="endeffL";
  const char* object="obj_0";
  const char* realObject="box1";

  //-- that's the "working" config,
  rai::KinematicWorld C;
  C = OP.ctrl_config.get();
  arr x0 = C.getFrameState();
  arr q0 = C.getJointState();
  StringA joints = C.getJointNames();
  C.watch(true);

  //compute a grasp
  chooseBoxGrasp(C, endeff, object);
  C.watch(true);
  arr grasp = C.getJointState();

  //compute a path from x0 to grasp
  C.setFrameState(x0);
  auto path = computePath(C, grasp, joints, endeff, .0, .8);

  //open the gripper
//  G.open();

  //execute the path
  {
    auto ctrlpath = addCtrlTask(OP.ctrl_tasks, OP.ctrl_config, "path", FS_qItself, joints, make_shared<MotionProfile_Path>(path.first, path.second.last()));
    wait(+ctrlpath);
  }
  C.setJointState(OP.ctrl_state.get()->q, joints);

  //close gripper
//  G.close();
  //  K.watch(true);

  //attach
  OP.sim_config.set()->attach(endeff, realObject);
  C.attach(endeff, object);
  C.watch(true);

  path = computePath(C, q0, joints, endeff, .2, .8);
  {
    auto ctrlpath = addCtrlTask(OP.ctrl_tasks, OP.ctrl_config, "path", FS_qItself, joints, make_shared<MotionProfile_Path>(path.first, path.second.last()));
    wait(+ctrlpath);
  }

  rai::wait();
}

#endif
//===========================================================================

int main(int argc,char** argv){
  rai::initCmdLine(argc,argv);

  test();

//  testPerception();

//  test_operate();
//  opHighLevel();

  return 0;
}


