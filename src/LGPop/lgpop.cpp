#include "lgpop.h"

#include <CameraCalibration/cameraCalibration.h>

#include <iomanip>

#include <Gui/viewer.h>

#include <Kin/cameraview.h>
#include <Kin/kinViewer.h>

#include <Franka/controlEmulator.h>
#include <Franka/franka.h>
#include <Franka/gripper.h>
#include <Franka/help.h>

#include <RealSense/RealSenseThread.h>

#include <RosCom/rosCamera.h>

#include <Perception/perceptViewer.h>
#include <Perception/perceptSyncer.h>

#include <FlatVision/flatVision.h>


struct self_LGPop{
  ptr<FrankaGripper> G_right;
  ptr<FrankaGripper> G_left;

  ptr<TaskControlThread> controller;
  ptr<FlatVisionThread> flatVision;

};

LGPop::LGPop(OpMode _opMode, const char* worldFileName)
  : opMode(_opMode), self(make_shared<self_LGPop>()){

  if(worldFileName) {
    rawModel.addFile(worldFileName);
  } else {
    rawModel.addFile(rai::raiPath("../model/pandaStation/pandaStation.g"));
  }

  rawModel.optimizeTree();
  q_home = rawModel.getJointState();
  q_freeView = q_home;

  ctrl_config.set() = rawModel;

  if(opMode==SimulationMode){
    sim_config.set() = rawModel;
    sim_config.name() = "HIDDEN SIMULATION config";
    processes.append( make_shared<KinViewer>(sim_config) );
  }

  armPoseCalib.set() = zeros(2,6);

  // init ctrl_state msg
  {
    auto set = ctrl_state.set();
    rawModel.getJointState(set->q, set->qDot);
    set->tauExternal.resize(set->q.N).setZero();
  } 
}

LGPop::~LGPop(){
  delete tci;
  reportCycleTimes();
}

void LGPop::runRobotControllers(OpMode _ctrlOpMode){
  if(_ctrlOpMode==SimulationMode || opMode==SimulationMode){
    ptr<Thread> F_emul = make_shared<ControlEmulator>(sim_config, ctrl_ref, ctrl_state);
    ptr<Thread> G_emul = make_shared<GripperEmulator>();
    processes.append({F_emul, G_emul});
  }else{
    ptr<Thread> F_right = make_shared<FrankaThread>(ctrl_ref, ctrl_state, 0, franka_getJointIndices(rawModel,'R'));
    ptr<Thread> F_left =  make_shared<FrankaThread>(ctrl_ref, ctrl_state, 1, franka_getJointIndices(rawModel,'L'));
    self->G_right = make_shared<FrankaGripper>(0);
    self->G_right->name = "gripperRight";
    self->G_left =  make_shared<FrankaGripper>(1);
    self->G_left->name = "gripperLeft";
    processes.append({F_right, F_left, std::dynamic_pointer_cast<Thread>(self->G_right), std::dynamic_pointer_cast<Thread>(self->G_left)});
  }
}


void LGPop::runTaskController(int verbose){
  ptr<Thread> TC = make_shared<TaskControlThread>(ctrl_config, ctrl_ref, ctrl_state, ctrl_tasks, new TaskControlMethodInverseKinematics(ctrl_config.get()));
  processes.append(TC);
  if(verbose){
    ctrl_config.name() = "ctrl_config";
    ptr<Thread> view = make_shared<KinViewer>(ctrl_config, .1);
    processes.append(view);
  }

  tci = new TaskControlInterface(ctrl_config, ctrl_tasks);
  tci->ctrl_tasks.waitForRevisionGreaterThan(20);
}

void LGPop::runCamera(int verbose){
  if(opMode==RealMode){
//    auto cam = make_shared<RealSenseThread>(cam_color, cam_depth);
    rosCamera = make_shared<RosCamera>(cam_color, cam_depth, "cameraRosNode", "/camera/rgb/image_raw", "/camera/depth/image_rect");


    arr PInv;

    //PInv.append(~ARR(0.00186463, -6.38537e-06, -0.419071, -0.0145604));
    //PInv.append(~ARR(-1.45655e-05, -0.00186804, 0.252649, 0.330979));
    //PInv.append(~ARR(4.51965e-06, 7.21979e-05, -1.01518, 1.81644));
    //uintA crop = {95, 80, 80, 10};

    PInv.append(~ARR(0.00186622, -9.56714e-06, -0.586458, -0.0151291));
    PInv.append(~ARR(-1.23487e-05, -0.00186911, 0.307741, 0.330753));
    PInv.append(~ARR(6.61791e-06, 7.68097e-05, -1.01703, 1.81519));
    uintA crop = {5, 30, 50, 13};

    cam_PInv.set() = PInv;
    cam_crop.set() = crop;

    cam_depth.waitForNextRevision();
  }
  if(opMode==SimulationMode){
    auto cam = make_shared<rai::Sim_CameraView>(sim_config, cam_color, cam_depth, .1, "camera");
    processes.append(std::dynamic_pointer_cast<Thread>(cam));

    arr PInv;
    PInv.append(~ARR(0.00310267, -4.8012e-08, -0.69714, -0.0136218));
    PInv.append(~ARR(2.46959e-07, -0.00310676, 0.309154, 0.330402));
    PInv.append(~ARR(1.22987e-10, 2.93568e-10, -1, 1.81456));


    cam_PInv.set() = PInv;
  }
  if(verbose){
    cam_color.name() = "cam_color";
    ptr<Thread> view1 = make_shared<ImageViewer>(cam_color, .1);
    cam_depth.name() = "cam_depth";
    ptr<Thread> view2 = make_shared<ImageViewerFloat>(cam_depth, .1, 128.f);
    processes.append({view1, view2});
  }
}

void LGPop::runPerception(int verbose){
  //-- compute model view with robot mask and depth
  franka_setFrameMaskMapLabels(ctrl_config.set());
  ptr<Thread> masker = make_shared<rai::Sim_CameraView>(ctrl_config, model_segments, model_depth,
                                                        .05, "camera", true);
  processes.append(masker);
  if(verbose>1){
    model_segments.name()="model_segments";
    ptr<Thread> viewer = make_shared<ImageViewer>(model_segments);
    processes.append(viewer);

//    model_depth.name()="model_depth";
//    ptr<Thread> viewer2 = make_shared<ImageViewerFloat>(model_depth);
//    processes.append(viewer2);
  }

  //-- big OpenCV process that generates basic percepts
  self->flatVision =
      make_shared<FlatVisionThread>(ctrl_config, objects,
                                    cam_color, cam_depth, model_segments, model_depth,
                                    cam_crop, cam_PInv, armPoseCalib, verbose);
  self->flatVision->name="explainPixels";
  processes.append(std::dynamic_pointer_cast<Thread>(self->flatVision));
}

void LGPop::runCalibration(rai::LeftRight leftRight) {
  rai::KinematicWorld K;
  K.addFile(rai::raiPath("../model/pandaStation/cameraCalibration.g"));

  arr q0;
  if(leftRight == rai::_right) {
    q0 = {0.0, 0.0, -0.5, 1.0, 0.0,  0.1, -1.7, -1.3, 0.0, 1.6, 2.0, 1.7, 0.0, -1.6};
  } else {
    q0 = {0.0, 0.0, 1.0, -0.5, -0.1, 0.0, -1.3, -1.7, -1.6, 0.0, 1.7, 2.0, 0.0, 0.0};
  }

  {
    auto t = tci->addCtrlTaskSineMP("calib", FS_qItself, {}, 5.0, 300.0, 30.0);
    t->setTarget(q0);
    wait(+t);
  }

  K.setJointState(ctrl_state.get()->q);

  K.watch(true);

  arr hsvFilter = ARR(60, 101, 100, 82, 255, 255).reshape(2, 3);

  char LR;
  if(leftRight == rai::_right) {
    LR = 'R';
  } else {
    LR = 'L';
  }

  CameraCalibration cc(K, STRING("calibrationVolume" << LR), STRING("calibrationMarker" << LR), cam_crop.get());
  cc.set_q0(q0);
  cc.hsvFilter = hsvFilter;

  arr targets = cc.generateTargets({4});
  for(uint i = 0; i < targets.d0; i++) {
    cc.updateWorldState(ctrl_config.get());
    arr path = cc.generatePathToTarget(targets[i], leftRight == rai::_right);
    if(!path.N) continue;

    {
      auto t = tci->addCtrlTaskSineMP("calib", FS_qItself, {}, 5.0, 300.0, 30.0);
      t->setTarget(path[path.d0-1]);
      wait(+t);
    }

    rai::wait(1.0);

    cc.updateWorldState(ctrl_config.get());

    cc.captureDataPoint(cam_color.get(), cam_depth.get());

    rai::system("mkdir -p cameraCalibrationData");
    rai::String fileName;
    fileName << "cameraCalibrationData/";
    if(opMode == SimulationMode) {
      fileName << "simulation";
    } else {
      fileName << "real";
    }
    if(leftRight == rai::_right) {
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



void LGPop::perception_setSyncToConfig(bool _syncToConfig){
  for(ptr<Thread>& thread: processes) {
    std::shared_ptr<FlatVisionThread> flatVision = std::dynamic_pointer_cast<FlatVisionThread>(thread);
    if(flatVision){
      flatVision->syncToConfig = _syncToConfig;
    }
  }
}

void LGPop::perception_updateBackgroundModel(bool update) {
  for(ptr<Thread>& thread: processes) {
    std::shared_ptr<FlatVisionThread> flatVision = std::dynamic_pointer_cast<FlatVisionThread>(thread);
    if(flatVision){
      flatVision->updateBackground = update;
    }
  }
}

void LGPop::saveBackgroundModel(const char *name) {
  for(ptr<Thread>& thread: processes) {
    std::shared_ptr<FlatVisionThread> flatVision = std::dynamic_pointer_cast<FlatVisionThread>(thread);
    if(flatVision){
      flatVision->exBackground.saveBackgroundModel(name);
    }
  }
}

void LGPop::loadBackgroundModel(const char *name) {
  for(ptr<Thread>& thread: processes) {
    std::shared_ptr<FlatVisionThread> flatVision = std::dynamic_pointer_cast<FlatVisionThread>(thread);
    if(flatVision){
      flatVision->exBackground.loadBackgroundModel(name);
    }
  }
}

void LGPop::stopPerception() {
  for(ptr<Thread>& thread: processes) {
    std::shared_ptr<FlatVisionThread> flatVision = std::dynamic_pointer_cast<FlatVisionThread>(thread);
    if(flatVision){
      flatVision->threadClose();
    }
  }
}

void LGPop::pauseProcess(const char* name, bool resume){
  for(ptr<Thread>& thread: processes) {
    if(thread->name==name){
      if(!resume) thread->threadStop();
    }
  }
}

bool LGPop::execGripper(rai::OpenClose openClose, rai::LeftRight leftRight) {
  if(opMode == LGPop::SimulationMode) return true;
  if(leftRight == rai::_left) {
    if(openClose == rai::_open) {
      self->G_left->open(0.075, 0.1);
    } else {
      self->G_left->close();
    }

  } else if(leftRight == rai::_right) {
    if(openClose == rai::_open) {
      self->G_right->open(0.075, 0.1);
    } else {
      self->G_right->close();
    }
  } else {
    HALT("weird")
  }
}

ptr<CtrlTask> LGPop::execPath(const arr& path, const arr& times, StringA joints, bool _wait){
  /*ptr<CtrlTask> ctrlpath = addCtrlTask(ctrl_tasks, ctrl_config, "path", FS_qItself, joints, make_shared<MotionProfile_Path>(path, times));
  if(_wait) wait(+ctrlpath);
  rai::wait();
  return ctrlpath;*/
}

std::shared_ptr<CtrlTask> LGPop::moveToFreeView() {
  auto t = tci->addCtrlTask("qItself", FS_qItself, {}, make_shared<MotionProfile_Sine>(q_freeView, 5.0));
  wait(+t);
  return t;
}

void LGPop::reportCycleTimes(){
  cout <<"Cycle times for all Threads (msec):" <<endl;
  for(ptr<Thread>& thread: processes) {
    cout <<std::setw(30) <<thread->name <<" : " <<thread->timer.report() <<endl;
  }
}

void LGPop::updateArmPoseCalibInModel(){
  arr calib = armPoseCalib.get();

  {
    auto Kset = ctrl_config.set();
    double step=.1;

    rai::Frame *f = Kset->getFrameByName("L_panda_link0");
    f->Q.pos.x += step * calib(0,0);
    f->Q.pos.y += step * calib(0,1);
    f->Q.pos.z += 0.2*step * calib(0,2);
    f->Q.rot.addX(0.2*step*calib(0,3));
//    f->Q.rot.addY(0.2*step*calib(0,4));
//    cout <<"calib LEFT =" <<f->Q <<endl;

    f = Kset->getFrameByName("R_panda_link0");
    f->Q.pos.x += step * calib(1,0);
    f->Q.pos.y += step * calib(1,1);
    f->Q.pos.z += 0.2*step * calib(1,2);
    f->Q.rot.addX(0.2*step*calib(1,3));
//    f->Q.rot.addY(0.2*step*calib(1,4));
//    cout <<"calib RIGHT=" <<f->Q <<endl;

    Kset->calc_fwdPropagateFrames();
  }
}

void LGPop::ctrl_attach(const char* a, const char* b){
  ctrl_config.set()->attach(a, b);
}

void LGPop::sim_attach(const char* a, const char* b){
  if(opMode==SimulationMode){
    sim_config.set()->attach(a, b);
  }
}

void LGPop::sim_addRandomBox(const char* name){
  arr size = {rnd.uni(.04,.1), rnd.uni(.1,.5), rnd.uni(.04, .2), .01 };
  arr color = {1.,.5,.5};
  arr pos = { rnd.uni(-1.,1.), rnd.uni(-.5,.5), .5*size(2)+.009 };
  rai::Quaternion rot;
  rot.setRad(rnd.uni(-RAI_PI,RAI_PI), 0,0,1);

  sim_config.set()->addObject(name, "table", rai::ST_ssBox, size, color, pos, rot.getArr4d());
}

