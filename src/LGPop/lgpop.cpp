#include "lgpop.h"

#include <iomanip>

#include <Gui/viewer.h>

#include <Kin/cameraview.h>
#include <Kin/kinViewer.h>

#include <Franka/controlEmulator.h>
#include <Franka/franka.h>
#include <Franka/gripper.h>
#include <Franka/help.h>

#include <RealSense/RealSenseThread.h>

#include <Perception/perceptViewer.h>
#include <Perception/perceptSyncer.h>

#include <FlatVision/flatVision.h>

struct self_LGPop{
  ptr<FrankaGripper> G_right;
  ptr<FrankaGripper> G_left;

  ptr<TaskControlThread> controller;
  ptr<FlatVisionThread> flatVision;

};

LGPop::LGPop(OpMode _opMode)
  : opMode(_opMode), self(make_shared<self_LGPop>()){

  rawModel.addFile(rai::raiPath("../model/pandaStation.g"));
  rawModel.optimizeTree();
  q_home = rawModel.getJointState();

  ctrl_config.set() = rawModel;

  if(true || opMode==SimulationMode){
    sim_config.set() = rawModel;
    sim_config.name() = "HIDDEN SIMULATION config";
    processes.append( make_shared<KinViewer>(sim_config) );
  }

  cam_pose.set() = rawModel["camera"]->X.getArr7d();

  armPoseCalib.set() = zeros(2,6);

  {
    auto set = ctrl_state.set();
    rawModel.getJointState(set->q, set->qdot);
  }
}

LGPop::~LGPop(){
  reportCycleTimes();
}

void LGPop::runRobotControllers(OpMode _ctrlOpMode){
  if(_ctrlOpMode==SimulationMode || opMode==SimulationMode){
    ptr<Thread> F_emul = make_shared<ControlEmulator>(sim_config, ctrl_ref, ctrl_state);
    ptr<Thread> G_emul = make_shared<GripperEmulator>();
    processes.append({F_emul, G_emul});
  }else{
    //ptr<Thread> F_right = make_shared<FrankaThread>(ctrl_ref, ctrl_state, 0, franka_getJointIndices(rawModel,'R'));
    ptr<Thread> F_left =  make_shared<FrankaThread>(ctrl_ref, ctrl_state, 1, franka_getJointIndices(rawModel,'L'));
    //self->G_right = make_shared<FrankaGripper>(0);
    self->G_left =  make_shared<FrankaGripper>(1);
    processes.append({/*F_right,*/ F_left,
                      /*std::dynamic_pointer_cast<Thread>(self->G_right),*/
                      std::dynamic_pointer_cast<Thread>(self->G_left)});
  }
}

void LGPop::runTaskController(int verbose){
  self->controller = make_shared<TaskControlThread>(ctrl_config, ctrl_ref, ctrl_state, ctrl_tasks);
  processes.append(std::dynamic_pointer_cast<Thread>(self->controller));
  if(verbose){
    ctrl_config.name() = "ctrl_config";
    ptr<Thread> view = make_shared<KinViewer>(ctrl_config, .1);
    processes.append(view);
  }
}

void LGPop::runCamera(int verbose){
  if(opMode==RealMode){
    auto cam = make_shared<RealSenseThread>(cam_color, cam_depth);
    cam_depth.waitForNextRevision();
    cam_Fxypxy.set() = cam->depth_fxypxy;
    cout <<"RS calib=" <<cam_Fxypxy.get()() <<endl;
    processes.append(std::dynamic_pointer_cast<Thread>(cam));
  }
  if(opMode==SimulationMode){
    auto cam = make_shared<rai::Sim_CameraView>(sim_config, cam_color, cam_depth, .1, "camera");
    cam_Fxypxy.set() = cam->getFxypxy();
    processes.append(std::dynamic_pointer_cast<Thread>(cam));
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
                                    cam_pose, cam_Fxypxy, armPoseCalib, verbose);
  self->flatVision->name="explainPixels";
  processes.append(std::dynamic_pointer_cast<Thread>(self->flatVision));
}

void LGPop::runCalibration(){
  NIY;
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

void LGPop::pauseProcess(const char* name, bool resume){
  for(ptr<Thread>& thread: processes) {
    if(thread->name==name){
      if(!resume) thread->threadStop();
    }
  }
}

bool LGPop::execGripper(rai::OpenClose openClose, rai::LeftRight leftRight){
  if(leftRight == rai::_left) {
    if(openClose == rai::_open) {
      self->G_left->open(0.072, 0.1);
    } else {
      self->G_left->close();
    }

  } else if(leftRight == rai::_right) {

  } else {
    HALT("weird")
  }
}

ptr<CtrlTask> LGPop::execPath(const arr& path, const arr& times, StringA joints, bool _wait){
  ptr<CtrlTask> ctrlpath = addCtrlTask(ctrl_tasks, ctrl_config, "path", FS_qItself, joints, make_shared<MotionProfile_Path>(path, times));
  if(_wait) wait(+ctrlpath);
  rai::wait();
  return ctrlpath;
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
