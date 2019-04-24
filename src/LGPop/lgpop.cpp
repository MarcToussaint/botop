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

LGPop::LGPop(bool _simulationMode)
  : simulationMode(_simulationMode){

  rawModel.addFile(rai::raiPath("../model/pandaStation.g"));
  rawModel.optimizeTree();
  q_home = rawModel.getJointState();

  ctrl_config.set() = rawModel;

  if(simulationMode){
    sim_config.set() = rawModel;
    sim_config.name() = "SIMULATION config";
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

void LGPop::runRobotControllers(bool simuMode){
  if(simuMode || simulationMode){
    ptr<Thread> F_emul = make_shared<ControlEmulator>(sim_config, ctrl_ref, ctrl_state);
    ptr<Thread> G_emul = make_shared<GripperEmulator>();
    processes.append({F_emul, G_emul});
  }else{
    ptr<Thread> F_right = make_shared<FrankaThread>(ctrl_ref, ctrl_state, 0, franka_getJointIndices(rawModel,'R'));
    ptr<Thread> F_left =  make_shared<FrankaThread>(ctrl_ref, ctrl_state, 1, franka_getJointIndices(rawModel,'L'));
    ptr<Thread> G_right = make_shared<FrankaGripper>(0);
    ptr<Thread> G_left =  make_shared<FrankaGripper>(1);
    processes.append({F_right, F_left, G_right, G_left});
  }
}

void LGPop::runTaskController(int verbose){
  ptr<Thread> TC = make_shared<TaskControlThread>(ctrl_config, ctrl_ref, ctrl_state, ctrl_tasks);
  processes.append(TC);
  if(verbose){
    ctrl_config.name() = "ctrl_config";
    ptr<Thread> view = make_shared<KinViewer>(ctrl_config, .1);
    processes.append(view);
  }
}

void LGPop::runCamera(int verbose){
  if(!simulationMode){
    auto cam = make_shared<RealSenseThread>(cam_color, cam_depth);
    cam_depth.waitForNextRevision();
    cam_Fxypxy.set() = cam->depth_fxypxy;
    cout <<"RS calib=" <<cam_Fxypxy.get()() <<endl;
    processes.append(std::dynamic_pointer_cast<Thread>(cam));
  }else{
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
  ptr<Thread> flatVision =
      make_shared<FlatVisionThread>(ctrl_config, cam_color, cam_depth, model_segments, model_depth, cam_pose, cam_Fxypxy, armPoseCalib, verbose);
  flatVision->name="explainPixels";
  processes.append(flatVision);

#if 0
  //-- percept filter and integration in model
  ptr<Thread> filter = make_shared<SyncFiltered> (percepts, ctrl_config);
  filter->name="syncer";
  processes.append(filter);
  if(verbose>3){
    ptr<Thread> view = make_shared<PerceptViewer>(percepts, ctrl_config);
    processes.append(view);
//    ptr<Thread> view2 = make_shared<KinViewer>(ctrl_config);
  }
#endif
}

void LGPop::runCalibration(){
  NIY;
}

void LGPop::pauseProcess(const char* name, bool resume){
  for(ptr<Thread>& thread: processes) {
    if(thread->name==name){
      if(!resume) thread->threadStop();
    }
  }
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

void LGPop::sim_addRandomBox(const char* name){
  arr size = {rnd.uni(.04,.1), rnd.uni(.1,.5), rnd.uni(.04, .2), .01 };
  arr color = {1.,.5,.5};
  arr pos = { rnd.uni(-1.,1.), rnd.uni(-.5,.5), .5*size(2)+.009 };
  rai::Quaternion rot;
  rot.setRad(rnd.uni(-RAI_PI,RAI_PI), 0,0,1);

  sim_config.set()->addObject(name, rai::ST_ssBox, size, color, -1., "table", pos, rot.getArr4d());
//      File("../../model/stick.g", "table", rai::Transformation({-.2,-.2,.05}, 0));


}
