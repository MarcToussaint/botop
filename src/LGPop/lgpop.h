#pragma once

#include <Kin/kin.h>
#include <Kin/frame.h>

#include <NewControl/ctrlMsgs.h>
#include <NewControl/TaskControlThread.h>

#include <Perception/percept.h>
#include <FlatVision/helpers.h>

#include <Franka/franka.h>

namespace rai {
  enum OpenClose { _open, _close };
  enum LeftRight { _left, _right };
}

struct RosCamera;

struct LGPop{
  enum OpMode { NoMode, SimulationMode, RealMode };

  OpMode opMode;
  rai::KinematicWorld rawModel; //should be constant, and unchanged from loaded model
  arr q_home; //pose of loaded model

  //-- simulation variables (when run in simulationMode)
  Var<rai::KinematicWorld> sim_config; //real-synced configuration

  //-- control variables
  Var<rai::KinematicWorld> ctrl_config; //real-synced configuration
  Var<CtrlCmdMsg> ctrl_ref;                //control reference
  Var<CtrlStateMsg> ctrl_state;              //control state
  Var<CtrlTaskL> ctrl_tasks;            //set of control tasks

  //-- camera variables
  Var<floatA> cam_depth; //camera output
  Var<byteA> cam_color;  //camera output
  Var<arr> cam_pose;     //camera pose
  Var<arr> cam_PInv;   //camera inverse projection matrix 3x4

  //-- model camera (predicated images) variables
  Var<byteA> model_segments; //output of model camera (segment IDs)
  Var<floatA> model_depth;   //output of model camera (segment IDs)

  //-- calibration variables
  Var<arr> armPoseCalib; //(2x6 matrix: (dx,dy,dz, rx,ry,rz) (with trans and infinitesimal rot; for both arms)

  //-- perception results
  Var<rai::Array<ptr<Object>>> objects;

  //-- list of all processes
  rai::Array<ptr<Thread>> processes;

  ptr<RosCamera> rosCamera;



  LGPop(OpMode _opMode=SimulationMode);

  ~LGPop();

  //-- running processes
  void runRobotControllers(OpMode _ctrlOpMode=NoMode);
  void runTaskController(int verbose=0);
  void runCamera(int verbose=0);
  void runPerception(int verbose=0);
  void runCalibration();

  void perception_setSyncToConfig(bool _syncToConfig);

  void perception_updateBackgroundModel(bool update);
  void saveBackgroundModel(const char* name = "backgroundModel");
  void loadBackgroundModel(const char* name = "backgroundModel");

  void pauseProcess(const char* name, bool resume=false);

  //--
  bool execGripper(rai::OpenClose openClose, rai::LeftRight leftRight);
  ptr<CtrlTask> execPath(const arr& path, const arr& times, StringA joints, bool wait);



  void reportCycleTimes();

  void updateArmPoseCalibInModel();

  void ctrl_attach(const char* a, const char* b);
  void sim_attach(const char* a, const char* b);
  void sim_addRandomBox(const char* name="randomBox");


private:
  std::shared_ptr<struct self_LGPop> self;
};


/*
 * waitForStableObjectPercept();
 * LGP plan
 * execute plan
 */

double shapeSize(const rai::KinematicWorld& K, const char* name, uint i=1);
