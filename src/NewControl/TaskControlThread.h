/*  ------------------------------------------------------------------
    Copyright (c) 2017 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include <Core/thread.h>
#include <NewControl/ctrlMsgs.h>
#include <NewControl/taskControl.h>


struct TaskControlThread : Thread {

  Var<rai::KinematicWorld> ctrl_config;
  Var<CtrlCmdMsg> ctrl_ref;
  Var<CtrlStateMsg> ctrl_state;
  Var<CtrlTaskL> ctrl_tasks;
  TaskControlMethod* tcm;

  arr q_real, qdot_real, tauExternal; //< real state
  arr q0; //< homing pose
  //arr Hmetric;

  bool requiresInitialSync;
  int verbose;
  
  TaskControlThread(const Var<rai::KinematicWorld>& _ctrl_config,
                    const Var<CtrlCmdMsg>& _ctrl_ref,
                    const Var<CtrlStateMsg>& _ctrl_state,
                    const Var<CtrlTaskL>& _ctrl_tasks,
                    TaskControlMethod* tcm);
  ~TaskControlThread();

  arr whatsTheForce(const ptr<CtrlTask>& t);

  void step();
};


// ----------------------------------------------------------------------------

struct TaskControlInterface {
  Var<rai::KinematicWorld> ctrl_config;
  Var<CtrlTaskL> ctrl_tasks;

  TaskControlInterface(const Var<rai::KinematicWorld>& ctrl_config, const Var<CtrlTaskL>& ctrl_tasks);

  ptr<CtrlTask> addCtrlTask(const char* name, const ptr<Feature>& taskMap, const ptr<MotionProfile>& mp, bool active = true);
  ptr<CtrlTask> addCtrlTask(const char* name, FeatureSymbol fs, const StringA& frames, const ptr<MotionProfile>& mp, bool active = true);

  ptr<CtrlTask> addCtrlTask(const char* name, FeatureSymbol fs, const StringA& frames, const ptr<MotionProfile>& mp, double kp, double kd, const arr& C = arr(), bool active = true);


  ptr<CtrlTask> addCtrlTaskSineMP(const char* name, const ptr<Feature>& taskMap, double duration, bool active = true);
  ptr<CtrlTask> addCtrlTaskSineMP(const char* name, FeatureSymbol fs, const StringA& frames, double duration, bool active = true);
  ptr<CtrlTask> addCtrlTaskPD(const char* name, const ptr<Feature>& taskMap, double decayTime, double dampingRatio, bool active = true);
  ptr<CtrlTask> addCtrlTaskPD(const char* name, FeatureSymbol fs, const StringA& frames, double decayTime, double dampingRatio, bool active = true);

  ptr<CtrlTask> addCtrlTaskSineMP(const char* name, FeatureSymbol fs, const StringA& frames, double duration,  double kp, double kd, const arr& C = arr(), bool active = true);

  ptr<CtrlTask> addCtrlTaskConst(const char* name, const ptr<Feature>& taskMap, const arr& y_target = arr(), bool active = true);
  ptr<CtrlTask> addCtrlTaskConst(const char* name, FeatureSymbol fs, const StringA& frames, const arr& y_target = arr(), bool active = true);

  ptr<CtrlTask> addCtrlTaskConst(const char* name, FeatureSymbol fs, const StringA& frames, const arr& y_target = arr(), double kp = 0.0, double kd = 0.0, const arr& C = arr(), bool active = true);


  ptr<CtrlTask> addCtrlTaskConstVel(const char* name, const ptr<Feature>& taskMap, const arr& v_target = arr(), bool active = true);
  ptr<CtrlTask> addCtrlTaskConstVel(const char* name, FeatureSymbol fs, const StringA& frames, const arr& v_target = arr(), bool active = true);

  ptr<CtrlTask> addCtrlTaskConstVel(const char* name, FeatureSymbol fs, const StringA& frames, const arr& v_target = arr(), double kd = 0.0, const arr& C = arr(), bool active = true);


  void setTarget(ptr<CtrlTask>& ct, const arr& y_ref, const arr& v_ref = NoArr);
  void setVTarget(ptr<CtrlTask>& ct, const arr& v_ref);

  void activateCtrlTask(ptr<CtrlTask>& ct);
  void deactivateCtrlTask(ptr<CtrlTask>& ct);

  void removeCtrlTask(const ptr<CtrlTask>& ct);

  arr forceInTaskSpace(const ptr<CtrlTask>& ct);
};




ptr<CtrlTask> addCompliance(Var<CtrlTaskL>& ctrlTasks,
                            Var<rai::KinematicWorld>& ctrl_config,
                            const char* name, FeatureSymbol fs, const StringA& frames,
                            const arr& compliance);


