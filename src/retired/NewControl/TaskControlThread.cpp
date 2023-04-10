/*  ------------------------------------------------------------------
    Copyright (c) 2017 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "TaskControlThread.h"
#include <Kin/frame.h>

TaskControlThread::TaskControlThread(const Var<rai::KinematicWorld>& _ctrl_config,
                                     const Var<CtrlCmdMsg>& _ctrl_ref,
                                     const Var<CtrlStateMsg>& _ctrl_state,
                                     const Var<CtrlTaskL>& _ctrl_tasks,
                                     TaskControlMethod* tcm)
  : Thread("TaskControlThread", .01),
    ctrl_config(this, _ctrl_config),
    ctrl_ref(this, _ctrl_ref),
    ctrl_state(this, _ctrl_state),
    ctrl_tasks(this, _ctrl_tasks),
    tcm(tcm),
    requiresInitialSync(true),
    verbose(0)
{

  double hyper = rai::getParameter<double>("hyperSpeed", -1.);
  if(hyper>0.) this->metronome.reset(.01/hyper);

  //memorize the "NULL position", which is the initial model position
  q0 = ctrl_config.get()->getJointState();
  q_real = q0;
  qdot_real = zeros(q0.N);

  //Hmetric = rai::getParameter<double>("Hrate", .1)*ctrl_config.get()->getHmetric();

  threadLoop();
}

TaskControlThread::~TaskControlThread() {
  threadClose();
  delete tcm;
}

arr TaskControlThread::whatsTheForce(const ptr<CtrlTask>& t){
  return pseudoInverse(~t->J_y)*tauExternal;
}


#if 1

void TaskControlThread::step() {  
  //-- initial initialization
  if(requiresInitialSync){
    if(ctrl_state.getRevision()>1) {
      {
        auto state = ctrl_state.get();
        q_real = state->q;
        qdot_real = state->qDot;
      }
      ctrl_config.set()->setJointState(q_real, qdot_real);
      requiresInitialSync = false;
    } else{
      LOG(0) << "waiting for ctrl_state messages...";
      return;
    }
  }

  //-- read current state
  CtrlStateMsg ctrlStateMsg;
  {
    auto state = ctrl_state.get();
    ctrlStateMsg = ctrl_state();
    if(state->q.N){
      q_real = state->q;
      qdot_real = state->qDot;
      tauExternal = state->tauExternal;
    }
  }
  //-- update kinematic world for controller
  ctrl_config.set()->setJointState(q_real, qdot_real);

  ctrl_tasks.writeAccess();
  ctrl_config.readAccess();

  //-- update control tasks
  for(CtrlTask* t: ctrl_tasks()) t->update(0.01, ctrl_config(), tauExternal);

  CtrlCmdMsg ctrlCmdMsg;

  tcm->calculate(ctrlCmdMsg, ctrlStateMsg, ctrl_tasks(), ctrl_config());

  // TODO tcm->checkSafety(ctrlCmdMsg, ctrlStateMsg, ctrl_tasks(), ctrl_config());

  ctrl_tasks.deAccess();
  ctrl_config.deAccess();

  //-- safety checks come here
  // MORE TODO
  // limit dq
#if 0
  double maxQStep = 2e-1;
  arr dq = ctrlCmdMsg.qRef - q_real;
  double l = length(dq);
  if(l > maxQStep) {
    dq *= maxQStep/l;
    cout << "limit max dq" << endl;
    ctrlCmdMsg.qRef = q_real + dq;
  }
  // limit qDotRef
  double maxQDot = 2.0;
  if(absMax(ctrlCmdMsg.qDotRef) > maxQDot) {
    cout << "limit max qDot exceeded " << ctrlCmdMsg.qDotRef << endl;
    return;
  }
#endif

  ctrl_ref.set() = ctrlCmdMsg;
}

#else
void TaskControlThread::step() {
  //-- initial initialization
  if(requiresInitialSync){
    if(ctrl_state.getRevision()>1) {
      {
        auto state = ctrl_state.get();
        q_real = state->q;
        qdot_real = state->qdot;
      }
      ctrl_config.set()->setJointState(q_real, qdot_real);
      requiresInitialSync = false;
    } else{
      LOG(0) << "waiting for ctrl_state messages...";
      return;
    }
  }

  //-- read current state
  {
    auto state = ctrl_state.get();
    if(state->q.N){
      q_real = state->q;
      qdot_real = state->qdot;
      torques_real = state->u_bias;
    }
  }
  ctrl_config.set()->setJointState(q_real, qdot_real);

  arr P_compliance;
  arr qRef = q_real;
  arr qDotRef = zeros(qdot_real.N);
  {
    auto K = ctrl_config.set();

    if(false && !(step_count%20)){
      rai::String txt;
      txt <<"TaskControlThread ctrl_config " <<step_count;
      for(CtrlTask *t:ctrl_tasks.get()()){ txt <<'\n'; t->reportState(txt); }
      K->watch(false, txt); //only for debugging
    }

    ctrl_tasks.writeAccess();
    for(CtrlTask* t: ctrl_tasks()) t->update(.01, K);

    TaskControlMethods taskController(Hmetric);

    //-- get compliance projection matrix
    P_compliance = taskController.getComplianceProjection(ctrl_tasks());

    //-- compute IK step
    double maxQStep = 2e-1;
    arr dq;

    dq = taskController.inverseKinematics(ctrl_tasks(), qDotRef, P_compliance); //don't include a null step
    if(dq.N){
      double l = length(dq);
      if(l>maxQStep) dq *= maxQStep/l;
      qRef += dq;
    }

#if 0
    //set/test the new configuration
    K->setJointState(q_model, qdot_model); //DONT! the configuration should stay on real; use a separate one for safty checks
    if(useSwift) K->stepSwift();
    for(CtrlTask* t: ctrl_tasks()) t->update(.0, K); //update without time increment
    double cost = taskController.getIKCosts(ctrl_tasks());
//    IK_cost.set() = cost;

    //check the costs
    if(cost>1000.) { //reject!
      LOG(-1) <<"HIGH COST IK! " <<cost;
      q_model -= .9*dq;
      K->setJointState(q_model, qdot_model);
      if(useSwift) K->stepSwift();
      for(CtrlTask* t: ctrl_tasks()) t->update(.0, K); //update without time increment
    }
#endif

    if(verbose) taskController.reportCurrentState(ctrl_tasks());

    ctrl_tasks.deAccess();
  }


  //TODO: construct force tasks
  //    //-- compute the force feedback control coefficients
  //    uint count=0;
  //    ctrl_tasks.readAccess();
  //    taskController.tasks = ctrl_tasks();
  //    for(CtrlTask *t : taskController.tasks) {
  //      if(t->active && t->f_ref.N){
  //        count++;
  //        if(count!=1) HALT("you have multiple active force control tasks - NIY");
  //        t->getForceControlCoeffs(refs.fL, refs.u_bias, refs.KiFTL, refs.J_ft_invL, realWorld);
  //      }
  //    }
  //    if(count==1) refs.Kp = .5;
  //    ctrl_tasks.deAccess();

  //-- output: set variables
  if(true){
    CtrlMsg refs;
    refs.q = qRef;
    refs.qdot = qDotRef;
    refs.P_compliance = P_compliance;
    refs.fL_gamma = 1.;
    refs.fL = zeros(6);
    refs.fR = zeros(6);
    refs.KiFTL.clear();
    refs.J_ft_invL.clear();
    refs.u_bias = zeros(q_real.N);
    refs.intLimitRatio = 1.;
    refs.qd_filt = .99;

    //-- send the computed movement to the robot
    ctrl_ref.set() = refs;
  }
}

#endif






TaskControlInterface::TaskControlInterface(const Var<rai::KinematicWorld>& ctrl_config, const Var<CtrlTaskL>& ctrl_tasks)
    : ctrl_config(ctrl_config),
      ctrl_tasks(ctrl_tasks) {}

ptr<CtrlTask> TaskControlInterface::addCtrlTask(const char* name, const ptr<Feature>& taskMap, const ptr<MotionProfile>& mp, bool active) {
  ptr<CtrlTask> ct = make_shared<CtrlTask>(name, taskMap, mp);
  ct->active = active;
  ct->update(0., ctrl_config.get(), zeros(ctrl_config.get()->getJointStateDimension())); // initialize control task with current values
  ct->ctrlTasks = &ctrl_tasks;
  ctrl_tasks.set()->append(ct.get());
  return ct;
}

ptr<CtrlTask> TaskControlInterface::addCtrlTask(const char* name, FeatureSymbol fs, const StringA& frames, const ptr<MotionProfile>& mp, bool active) {
  return addCtrlTask(name, symbols2feature(fs, frames, ctrl_config.get()), mp, active);
}

ptr<CtrlTask> TaskControlInterface::addCtrlTask(const char *name, FeatureSymbol fs, const StringA &frames, const ptr<MotionProfile> &mp, double kp, double kd, const arr &C, bool active) {
  ptr<CtrlTask> ct = make_shared<CtrlTask>(name, symbols2feature(fs, frames, ctrl_config.get()), mp, kp, kd, C);
  ct->active = active;
  ct->update(0., ctrl_config.get(), zeros(ctrl_config.get()->getJointStateDimension())); // initialize control task with current values
  ct->ctrlTasks = &ctrl_tasks;
  ctrl_tasks.set()->append(ct.get());
  return ct;
}

ptr<CtrlTask> TaskControlInterface::addCtrlTaskSineMP(const char* name, const ptr<Feature>& taskMap, double duration, bool active) {
  return addCtrlTask(name, taskMap, make_shared<MotionProfile_Sine>(arr(), duration), active);
}

ptr<CtrlTask> TaskControlInterface::addCtrlTaskSineMP(const char* name, FeatureSymbol fs, const StringA& frames, double duration, bool active) {
  return addCtrlTask(name, symbols2feature(fs, frames, ctrl_config.get()), make_shared<MotionProfile_Sine>(arr(), duration), active);
}

ptr<CtrlTask> TaskControlInterface::addCtrlTaskPD(const char* name, const ptr<Feature>& taskMap, double decayTime, double dampingRatio, bool active) {
  return addCtrlTask(name, taskMap, make_shared<MotionProfile_PD>(arr(), decayTime, dampingRatio), active);
}

ptr<CtrlTask> TaskControlInterface::addCtrlTaskPD(const char* name, FeatureSymbol fs, const StringA& frames, double decayTime, double dampingRatio, bool active) {
  return addCtrlTask(name, symbols2feature(fs, frames, ctrl_config.get()), make_shared<MotionProfile_PD>(arr(), decayTime, dampingRatio), active);
}

ptr<CtrlTask> TaskControlInterface::addCtrlTaskSineMP(const char *name, FeatureSymbol fs, const StringA &frames, double duration, double kp, double kd, const arr &C, bool active) {
  ptr<CtrlTask> ct = make_shared<CtrlTask>(name, symbols2feature(fs, frames, ctrl_config.get()), make_shared<MotionProfile_Sine>(arr(), duration), kp, kd, C);
  ct->active = active;
  ct->update(0., ctrl_config.get(), zeros(ctrl_config.get()->getJointStateDimension())); // initialize control task with current values
  ct->ctrlTasks = &ctrl_tasks;
  ctrl_tasks.set()->append(ct.get());
  return ct;
}

ptr<CtrlTask> TaskControlInterface::addCtrlTaskConst(const char* name, const ptr<Feature>& taskMap, const arr& y_target, bool active) {
  return addCtrlTask(name, taskMap, make_shared<MotionProfile_Const>(y_target), active);
}

ptr<CtrlTask> TaskControlInterface::addCtrlTaskConst(const char* name, FeatureSymbol fs, const StringA& frames, const arr& y_target, bool active) {
  return addCtrlTask(name, symbols2feature(fs, frames, ctrl_config.get()), make_shared<MotionProfile_Const>(y_target), active);
}

ptr<CtrlTask> TaskControlInterface::addCtrlTaskConst(const char *name, FeatureSymbol fs, const StringA &frames, const arr &y_target, double kp, double kd, const arr &C, bool active) {
  ptr<CtrlTask> ct = make_shared<CtrlTask>(name, symbols2feature(fs, frames, ctrl_config.get()), make_shared<MotionProfile_Const>(y_target), kp, kd, C);
  ct->active = active;
  ct->update(0., ctrl_config.get(), zeros(ctrl_config.get()->getJointStateDimension())); // initialize control task with current values
  ct->ctrlTasks = &ctrl_tasks;
  ctrl_tasks.set()->append(ct.get());
  return ct;
}

ptr<CtrlTask> TaskControlInterface::addCtrlTaskConstVel(const char* name, const ptr<Feature>& taskMap, const arr& v_target, bool active) {
  return addCtrlTask(name, taskMap, make_shared<MotionProfile_ConstVel>(v_target), active);
}

ptr<CtrlTask> TaskControlInterface::addCtrlTaskConstVel(const char* name, FeatureSymbol fs, const StringA& frames, const arr& v_target, bool active) {
  return addCtrlTask(name, symbols2feature(fs, frames, ctrl_config.get()), make_shared<MotionProfile_ConstVel>(v_target), active);
}

ptr<CtrlTask> TaskControlInterface::addCtrlTaskConstVel(const char *name, FeatureSymbol fs, const StringA &frames, const arr &v_target, double kd, const arr &C, bool active) {
  ptr<CtrlTask> ct = make_shared<CtrlTask>(name, symbols2feature(fs, frames, ctrl_config.get()), make_shared<MotionProfile_ConstVel>(v_target), 0.0, kd, C);
  ct->active = active;
  ct->update(0., ctrl_config.get(), zeros(ctrl_config.get()->getJointStateDimension())); // initialize control task with current values
  ct->ctrlTasks = &ctrl_tasks;
  ctrl_tasks.set()->append(ct.get());
  return ct;
}

void TaskControlInterface::setTarget(ptr<CtrlTask>& ct, const arr& y_ref, const arr& v_ref) {
  ctrl_tasks.writeAccess();
  ct->setTarget(y_ref, v_ref);
  ctrl_tasks.deAccess();
}

void TaskControlInterface::setVTarget(ptr<CtrlTask>& ct, const arr& v_ref) {
  ctrl_tasks.writeAccess();
  ct->setTarget(arr(), v_ref);
  ctrl_tasks.deAccess();
}

void TaskControlInterface::activateCtrlTask(ptr<CtrlTask>& ct) {
  ctrl_tasks.writeAccess();
  ct->active = true;
  ctrl_tasks.deAccess();
}

void TaskControlInterface::deactivateCtrlTask(ptr<CtrlTask>& ct) {
  ctrl_tasks.writeAccess();
  ct->active = false;
  ctrl_tasks.deAccess();
}

void TaskControlInterface::removeCtrlTask(const ptr<CtrlTask>& ct) {
  ctrl_tasks.set()->removeValue(ct.get());
}

arr TaskControlInterface::forceInTaskSpace(const ptr<CtrlTask>& ct) {
  //return pseudoInverse(~ct->J_y)*torques_real;
}




#if 0
ptr<CtrlTask> addCompliance(Var<CtrlTaskL>& ctrl_tasks,
                            Var<rai::KinematicWorld>& ctrl_config,
                            const char* name, FeatureSymbol fs, const StringA& frames,
                            const arr& compliance){
  ptr<CtrlTask> t = make_shared<CtrlTask>(name, symbols2feature(fs, frames, ctrl_config.get()));
  t->compliance = compliance;
  t->ctrlTasks = &ctrl_tasks;
  ctrl_tasks.set()->append(t.get());
  return t;
}






#endif
