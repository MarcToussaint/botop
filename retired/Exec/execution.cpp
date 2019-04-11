#include "execution.h"

#include <RosCom/roscom.h>
#include <Exec/komo_fine.h>
#include <Sim/simulationIO_self.h>

LGPExecution::LGPExecution(bool _useRealRobot, bool _useSimulation, const char *modelFile)
  : S(false, modelFile, .01),
    R(_useRealRobot),
    K(modelFile),
    useRealRobot(_useRealRobot),
    useSimulation(_useSimulation){

  guiPauses = rai::getParameter<bool>("guiPauses", false);
  timeScale = rai::getParameter<double>("timeScale", 1.);

//  K.displayDot();

  allJoints = K.getJointNames();
  for(rai::Frame *f: K.frames){
    if(f->joint && f->joint->dim==1 && f->name.contains("iiwa")) armJoints.append(f->name);
    if(f->joint && f->joint->dim==1 && f->name.contains("gripper")) gripperJoints.append(f->name);
  }

  q_home = K.getJointState(armJoints);

  //-- move all objects to random/zero places!! Must not assume that the model knows real poses
//  for(rai::Frame *a:K.frames) if(a->ats["percept"]){
//    a->X.setZero();
//    a->Q.setZero();
//  }
  K.calc_fwdPropagateFrames();
  arr q = K.getJointState();
  q.setZero();
  K.setJointState(q);

  K.gl().title = "ExecutionModel";
  K.watch(guiPauses, "initialization");

  if(useRealRobot){
    syncModelJointStateWithRealOrSimulation();
  }else{
    setModelJointState(armJoints, q_home);
  }

  if(useSimulation){
    startSimulation();
  }

  syncModelObjectPosesWithRealOrSimulation(20);
}

void LGPExecution::startSimulation(){
  useSimulation = true;
  S.threadLoop();
}

void LGPExecution::stopSimulation(){
  S.threadStop();
  useSimulation = false;
}

void LGPExecution::syncModelObjectPosesWithRealOrSimulation(uint average){
  if(useRealRobot){
    StringA objects = R.getObjectNames();
    cout << "NUMOBJECTS: " << objects.size() << endl;
    arr poses = R.getObjectPoses(objects);
    if(average>1){
      for(uint a=1;a<average;a++){
        rai::wait(.1);
        poses += R.getObjectPoses(objects);
      }
      poses /= (double)average;
    }

    for(uint i=0;i<objects.N;i++){
        rai::Frame *f = K[objects(i)];
        if(!f){ //not exists
            f = new rai::Frame(K);
            f->name = objects(i);
            rai::Shape *s = new rai::Shape(*f);
            s->type() = rai::ST_marker;
        }
        f->X.set(poses[i]);
        if(f->parent) f->Q = f->X / f->parent->X;
        cout << "synced tracked Obj: " << f->name << " with pose: " << f->X << endl;
    }
    K.calc_fwdPropagateFrames();

    K.watch(guiPauses, "synced objects");
  }else{
      Var<rai_msgs::StringA> objectNames("objectNames");
      objectNames.waitForNextRevision();
      StringA objs = conv_StringA2StringA(objectNames.get());

      Var<rai_msgs::arr> objectPoses("objectPoses");
      objectPoses.waitForNextRevision();
      arr poses = conv_arr2arr(objectPoses.get());

      CHECK_EQ(objs.N, poses.d0, "");

      for(uint i=0;i<objs.N;i++){
        rai::Frame *f = K[objs(i)];
        CHECK(f, "");
        f->X.set(poses[i]);
        if(f->parent) f->Q = f->X / f->parent->X;
      }
      K.calc_fwdPropagateFrames();

      K.watch(guiPauses, "synced objects");
  }
}

void LGPExecution::setModelJointState(const StringA &joints, const arr &q){
  K.setJointState(q, joints);
  K.watch(guiPauses, "set joints");
}

void LGPExecution::syncModelJointStateWithRealOrSimulation(){
  if(useRealRobot){
      //sync with the current robot state
    StringA joints = R.getJointNames();
    arr q = R.getJointPositions(joints);
    setModelJointState(joints, q);
    S.self->SIM.setJointState(joints, q);
  } else if(useSimulation){
    S.self->SIM.setUsedRobotJoints(allJoints);
    K.setJointState( S.self->SIM.getJointState() );
  }
  K.watch(guiPauses, "synced joints");
}

void LGPExecution::waitForCompletion(){
  if(useSimulation){
    Var<std_msgs::Float64> ttg("timeToGo");
    uint r = ttg.getRevision();
    ttg.waitForRevisionGreaterThan(r+10);
    for(;;){
      ttg.waitForNextRevision();
      if(ttg.get()->data<=0.) break;
    }
  }
}

void LGPExecution::attach(const char *a, const char *b){
  StringA cmd = {"attach", a, b};
  Var<rai_msgs::StringA> command("command");
  command.set() = conv_StringA2StringA( cmd );
  syncModelJointStateWithRealOrSimulation();

  if(cmd(0)=="attach"){
    rai::Frame *a = K.getFrameByName(cmd(1));
    rai::Frame *b = K.getFrameByName(cmd(2));
    b = b->getUpwardLink();

    if(b->parent) b->unLink();
    b->linkFrom(a, true);
    (new rai::Joint(*b)) -> type=rai::JT_rigid;
    K.calc_q();
  }

  K.watch(guiPauses, STRING(cmd));
}

void LGPExecution::openGripper(){
  arr q = arr(1,1, {.08});
  if(useSimulation){
    executeMotion(gripperJoints, q, {1.});
  }
  if(useRealRobot){
    R.execGripper(100.);
  }
}

void LGPExecution::closeGripper(){
  arr q = arr(1,1, {.01});
  if(useSimulation){
    executeMotion(gripperJoints, q, {1.});
  }
  if(useRealRobot){
    R.execGripper(0.);
  }
}

void LGPExecution::executeMotion(const StringA& joints, const arr &q, const arr &tau){
  displayAndValidate(joints, q, tau);

  //send to simulation
  if(useSimulation){
    Var<rai_msgs::MotionReference> ref("MotionReference");

    ref.writeAccess();
    ref->joints = conv_StringA2stdStringVec(joints);
    ref->x = conv_arr2arr(q);
    ref->t = conv_arr2arr(tau);
    ref->append = true;
    ref->revision = 0;
    ref.deAccess(); //publishes automatically

    waitForCompletion();
  }
  if(useRealRobot){
    R.executeMotion(joints, q, timeScale * tau);
  }
  syncModelJointStateWithRealOrSimulation();

  K.watch(guiPauses, "final state of executed motion");
}

std::pair<arr,arr> LGPExecution::computePreciseMotion(const arr &to, bool initialUp, bool finalDown){
  KOMO_fineManip komo(K);

  komo.setPathOpt(1., 20, 3.);
  komo.setGoTo(to, armJoints, "endeff", initialUp?.1:.0, finalDown?.9:.0);
  komo.reset();
  komo.run();
  cout <<komo.getReport(false) <<endl;
//  while(komo.displayTrajectory(.1, true));

  arr x = komo.getPath(armJoints);
//  cout <<x[komo.T-1] <<endl <<to <<endl <<"ERROR=" <<sumOfSqr(to - x[komo.T-1]) <<endl;
  x[komo.T-1] = to; //hard overwrite of last config!

  arr tau = komo.getPath_times();

  K.setJointState(to, armJoints);
  K.watch(guiPauses, "comp end motion");

  return {x, tau};
}

struct PlanDrawer : GLDrawer {
  rai::KinematicWorld& K;
  const arr& q0;
  const StringA& joints; const arr& q; const arr& tau;
  PlanDrawer(rai::KinematicWorld& K, const arr& q0, const StringA& joints, const arr& q, const arr& tau): K(K), q0(q0), joints(joints), q(q), tau(tau) {}
  void glDraw(OpenGL& gl) {
    K.setJointState(q0, joints);
    K.glDraw(gl);
    for(uint i=0; i<q.d0; i++) {
      K.setJointState(q[i], joints);
      K.glDraw(gl);
    }
  }
};

void LGPExecution::displayAndValidate(const StringA& joints, const arr& q, const arr& tau){
  arr q0 = K.getJointState();

  syncModelJointStateWithRealOrSimulation();

  arr q_now = K.getJointState(joints);

  rai::String txt;
  txt <<"VALIDATE with ENTER -- ESC will quit";

  if(q.d0>1){
    double startVel, endVel, maxVel=0.;
    startVel = length(q[0]-q_now)/(tau(0));
    endVel = length(q[-1]-q[-2])/(tau(-1)-tau(-2));
    for(uint t=1;t<q.d0;t++){
      double v = length(q[t]-q[t-1])/(tau(t)-tau(t-1));
      if(v>maxVel) maxVel = v;
    }
    txt <<"\nv0=" <<startVel <<" vT=" <<endVel <<" vMax=" <<maxVel;
  }
  if(joints.N<=3){
    txt <<"\n" <<joints;
  }
  txt <<"\n";

  PlanDrawer planDrawer(K, q_now, joints, q, tau);
  K.gl().remove(K);
  K.gl().add(planDrawer);
  for(;;){
    int key = K.gl().watch(txt);

    if(key==13){ //validated
      K.gl().remove(planDrawer);
      K.gl().add(K);
      K.setJointState(q0);
      K.gl().update("validated");
      return;
    }

    if(key==27){
      LOG(0) <<"NO VALIDATION - exiting";
      K.gl().closeWindow();
      exit(0);
    }
  }
}

arr LGPExecution::computePreciseGrasp(const arr &roughPose, const char *obj){
  KOMO_fineManip komo(K);
  komo.world.setJointState(roughPose, armJoints);
  komo.setIKOpt();
  komo.setIKGrasp("endeff", obj);
  komo.reset();
  komo.run();
  cout <<komo.getReport(false) <<endl;
//  while(komo.displayTrajectory(-1, true));

  arr x = komo.configurations.last()->getJointState(armJoints);
  K.setJointState(x, armJoints);
  K.watch(guiPauses, "comp Grasp");

  return x;
}

arr LGPExecution::computePrecisePlace(const arr &roughPose, const char *obj, const char *onto, const rai::Transformation& Q){
  KOMO_fineManip komo(K);
  komo.world.setJointState(roughPose, armJoints); //this initializes the zero-pose to rough... regularization
  komo.setIKOpt();
  komo.setIKPlace(obj, onto, Q);
  komo.reset();
  komo.run();
  cout <<komo.getReport(false) <<endl;
//  while(komo.displayTrajectory(-1, true));

  arr x = komo.configurations.last()->getJointState(armJoints);
  K.setJointState(x, armJoints);
  K.watch(guiPauses, "comp Place");

  return x;
}

arr LGPExecution::getJointConfiguration(ptr<KOMO> plan, double phase){
  const rai::KinematicWorld& K_phase = plan->getConfiguration(phase);
  return K_phase.getJointState(armJoints);
}

rai::Transformation LGPExecution::getObjectPose(ptr<KOMO> plan, double phase, const char *obj){
  rai::KinematicWorld& K_phase = plan->getConfiguration(phase);
  return K_phase[obj]->X;
}

ptr<KOMO> LGPExecution::planSkeleton(const Skeleton &S){
  double maxPhase = 1.;
  for(const SkeletonEntry& s:S){
    if(s.phase0 > maxPhase) maxPhase = s.phase0;
    if(s.phase1 > maxPhase) maxPhase = s.phase1;
  }
  maxPhase += .5;

  ptr<KOMO> komo = std::make_shared<KOMO>();
  komo->setModel(K, true);
  komo->setPathOpt(maxPhase, 10., 2.);
  komo->setCollisions(false);
  komo->deactivateCollisions("table1", "iiwa_link_0_0");
  komo->activateCollisions("table1", "stick");
  komo->activateCollisions("table1", "stickTip");

  komo->setSkeleton(S);

  komo->reset();
  //  komo->reportProblem();
  komo->animateOptimization = 0;
  komo->run();
  //   komo->checkGradients();

  cout <<komo->getReport(false) <<endl;
//  komo->reportEffectiveJoints();
//  komo->reportProxies();
//  while(komo->displayTrajectory(.1));

  return komo;
}

