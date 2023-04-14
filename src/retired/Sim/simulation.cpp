#include "simulation.h"

#include <Algo/spline.h>
#include <Kin/kin_swift.h>
#include <Kin/proxy.h>

arr computeNextFeasibleConfiguration(rai::KinematicWorld& K, arr q_ref, StringA& jointsInLimit, StringA& collisionPairs);

struct Sensor{
  rai::String name;
  rai::Camera cam;
  uint width=640, height=480;
  byteA backgroundImage;
  Sensor();
};

struct Simulation_self{
  rai::Array<Sensor> sensors;

  Mutex threadLock;

  rai::KinematicWorld K_compute;
  OpenGL gl;

  StringA currentlyUsedJoints; //the joints that the spline refers to
  rai::Spline refSpline; // reference spline constructed from ref
  arr refPoints, refTimes; // the knot points and times of the spline
  double phase=0.; // current phase in the spline
  double dt; // time stepping interval
  uint stepCount=0; // number of simulation steps
};

Simulation::Simulation(const char *modelFile, double dt)
  : K(modelFile){
  self = new Simulation_self;
  self->currentlyUsedJoints = K.getJointNames();
  self->dt = dt;

  self->gl.title = "Simulation";
  self->gl.camera.setDefault();
  self->gl.add(glStandardScene);
  self->gl.add(K);
//  self->gl.update();

  self->K_compute = K;
  self->K_compute.swift().deactivate(K["table1"], K["iiwa_link_0_0"]);
  self->K_compute.swift().deactivate(K["table1"], K["stick"]);
  self->K_compute.swift().deactivate(K["table1"], K["stickTip"]);
}

Simulation::~Simulation(){
  delete self;
}

void Simulation::stepKin(){
  auto lock = self->threadLock();

  double maxPhase = 0.;
  if(self->refSpline.knotPoints.N){
    //read out the new reference
    self->phase += self->dt;
    maxPhase = self->refSpline.knotTimes.last();
    arr q_ref = self->refSpline.eval(self->phase);
    if(self->phase>maxPhase){ //clear spline buffer
      q_ref = self->refPoints[-1];
      stop(true);
    }
//    self->K_ref.setJointState(q_ref); //for display only
//    timeToGo.set() = conv_double2Float64(maxPhase-phase);

    //compute feasible q_next (collision & limits)
    StringA jointsInLimit, collisionPairs;
//    arr q_next = computeNextFeasibleConfiguration(K, q_ref, jointsInLimit, collisionPairs);
    //    set the simulation's joint state
    //    K.setJointState(q_next);

//    setJointStateSafe(q_ref, jointsInLimit, collisionPairs);
    setJointState(self->currentlyUsedJoints, q_ref);
//    if(jointsInLimit.N)
//      cout <<"LIMITS:" <<jointsInLimit <<endl;
    if(collisionPairs.N)  cout <<"COLLISIONS:" <<collisionPairs <<endl;

    //display
//    K_disp.setJointState(K.q);
  }

  if(!(self->stepCount%10))
    self->gl.update(STRING("step=" <<self->stepCount <<" phase=" <<self->phase <<" timeToGo=" <<maxPhase-self->phase <<" #ref=" <<self->refSpline.knotPoints.d0));

  self->stepCount++;

}

void Simulation::setJointState(const StringA &joints, const arr &q_ref){
  auto lock = self->threadLock();

  K.setJointState(q_ref, joints);
}

void Simulation::setJointStateSafe(arr q_ref, StringA &jointsInLimit, StringA &collisionPairs){
  auto lock = self->threadLock();

  arr q = q_ref;
  arr q0 = K.getJointState(self->currentlyUsedJoints);

  self->K_compute.setJointState(q0, self->currentlyUsedJoints);
  rai::KinematicWorld& KK = self->K_compute;

  jointsInLimit.clear();
  collisionPairs.clear();

  //-- first check limits -> box constraints -> clip
  for(rai::Joint *j:KK.fwdActiveJoints){
    bool active=false;
    if(j->limits.N){
      for(uint d=0;d<j->dim;d++){
        if(q(j->qIndex+d) < j->limits(0)){
          q(j->qIndex+d) = j->limits(0);
          active=true;
        }
        if(q(j->qIndex+d) > j->limits(1)){
          q(j->qIndex+d) = j->limits(1);
          active=true;
        }
      }
    }
    if(active) jointsInLimit.append(j->frame.name);
  }
  q_ref = q;

  //-- optimize s.t. collisions
  KK.setJointState(q, self->currentlyUsedJoints);
  KK.stepSwift();

//  KK.kinematicsPenetrations(y);

  arr y;
  double margin = .03;
  KK.kinematicsProxyCost(y, NoArr, margin);

  for(rai::Proxy& p:KK.proxies) if(p.d<margin){
    collisionPairs.append({p.a->name, p.b->name});
  }
  collisionPairs.reshape(2, collisionPairs.N/2);

  if(y.scalar()>.9 || collisionPairs.N){
    //back to old config
    q = q0;

    //simple IK to find config closest to q_ref in nullspace of contact
    arr y, J, JJ, invJ, I=eye(KK.q.N);
    KK.setJointState(q, self->currentlyUsedJoints);
    KK.stepSwift();
    KK.kinematicsProxyCost(y, J, margin);

    uint k;
    for(k=0; k<10; k++) {
      JJ = J*~J;
      for(uint i=0;i<JJ.d0;i++) JJ(i,i) += 1e-5;
      invJ = ~J * inverse_SymPosDef(JJ);

      arr qstep_nullspace = q_ref - q;
      if(length(qstep_nullspace)>.1) qstep_nullspace = .1/length(qstep_nullspace) * qstep_nullspace;

      arr qstep = invJ * (ARR(.9) - y); //.9 is the target for the proxy cost (1. is contact, 0. is margin))
      qstep += (I - invJ*J) * qstep_nullspace;

      if(absMax(qstep)<1e-6) break;

      q += qstep;

      KK.setJointState(q, self->currentlyUsedJoints);
      KK.stepSwift();
      KK.kinematicsProxyCost(y, J, margin);
    }

    if(y.scalar()>1.){
      //failed -> back to old config
      q=q0;
      KK.setJointState(q, self->currentlyUsedJoints);
      KK.stepSwift();
      KK.kinematicsProxyCost(y, J, margin);
    }

    if(y.scalar()>1.){
      LOG(-1) <<"what's going on??? y=" <<y;
    }
  }

  K.setJointState(q, self->currentlyUsedJoints);
}

void Simulation::setUsedRobotJoints(const StringA& joints){
  auto lock = self->threadLock();

  if(self->currentlyUsedJoints!=joints){
    if(self->refPoints.N){
      LOG(-1) <<"you changed the robot joints before the spline was done -- killing spline execution";
      stop(true);
    }
    self->currentlyUsedJoints=joints;
  }
}


void Simulation::exec(const arr &x, const arr &t, bool append){
  auto lock = self->threadLock();

  if(!self->refTimes.N) append=false;

  if(x.d1 != self->currentlyUsedJoints.N){
    LOG(-1) <<"you're sending me a motion reference of wrong dimension!"
          <<"\n  I'm ignoring this"
         <<"\n  my dimension=" <<self->currentlyUsedJoints.N <<"  your message=" <<x.d1
        <<"\n  my joints=" <<self->currentlyUsedJoints;
    return;
  }

  if(append){
    self->refPoints.append(x);
    double last = self->refTimes.last();
    self->refTimes.append(t+last);
    self->refSpline.set(2, self->refPoints, self->refTimes);
  }else{
    self->refPoints = x;
    self->refTimes = t;
    if(self->refTimes(0)>0.){ //the given reference does not have a knot for NOW -> copy the current joint state
      self->refTimes.prepend(0.);
      self->refPoints.prepend(K.getJointState(self->currentlyUsedJoints));
    }
    self->refSpline.set(2, self->refPoints, self->refTimes);
    self->phase=0.;
  }
}

void Simulation::exec(const StringA& command){
  auto lock = self->threadLock();

  LOG(0) <<"CMD = " <<command <<endl;
  if(command(0)=="attach"){
    rai::Frame *a = K.getFrameByName(command(1));
    rai::Frame *b = K.getFrameByName(command(2));
    b = b->getUpwardLink();

    if(b->parent) b->unLink();
    b->linkFrom(a, true);
    (new rai::Joint(*b)) -> type=rai::JT_rigid;
    K.calc_q();
  }
}

void Simulation::stop(bool hard){
  auto lock = self->threadLock();

  self->phase=0.;
  self->refPoints.clear();
  self->refTimes.clear();
  self->refSpline.clear();
}

double Simulation::getTimeToGo(){
  auto lock = self->threadLock();

  double maxPhase = 0.;
  if(self->refSpline.knotPoints.N) maxPhase = self->refSpline.knotTimes.last();
  return maxPhase - self->phase;
}

arr Simulation::getJointState(){
  auto lock = self->threadLock();

  return K.getJointState(self->currentlyUsedJoints);
}

arr Simulation::getObjectPoses(const StringA &objects){
  auto lock = self->threadLock();

  FrameL objs;
  if(objects.N){
    for(const rai::String& s:objects) objs.append(K[s]);
  }else{//non specified... go through the list and pick 'percets'
    for(rai::Frame *a:K.frames){
      if(a->ats["percept"]) objs.append(a);
    }
  }

  arr X(objs.N, 7);
  for(uint i=0;i<objs.N;i++) X[i] = objs.elem(i)->X.getArr7d();
  return X;
}

StringA Simulation::getJointNames(){
  auto lock = self->threadLock();

  return K.getJointNames();
}

StringA Simulation::getObjectNames(){
  auto lock = self->threadLock();

  StringA objs;
  for(rai::Frame *a:K.frames){
    if(a->ats["percept"]) objs.append(a->name);
  }
  return objs;
}

void Simulation::glDraw(OpenGL &gl){
  glStandardLight(NULL);
   //  glEnable(GL_LIGHTING);

  for(Sensor& sen:self->sensors){
    glDrawCamera(sen.cam);
    glDrawText(STRING("SENSOR " <<sen.name), 0., 0., 0.);
  }
}

//=============================================================================

#if 0
arr computeNextFeasibleConfiguration(rai::KinematicWorld& K, arr q_ref, StringA& jointsInLimit, StringA& collisionPairs){
  arr q = q_ref;
  arr q0 = K.getJointState();

  jointsInLimit.clear();
  collisionPairs.clear();

  //-- first check limits -> box constraints -> clip
  for(rai::Joint *j:K.fwdActiveJoints){
    bool active=false;
    if(j->limits.N){
      for(uint d=0;d<j->dim;d++){
        if(q(j->qIndex+d) < j->limits(0)){
          q(j->qIndex+d) = j->limits(0);
          active=true;
        }
        if(q(j->qIndex+d) > j->limits(1)){
          q(j->qIndex+d) = j->limits(1);
          active=true;
        }
      }
    }
    if(active) jointsInLimit.append(j->frame.name);
  }
  q_ref = q;

  //-- optimize s.t. collisions
  K.setJointState(q);
//  K.stepSwift();

  arr y;
  K.kinematicsPenetrations(y);
  for(uint i=0;i<y.N;i++) if(y(i)>0.){
    collisionPairs.append({K.proxies(i).a->name, K.proxies(i).b->name});
  }
  collisionPairs.reshape(2, collisionPairs.N/2);

  double margin = .03;
  K.kinematicsProxyCost(y, NoArr, margin);
  if(y.scalar()>.9 || collisionPairs.N){
    //back to old config
    q = q0;

    //simple IK to find config closest to q_ref in nullspace of contact
    arr y, J, JJ, invJ, I=eye(K.q.N);
    K.setJointState(q);
    K.stepSwift();
    K.kinematicsProxyCost(y, J, margin);

    uint k;
    for(k=0; k<10; k++) {
      JJ = J*~J;
      for(uint i=0;i<JJ.d0;i++) JJ(i,i) += 1e-5;
      invJ = ~J * inverse_SymPosDef(JJ);

      arr qstep_nullspace = q_ref - q;
      if(length(qstep_nullspace)>.1) qstep_nullspace = .1/length(qstep_nullspace) * qstep_nullspace;

      arr qstep = invJ * (ARR(.9) - y); //.9 is the target for the proxy cost (1. is contact, 0. is margin))
      qstep += (I - invJ*J) * qstep_nullspace;

      if(absMax(qstep)<1e-6) break;

      q += qstep;

      K.setJointState(q);
      K.stepSwift();
      K.kinematicsProxyCost(y, J, margin);
    }

    if(y.scalar()>1.){
      //failed -> back to old config
      q=q0;
      K.setJointState(q);
      K.stepSwift();
      K.kinematicsProxyCost(y, J, margin);
    }

    if(y.scalar()>1.){
      LOG(-1) <<"what's going on??? y=" <<y;
    }
  }

  return q;
}
#endif

