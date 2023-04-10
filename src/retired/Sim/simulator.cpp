#include "simulator.h"

#include <Kin/kin_swift.h>
#include <Kin/frame.h>
#include <Kin/proxy.h>

//=============================================================================

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

Simulator::Simulator(const char *modelFile, double dt)
  : ROS("simulator"),
    K(modelFile),
    K_disp(K), K_ref(K),
    gl("SIM"),
    ref("MotionReference"),
    command("command"),
    currentQ("currentQ"),
    timeToGo("timeToGo"),
    dt(dt){

  //K.swift().deactivate(K["coll_pedestal"], K["coll_box_bottom"]);

  //setup display
  gl.camera.setDefault();
  gl.add(glStandardScene);
  //    gl.add(K_ref); //don't display the reference, only the actual state
  gl.add(K_disp);
  gl.update();

  //setup ros communication
  sub_ref = ROS.subscribe(ref);
  sub_command = ROS.subscribe(command);
  pub_timeToGo = ROS.publish(timeToGo);
  pub_currentQ = ROS.publish(currentQ);
}

void Simulator::step(){
  //-- check for reference update
  if(ref.hasNewRevision()){
    ref.readAccess();
    //StringA joints = ref->joints; //is ignored for now!!!
    arr t = conv_arr2arr(ref->t);
    arr x = conv_arr2arr(ref->x);
    bool append = ref->append;
    cout <<"MotionReference t=" <<dt*stepCount <<"s revision=" <<ref.getRevision() <<' ' <<ref->revision <<endl;
    ref.deAccess();

    if(!refTimes.N) append=false;

    if(append){
      refPoints.append(x);
      double last = refTimes.last();
      for(double tau:t) refTimes.append(last += tau);
      reference.set(2, refPoints, refTimes);
    }else{
      refPoints = x;
      refTimes.clear();
      double last = 0.;
      for(double tau:t) refTimes.append(last += tau);
      if(refTimes(0)>0.){ //the given reference does not have a knot for NOW -> copy the current joint state
        refTimes.prepend(0.);
        refPoints.prepend(K.q);
      }
      reference.set(2, refPoints, refTimes);
      phase=0.;
    }
  }

  //-- when reference is given, compute movement
  double maxPhase = 0.;
  if(reference.knotPoints.N){
    //read out the new reference
    phase += dt;
    maxPhase = reference.knotTimes.last();
    if(phase>maxPhase) phase=maxPhase;
    arr q_ref = reference.eval(phase);
    K_ref.setJointState(q_ref); //for display only
    timeToGo.set() = conv_double2Float64(maxPhase-phase);

    //compute feasible q_next (collision & limits)
    StringA jointsInLimit, collisionPairs;
    arr q_next = computeNextFeasibleConfiguration(K, q_ref, jointsInLimit, collisionPairs);

    //set the simulation's joint state
    K.setJointState(q_next);

    //display
    K_disp.setJointState(K.q);
  }

  if(!(stepCount%10))
    gl.update(STRING("step=" <<stepCount <<" phase=" <<phase <<" timeToGo=" <<maxPhase-phase <<" #ref=" <<reference.knotPoints.d0));

  stepCount++;
}

void Simulator::loop(){
  //loop
  Metronome tictac(dt);
  for(;stepCount<10000;){
    tictac.waitForTic();
    step();
  }
}
