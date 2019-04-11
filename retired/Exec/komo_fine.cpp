#include "komo_fine.h"
#include <Kin/taskMaps.h>
#include <Kin/TM_InsideBox.h>
#include <Kin/TM_linTrans.h>
#include <Kin/frame.h>

double shapeSize(const rai::KinematicWorld& K, const char* name, uint i=2);

void KOMO_fineManip::setFineLift(double time, const char *endeff){
  setTask(time+.1, time+.2, new TM_Default(TMT_pos, world, endeff), OT_sos, {0.,0.,.2}, 1e2, 1);
}

void KOMO_fineManip::setFineHoming(double time, const char *gripper){
  uintA bodies;
  rai::Joint *j;
  for(rai::Frame *f:world.frames){
    if(f->name!=gripper && (j=f->joint) && !j->constrainToZeroVel && j->qDim()>0) bodies.append(f->ID);
  }
  setTask(time-.1, time, new TM_qItself(bodies, true), OT_sos, NoArr, 1e2);
}


void KOMO_fineManip::setFineGrasp(double time, const char *endeff, const char *object, const char *gripper){
  rai::KinematicWorld& K = world;
  StringA joints = K.getJointNames();

  setKinematicSwitch(time, true, "JT_trans3", endeff, object);
  //    setKinematicSwitch(time, true, "ballZero", endeff, object);

  //    setKinematicSwitch(time, true, "insert_transX", NULL, object);

  //height to grasp
  double h = .5*shapeSize(world, object) -.02;
  setTask(time-.1, time, new TM_Default(TMT_posDiff, K, endeff, NoVector, object), OT_sos, {0.,0.,h}, 1e2);

  //vertical
  setTask(time-.35, time, new TM_Default(TMT_vec, K, endeff, Vector_z), OT_sos, {0.,0.,1.}, 1e1);
  //downward motion
  setTask(time-.3, time-.2, new TM_Default(TMT_pos, K, endeff), OT_sos, {0.,0.,-.2}, 1e2, 1);
  //anti-podal
  //    setTask(time-.3, time, new TM_Default(TMT_vecAlign, K, endeff, Vector_y, object, Vector_x), OT_sos, NoArr, 1e1);
  //    setTask(time-.3, time, new TM_Default(TMT_vecAlign, K, endeff, Vector_y, object, Vector_z), OT_sos, NoArr, 1e1);
  //insideBox
  //    setTask(time-.1, time, new TM_InsideBox(K, endeff, NoVector, object, .02), OT_ineq, NoArr, 1e2);
  //open gripper
  setTask(time-.5, time-.1, new TM_qItself(QIP_byJointNames, {gripper}, K), OT_sos, {.06}, 1e2);
  //close gripper
  setTask(time, time, new TM_qItself(QIP_byJointNames, {gripper}, K), OT_sos, {.0}, 1e2);

  //hold still
  joints.removeValue(gripper);
  setTask(time-.1, time+.05, new TM_qItself(QIP_byJointNames, joints, K), OT_eq, NoArr, 1e1, 1);
}

void KOMO_fineManip::setFinePlace(double time, const char *endeff, const char *object, const char *placeRef, const char *gripper){
  rai::KinematicWorld& K = world;
  StringA joints = K.getJointNames();

  //connect object to placeRef
  rai::Transformation rel = 0;
  rel.pos.set(0,0, .5*(shapeSize(world, object) + shapeSize(world, placeRef)));
  setKinematicSwitch(time, true, "transXYPhiZero", placeRef, object, rel );

  //vertical
  setTask(time-.4, time, new TM_Default(TMT_vec, K, endeff, Vector_z), OT_sos, {0.,0.,1.}, 1e2);
  //downward motion
  setTask(time-.35, time-.25, new TM_Default(TMT_pos, K, endeff), OT_sos, {0.,0.,-.2}, 1e2, 1);
  //insideBox
  setTask(time, time, new TM_AboveBox(world, object, placeRef, .1), OT_ineq, NoArr, 1e2);
  //close gripper
  setTask(time-1., time-.15, new TM_qItself(QIP_byJointNames, {gripper}, K), OT_sos, {.0}, 1e2);
  //open gripper
  setTask(time-.05, time, new TM_qItself(QIP_byJointNames, {gripper}, K), OT_sos, {.06}, 1e2);

  //hold still
  joints.removeValue(gripper);
  setTask(time-.15, time+.05, new TM_qItself(QIP_byJointNames, joints, K), OT_eq, NoArr, 1e1, 1);
}

void KOMO_fineManip::setFinePlaceFixed(double time, const char *endeff, const char *object, const char *placeRef, const rai::Transformation& relPose, const char* gripper){
  rai::KinematicWorld& K = world;
  StringA joints = K.getJointNames();

  //connect object to table
  setKinematicSwitch(time, true, "rigidZero", placeRef, object, relPose );

  //vertical
  setTask(time-.4, time, new TM_Default(TMT_vec, K, endeff, Vector_z), OT_sos, {0.,0.,1.}, 1e2);
  //downward motion
  setTask(time-.35, time-.25, new TM_Default(TMT_pos, K, endeff), OT_sos, {0.,0.,-.2}, 1e2, 1);
  //close gripper
  setTask(time-1., time-.15, new TM_qItself(QIP_byJointNames, {gripper}, K), OT_sos, {.0}, 1e2);
  //open gripper
  setTask(time-.05, time, new TM_qItself(QIP_byJointNames, {gripper}, K), OT_sos, {.06}, 1e2);

  //hold still
  joints.removeValue(gripper);
  setTask(time-.15, time+.05, new TM_qItself(QIP_byJointNames, joints, K), OT_eq, NoArr, 1e1, 1);
}

void KOMO_fineManip::setIKGrasp(const char *endeff, const char *object){
  rai::KinematicWorld& K = world;

  //height to grasp
  double h = .5*shapeSize(world, object);
#if 0
  setTask(0.,0.,
          new TM_LinTrans(new TM_Default(TMT_pos, K, endeff, {0.,0.,.05}, object), {2,3,{0.,1.,0., 0.,0.,1.}}, {}),
          OT_eq, {0.,h}, 1e2);

  //insideBox
  //    setTask(1.,1., new TM_InsideBox(K, endeff, {0.,0.,.06}, object, .02), OT_ineq, NoArr, 1e2);
  core_setTouch(0., 0., endeff, object);
#else
  setTask(0.,0.,
          new TM_Default(TMT_pos, K, endeff, {0.,0.,.055}, object),
          OT_eq, {0.,0., h}, 1e2);
#endif

  //anti-podal
  setTask(0.,0., new TM_Default(TMT_vecAlign, K, endeff, Vector_y, object, Vector_x), OT_eq, NoArr, 1e1);
  setTask(0.,0., new TM_Default(TMT_vecAlign, K, endeff, Vector_y, object, Vector_z), OT_eq, NoArr, 1e1);

  //vertical
  setTask(0.,0., new TM_Default(TMT_vec, K, endeff, -Vector_z), OT_eq, {0.,0.,1.}, 1e1 );
}

void KOMO_fineManip::setIKPlace(const char *object, const char *placeRef, const rai::Transformation& worldPose){
  rai::KinematicWorld& K = world;
#if 1
  //height to place
  double h = .5*(shapeSize(world, object) + shapeSize(world, placeRef));
  setTask(0.,0.,
          new TM_LinTrans(new TM_Default(TMT_posDiff, K, object, NoVector, placeRef), {1,3,{0.,0.,1.}}, {}),
          OT_eq, {h}, 1e2);

  //vertical
  setTask(0.,0., new TM_Default(TMT_vecAlign, K, object, Vector_z, placeRef, Vector_x), OT_eq, NoArr, 1e1);
  setTask(0.,0., new TM_Default(TMT_vecAlign, K, object, Vector_z, placeRef, Vector_y), OT_eq, NoArr, 1e1);
#endif

  if(&worldPose){
    setTask(0.,0., new TM_Default(TMT_pos, K, object), OT_eq, worldPose.pos.getArr(), 1e2);
    Task *t = setTask(0.,0., new TM_Default(TMT_quat, K, object), OT_sos, worldPose.rot.getArr4d(), 1e2);
    t->map->flipTargetSignOnNegScalarProduct=true;
  }
}

void KOMO_fineManip::setIKPlaceFixed(const char *object, const rai::Transformation& worldPose){
  rai::KinematicWorld& K = world;

  setTask(1.,1., new TM_Default(TMT_posDiff, K, object), OT_eq, worldPose.pos.getArr(), 1e1);
  Task *t = setTask(1.,1., new TM_Default(TMT_quatDiff, K, object), OT_eq, {}, 1e1);
  t->map->flipTargetSignOnNegScalarProduct=true;

  //vertical
  //    setTask(time-.4, time, new TM_Default(TMT_vec, K, endeff, Vector_z), OT_sos, {0.,0.,1.}, 1e2);

}

void KOMO_fineManip::setIKPush(const char* stick, const char* object, const char* table){

  setKS_slider(1., true, object, "slider1", table);

  //stick normal alignes with slider direction
  setTask(1.,1. , new TM_Default(TMT_vecAlign, world, stick, -Vector_y, "slider1b", Vector_x), OT_sos, {1.}, 1e2);
  //stick horizontal is orthogonal to world vertical
  setTask(1.,1. , new TM_Default(TMT_vecAlign, world, stick, Vector_x, NULL, Vector_z), OT_sos, {0.}, 1e2);

  double dist = .5*shapeSize(world, object, 0)+.01;
  setTask(1.,1. , new TM_InsideBox(world, "slider1b", {dist, .0, .0}, stick), OT_ineq);
  //  setTask(startTime, endTime, new TM_Default(TMT_posDiff, world, stick, NoVector, "slider1b", {dist, .0, .0}), OT_sos, {}, 1e2);


  //  setTask(1.,1., new TM_AboveBox(world, object, table), OT_ineq, NoArr, 1e2);

  //#if 1
  //  //connect object to placeRef
  //  rai::Transformation rel = 0;
  //  rel.pos.set(0,0, .5*(shapeSize(world, object) + shapeSize(world, table)));
  //  setKinematicSwitch(endTime, true, "transXYPhiZero", table, object, rel );
  //#endif

  //  if(stepsPerPhase>2){ //velocities down and up
  //    setTask(startTime-.3, startTime-.1, new TM_Default(TMT_pos, world, stick), OT_sos, {0.,0., -.2}, 1e2, 1); //move down
  //    setTask(startTime-.05, startTime-.0, new TM_Default(TMT_pos, world, stick), OT_sos, {0.,0., 0}, 1e2, 1); //hold still
  //    setTask(endTime+.0, endTime+.05, new TM_Default(TMT_pos, world, stick), OT_sos, {0.,0., 0}, 1e2, 1); //hold still
  //    setTask(endTime+.1, endTime+.3, new TM_Default(TMT_pos, world, stick), OT_sos, {0.,0., .2}, 1e2, 1); // move up
  //  }
}


void KOMO_fineManip::setGoTo(const arr &q, const StringA& joints, const char* endeff, double up, double down){
  rai::KinematicWorld& K = world;

  if(endeff){
    arr profile(T, 3);
    profile.setZero();

    if(up>0.){
      uint t0=up*T+1;
      for(uint t=0;t<t0;t++) profile[t] = ARR(0.,0., .3*((double(t)/t0)));
      setTask(0., up, new TM_Default(TMT_posDiff, K, endeff), OT_sos, profile, 1e2, 1);
    }

    if(down>0.){
      uint t0=down*T-1;
      for(uint t=t0;t<T;t++) profile[t] = ARR(0.,0., -.3*(1.-double(t-t0)/(T-1-t0)));
      setTask(down, 1., new TM_Default(TMT_posDiff, K, endeff), OT_sos, profile, 1e2, 1);
    }
  }

  if(!joints.N){
    setTask(1.,1., new TM_qItself(), OT_eq, q, 1e1);
  }else{
    setTask(1.,1., new TM_qItself(QIP_byJointNames, joints, K), OT_eq, q, 1e1);
  }

  setSlow(0.,0., 1e2, true);
  setSlow(1.,1., 1e2, true);
}

/// translate a list of facts (typically facts in a FOL state) to LGP tasks
void KOMO_fineManip::setIKAbstractTask(const StringA& symbols, int verbose){
  double time=1.;
  if(symbols(0)=="home")            setHoming(1., 1.);
  else if(symbols(0)=="grasp")      setIKGrasp(symbols(1+0), symbols(1+1));
  else if(symbols(0)=="place"){
    if(symbols.N==2)                setPlace(time, NULL, symbols(1+0), symbols(1+1), verbose);
    else                            setPlace(time, symbols(1+0), symbols(1+1), symbols(1+2), verbose);
  }else if(symbols(0)=="handover")  setHandover(time, symbols(1+0), symbols(1+1), symbols(1+2), verbose);
  else if(symbols(0)=="push")       setIKPush(symbols(1+0), symbols(1+1), symbols(1+2)); //TODO: the +1. assumes pushes always have duration 1
  else if(symbols(0)=="slide") {     NIY;} //setSlide(time, symbols(1+0), symbols(1+1), symbols(1+2), verbose);
  else if(symbols(0)=="slideAlong") setSlideAlong(time, symbols(1+0), symbols(1+1), symbols(1+2), verbose);
  else if(symbols(0)=="drop")       setDrop(time, symbols(1+0), symbols(1+1), symbols(1+2), verbose);
  else HALT("UNKNOWN komo TAG: '" <<symbols(0) <<"'");
}
