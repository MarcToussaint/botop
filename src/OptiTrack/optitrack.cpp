#include "optitrack.h"

#ifdef RAI_OPTITRACK

#include "motioncapture_optitrack.h"

#include <Kin/frame.h>


rai::Transformation rb2pose(const libmotioncapture::RigidBody& rb){
  rai::Transformation X;
  X.pos.set(rb.position()(0), rb.position()(1), rb.position()(2));
  X.rot.set(rb.rotation().w(), rb.rotation().vec()(0), rb.rotation().vec()(1), rb.rotation().vec()(2));
  return X;
}

void poseFilter(rai::Transformation& X, double alpha, const rai::Transformation& signal){
  CHECK_GE(alpha, 0., "");
  CHECK_LE(alpha, 1., "");
  if(alpha>0.){
    double posErr = (X.pos - signal.pos).length();
    double quatErr = 1. - fabs(rai::quat_scalarProduct(X.rot, signal.rot));
    if(posErr > .5 || quatErr > .2){
      LOG(-1) <<"rejecting jumping optitrack signal! err:" <<posErr <<' ' <<quatErr;
//      return;
    }
    X.pos = alpha*X.pos + (1.-alpha)*signal.pos;
    X.rot.setInterpolate(1.-alpha, X.rot, signal.rot);
  }else{
    X = signal;
  }
}

namespace rai{

  OptiTrack::OptiTrack() : Thread("OptitrackThread", 0.){
    mocap = libmotioncapture::MotionCapture::connect("optitrack", rai::getParameter<rai::String>("optitrack/host", "130.149.82.29").p);
    threadLoop();
  }

  OptiTrack::~OptiTrack(){
    threadClose();
    delete mocap;
  }

  void OptiTrack::pull(rai::Configuration& C){
    //-- update configuration
    //we need a base frame
    const char* name = "optitrack_base";
    rai::Frame *base = C.getFrame(name, false);
    if(!base){
      LOG(0) <<"creating new frame 'optitrack_base'";
      base = C.addFrame(name);
      base->setShape(rai::ST_marker, {.3});
      base->setColor({.8, .8, .2});
      if(rai::checkParameter<arr>("optitrack_baseFrame")){
        arr X = rai::getParameter<arr>("optitrack_baseFrame");
        base->setPose(rai::Transformation(X));
      }else{
        LOG(0) <<"WARNING: optitrack_baseFrame not set - using non-calibrated default [0 0 0]!";
      }
    }

    std::lock_guard<std::mutex> lock(mux);

    //loop through all ot signals
    for (auto const& obj: poses) {
      //get (or create) frame for that body
      const char* name = obj.first.c_str();
      rai::Frame *f = C.getFrame(name, false);
      if(!f){//create a new marker frame!
        LOG(0) <<"creating new frame '" <<name <<"'";
        f = C.addFrame(name);
        f->setParent(base);
        f->setShape(rai::ST_marker, {.02});
        f->setColor({.8, .8, .2});
      }

      //set pose of frame
      f->set_Q() = obj.second;
    }
  }

  void OptiTrack::step(){
    // Get a frame
    mocap->waitForNextFrame();
    //uint64_t timeStamp = mocap->timeStamp();

    std::map<std::string, libmotioncapture::RigidBody> rigidBodies = mocap->rigidBodies();

    std::lock_guard<std::mutex> lock(mux);

    //loop through all ot signals
    for (auto const& item: rigidBodies) {
      if(item.second.occluded()) continue;

      const char* name = item.second.name().c_str();
      auto pose = poses.find(name);

      if(pose!=poses.end()){
        //filter an existing entry
        poseFilter(pose->second, filter, rb2pose(item.second));
      }else{
        //create a new entry
        LOG(0) <<"adding signal to entries '" <<name <<"'";
        poses.insert({name, rb2pose(item.second)});
      }
    }
  }  
} //namespace

#else

rai::OptiTrack::OptiTrack() : Thread("OptitrackThread") { NICO }
rai::OptiTrack::~OptiTrack(){ NICO }
void rai::OptiTrack::pull(rai::Configuration& C){ NICO }
void rai::OptiTrack::step(){ NICO }

#endif
