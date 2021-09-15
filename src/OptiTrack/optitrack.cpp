#include "optitrack.h"

#include "motioncapture_optitrack.h"

#include <Kin/frame.h>

namespace rai{

  OptiTrack::OptiTrack() {
    mocap = libmotioncapture::MotionCapture::connect("optitrack", rai::getParameter<rai::String>("optitrack/host", "130.149.82.29").p);
  }

  OptiTrack::~OptiTrack(){
    delete mocap;
  }

  void OptiTrack::pull(rai::Configuration& C){
    // Get a frame
    mocap->waitForNextFrame();
    uint64_t timeStamp = mocap->timeStamp();

    std::map<std::string, libmotioncapture::RigidBody> rigidBodies = mocap->rigidBodies();

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

    //loop through all ot signals
    for (auto const& item: rigidBodies) {
      //get (or create) frame for that body
      const char* name = item.first.c_str();
      rai::Frame *f = C.getFrame(name, false);
      if(!f){//create a new marker frame!
        LOG(0) <<"creating new frame '" <<name <<"'";
        f = C.addFrame(name);
        f->setParent(base);
        f->setShape(rai::ST_marker, {.1});
        f->setColor({.8, .8, .2, .5});
      }

      //set pose of frame
      const auto& rigidBody = item.second;
      if (rigidBody.occluded() == false) {
        const Eigen::Vector3f& position = rigidBody.position();
        const Eigen::Quaternionf& rotation = rigidBody.rotation();
        {
          auto Q = f->set_Q();
          Q->pos.set(position(0), position(1), position(2));
          Q->rot.set(rotation.w(), rotation.vec()(0), rotation.vec()(1), rotation.vec()(2));
        }
      }
    }
  }

} //namespace
