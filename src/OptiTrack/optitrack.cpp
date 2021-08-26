#include "optitrack.h"

#include "motioncapture_optitrack.h"

#include <Kin/frame.h>

OptiTrack::OptiTrack(rai::Configuration& _C) : C(_C) {
    mocap = libmotioncapture::MotionCapture::connect("optitrack",
                                                     rai::getParameter<rai::String>("optitrack/host").p);
}

OptiTrack::~OptiTrack(){
    delete mocap;
}

void OptiTrack::step(){
    // Get a frame
    mocap->waitForNextFrame();
    uint64_t timeStamp = mocap->timeStamp();

    std::map<std::string, libmotioncapture::RigidBody> rigidBodies = mocap->rigidBodies();

    //-- update configuration
    for (auto const& item: rigidBodies) {
        //get (or create) frame
        const char* name = item.first.c_str();
        rai::Frame *f = C.getFrame(name, false);
        if(!f){//create a new marker frame!
            LOG(0) <<"creating new frame '" <<name <<"'";
            f = C.addFrame(name);
            f->setShape(rai::ST_marker, {.3});
            f->setColor({.8, .8, .2});
        }

        //set pose of frame
        const auto& rigidBody = item.second;
        if (rigidBody.occluded() == false) {
            const Eigen::Vector3f& position = rigidBody.position();
            const Eigen::Quaternionf& rotation = rigidBody.rotation();
            {
                auto X = f->set_X();
                X->pos.set(position(0), position(1), position(2));
                X->rot.set(rotation.w(), rotation.vec()(0), rotation.vec()(1), rotation.vec()(2));
            }
        }
    }

    C.watch();
}
