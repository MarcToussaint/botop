#pragma once

#include <Kin/kin.h>
#include <Kin/frame.h>

enum PixelLabel : byte { PL_unexplained=0x00, PL_nosignal=0x01, PL_noise=0x02, PL_toofar=0x03,
                         PL_background=0x10,
                         PL_robot=0x20,
                         PL_novelPercepts=0x40,
                         PL_objects=0x80,
                         PL_closeToObject=0xc0,
                         PL_max=0xff};

inline uintA franka_getJointIndices(const rai::Configuration& C, char L_or_R){
  CHECK(C._state_indexedJoints_areGood , "need to ensure_q (indexed joints) before!");
  StringA jointNames;
  for(uint i=1;i<=7;i++){
    jointNames.append(STRING(L_or_R <<"_panda_joint" <<i));
  }
  FrameL joints = C.getFrames(jointNames);
  uintA qIndices(7);
  for(uint i=0;i<joints.N;i++) qIndices(i) = joints(i)->joint->qIndex;
  return qIndices;
}


inline byteA franka_getFrameMaskMap(const rai::Configuration& K){
  byteA frameMaskMap(K.frames.N); //map each frame in the image to a mask byte (here just 0 or 0xff)
  frameMaskMap.setZero();
  for(rai::Frame *f:K.frames){
    if(f->shape){
      if(f->getUpwardLink()->name.startsWith("l_")){
        frameMaskMap(f->ID)=PL_robot;
      }
      if(f->getUpwardLink()->name.startsWith("r_")){
        frameMaskMap(f->ID)=PL_robot+1;
      }
      if(f->getUpwardLink()->name.startsWith("perc_")){
        int id;
        f->getUpwardLink()->name >>"perc_" >>id;
        frameMaskMap(f->ID)=PixelLabel(PL_robot+id);
      }
//      cout <<f->ID <<' ' <<f->name <<' ' <<frameMaskMap(f->ID) <<endl;
    }
  }
  return frameMaskMap;
}

inline void franka_setFrameMaskMapLabels(rai::Configuration& K){
  for(rai::Frame *f:K.frames){
    if(f->shape){
      if(f->getUpwardLink()->name.startsWith("l_")){
        f->ats->getNew<int>("label") = PL_robot;
      }
      if(f->getUpwardLink()->name.startsWith("r_")){
        f->ats->getNew<int>("label") = PL_robot+1;
      }
    }
  }
}

