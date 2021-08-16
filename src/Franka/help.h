#pragma once

#include <Kin/kin.h>
#include <Kin/frame.h>
#include <FlatVision/helpers.h>

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

