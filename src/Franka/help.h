#include <Kin/kin.h>
#include <ExplainPixels/labels.h>

inline uintA franka_getJointIndices(const rai::KinematicWorld& K, char L_or_R){
  StringA joints;
  for(uint i=1;i<=7;i++){
    joints.append(STRING(L_or_R <<"_panda_joint" <<i));
  }
  return  K.getQindicesByNames(joints);
}


inline byteA franka_getFrameMaskMap(const rai::KinematicWorld& K){
  byteA frameMaskMap(K.frames.N); //map each frame in the image to a mask byte (here just 0 or 0xff)
  frameMaskMap.setZero();
  for(rai::Frame *f:K.frames){
    if(f->shape && f->shape->visual){
      if(f->getUpwardLink()->name.startsWith("L_")){
        frameMaskMap(f->ID)=PL_robot;
      }
      if(f->getUpwardLink()->name.startsWith("R_")){
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

inline void franka_setFrameMaskMapLabels(rai::KinematicWorld& K){
  for(rai::Frame *f:K.frames){
    if(f->shape && f->shape->visual){
      if(f->getUpwardLink()->name.startsWith("L_")){
        f->ats.getNew<int>("label") = PL_robot;
      }
      if(f->getUpwardLink()->name.startsWith("R_")){
        f->ats.getNew<int>("label") = PL_robot+1;
      }
    }
  }
}

