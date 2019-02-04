#include <Kin/kin.h>

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
        frameMaskMap(f->ID)=0x80;
      }
      if(f->getUpwardLink()->name.startsWith("R_")){
        frameMaskMap(f->ID)=0x8f;
      }
      cout <<f->ID <<' ' <<f->name <<' ' <<frameMaskMap(f->ID) <<endl;
    }
  }
  return frameMaskMap;
}
