#include <Kin/kin.h>

struct KomoArucoTracker{
  FrameL cams;
  arrA Fxycxy, Distortion;

  rai::Frame *obj;
  FrameL arucos;

  KomoArucoTracker(rai::Configuration& C, str obj_name);

};
