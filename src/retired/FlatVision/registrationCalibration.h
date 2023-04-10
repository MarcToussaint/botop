#include <Core/array.h>

struct RegReturn{
  arr calib;
  double depthError;
  double matchError;
};

RegReturn registrationCalibration(const byteA& cam_color, const floatA& cam_depth, const floatA& cam_mask,
                                  const byteA& model_color, const floatA& model_depth, const floatA& model_mask,
                                  bool useDepth=true,
                                  int padding=20,
                                  int verbose=1);
