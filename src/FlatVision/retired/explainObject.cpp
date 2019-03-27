#include <Perception/opencv.h>
#include <opencv2/opencv.hpp>

#include "explainObject.h"
#include "registrationCalibration.h"

void ExplainObject::compute(byteA& pixelLabels,
                             const byteA& cam_color, const floatA& cam_depth,
                             const byteA& model_segments, const floatA& model_depth) {

  CHECK(label & PL_objects, "is this really an object label?");

  auto reg = registrationCalibration(cam_color, cam_depth, convert<float>(pixelLabels==(byte)PL_unexplained),
                                       byteA(), model_depth, convert<float>(model_segments==(byte)label), verbose);

  for(uint i=0;i<model_segments.N;i++){
    if(model_segments.elem(i)==label) pixelLabels.elem(i)=label;
  }

  if(verbose>0){
    cv::Mat cv_labels = conv_Arr2CvRef(pixelLabels);
    cv::imshow("labels after exObjects", cv_labels);
    cv::waitKey(1);
  }
}
