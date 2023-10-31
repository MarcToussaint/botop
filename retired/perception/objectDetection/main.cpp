#include <Perception/opencv.h>
#include <Core/array.h>
#include <opencv2/opencv.hpp>
#include <RealSense/RealSenseThread.h>
#include <BackgroundSubtraction/BackgroundSubtractionThread.h>
#include <BackgroundSubtraction/cv_depth_backgroundSubstraction.h>
#include <Operate/op.h>
#include <Gui/viewer.h>
#include <Kin/frame.h>

void testBGSThread() {
  RealSenseThread RS;
//  BackgroundSubtractionThread bgs(RS.color);
  LOG(0) <<"recording background statistics..";
//  bgs.learnBackgroundModel(100);
  LOG(0) <<"..done";

  RS.depth.waitForNextRevision();
  Depth2PointCloud cvt2pcl(RS.depth, RS.depth_fxycxy(0), RS.depth_fxycxy(1), RS.depth_fxycxy(2), RS.depth_fxycxy(3));
  PointCloudViewer pcview(cvt2pcl.points, RS.color, 1.);

  Var<floatA> cameraDepth(RS.depth);
  Var<byteA> cameraColor(RS.color);
  CV_BackgroundSubstraction BS;
  BS.threshold = .01;
  BS.verbose = 1;
  auto depthpipeline = loop([&cameraDepth, &cameraColor, &BS](){
    cameraDepth.waitForNextRevision();
    floatA depth = cameraDepth.get();
    byteA color = cameraColor.get();
    byteA mask;
    mask.resize(depth.d0, depth.d1).setZero();
    auto lock = cvMutex();
    BS.compute(color, depth, mask);
    return 0;
  });

  rai::wait();
}

int main(int argc,char **argv){
  rai::initCmdLine(argc, argv);
  testBGSThread();
  return 0;
}

