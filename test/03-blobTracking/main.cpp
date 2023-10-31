#include <Perception/opencv.h> //always include this first! OpenCV headers define stupid macros
#include <Perception/opencvCamera.h>
#include <Geo/depth2PointCloud.h>

#include <Kin/frame.h>
#include <Kin/viewer.h>

#include <Core/array.h>
#include <Gui/opengl.h>
#include <RealSense/RealSenseThread.h>
#include <Gui/viewer.h>
#include <Core/thread.h>

arr getHsvBlobPosition(cv::Mat& rgb, cv::Mat& depth, const arr& hsvFilter, const arr& fxycxy){
  //blur
  cv::blur(rgb, rgb, cv::Size(3,3));

  //convert to BGR -> RGB -> HSV
  cv::Mat hsv, mask;
  cv::cvtColor(rgb, rgb, cv::COLOR_RGB2BGR);
  cv::cvtColor(rgb, hsv, cv::COLOR_BGR2HSV);

  //find red areas
  cv::inRange(hsv,
              cv::Scalar(hsvFilter(0,0), hsvFilter(0,1), hsvFilter(0,2)),
              cv::Scalar(hsvFilter(1,0), hsvFilter(1,1), hsvFilter(1,2)), mask);

  //find contours
  std::vector<std::vector<cv::Point> > contours;
  cv::findContours(mask, contours, cv::RETR_LIST, cv::CHAIN_APPROX_SIMPLE);

  //get largest contour
  arr sizes(contours.size());
  for(uint i=0; i<contours.size(); i++) sizes(i) = cv::contourArea(cv::Mat(contours[i]));
  uint largest = argmax(sizes);

  //draw the contour interior into the mask
  mask = cv::Scalar(0);
  cv::drawContours(mask, contours, largest, cv::Scalar(128), cv::FILLED);

  // grab the depth values and mean x,y coordinates
  floatA depthValues;
  double objX=0.,objY=0.;
  for(int y=0;y<mask.rows;y++) for(int x=0;x<mask.cols;x++){
    if(mask.at<byte>(y,x)){
      float d = depth.at<float>(y,x);
      if(d>.1 && d<2.){
        depthValues.append(d);
        objX += x;
        objY += y;
      }
    }
  }

  arr blobPosition;
  if(depthValues.N>200.){
    objX /= double(depthValues.N);
    objY /= double(depthValues.N);

    // median
    double objDepth = depthValues.median_nonConst();
    // mean
    //double objDepth = sum(depthValues)/double(depthValues.N);

    if(objDepth>.1 && objDepth < 1.5){ //accept new position only when object is in reasonable range
      // image coordinates
      blobPosition= {objX, objY, objDepth};

      // camera coordinates
      depthData2point(blobPosition, fxycxy); //transforms the point to camera xyz coordinates

      // world coordinates
      //cameraFrame->X.applyOnPoint(objCoords); //transforms into world coordinates
    }
  }

  if(rgb.total()>0 && depth.total()>0){
    if(contours.size()){
      cv::drawContours( rgb, contours, largest, cv::Scalar(255,0,0), 2, 8);
      cv::drawContours( depth, contours, largest, cv::Scalar(0), 2, 8);
    }
    cv::imshow("rgb", rgb);
    cv::imshow("depth", 0.5*depth); //white=2meters
    cv::imshow("mask", mask);
  }

  return blobPosition;
}

void tracking2(){
  // configuration - only for display
  rai::Configuration C;
  rai::Frame *cameraFrame = C.addFrame("camera");
  rai::Frame *obj = C.addFrame("obj", "camera");
  cameraFrame
      ->setPosition({0,0,0.8})
      .setQuaternion({1,1,0,0})
      .setShape(rai::ST_marker, {.2});
  obj->setShape(rai::ST_sphere, {.05});
  C.view();

  // launch camera
  RealSenseThread RS("cam");

  // grab the camera intrinsics
  arr fxycxy = RS.fxycxy;

  // set hsv filter parameters
  arr hsvFilter = rai::getParameter<arr>("hsvFilter").reshape(2,3);

  // looping
  for(uint i=0;i<10000;i++){
    RS.depth.waitForNextRevision();

    // grap copies of rgb and depth
    cv::Mat rgb = CV(RS.image.get()).clone();
    cv::Mat depth = CV(RS.depth.get()).clone();

    if(rgb.rows != depth.rows) continue;

    // blur
    arr pos = getHsvBlobPosition(rgb, depth, hsvFilter, fxycxy);
    cout <<"blob position: " <<pos <<endl;

    if(pos.N){
      obj->setRelativePosition(pos); // in world coordinate
      C.view(false);
    }

    int key = cv::waitKey(1);
    if((key&0xff)=='q') break;
    if((key&0xff)=='r'){
      rai::initParameters(0,0,true);
      hsvFilter = rai::getParameter<arr>("hsvFilter").reshape(2,3);
    }
  }
}

int main(int argc,char **argv){
  rai::initCmdLine(argc,argv);

  tracking2();

  LOG(0) <<" === bye bye ===\n used parameters:\n" <<rai::params() <<'\n';

  return 0;
}
