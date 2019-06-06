#include <Perception/opencv.h>
#include <opencv2/opencv.hpp>
#include <opencv_reg/mapaffine.hpp>
#include <opencv_reg/mappergradshift.hpp>
#include <opencv_reg/mappergradeuclid.hpp>
#include <opencv_reg/mappergradsimilar.hpp>
#include <opencv_reg/mappergradaffine.hpp>
#include <opencv_reg/mapperpyramid.hpp>

#include "registrationCalibration.h"
#include "helpers.h"

#include <Gui/opengl.h>
#include <Algo/MLcourse.h>

RegReturn registrationCalibration(const byteA& cam_color,
                                  const floatA& _cam_depth,
                                  const floatA& cam_mask,
                                  const byteA& model_color,
                                  const floatA& model_depth,
                                  const floatA& model_mask,
                                  bool useDepth,
                                  int padding,
                                  int verbose){


  floatA cam_depth = _cam_depth;
  for(uint i=0;i<cam_depth.N;i++){
    if(cam_depth.elem(i)<.001) cam_depth.elem(i) = model_depth.elem(i);
  }

  arr calib = zeros(6);

  //-- from the mask, count pixels
  floatA mask = model_mask;
  if(cam_mask.N) mask = mask%cam_mask;
  uint maskN=0;
  for(float& m:mask) if(m>0.f) maskN++;
  if(maskN<10){
    cout <<"REGIS: modelMask*cam_mask<10" <<endl;
    return{ calib, -1., -1. };
  }

  //-- collect all pixels in a dataset for redisual depth regression
  arr X(maskN, 3);
  arr Y(maskN);
  double avgDepth=0.;
  uint k=0;
  int H=mask.d0, W=mask.d1;
  for(int y=0;y<H;y++) for(int x=0;x<W;x++){
    int i=y*W+x;
    if(mask.elem(i)>0.f){
      X(k,0) = double(x)/W-.5;
      X(k,1) = double(y)/H-.5;
      X(k,2) = 1.;
      Y(k) = model_depth.elem(i) - cam_depth.elem(i); //target: error between model and cam
      avgDepth += model_depth.elem(i);
      k++;
    }
  }
  CHECK_EQ(k, maskN, "");
  avgDepth /= double(maskN);
  double depthError = sqrt(sumOfSqr(Y)/double(maskN));
//  cout <<"depthError=" <<depthError <<endl;

  //-- ridge regression of residual
  arr beta = ridgeRegression(X, Y, 1e-10);
  calib(2) = beta.elem(2); //bias is translation in z
  calib(3) = beta.elem(1); //tilt in y is rotation about x
  calib(4) = -beta.elem(0); //tilt in x is rotation about y

  //-- from the mask get the rect and enlarge it a bit
  intA rect = nonZeroRect(mask, .5);
  extendRect(rect, padding, mask.d0, mask.d1);
  cv::Rect cv_rect(cv::Point(rect(0), rect(1)), cv::Point(rect(2), rect(3)));

  //-- get the cropped depth images in model and cam
  cv::Mat crop_camImg,crop_modImg;
  if(useDepth){
    crop_camImg = CV(cam_depth)(cv_rect);
    crop_modImg = CV(model_depth)(cv_rect);
  }else{
    crop_camImg = CV(cam_color)(cv_rect);
    crop_modImg = CV(model_color)(cv_rect);
  }
  cv::Mat crop_modMask = CV(model_mask)(cv_rect);
  cv::Mat crop_camMask;
  if(cam_mask.N) crop_camMask = CV(cam_mask)(cv_rect);

  //-- compute the euclidean (shift+rotation) registration
  cv::Ptr<cv::reg::Mapper> R(dynamic_cast<cv::reg::Mapper*>(new cv::reg::MapperGradShift));
  cv::reg::MapperPyramid RR(R);
  RR.numLev_=6;
  RR.numIterPerScale_=2;
  RR.stepSize_=.5;
  double matchError=-123.;
  cv::Ptr<cv::reg::Map> map = RR.calculate(crop_modImg, crop_modMask, crop_camImg, crop_camMask, cv::Ptr<cv::reg::Map>(), &matchError);

  //-- grab the 3 parameters dx, dy, phi
  cv::reg::MapShift* smap = dynamic_cast<cv::reg::MapShift*>(&*map);
  if(smap){
    calib(0) = smap->getShift()(0); //translation in x
    calib(1) = smap->getShift()(1); //translation in y
    calib(5) = 0.; //rotation about z
  }else{
    cv::reg::MapAffine* amap = dynamic_cast<cv::reg::MapAffine*>(&*map);
    calib(0) = amap->getShift()(0); //translation in x
    calib(1) = amap->getShift()(1); //translation in y
    calib(5) = amap->getLinTr()(2); //rotation about z
  }
//  static int i=0;
//  cout <<i++ <<" TRANFORMATION " <<calib <<endl;

  if(verbose){ //display things
    std::vector<std::vector<cv::Point> > cv_contours;

    cv::Mat cv_mask;
    CV(mask).convertTo(cv_mask, CV_8UC1, 255.);
    cv::Mat cv_mask2;
    cv::threshold(cv_mask, cv_mask2, 128, 255, cv::THRESH_BINARY);
    map->inverseMap()->inverseWarp(cv_mask2, cv_mask2); //display the fitted contour instead of the predicted
    cv::findContours(cv_mask2, cv_contours, cv::RETR_LIST, cv::CHAIN_APPROX_SIMPLE);


    cv::Mat cv_col2 = CV(cam_color).clone();
    cv::Scalar colo(255,0,0);
    cv::rectangle( cv_col2, cv_rect.tl(), cv_rect.br(), colo, 2, 8, 0 );
    for(uint i=0; i<cv_contours.size(); i++){
      cv::drawContours(cv_col2, cv_contours, i, colo, 1, 8);
    }

//    cv_col2(cv_rect) = tmp;

    cv::Mat cv_depth2 = CV(cam_depth).clone();
    cv::Scalar colo2(0);
    cv::rectangle( cv_depth2, cv_rect.tl(), cv_rect.br(), colo2, 2, 8, 0 );
    for(uint i=0; i<cv_contours.size(); i++){
      cv::drawContours(cv_depth2, cv_contours, i, colo2, 1, 8);
    }

    cv::cvtColor(cv_col2, cv_col2, cv::COLOR_RGB2BGR);

    cv::imshow("color", cv_col2);
    //-- rescale the depth values to inverval ???
//    cv::imshow("mask", cv_mask);
//    cv::imshow("color", cv_col);
    cv::imshow("depth", cv_depth2);
//    cv::imshow("mdepth1", cv_mdepth);
//    cv::imshow("mdepth2", cv_reg_mdepth);

    cv::waitKey(1);
  }

  return {calib, depthError, matchError};
}
