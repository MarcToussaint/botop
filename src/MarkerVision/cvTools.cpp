#include "cvTools.h"

#include <Perception/opencv.h>

void makeHomogeneousImageCoordinate(arr& u){
  u(0) *= u(2);
  u(1) *= u(2);
  u.append(1.);
}

void decomposeInvProjectionMatrix(arr& K, arr& R, arr& t, const arr& P){
  t = P.col(3); t.resizeCopy(3);
  arr KRt = inverse(P.sub(0,2,0,2));
  lapack_RQ(K, R, KRt);
  //cout <<"K*R" <<K*R <<"\nKR:" <<KRt <<endl;
  transpose(R);
  {
    //transforms the solution to be consistent with convention $K(2,2)\approx 1$
    arr I=eye(3);
    uint num=0;
    for(uint i=0;i<3;i++) if(K(i,i)<0.){ I(i,i) = -1.; num++; }
    if(num){
      if(num!=2) LOG(-1) <<"??? is your cam left handed ???";
      R = R*I;
      K = K*I;
    }
  }
}

#ifdef RAI_OPENCV

arr getHsvBlobImageCoords(byteA& _rgb, floatA& _depth, const arr& hsvFilter, int verbose, arr& histograms){
  cv::Mat rgb = CV(_rgb);
  cv::Mat depth = CV(_depth);

  //blur
  cv::blur(rgb, rgb, cv::Size(3,3));

  //convert to BGR -> RGBPYBIND11_PYTHON_VERSION -> HSV
  cv::Mat hsv, mask;
  cv::cvtColor(rgb, rgb, cv::COLOR_RGB2BGR);
  cv::cvtColor(rgb, hsv, cv::COLOR_BGR2HSV);

  //find red areas
  cv::inRange(hsv,
              cv::Scalar(hsvFilter(0,0), hsvFilter(0,1), hsvFilter(0,2)),
              cv::Scalar(hsvFilter(1,0), hsvFilter(1,1), hsvFilter(1,2)), mask);

  if(verbose>0 && rgb.total()>0 && depth.total()>0){
    cv::imshow("rgb", rgb);
    //cv::imshow("depth", depth); //white=1meters
    cv::imshow("mask", mask);
    cv::waitKey(1);
  }


  //find contours
  std::vector<std::vector<cv::Point> > contours;
  cv::findContours(mask, contours, cv::RETR_LIST, cv::CHAIN_APPROX_SIMPLE);

  if(!contours.size()){
    if(verbose>0) cv::waitKey();
    return {};
  }

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
      if(d>.1 && d<1.){
        depthValues.append(d);
        objX += x;
        objY += y;
        if(!!histograms){
          if(!histograms.N) histograms.resize(3,256).setZero();
          cv::Vec3b c=hsv.at<cv::Vec3b>(y,x);
          histograms(0,c.val[0]) ++;
          histograms(1,c.val[1]) ++;
          histograms(2,c.val[2]) ++;
        }
      }
    }
  }

  arr blobPosition;
  if(depthValues.N>20.){
    objX /= double(depthValues.N);
    objY /= double(depthValues.N);

    // median
    double objDepth = depthValues.median_nonConst();
    // mean
    //    double objDepth = sum(depthValues)/double(depthValues.N);

    if(objDepth>.1 && objDepth < 1.){ //accept new position only when object is in reasonable range
      // image coordinates
      blobPosition= {objX, objY, objDepth};


      // world coordinates
      //cameraFrame->X.applyOnPoint(objCoords); //transforms into world coordinates
    }
  }else{
    LOG(0) <<"small blob size: " <<depthValues.N;
  }

  if(rgb.total()>0 && depth.total()>0){
    if(contours.size()){
      cv::drawContours( rgb, contours, largest, cv::Scalar(0,0,255), 1, 8);
      cv::drawContours( depth, contours, largest, cv::Scalar(0), 1, 8);
    }
    if(verbose>0){
      cv::imshow("rgb", rgb);
      //cv::imshow("depth", depth); //white=1meters
      cv::imshow("mask", mask);
      cv::waitKey(1);
      if(!blobPosition.N) cv::waitKey();
    }
  }

  if(verbose>1) cv::waitKey();

  return blobPosition;
}

//===========================================================================

const char* CameraCalibrationHSVGui::window_detection_name = "HSV Filter Selection";

#define SELF ((CameraCalibrationHSVGui*)self)

void CameraCalibrationHSVGui::on_low_H_thresh_trackbar(int, void* self) {
  SELF->low_H = cv::min(SELF->high_H-1, SELF->low_H);
  cv::setTrackbarPos("Low H", window_detection_name, SELF->low_H);
}

void CameraCalibrationHSVGui::on_high_H_thresh_trackbar(int, void* self) {
  SELF->high_H = cv::max(SELF->high_H, SELF->low_H+1);
  cv::setTrackbarPos("High H", window_detection_name, SELF->high_H);
}

void CameraCalibrationHSVGui::on_low_S_thresh_trackbar(int, void* self) {
  SELF->low_S = cv::min(SELF->high_S-1, SELF->low_S);
  cv::setTrackbarPos("Low S", window_detection_name, SELF->low_S);
}

void CameraCalibrationHSVGui::on_high_S_thresh_trackbar(int, void* self) {
  SELF->high_S = cv::max(SELF->high_S, SELF->low_S+1);
  cv::setTrackbarPos("High S", window_detection_name, SELF->high_S);
}

void CameraCalibrationHSVGui::on_low_V_thresh_trackbar(int, void* self) {
  SELF->low_V = cv::min(SELF->high_V-1, SELF->low_V);
  cv::setTrackbarPos("Low V", window_detection_name, SELF->low_V);
}

void CameraCalibrationHSVGui::on_high_V_thresh_trackbar(int, void* self) {
  SELF->high_V = cv::max(SELF->high_V, SELF->low_V+1);
  cv::setTrackbarPos("High V", window_detection_name, SELF->high_V);
}


CameraCalibrationHSVGui::CameraCalibrationHSVGui() {
  cv::namedWindow(window_detection_name);
  cv::createTrackbar("Low H", window_detection_name, &low_H, high_H, on_low_H_thresh_trackbar, this);
  cv::createTrackbar("High H", window_detection_name, &high_H, high_H, on_high_H_thresh_trackbar, this);
  cv::createTrackbar("Low S", window_detection_name, &low_S, high_S, on_low_S_thresh_trackbar, this);
  cv::createTrackbar("High S", window_detection_name, &high_S, high_S, on_high_S_thresh_trackbar, this);
  cv::createTrackbar("Low V", window_detection_name, &low_V, high_V, on_low_V_thresh_trackbar, this);
  cv::createTrackbar("High V", window_detection_name, &high_V, high_V, on_high_V_thresh_trackbar, this);
}


arr CameraCalibrationHSVGui::getHSV() {
  cv::waitKey(1);
  arr hsv = arr{(double)low_H, (double)low_S, (double)low_V, (double)high_H, (double)high_S, (double)high_V};
  return hsv.reshape(2,3);
}

#else

arr getHsvBlobImageCoords(byteA& _rgb, floatA& _depth, const arr& hsvFilter, int verbose, arr& histograms){ NICO }
void CameraCalibrationHSVGui::on_low_H_thresh_trackbar(int, void* self) { NICO }
void CameraCalibrationHSVGui::on_high_H_thresh_trackbar(int, void* self) { NICO }
void CameraCalibrationHSVGui::on_low_S_thresh_trackbar(int, void* self) { NICO }
void CameraCalibrationHSVGui::on_high_S_thresh_trackbar(int, void* self) { NICO }
void CameraCalibrationHSVGui::on_low_V_thresh_trackbar(int, void* self) { NICO }
void CameraCalibrationHSVGui::on_high_V_thresh_trackbar(int, void* self) { NICO }
CameraCalibrationHSVGui::CameraCalibrationHSVGui() { NICO }
arr CameraCalibrationHSVGui::getHSV() { NICO }

#endif //RAI_OPENCV
