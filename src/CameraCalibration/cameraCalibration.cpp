
#include <Perception/opencv.h>
#include "cameraCalibration.h"


#include <Kin/frame.h>
#include <KOMO/komo.h>
#include <Kin/TM_qItself.h>


CameraCalibration::CameraCalibration() {

}

CameraCalibration::CameraCalibration(const rai::KinematicWorld& world, const char* calibrationVolumeFrameName, const char* calibrationMarkerName, const uintA &cameraCrop)
  : K(world) {
  calibrationVolumeFrame = K.getFrameByName(calibrationVolumeFrameName);
  calibrationMarkerFrame = K.getFrameByName(calibrationMarkerName);
  this->cameraCrop = cameraCrop;
  clearData();
}

CameraCalibration::~CameraCalibration() {}

void CameraCalibration::updateWorldState(const rai::KinematicWorld& world) {
  K.setJointState(world.getJointState());
}

void CameraCalibration::set_q0(const arr& _q0) {
  q0 = _q0;
}


arr CameraCalibration::generateTargets(uintA resolution) {
  if(resolution.N == 1) {
    resolution = {resolution(0), resolution(0), resolution(0)};
  }
  resolution(0) -= 1;
  resolution(1) -= 1;
  resolution(2) -= 1;

  arr size = calibrationVolumeFrame->shape->size;
  arr posGrid = grid({-size(0)/2., -size(1)/2., -size(2)/2.}, {size(0)/2., size(1)/2., size(2)/2.}, resolution);
  arr targetPositions;
  for(uint i = 0; i < posGrid.d0; i++) {
    targetPositions.append(~(calibrationVolumeFrame->X * rai::Vector(posGrid[i])).getArr());
  }
  return targetPositions;
}

arr CameraCalibration::generatePathToTarget(const arr& target, bool direction) {
  KOMO komo(K, true);
  komo.setTiming(1., 20, 5., 2);
  komo.setSquaredQuaternionNorms();
  komo.setSquaredQAccVelHoming(0.0, -1.0, 1.0, 0.0, 0.01);
  komo.add_collision(true);
  komo.add_jointLimitsNew();
//  komo.add_jointLimits(true, 0.01);

  komo.addObjective({0.9, -1.0}, OT_sos, FS_position, {calibrationMarkerFrame->name}, ARR(100.0), target);
  if(direction) {
    komo.addObjective({0.9, -1.0}, OT_sos, FS_vectorZ, {calibrationMarkerFrame->name}, ARR(100.0), ARR(1.0, 0.0, 0.0));
  } else {
    komo.addObjective({0.9, -1.0}, OT_sos, FS_vectorZ, {calibrationMarkerFrame->name}, ARR(100.0), ARR(-1.0, 0.0, 0.0));
  }
  komo.addObjective({0.9, -1.0}, OT_sos, FS_vectorX, {calibrationMarkerFrame->name}, ARR(100.0), ARR(0.0, 1.0, 0.0));
  if(q0.N) {
    komo.addObjective(0.0, -1.0, make_shared<TM_qItself>(), OT_sos, q0, .02, 0);
  }
  komo.optimize();
  double constraints = komo.getConstraintViolations();
//  double costs = komo.getCosts();

//  komo.displayTrajectory(-0.1, true, false);

  if(constraints < 0.05) {
    return komo.getPath();
  } else {
    return NoArr;
  }
}

void CameraCalibration::clearData() {
  XData.clear();
  xData.clear();
  xDataRaw.clear();
}

void CameraCalibration::addDataPoint(const arr& X, const arr& x) {
  XData.append(~X);
  xData.append(~convertPixel2CameraCoordinate(x));
  xDataRaw.append(~x);
}

void CameraCalibration::saveData(const char *fileName) {
  FILE(STRING(fileName << "_X.data")) << XData;
  FILE(STRING(fileName << "_x.data")) << xData;
  FILE(STRING(fileName << "_xRaw.data")) << xDataRaw;
}

void CameraCalibration::loadData(const char *fileName) {
  clearData();
  XData << FILE(STRING(fileName << "_X.data"));
  xData << FILE(STRING(fileName << "_x.data"));
//  xDataRaw << FILE(STRING(fileName << "_xRaw.data"));
  XData.reshape(XData.d0/3, 3);
  xData.reshape(xData.d0/3, 3);
//  xDataRaw.reshape(xDataRaw.d0/3, 3);
}

arr CameraCalibration::extractWorldCoordinate(const rai::KinematicWorld& world) {
  return world.getFrameByName(calibrationMarkerFrame->name)->getPosition();
}

void CameraCalibration::captureDataPoint(const byteA& rgb, const floatA& depth) {
  arr pixelCoords = extractPixelCoordinate(rgb, depth, hsvFilter, blurring);
  if(!pixelCoords.N) return;
  arr X = extractWorldCoordinate(K);
  addDataPoint(X, pixelCoords);
}

void CameraCalibration::calibrateCamera() {
  PInv = computePInv(XData, xData);
  P = computeP(XData, xData);
  decomposeP(I, R, t, P);
}

arr CameraCalibration::computeP(const arr& X, const arr& x) {
  arr XHom(X.d0, 4);
  for(uint i = 0; i < XHom.d0; i++) {
    arr tmp = X[i]();
    tmp.append(1.0);
    XHom[i] = tmp;
  }
  return ~x*XHom*inverse_SymPosDef(~XHom*XHom);
}

void CameraCalibration::decomposeP(arr& K, arr& R, arr& t, const arr& P) {
  arr KR(3, 3), KRt(3, 1);
  P.getMatrixBlock(KR, 0, 0);
  P.getMatrixBlock(KRt, 0, 3);
  lapack_RQ(K, R, KR);
  t = -inverse(KR)*KRt;
  t.reshape(3);
}

arr CameraCalibration::computePInv(const arr &X, const arr &x) {
  arr xHom(x.d0, 4);
  for(uint i = 0; i < xHom.d0; i++) {
    arr tmp = x[i]();
    tmp.append(1.0);
    xHom[i] = tmp;
  }
  return ~X*xHom*inverse_SymPosDef(~xHom*xHom);
}

double CameraCalibration::computeError(const arr &X, const arr &x, const arr &PInv) {
  arr xHom(x.d0, 4);
  for(uint i = 0; i < xHom.d0; i++) {
    arr tmp = x[i]();
    tmp.append(1.0);
    xHom[i] = tmp;
  }
  return sqrt(sumOfSqr(xHom*~PInv - X)/double(X.d0));
}



arr CameraCalibration::extractPixelCoordinate(const byteA& _rgb, const floatA& _depth, const arr& hsvFilter, bool blurring) {
  uint cL = cameraCrop(0), cR = cameraCrop(1), cT = cameraCrop(2), cB = cameraCrop(3);
  byteA rgbTmp = _rgb.sub(cT,-cB,cL,-cR,0,-1);
  floatA depthTmp = _depth.sub(cT,-cB,cL,-cR);

  cv::Mat rgb = CV(rgbTmp).clone();
  cv::Mat depth = CV(depthTmp).clone();

  arr cameraCoords;

  if(blurring) {
    cv::blur(rgb, rgb, cv::Size(3,3));
  }

  // convert to hsv scheme
  cv::Mat hsv;
//  cv::cvtColor(rgb, hsv, cv::COLOR_BGR2RGB);
  cv::cvtColor(rgb, hsv, cv::COLOR_RGB2HSV);

  // extract mask
  cv::Mat mask;
  cv::inRange(hsv,
              cv::Scalar(hsvFilter(0,0), hsvFilter(0,1), hsvFilter(0,2)),
              cv::Scalar(hsvFilter(1,0), hsvFilter(1,1), hsvFilter(1,2)),
              mask);

  /*
  cv::Mat maskedRGB;
  rgb.copyTo(maskedRGB, mask);
  cv::imshow("Object Detection", maskedRGB);
  */

  std::vector<std::vector<cv::Point> > contours;
  cv::findContours(mask, contours, cv::RETR_LIST, cv::CHAIN_APPROX_SIMPLE);

  if(!contours.size()) return arr();

  // find largest ball
  arr sizes;
  sizes.resize(contours.size());
  for(uint i=0; i < contours.size(); i++) sizes(i) = cv::contourArea(cv::Mat(contours[i]));

  uint largest = sizes.argmax();
  double size = sizes(largest);

  // draw the contour interior into the mask
  mask = cv::Scalar(0);
  if(size > 10.) {
    cv::drawContours(mask, contours, largest, cv::Scalar(128), CV_FILLED);

    if(false) {
      cv::imshow("mask", mask);
      cv::waitKey(20);
    }

    // grab the depth values and mean x,y coordinates
    floatA depthValues;
    double objX=0.,objY=0.;
    for(int y=0;y<mask.rows;y++) for(int x=0;x<mask.cols;x++){
      if(mask.at<byte>(y,x)){
        float d = depth.at<float>(y,x);
        if(d > .1 && d < 1.2){
          depthValues.append(d);
          objX += x;
          objY += y;
        }
      }
    }

    if(depthValues.N){
      objX /= double(depthValues.N);
      objY /= double(depthValues.N);

      /*
      floatA depthValuesCopy = depthValues;
      double objDepthMedian = depthValuesCopy.median_nonConst();

      double maxDistFromMedian = 0.016;
      double objDepth = 10.0;
      for(uint i = 0; i < depthValues.N; i++) {
        double currentDepth = depthValues(i);
        if(currentDepth < objDepth && fabs(currentDepth-objDepthMedian) < maxDistFromMedian) {
          objDepth = currentDepth;
        }
      }

      cout << objDepth << endl;
      */

      // median
      double objDepth = depthValues.median_nonConst();
      // mean
      //double objDepth = sum(depthValues)/double(depthValues.N);

      cameraCoords = {objX, objY, objDepth};

      cv::drawContours( rgb, contours, largest, cv::Scalar(0, 255, 0), 3, 8);
      //cv::drawContours( depth, contours, largest, cv::Scalar(0), 1, 8);

      if(true) {
        cv::imshow("Object Detection", rgb);
        cv::waitKey(20);
      }
    }
  }
  return cameraCoords;
}

arr CameraCalibration::convertPixel2CameraCoordinate(const arr& pixelCoordinate) {
  arr cameraCoordinate = pixelCoordinate;
  cameraCoordinate(0) *= cameraCoordinate(2);
  cameraCoordinate(1) *= cameraCoordinate(2);
  return cameraCoordinate;
}




//=============================================================================

const char* CameraCalibrationHSVGui::window_detection_name = "Object Detection";
int CameraCalibrationHSVGui::low_H = 0;
int CameraCalibrationHSVGui::low_S = 0;
int CameraCalibrationHSVGui::low_V = 0;
int CameraCalibrationHSVGui::high_H = 180;
int CameraCalibrationHSVGui::high_S = 255;
int CameraCalibrationHSVGui::high_V = 255;

CameraCalibrationHSVGui::CameraCalibrationHSVGui() {
  cv::namedWindow(window_detection_name);
  cv::createTrackbar("Low H", window_detection_name, &low_H, max_value_H, on_low_H_thresh_trackbar);
  cv::createTrackbar("High H", window_detection_name, &high_H, max_value_H, on_high_H_thresh_trackbar);
  cv::createTrackbar("Low S", window_detection_name, &low_S, max_value, on_low_S_thresh_trackbar);
  cv::createTrackbar("High S", window_detection_name, &high_S, max_value, on_high_S_thresh_trackbar);
  cv::createTrackbar("Low V", window_detection_name, &low_V, max_value, on_low_V_thresh_trackbar);
  cv::createTrackbar("High V", window_detection_name, &high_V, max_value, on_high_V_thresh_trackbar);
}

arr CameraCalibrationHSVGui::getHSV() {
  arr hsv;
  hsv.append(~ARR(low_H, low_S, low_V));
  hsv.append(~ARR(high_H, high_S, high_V));
  return hsv;
}
