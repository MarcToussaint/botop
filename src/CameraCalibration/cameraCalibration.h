#ifndef CAMERACALIBRATION_H
#define CAMERACALIBRATION_H

#include <Perception/opencv.h>
#include <Core/array.h>
#include <Kin/kin.h>

struct CameraCalibration {

  rai::KinematicWorld K;
  rai::Frame* calibrationVolumeFrame;
  rai::Frame* calibrationMarkerFrame;

  uintA cameraCrop;

  arr q0;

  bool blurring = true;
  arr hsvFilter;

  arr XData;
  arr xData;
  arr xDataRaw;

  arr P;
  arr PInv;
  arr I;
  arr R;
  arr t;


  CameraCalibration();
  CameraCalibration(const rai::KinematicWorld& world, const char* calibrationVolumeFrameName, const char* calibrationMarkerName, const uintA& cameraCrop);
  ~CameraCalibration();

  void updateWorldState(const rai::KinematicWorld& world);
  void set_q0(const arr& q0);

  arr generateTargets(uintA resolution);
  arr generatePathToTarget(const arr& target, bool direction = true);

  void clearData();
  void addDataPoint(const arr& X, const arr& x);
  void saveData(const char* fileName = "cameraCalibration");
  void loadData(const char* fileName);

  arr extractPixelCoordinate(const byteA& _rgb, const floatA& _depth, const arr& hsvFilter, bool blurring = true);
  arr convertPixel2CameraCoordinate(const arr& pixelCoordinate);
  arr extractWorldCoordinate(const rai::KinematicWorld& world);

  void captureDataPoint(const byteA& rgb, const floatA& depth);

  void calibrateCamera();

  arr computeP(const arr& X, const arr& x);
  void decomposeP(arr& K, arr& R, arr& t, const arr& P);

  arr computePInv(const arr& X, const arr& x);

  double computeError(const arr& X, const arr& x, const arr& PInv);
};


// adapted from https://docs.opencv.org/3.4/da/d97/tutorial_threshold_inRange.html
struct CameraCalibrationHSVGui {

  const int max_value_H = 360/2;
  const int max_value = 255;
  static const char* window_detection_name;
  static int low_H, low_S, low_V;
  static int high_H, high_S, high_V;

  CameraCalibrationHSVGui();

  arr getHSV();

  static void on_low_H_thresh_trackbar(int, void *) {
    low_H = cv::min(high_H-1, low_H);
    cv::setTrackbarPos("Low H", window_detection_name, low_H);
  }

  static void on_high_H_thresh_trackbar(int, void *) {
    high_H = cv::max(high_H, low_H+1);
    cv::setTrackbarPos("High H", window_detection_name, high_H);
  }

  static void on_low_S_thresh_trackbar(int, void *) {
    low_S = cv::min(high_S-1, low_S);
    cv::setTrackbarPos("Low S", window_detection_name, low_S);
  }

  static void on_high_S_thresh_trackbar(int, void *) {
    high_S = cv::max(high_S, low_S+1);
    cv::setTrackbarPos("High S", window_detection_name, high_S);
  }

  static void on_low_V_thresh_trackbar(int, void *) {
    low_V = cv::min(high_V-1, low_V);
    cv::setTrackbarPos("Low V", window_detection_name, low_V);
  }

  static void on_high_V_thresh_trackbar(int, void *) {
    high_V = cv::max(high_V, low_V+1);
    cv::setTrackbarPos("High V", window_detection_name, high_V);
  }
};


#endif // CAMERACALIBRATION_H
