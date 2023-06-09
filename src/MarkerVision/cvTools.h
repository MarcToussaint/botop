#include <Core/array.h>

arr getHsvBlobImageCoords(byteA& _rgb, floatA& _depth, const arr& hsvFilter);

void makeHomogeneousImageCoordinate(arr& u, uint imgHeight){
  u(1) = double(imgHeight-1)-u(1);
  u(2) *= -1.;
  u(0) *= u(2);
  u(1) *= u(2);
  u.append(1.);
}

struct CameraCalibrationHSVGui {
  static const char* window_detection_name;
  int low_H=0, low_S=0, low_V=0;
  int high_H=180, high_S=255, high_V=255;

  CameraCalibrationHSVGui();

  arr getHSV();

protected:
  static void on_low_H_thresh_trackbar(int, void* self);
  static void on_high_H_thresh_trackbar(int, void* self);
  static void on_low_S_thresh_trackbar(int, void* self);
  static void on_high_S_thresh_trackbar(int, void* self);
  static void on_low_V_thresh_trackbar(int, void* self);
  static void on_high_V_thresh_trackbar(int, void* self);
};

