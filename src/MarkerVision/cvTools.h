#include <Core/array.h>

void makeHomogeneousImageCoordinate(arr& u);

void decomposeInvProjectionMatrix(arr& K, arr& R, arr& t, const arr& P);

arr getHsvBlobImageCoords(byteA& _rgb, floatA& _depth, const arr& hsvFilter, int verbose=0, arr& histograms=NoArr);

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

