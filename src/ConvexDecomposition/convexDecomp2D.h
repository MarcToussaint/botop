#ifndef ACD2D_H
#define ACD2D_H

#include <Core/array.h>
#include <opencv2/opencv.hpp>

struct PolygonUtils {
  static bool checkIntersection(const cv::Point& p1, const cv::Point& p2, const cv::Point& p3, const cv::Point& p4);
  static bool checkIfLineFromContourIntersectsItself(const std::vector<cv::Point>& contour, size_t i, size_t j);
  static float calcConvexityDefectOfContour(const std::vector<cv::Point>& contour);
};

struct ACD2D_kParts {
  ACD2D_kParts();

  bool checkIfCutLineIsInside(const std::vector<cv::Point>& contour, size_t i, size_t j);

  std::vector<std::vector<cv::Point> > decompose(const std::vector<cv::Point>& contour, uint k = 2, float CDTolerance = -1);

  std::vector<std::vector<cv::Point> > decomposeIntoTwo(const std::vector<cv::Point>& contour);
};

#endif // ACD2D_H
