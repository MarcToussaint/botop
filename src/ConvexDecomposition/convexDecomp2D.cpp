#include "convexDecomp2D.h"
#include <opencv2/opencv.hpp>

bool PolygonUtils::checkIntersection(const cv::Point& p1, const cv::Point& p2, const cv::Point& p3, const cv::Point& p4) {
  float denominator = (p1.x - p2.x)*(p3.y - p4.y) - (p1.y - p2.y)*(p3.x - p4.x);
  if(fabs(denominator) < 1e-5) {
    return false;
  }
  float num1 = (p1.x - p3.x)*(p3.y - p4.y) - (p1.y - p3.y)*(p3.x - p4.x);
  float num2 = (p1.x - p2.x)*(p1.y - p3.y) - (p1.y - p2.y)*(p1.x - p3.x);
  float t1 = num1/denominator;
  float t2 = -num2/denominator;
  return ((0.0 < t1) && (t1 < 1.0)) && ((0.0 < t2) && (t2 < 1.0));
  //return ((0.0 <= t1) && (t1 <= 1.0)) && ((0.0 < t2) && (t2 < 1.0));
}

bool PolygonUtils::checkIfLineFromContourIntersectsItself(const std::vector<cv::Point>& contour, size_t i, size_t j) {
  for(size_t k = 0; k < contour.size()-1; k++) {
    if(checkIntersection(contour[k], contour[k+1], contour[i], contour[j])) {
      return true;
    }
  }
  return false;
}

float PolygonUtils::calcConvexityDefectOfContour(const std::vector<cv::Point>& contour) {
  if(contour.size() <= 3) {
    return 0.0f;
  }
  std::vector<int> cHInd;
  cv::convexHull(contour, cHInd, false, false);
  std::vector<cv::Vec4i> convDefects;
  cv::convexityDefects(contour, cHInd, convDefects);
  float CD = 0.0f;
  for(auto cd : convDefects) {
    CD += ((float)cd[3])/256.0f;
  }
  return CD;
}


//==========================================================================================


ACD2D_kParts::ACD2D_kParts() {

}

bool ACD2D_kParts::checkIfCutLineIsInside(const std::vector<cv::Point>& contour, size_t i, size_t j) {
  bool lineIntersect = PolygonUtils::checkIfLineFromContourIntersectsItself(contour, i, j);
  if(lineIntersect) {
    return false;
  }
  cv::Point2f m(0.5f*((float)contour[i].x+(float)contour[j].x), 0.5f*((float)contour[i].y+(float)contour[j].y));
  float pol = cv::pointPolygonTest(contour, m, false);
  if(pol > 0) {
    return true;
  }
  return false;
}

std::vector<std::vector<cv::Point> > ACD2D_kParts::decompose(const std::vector<cv::Point>& contour, uint k, float CDTolerance) {
  // if sufficiently "convex", do not decompose at all
  if(CDTolerance >= 0.0f) {
    if(PolygonUtils::calcConvexityDefectOfContour(contour) < CDTolerance) {
      std::vector<std::vector<cv::Point> > dc = {contour};
      return dc;
    }
  }

  // decompose into two pieces
  if(k == 2) {
    return decomposeIntoTwo(contour);
  }

  return std::vector<std::vector<cv::Point> >();
}


std::vector<std::vector<cv::Point> > ACD2D_kParts::decomposeIntoTwo(const std::vector<cv::Point>& contour) {
  float bestCD = std::numeric_limits<float>::infinity();
  size_t best_i;
  size_t best_j;

  for(size_t i = 0; i < contour.size() - 2; i++) {
    for(size_t jT = i + 2; jT < contour.size(); jT++) {
      size_t j = jT % contour.size();
      if(checkIfCutLineIsInside(contour, i, j)) {
        std::vector<cv::Point> subC2(contour.begin()+i, contour.begin()+j+1);
        std::vector<cv::Point> subC1(contour.begin(), contour.begin()+i+1);
        subC1.insert(subC1.end(), contour.begin()+j, contour.end());

        if(!(subC1.size() >= 3 && subC2.size() >= 3)) continue; // probably unnecessary
        if(!(cv::contourArea(subC1) > 0 && cv::contourArea(subC2) > 0)) continue; // probably unnecessary

        float CD = 0.0f;
        CD += PolygonUtils::calcConvexityDefectOfContour(subC1);
        CD += PolygonUtils::calcConvexityDefectOfContour(subC2);

        if(CD < bestCD) {
          bestCD = CD;
          best_i = i;
          best_j = j;
        }
      }
    }
  }
  std::vector<cv::Point> subC2(contour.begin()+best_i, contour.begin()+best_j+1);
  std::vector<cv::Point> subC1(contour.begin(), contour.begin()+best_i+1);
  subC1.insert(subC1.end(), contour.begin()+best_j, contour.end());

  std::vector<std::vector<cv::Point> > dc = {subC1, subC2};
  return dc;
}

