#include <Perception/opencv.h>
#include <opencv2/opencv.hpp>

#include <Gui/opengl.h>

#include "explainNovels.h"

void ExplainNovelPercepts::compute(byteA& pixelLabels,
                                   const byteA& cam_color, const floatA& cam_depth) {

  //-- initialize unexplained filter
  if(!countUnexplained.N) countUnexplained.resizeAs(pixelLabels).setZero();

  //-- assign non-stable unexplained pixels as noise
  for(uint i=0;i<pixelLabels.N;i++){
    byte& l = pixelLabels.p[i];
    byte& c = countUnexplained.p[i];

    //count how often consecutive signals
    if(l) c=0; //is already explained -> reset count
    else{ if(c<255) c++; }

    //when unexplained, but not stably -> noise
    if(!l && c<3) l=PL_noise;
  }

  //..now we only have stable unexplained pixellabels left

  //-- compute contours
  std::vector<std::vector<cv::Point> > contours;
  cv::Mat cv_labels = CV(pixelLabels);
  cv::Mat bin = (cv_labels == PL_unexplained);
  cv::findContours(bin, contours, cv::RETR_LIST, cv::CHAIN_APPROX_SIMPLE);

  int boxSizeLimit=20;
  int sizeLimit=500;

  //-- approximate contours with polygons + get bounding rects and circles
  uint C  = contours.size();
  std::vector<std::vector<cv::Point> > contours_poly(C);
  std::vector<std::vector<cv::Point> > contours_hull(C);
  std::vector<cv::Rect> boundRect(C);
  std::vector<cv::Point2f> center(C);
  std::vector<float> radius(C);
  std::vector<double> size(C);
  std::vector<int> label(C);
  uint numPercepts=0;
  for(uint i=0; i<C; i++){
    cv::approxPolyDP( cv::Mat(contours[i]), contours_poly[i], 3, true );
    cv::convexHull( cv::Mat(contours_poly[i]), contours_hull[i], false );

    boundRect[i] = cv::boundingRect( cv::Mat(contours[i]) );
    size[i] = cv::contourArea(cv::Mat(contours[i]));
    cv::minEnclosingCircle( cv::Mat(contours_poly[i]), center[i], radius[i] );

    if(size[i]>sizeLimit &&
       (boundRect[i].width>boxSizeLimit ||
        boundRect[i].height>boxSizeLimit)){
      label[i]=numPercepts;
      numPercepts++;
    }else{
      label[i]=-1;
    }
  }

  //-- assign pixelLabels by drawing the contour polygon or hull directly into the image with the right labels!
  cv::Mat cv_pixelLabels = CV(pixelLabels);
  for(uint i=0; i<C; i++){
    if(label[i]>=0){
      cv::Scalar col(PL_novelPercepts+label[i]);
      cv::drawContours(cv_pixelLabels, contours, i, col, CV_FILLED); //POLY!
    }
  }

  //-- create output percepts
  flats.resize(numPercepts);
  for(uint i=0; i<C; i++){
    if(label[i]>=0){
      FlatPercept& p=flats(label[i]);
      p.label=PixelLabel(PL_novelPercepts+label[i]);
      p.done=PS_fresh;
      p.x=center[i].x;
      p.y=center[i].y;
      p.radius = radius[i];
      p.size = size[i];
      p.rect = ARRAY<int>(boundRect[i].x,
                          boundRect[i].y,
                          boundRect[i].x+boundRect[i].width,
                          boundRect[i].y+boundRect[i].height);
      //contour polygon
      conv_pointVec_arr(p.polygon, contours_poly[i]);
      //contour hull
      conv_pointVec_arr(p.hull, contours_hull[i]);
    }
  }

  if(verbose>0){
    cv::imshow("labels after exNovel", cv_pixelLabels);

    cv::Mat cv_color = CV(cam_color).clone();
    for(uint i=0,k=0; i<contours.size(); i++){
      if(label[i]>=0){
        byte col[3];
        id2color(col, k+1);
        cv::Scalar colo(col[0], col[1], col[2]);
        cv::drawContours( cv_color, contours, i, colo, 2, 8);
        cv::drawContours( cv_color, contours_hull, i, colo, 2, 8);
        rectangle( cv_color, boundRect[i].tl(), boundRect[i].br(), colo, 2, 8, 0 );
//        circle( cv_color, center[i], (int)radius[i], colo, 2, 8, 0 );
        k++;
      }
    }

    cv::cvtColor(cv_color, cv_color, cv::COLOR_RGB2BGR);
    cv::imshow("novel contours", cv_color);
    cv::waitKey(1);
  }
}
