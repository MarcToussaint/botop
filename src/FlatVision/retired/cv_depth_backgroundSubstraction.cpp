#include <Perception/opencv.h>
#include "cv_depth_backgroundSubstraction.h"
#include <Gui/opengl.h>

#include <opencv2/opencv.hpp>

void CV_BackgroundSubstraction::compute(const byteA& color, const floatA& depth, const byteA& _inputMask){
  if(depth.nd!=2) return;
  if(_inputMask.nd!=2) return;

  //-- check for no signal
  if(!noSignal.N) resizeAs(noSignal, depth);
  noSignal.setZero();
  for(uint i=0;i<depth.N;i++){
    if(depth.elem(i)<.001 || _inputMask.elem(i)) noSignal.elem(i)=0xff;
  }

  //-- rescale the depth values to inverval ???
  cv::Mat cv_depth = conv_Arr2CvRef(depth);
  cv_depth.convertTo(cv_depth, -1, 1.f/depthcut, .0);
  cv::threshold(cv_depth, cv_depth, 1., 1., cv::THRESH_TRUNC);

  //-- background: assign to deepest value ever observed
  if(!background.N){ background=depth; background.setZero(); background = .9; }
  if(!countDeeper.N){ resizeAs(countDeeper, background); countDeeper.setZero(); }
  if(!valueDeeper.N){ valueDeeper.resizeAs(background); valueDeeper.setZero(); }

  for(uint i=0;i<background.N;i++){
    if(noSignal.elem(i)){
      continue; //don't filter if you have no signal
    }

    float &d = depth.elem(i);
    float &b = background.elem(i);
    float &deeper = valueDeeper.elem(i);
    byte &count = countDeeper.elem(i);
    if(d > b){ //depth is deeper than background
      if(!count){ //1st time: initialize
        deeper = d;
      }else{      //multiple times in a row: store the least deepest value
        if(deeper > d) deeper = d;
      }
      count++;
    }else{
      count=0;    //reset count
    }
    if(count>=5){  //if count>=5, reassign background and reset count
      b=deeper;
      count=0;
    }
  }

  //-- fill holes in depth image
//  for(uint i=0;i<depth.N;i++){
//    if(depth.elem(i)<.001) depth.elem(i) = background.elem(i);
//  }


  //-- mask: background thresholding
  resizeAs(mask, depth);
  mask = 0xff;
  for(uint i=0;i<mask.N;i++){
    if(depth.elem(i)<.01
       || noSignal.elem(i)
       || depth.elem(i) > background.elem(i) - threshold) mask.elem(i)=0;
  }

  //-- stable: discard nonstable mask points
  if(!countStable.N) countStable.resizeAs(mask).setZero();
  for(uint i=0;i<mask.N;i++){
    byte& m = mask.elem(i);
    byte& c = countStable.elem(i);

    //count how often consecutive signals
    if(!m) c=0;
    else{ if(c<100) c++; }

    //only when 10 consecutive signals -> signal
    if(c<5) m=0;
  }


#if 1
  //-- compute contours
  std::vector<std::vector<cv::Point> > cv_contours;
  cv::Mat cv_mask = conv_Arr2CvRef(mask);
  cv::Mat bin2 = cv_mask.clone();
  cv::findContours(bin2, cv_contours, cv::RETR_LIST, cv::CHAIN_APPROX_SIMPLE);

  //-- approximate contours to polygons + get bounding rects and circles
  std::vector<std::vector<cv::Point> > contours_poly( cv_contours.size() );
  std::vector<cv::Rect> boundRect( cv_contours.size() );
  std::vector<cv::Point2f>center( cv_contours.size() );
  std::vector<float>radius( cv_contours.size() );
  uint numPercepts=0;
  for(uint i=0; i<cv_contours.size(); i++){
    approxPolyDP( cv::Mat(cv_contours[i]), contours_poly[i], 3, true );
    boundRect[i] = cv::boundingRect( cv::Mat(contours_poly[i]) );
    minEnclosingCircle( cv::Mat(contours_poly[i]), center[i], radius[i] );
    if(radius[i]>10) numPercepts++;
  }

  //-- create an output percept
  CVpercepts.resize(numPercepts+1);
  uint k=0;
  for(uint i=0; i<cv_contours.size(); i++){
    if(radius[i]>10){
      CV_BackgroundSubstraction_Percept& p=CVpercepts(k);
      p.x=center[i].x;
      p.y=center[i].y;
      p.radius = radius[i];
      p.rect = ARR(boundRect[i].x, boundRect[i].y, boundRect[i].width, boundRect[i].height); //< the top-left corner, as well as width and height of the rectangle
      //polygon contour
      p.polygon.resize(contours_poly[i].size(), 2);
      for(uint j=0;j<p.polygon.d0;j++){
        p.polygon(j,0) = contours_poly[i][j].x;
        p.polygon(j,1) = contours_poly[i][j].y;
      }
      //depth image from the rect
      p.depthRect.resize(boundRect[i].height+1, boundRect[i].width+1).setZero();
      p.backgroundRect.resize(boundRect[i].height+1, boundRect[i].width+1).setZero();
      for(int x=0;x<(int)p.depthRect.d1;x++) for(int y=0;y<(int)p.depthRect.d0;y++){
        if(boundRect[i].y+y>=mask.d0) continue;
        if(boundRect[i].x+x>=mask.d1) continue;
        if(mask(boundRect[i].y+y, boundRect[i].x+x)){
          p.depthRect(y,x)=depth(boundRect[i].y+y, boundRect[i].x+x);
          p.backgroundRect(y,x)=background(boundRect[i].y+y, boundRect[i].x+x);
        }
      }
//      for(int j=0; j<p.depthRect.d0; j++)
//        p.depthRect[j].setCarray(&depth(boundRect[i].y+j,boundRect[i].x), p.depthRect.d1);

      k++;
    }
  }
#endif

  //-- create an output percept for the intputMask (robot)
  cv::Rect inputMaskRect;
  if(true){
    CV_BackgroundSubstraction_Percept& p=CVpercepts.last();
    cv::Mat inputMaskPoints;
    cv::Mat inputMask = conv_Arr2CvRef(_inputMask);
    cv::findNonZero(inputMask, inputMaskPoints);
    if(inputMaskPoints.rows){
      inputMaskRect=boundingRect(inputMaskPoints);
      p.rect = ARR(inputMaskRect.x, inputMaskRect.y, inputMaskRect.width, inputMaskRect.height); //< the top-left corner, as well as width and height of the rectangle
    }else{
      p.rect = ARR(0,0, _inputMask.d1-1, _inputMask.d0-1);
    }
    //depth image from the rect
    p.depthRect.resize(inputMaskRect.height, inputMaskRect.width);
    for(int x=0;x<(int)p.depthRect.d1;x++) for(int y=0;y<(int)p.depthRect.d0;y++){
      if(!_inputMask(inputMaskRect.y+y, inputMaskRect.x+x)) p.depthRect(y,x)=0.;
      else p.depthRect(y,x)=depth(inputMaskRect.y+y, inputMaskRect.x+x);
    }
  }


  if(verbose>0){
    cv::Mat cv_background = conv_Arr2CvRef(background);
    cv::imshow("depth", cv_depth);
    cv::imshow("background", cv_background);
#if 1
    cv::imshow("mask", cv_mask);

    cv::Mat cv_countStable = conv_Arr2CvRef(countStable);
    cv::imshow("stable", cv_countStable);

    cv::Mat cv_color = conv_Arr2CvRef(color);
    for(uint i=0; i<cv_contours.size(); i++){
      if(radius[i]<=10) continue;
      byte col[3];
      id2color(col, i);
      cv::Scalar colo(col[0], col[1], col[2]);
      cv::drawContours( cv_color, contours_poly, i, colo, 2, 8);
      rectangle( cv_color, boundRect[i].tl(), boundRect[i].br(), colo, 2, 8, 0 );
      circle( cv_color, center[i], (int)radius[i], colo, 2, 8, 0 );
    }

    rectangle( cv_color, inputMaskRect.tl(), inputMaskRect.br(), cv::Scalar(0x80,0xff,0x80), 2, 8, 0 );

    cv::imshow("contour", cv_color);
#endif

    cv::waitKey(1);
  }
}

void CV_BackgroundSubstraction_Thread::step(){
  byteA _color = color.get();
  floatA _depth = depth.get();
  byteA _inputMask = mask.get();

  compute(_color, _depth, _inputMask);

  arr fxypxy = cameraFxypxy.get();
  CHECK(fxypxy.N, "need camera calibration parameters");
  if(false){
    auto P = percepts.set();
    P->clear();
    uint k=0;
    for(CV_BackgroundSubstraction_Percept& cv_p : CVpercepts){
      auto p = make_shared<PercMesh>();
      p->id=k++;
      //add the observed points to the mesh:
      depthData2pointCloud(p->mesh.V, cv_p.depthRect, fxypxy(0), fxypxy(1), fxypxy(2)-cv_p.rect(0), fxypxy(3)-cv_p.rect(1));
      p->mesh.V.reshape(p->mesh.V.N/3, 3);
      //add the background points to the mesh: (assuming object to rest on background)
      arr V2;
      depthData2pointCloud(V2, cv_p.backgroundRect, fxypxy(0), fxypxy(1), fxypxy(2)-cv_p.rect(0), fxypxy(3)-cv_p.rect(1));
      V2.reshape(V2.N/3, 3);
      p->mesh.V.append(V2);
      //make convex
      for(uint i=p->mesh.V.d0;i--;)  if(absMax(p->mesh.V[i])==0.)  p->mesh.V.delRows(i); //delete zero points
      p->mesh.makeConvexHull();
      p->mesh.C = id2color(p->id);
      //set the pose to be the camera pose:
      p->pose.set(cameraPose.get());
      P->append(std::dynamic_pointer_cast<Percept>(p));
      if(p->mesh.V.N) p->com = p->mesh.getCenter();
    }
  }
  //    LOG(1) <<"#percepts:" <<percepts.N;
}
