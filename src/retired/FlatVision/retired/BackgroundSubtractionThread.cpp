#include <opencv2/opencv.hpp>
#include <opencv2/video/background_segm.hpp>
#include <Perception/opencv.h>

#include "BackgroundSubtractionThread.h"

#if 0 //not compatible with opencv4

BackgroundSubtractionThread::BackgroundSubtractionThread(Var<byteA>& color, double rate)
  : Thread("BackgroundSubtraction", rate), color(this, color, false), rate(rate)  {
  bgs = new cv::BackgroundSubtractorMOG2(1000.0, 0.0);
  threadLoop();
}

BackgroundSubtractionThread::~BackgroundSubtractionThread() {
  threadClose();
  delete bgs;
}

void BackgroundSubtractionThread::learnBackgroundModel(uint nFrames) {
  threadStop();
  updateBackgroundModel = true;
  for(uint i = 0; i < nFrames; i++) {
    step();
    rai::wait(rate);
  }
  updateBackgroundModel = false;
  threadLoop();
}

void BackgroundSubtractionThread::open() {}

void BackgroundSubtractionThread::close() {}

void BackgroundSubtractionThread::step() {
  cv::Mat colorImg = conv_Arr2CvRef(color.get()()).clone();
  if(!colorImg.empty()) {
    cv::Mat cvMask;
    if(updateBackgroundModel) {
      bgs->operator()(colorImg, cvMask);
    } else {
      bgs->operator()(colorImg, cvMask, 0.0);
    }
    cv::threshold(cvMask, cvMask, 254, 255, cv::THRESH_BINARY);

    if(morphologicalCleaning) {
      // opening and closing to get rid of small stuff
      cv::Mat element = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(2*mSize+1, 2*mSize+1));
      cv::morphologyEx(cvMask, cvMask, cv::MORPH_OPEN, element);
      cv::morphologyEx(cvMask, cvMask, cv::MORPH_CLOSE, element);
    }

    if(contourFilling) {
      // fill holes (hopefully)
      cv::Mat contourImg(cvMask.size(), CV_8UC1, cv::Scalar(0));
      std::vector<std::vector<cv::Point> > contours;
      cv::findContours(cvMask, contours, cv::RETR_LIST, cv::CHAIN_APPROX_NONE);
      for(uint i = 0; i < contours.size(); i++) {
        cv::drawContours(contourImg, contours, i, cv::Scalar(255), -1);
      }
      mask.set()() = contourImg;
    } else {
      mask.set()() = cvMask;
    }
  }
}

#endif
