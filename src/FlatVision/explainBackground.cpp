#include <Perception/opencv.h>
#include <opencv2/opencv.hpp>

#include "explainBackground.h"
#include "helpers.h"

void ExplainBackground::compute(byteA& pixelLabels,
                                const byteA& cam_color, const floatA& cam_depth){
  CHECK_EQ(cam_depth.nd, 2, "");

  //-- initialize pixelLabels
  if(!pixelLabels.N) resizeAs(pixelLabels, cam_depth);
  pixelLabels.setZero();

  //-- check for no signal
  for(uint i=0;i<cam_depth.N;i++){
    if(cam_depth.elem(i)<.001) pixelLabels.elem(i)=PL_nosignal;
    if(cam_depth.elem(i)>farThreshold) pixelLabels.elem(i)=PL_toofar;
  }

  //-- initialize background and filters
  if(!background.N){ background=cam_depth; background.setZero(); background = .1; }
  if(!countDeeper.N){ resizeAs(countDeeper, background); countDeeper.setZero(); }
  if(!valueDeeper.N){ valueDeeper.resizeAs(background); valueDeeper.setZero(); }

  //-- filter background to deepest value ever (stably) observed
  for(uint i=0;i<background.N;i++){
    if(pixelLabels.elem(i)==PL_nosignal) continue; //don't filter if you have no signal

    float &d = cam_depth.elem(i);
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

  //-- assign pixels to background is deeper than background - threshold
  for(uint i=0;i<pixelLabels.N;i++){
    if(!pixelLabels.elem(i) &&
       (cam_depth.elem(i) > background.elem(i) - threshold)){
      pixelLabels.elem(i)=PL_background;
    }
  }

  if(verbose>0){
    cv::Mat cv_background = CV(background);
    cv::Mat cv_labels = CV(pixelLabels);
    cv::imshow("background", cv_background);
    cv::imshow("labels after exBackground", cv_labels);
    cv::waitKey(1);
  }
}
