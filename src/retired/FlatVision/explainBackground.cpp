#include <Perception/opencv.h>
#include <opencv2/opencv.hpp>

#include "explainBackground.h"
#include "helpers.h"

void ExplainBackground::compute(byteA& pixelLabels,
                                const byteA& cam_color, const floatA& cam_depth){
  CHECK_EQ(cam_depth.nd, 2, "");

  //-- initialize pixelLabels
  if(!pixelLabels.N) resizeAs(pixelLabels, cam_depth);

  //-- initialize pixel labels
  for(uint i=0;i<cam_depth.N;i++){
    pixelLabels.p[i] = PL_unexplained;
    if(cam_depth.p[i] < .4) pixelLabels.p[i]=PL_nosignal;
    if(std::isnan(cam_depth.p[i])) pixelLabels.p[i]=PL_nosignal;
    if(cam_depth.p[i]>farThreshold) pixelLabels.p[i]=PL_toofar;
  }

  //-- initialize background and filters
  if(!background.N){ background=cam_depth; background.setZero(); background = .1; }
  if(!countDeeper.N){ resizeAs(countDeeper, background); countDeeper.setZero(); }
  if(!valueDeeper.N){ valueDeeper.resizeAs(background); valueDeeper.setZero(); }

  //-- filter background to deepest value ever (stably) observed
  if(computeBackground) {
    for(uint i=0;i<background.N;i++) {
      if(pixelLabels.p[i]==PL_nosignal) continue; //don't filter if you have no signal

      float &d = cam_depth.p[i];
      float &b = background.p[i];
      float &deeper = valueDeeper.p[i];
      byte &count = countDeeper.p[i];
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
      }else{
        if(fabs(d-b)<.5*threshold){ //for very close pixels, smooth very slowly
          double alpha=.1;
          //b = (1-alpha)*b + alpha*d;
        }
      }
    }
  }


  //-- label pixels as background that are deeper than background - threshold
  for(uint i=0;i<pixelLabels.N;i++){
    if(!pixelLabels.p[i] &&
       (cam_depth.p[i] > background.p[i] - threshold)){
      pixelLabels.p[i]=PL_background;
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

void ExplainBackground::saveBackgroundModel(const char* name) {
  ofstream f1(STRING(name << ".background"));
  ofstream f2(STRING(name << ".countDeeper"));
  ofstream f3(STRING(name << ".valueDeeper"));
  background.write(f1, " ", "\n", "  ");
  countDeeper.write(f2, " ", "\n", "  ");
  valueDeeper.write(f3, " ", "\n", "  ");
//  FILE(STRING(name << ".background")) << background;
//  FILE(STRING(name << ".countDeeper")) << countDeeper;
//  FILE(STRING(name << ".valueDeeper")) << valueDeeper;

}

void ExplainBackground::loadBackgroundModel(const char* name) {
  ifstream f1(STRING(name << ".background"));
  ifstream f2(STRING(name << ".countDeeper"));
  ifstream f3(STRING(name << ".valueDeeper"));
  background.read(f1);
  countDeeper.read(f2);
  valueDeeper.read(f3);
  //  background << FILE(STRING(name << ".background"));
//  cout << background.d0 << " " << background.d1 << endl;
//  countDeeper << FILE(STRING(name << ".countDeeper"));
//  valueDeeper << FILE(STRING(name << ".valueDeeper"));
}



/*
  for(uint i=0;i<pixelLabels.N;i++){
   if(!pixelLabels.p[i] && (cam_depth.p[i] > 1.000)){
      pixelLabels.p[i]=PL_background;
    }
  }
*/
