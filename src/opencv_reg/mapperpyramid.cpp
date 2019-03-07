/*M///////////////////////////////////////////////////////////////////////////////////////
//
//  IMPORTANT: READ BEFORE DOWNLOADING, COPYING, INSTALLING OR USING.
//
//  By downloading, copying, installing or using the software you agree to this license.
//  If you do not agree to this license, do not download, install,
//  copy or use the software.
//
// Copyright (C) 2013, Alfonso Sanchez-Beato, all rights reserved.
// Third party copyrights are property of their respective owners.
//
// Redistribution and use in source and binary forms, with or without modification,
// are permitted provided that the following conditions are met:
//
//   * Redistribution's of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//
//   * Redistribution's in binary form must reproduce the above copyright notice,
//     this list of conditions and the following disclaimer in the documentation
//     and/or other materials provided with the distribution.
//
//   * The name of the copyright holders may not be used to endorse or promote products
//     derived from this software without specific prior written permission.
//
// This software is provided by the copyright holders and contributors "as is" and
// any express or implied warranties, including, but not limited to, the implied
// warranties of merchantability and fitness for a particular purpose are disclaimed.
// In no event shall the contributors be liable for any direct,
// indirect, incidental, special, exemplary, or consequential damages
// (including, but not limited to, procurement of substitute goods or services;
// loss of use, data, or profits; or business interruption) however caused
// and on any theory of liability, whether in contract, strict liability,
// or tort (including negligence or otherwise) arising in any way out of
// the use of this software, even if advised of the possibility of such damage.
//
//M*/

#include "precomp.hpp"
#include <vector>

#include <opencv2/imgproc/imgproc.hpp>
#include "mapperpyramid.hpp"

using namespace std;

namespace cv {
namespace reg {


////////////////////////////////////////////////////////////////////////////////////////////////////
MapperPyramid::MapperPyramid(Ptr<Mapper> baseMapper)
    : numLev_(3), numIterPerScale_(3), baseMapper_(*baseMapper)
{
  baseMapper_.stepSize_=stepSize_;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
Ptr<Map> MapperPyramid::calculate(InputArray _img1, InputArray _mask1, InputArray _img2, InputArray _mask2, Ptr<Map> init, double* error) const
{
    Mat img1 = _img1.getMat();
    Mat mask1 = _mask1.getMat();
    Mat img2, mask2;

    if(!init.empty()) {
        // We have initial values for the registration: we move img2 to that initial reference
        init->inverseWarp(_img2, img2);
        if(mask2.total())
          init->inverseWarp(_mask2, mask2);
    } else {
        init = baseMapper_.getMap();
        img2 = _img2.getMat();
        mask2 = _mask2.getMat();
    }

    cv::Ptr<Map> ident = baseMapper_.getMap();

    // Precalculate pyramid images
    vector<Mat> pyrIm1(numLev_), pyrIm2(numLev_), pyrMask1(numLev_), pyrMask2(numLev_);
    pyrIm1[0] = img1;
    pyrIm2[0] = img2;
    pyrMask1[0] = mask1;
    pyrMask2[0] = mask2;
    for(int im_i = 1; im_i < numLev_; ++im_i) {
        pyrDown(pyrIm1[im_i - 1], pyrIm1[im_i]);
        pyrDown(pyrIm2[im_i - 1], pyrIm2[im_i]);
        if(mask1.total())
          pyrDown(pyrMask1[im_i - 1], pyrMask1[im_i]);
        if(mask2.total())
          pyrDown(pyrMask2[im_i - 1], pyrMask2[im_i]);
    }

    Mat currImg1, currImg2, currMask1, currMask2;
    for(int lv_i = 0; lv_i < numLev_; ++lv_i) {
        currImg1  = pyrIm1[numLev_ - 1 - lv_i];
        currImg2  = pyrIm2[numLev_ - 1 - lv_i];
        currMask1 = pyrMask1[numLev_ - 1 - lv_i];
        currMask2 = pyrMask2[numLev_ - 1 - lv_i];
        // Scale the transformation as we are incresing the resolution in each iteration
        if(lv_i != 0) {
            ident->scale(2.);
        }
        int numIter = numIterPerScale_;
        if(lv_i==0){ //different parameters for coarsest level!!
          numIter = 10;
          baseMapper_.stepSize_ = .2;
        }else{
          numIter = numIterPerScale_;
          baseMapper_.stepSize_ = stepSize_;
        }
        for(int it_i = 0; it_i < numIter; ++it_i) {
            ident = baseMapper_.calculate(currImg1, currMask1, currImg2, currMask2, ident, error);
        }
    }

    init->compose(ident);
    return init;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
cv::Ptr<Map> MapperPyramid::getMap() const
{
    return cv::Ptr<Map>();
}


}}  // namespace cv::reg
