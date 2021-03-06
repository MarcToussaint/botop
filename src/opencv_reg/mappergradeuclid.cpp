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
#include "mappergradeuclid.hpp"
#include "mapaffine.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <iomanip>

namespace cv {
namespace reg {


////////////////////////////////////////////////////////////////////////////////////////////////////
MapperGradEuclid::MapperGradEuclid()
{
}


////////////////////////////////////////////////////////////////////////////////////////////////////
MapperGradEuclid::~MapperGradEuclid()
{
}


////////////////////////////////////////////////////////////////////////////////////////////////////
cv::Ptr<Map> MapperGradEuclid::calculate(
    InputArray _img1, InputArray _mask1, InputArray _img2, InputArray _mask2, cv::Ptr<Map> init, double* error) const
{
    Mat img1 = _img1.getMat();
    Mat mask1 = _mask1.getMat();
    Mat gradx, grady, imgDiff;
    Mat img2, mask2;

    CV_DbgAssert(_img1.size() == _img2.size());
    CV_DbgAssert(_img1.type() == _img2.type());

    if(!init.empty()) {
        // We have initial values for the registration: we move img2 to that initial reference
        init->inverseWarp(_img2, img2);
        if(mask2.total())
          init->inverseWarp(_mask2, mask2);
    } else {
        img2 = _img2.getMat();
        mask2 = _mask2.getMat();
    }

    // Matrices with reference frame coordinates
    Mat grid_r, grid_c;

    // Get gradient in all channels
    if(img1.type()==CV_32FC1 || img1.type()==CV_32FC3){
      gradient(img1, img2, gradx, grady, imgDiff);
      grid(img1, grid_r, grid_c);
    }else{
      Mat I1, I2;
      img1.convertTo(I1, CV_32FC3, 1./255.);
      img2.convertTo(I2, CV_32FC3, 1./255.);
      gradient(I1, I2, gradx, grady, imgDiff);
      grid(I1, grid_r, grid_c);
    }

    // Calculate parameters using least squares
    Matx<double, 3, 3> A;
    Vec<double, 3> b;
    // For each value in A, all the matrix elements are added and then the channels are also added,
    // so we have two calls to "sum". The result can be found in the first element of the final
    // Scalar object.

    if(mask1.total()){
      if(gradx.type()==CV_32FC3){
        Mat tmp;
        cvtColor(mask1, tmp, CV_GRAY2RGB);
        CV_Assert(tmp.type() == gradx.type());
        gradx = gradx.mul(tmp);
        grady = grady.mul(tmp);
        imgDiff = imgDiff.mul(tmp);
      }else{
        gradx = gradx.mul(mask1);
        grady = grady.mul(mask1);
        imgDiff = imgDiff.mul(mask1);
      }
    }
    if(mask2.total()){
      if(gradx.type()==CV_32FC3){
        Mat tmp;
        cvtColor(mask1, tmp, CV_GRAY2RGB);
        CV_Assert(tmp.type() == gradx.type());
        gradx = gradx.mul(tmp);
        grady = grady.mul(tmp);
        imgDiff = imgDiff.mul(tmp);
      }else{
        gradx = gradx.mul(mask2);
        grady = grady.mul(mask2);
        imgDiff = imgDiff.mul(mask2);
      }
    }

    Mat xIy_yIx = grid_c.mul(grady);
    xIy_yIx -= grid_r.mul(gradx);

    A(0, 0) = sum(sum(gradx.mul(gradx)))[0];
    A(0, 1) = sum(sum(gradx.mul(grady)))[0];
    A(0, 2) = sum(sum(gradx.mul(xIy_yIx)))[0];
    A(1, 1) = sum(sum(grady.mul(grady)))[0];
    A(1, 2) = sum(sum(grady.mul(xIy_yIx)))[0];
    A(2, 2) = sum(sum(xIy_yIx.mul(xIy_yIx)))[0];
    A(1, 0) = A(0, 1);
    A(2, 0) = A(0, 2);
    A(2, 1) = A(1, 2);

    b(0) = -sum(sum(imgDiff.mul(gradx)))[0];
    b(1) = -sum(sum(imgDiff.mul(grady)))[0];
    b(2) = -sum(sum(imgDiff.mul(xIy_yIx)))[0];

    // Calculate parameters. We use Cholesky decomposition, as A is symmetric.
    Vec<double, 3> k = A.inv(DECOMP_CHOLESKY)*b;

    k *= stepSize_;

    double cosT = cos(k(2));
    double sinT = sin(k(2));
    Matx<double, 2, 2> linTr(cosT, -sinT, sinT, cosT);
    Vec<double, 2> shift(k(0), k(1));

    cv::pow(imgDiff, 2., imgDiff);
    if(error){
      *error = sum(sum(imgDiff))[0] / sum(mask1)[0];
    }
    //verbose:
    std::cout <<"scale=" <<imgDiff.size() <<"  registration error=" <<std::setprecision(5) <<sum(sum(imgDiff))[0]/sum(mask1)[0] <<std::endl;

    if(init.empty()) {
        return Ptr<Map>(new MapAffine(linTr, shift));
    } else {
        Ptr<MapAffine> newTr(new MapAffine(linTr, shift));
        MapAffine* initPtr = dynamic_cast<MapAffine*>(&*init);
        Ptr<MapAffine> oldTr(new MapAffine(initPtr->getLinTr(), initPtr->getShift()));
        oldTr->compose(newTr);
        return oldTr;
   }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
cv::Ptr<Map> MapperGradEuclid::getMap() const
{
    return cv::Ptr<Map>(new MapAffine());
}


}}  // namespace cv::reg
