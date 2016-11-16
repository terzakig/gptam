// George Terzakis 2016
//
// University of Portsmouth
//
// Code based on PTAM by Klein and Murray (Copyright 2008 Isis Innovation Limited)

#ifndef __CALIB_CORNER_PATCH_H
#define __CALIB_CORNER_PATCH_H


#include <cv.hpp>
#include <highgui.hpp>
#include <core.hpp>
#include <cxcore.hpp>


#include "GCVD/Addedutils.h"

class CalibCornerPatch
{
public:
  struct Params
  {
    Params();
    cv::Mat_<float> m2Warp(); // 2x2!!!
    cv::Vec2f v2Pos;
    cv::Vec2f v2Angles;
    float dMean;
    float dGain;
  };
  
  CalibCornerPatch(int nSideSize = 8);
  //bool IterateOnImage(Params &params, CVD::Image<CVD::byte> &im);
  bool IterateOnImage(Params &params, cv::Mat_<uchar> &im);
  //bool IterateOnImageWithDrawing(Params &params, CVD::Image<CVD::byte> &im);
  bool IterateOnImageWithDrawing(Params &params, cv::Mat_<uchar> &im);

 protected:
  void MakeTemplateWithCurrentParams();
  void FillTemplate(cv::Mat_<float> &im, Params params);
  float Iterate(cv::Mat_<uchar> &im);
  Params mParams;
  cv::Mat_<float> mimTemplate;
  cv::Mat_<cv::Vec2f > mimGradients;
  cv::Mat_<cv::Vec2f > mimAngleJacs;
  
  void MakeSharedTemplate();
  static cv::Mat_<float> mimSharedSourceTemplate;

  float mdLastError;
};




#endif
