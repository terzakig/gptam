// -*- c++ -*-
// Copyright 2008 Isis Innovation Limited
//
// SmallBlurryImage - A small and blurry representation of an image.
// used by the relocaliser.

#ifndef __SMALLBLURRYIMAGE_H
#define __SMALLBLURRYIMAGE_H

#include "OpenCV.h"

#include "GCVD/SE2.h"
#include "GCVD/SE3.h"

#include "KeyFrame.h"
#include "ATANCamera.h"

#include "./GCVD/Addedutils.h"


class SmallBlurryImage
{
 public:
  SmallBlurryImage();
  
  SmallBlurryImage(KeyFrame &kf, double dBlur = 2.5);
  
  void MakeFromKF(KeyFrame &kf, double dBlur = 2.5);
  
  void MakeGradients();
  
  double ZMSSD(SmallBlurryImage &other);
  
  std::pair<SE2<>,double> IteratePosRelToTarget(SmallBlurryImage &other, int nIterations = 10);
  
  static SE3<> SE3fromSE2(SE2<> se2, ATANCamera camera);
  
protected:
  cv::Mat_<uchar> mimSmall;
  cv::Mat_<float> mimTemplate;

  
  cv::Mat_<cv::Vec<float, 2> > mimImageGradients;
  bool mbMadeGradients;

  static cv::Size2i mirSize;
};



#endif









