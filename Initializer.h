// George Terzakis 2016
//
// University of Portsmouth
//
// Code based on PTAM by Klein and Murray (Copyright 2008 Isis Innovation Limited)


// This is the SLAM initializer interface

#ifndef INITIALIZER_H
#define INITIALIZER_H

#include "ATANCamera.h"
#include "GCVD/SE3.h"
#include <vector>

#include "OpenCV.h"

using namespace RigidTransforms;

// The 2D matches (points/lines) in thw two camera views
struct InitializerMatch
{
  // To be filled in by MapMaker:
  cv::Point2i imPtFirst;   // Imagepoints
  cv::Point2i imPtSecond;  
  cv::Vec2f v2EucPlaneFirst; // first view normalized Euclidean coordinates
  cv::Vec2f v2EucPlaneSecond; // second view normalized Euclidean coordinates
  //double Z1;                  // Depth in the source (first) view
  cv::Matx<float, 2, 2> m2PixelProjectionGrad; // Maybe ready-made derivatives... For the case they are needed...
};


class Initializer
{
public:
  // compute and return relative pose (se3SecondCameraPose)
  virtual bool Compute(std::vector<InitializerMatch> vMatches, double dMaxPixelError, SE3<> &se3SecondCameraPose) = 0;
  Initializer(ATANCamera *pcam) : pCamera(pcam) {};

protected:
  ATANCamera *pCamera; // use this to project predicted matches on the image
		       // and cache the respective derivatives
 
};



#endif
