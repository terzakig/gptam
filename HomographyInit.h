// George Terzakis 2016
//
// University of Portsmouth
//
// Code based on PTAM by Klein and Murray (Copyright 2008 Isis Innovation Limited)

// HomographyInit.h 
// Declares the HomographyInit class and a few helper functions. 
//
// This class is used by MapMaker to bootstrap the map, and implements
// the homography decomposition of Faugeras and Lustman's 1988 tech
// report.
//
// Implementation according to Faugeras and Lustman

#ifndef __HOMOGRAPHY_INIT_H
#define __HOMOGRAPHY_INIT_H


#include "Initializer.h"

#include "ATANCamera.h"
#include "GCVD/SE3.h"
#include <vector>

#include "OpenCV.h"

using namespace RigidTransforms;

// Homography matches are 2D-2D matches expressed in normazlied Euclidean coordinates
/*struct HomographyMatch
{
  // To be filled in by MapMaker:
  cv::Point2i imPtFirst;   // Imagepoints!!!!! They are NECESSARY!
  cv::Point2i imPtSecond;  // YES THEY ARE! YES THEY ARE!
  cv::Vec2f v2EucPlaneFirst; // first view normalized Euclidean coordinates
  cv::Vec2f v2EucPlaneSecond; // second view normalized Euclidean coordinates
  cv::Matx<float, 2, 2> m2PixelProjectionGrad; // 2x2 ready-made derivatives of the projection in the 2nd view
					      // CAUTION WITH THESE DERIVATIVES!!!!!!!!
};*/

struct HomographyDecomposition 
// Storage for each homography decomposition
{
  // NOTE!!!! The following 3 fields are intermediate matrices in Faugeras' paper.
  //          They are NOT required in my decomposition function and I dont see how they
  //	      might be required in other algorithms; hence, I am discarding them (commented out for now, 
  //          but will be deleted in the future)
  //cv::Vec3f v3Tp;
  //cv::Matx<float, 3, 3> m3Rp; // 3x3
  //double d;                   // I dont know why this is here, but i know IT'S NOT THE PLANE DISTANCE!
  cv::Vec<float, 3> v3n; // plane normal
  
  // The recovered transformation from the 1st to the 2nd view.
  SE3<> se3SecondFromFirst; // The rigid transformation in fashion [R|t] which basically transforms 
			    // points in the first view to points in the second as p2 = (R + t*n')*p1
  int nScore; // how good is this homography? (assigned by ChooseBestDecomposition() )
};

class HomographyInit : public Initializer {
  
public:
  static constexpr double RANSAC_DEFAULT_THRESHOLD_BOUND = 5.0; 
  
  bool Compute(std::vector<InitializerMatch> vMatches, double dMaxPixelError, SE3<> &se3SecondCameraPose);
  HomographyInit(ATANCamera *pcam) : Initializer(pcam) {};
protected:
  //ATANCamera *pCamera; // use this to project predicted matches on the image
		       // and cache the respective derivatives
  cv::Matx<float, 3, 3> HomographyFromMatches(std::vector<InitializerMatch> vMatches);
  void BestHomographyFromMatches_MLESAC();
  void DecomposeHomography();
  void ChooseBestDecomposition();
  void RefineHomographyWithInliers();
  
  bool IsHomographyInlier(cv::Matx<float, 3, 3> m3Homography, InitializerMatch match);
  double MLESACScore(cv::Matx<float, 3, 3> m3Homography, InitializerMatch match);
  
  double mdMaxPixelErrorSquared;
  cv::Matx<float, 3, 3> mm3BestHomography;
  std::vector<InitializerMatch> mvMatches;
  std::vector<InitializerMatch> mvHomographyInliers;
  std::vector<HomographyDecomposition> mvDecompositions;
};



#endif
