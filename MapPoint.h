// George Terzakis 2016
//
// University of Portsmouth
//
// Code based on PTAM by Klein and Murray (Copyright 2008 Isis Innovation Limited)
// 


#ifndef __MAP_POINT_H
#define __MAP_POINT_H

#include "OpenCV.h"
#include "GCVD/timer.h" // just for timing


#include <set>
#include <memory>

struct KeyFrame;
struct TrackerData;
struct MapMakerData;

extern CvUtils::Timer timer;

struct MapPoint
{
public:
  typedef std::shared_ptr<MapPoint> Ptr; // managed pointer type to MapPoint
  
  
  // Constructor inserts sensible defaults and zeros pointers.
  inline MapPoint()
  {
    bBad = false;
    pTData = NULL;
    pMMData = NULL;
    nMEstimatorOutlierCount = 0;
    nMEstimatorInlierCount = 0;
   
    dCreationTime = CvUtils::timer.get_time();
  };
  
  // Where in the world is this point? The main bit of information, really.
  cv::Vec3f v3WorldPos;
  // Is it a dud? In that case it'll be moved to the trash soon.
  bool bBad;
  
  // What pixels should be used to search for this point?
  std::shared_ptr<KeyFrame> pPatchSourceKF; // The KeyFrame the point was originally made in
  int nSourceLevel;         // Pyramid level in source KeyFrame
  cv::Point2i irCenter;   // This is in level-coords in the source pyramid level
  
  // What follows next is a bunch of intermediate vectors - they all lead up
  // to being able to calculate v3PixelGo{Down,Right}_W, which the PatchFinder
  // needs for patch warping!
  // a) v3Cednter_NEC : The EUclidean unit-vector of the projection ray.
  // b) v3OneDownFromCenter_NEC : The Euclidean unit-vector of the projection ray correpsonding to the neighboring pixel below the feature.
  // c) v3OneRightFromCenter_NEC : The Euclidean unit-vector of the projection ray correpsonding to the neighboring pixel to the right of the feature.
  // d) v3Normal_NEC : This mysterious " normal: vector is essentially the the optical axis unit vector (negated) upon initialization.
  cv::Vec3f v3Center_NEC;             // Unit vector in Source-KF coords pointing at the patch center
  cv::Vec3f v3OneDownFromCenter_NEC;  // Unit vector in Source-KF coords pointing towards one pixel down of the patch center
  cv::Vec3f v3OneRightFromCenter_NEC; // Unit vector in Source-KF coords pointing towards one pixel right of the patch center
  cv::Vec3f v3Normal_NEC;             // Unit vector in Source-KF coords indicating patch normal
  
  cv::Vec3f v3PixelGoDown_W;           // 3-Vector in World coords corresponding to a one-pixel move down the source image
  cv::Vec3f v3PixelGoRight_W;          // 3-Vector in World coords corresponding to a one-pixel move right the source image
  
  void RefreshPixelVectors();        // Calculates above two vectors
  
  // Info for the Mapmaker (not to be trashed by the tracker:)
  std::shared_ptr<MapMakerData> pMMData;
  
  // Info for the Tracker (not to be trashed by the MapMaker:)
  std::shared_ptr<TrackerData> pTData;
  
  // Info provided by the tracker for the mapmaker:
  int nMEstimatorOutlierCount;
  int nMEstimatorInlierCount;
  
  // Random junk (e.g. for visualisation)
  double dCreationTime; //timer.get_time() time of creation
};

#endif
