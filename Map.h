// George Terzakis 2016
//
//  University of Portsmouth

//
// Code based on PTAM by Klein and Murray

#ifndef __MAP_H
#define __MAP_H

#include <vector>
#include "GCVD/SE3.h"

#include "OpenCV.h"

#include <memory>

struct MapPoint;
struct KeyFrame;

struct Map
{
  Map();
  inline bool IsGood() {return bGood;}
  void Reset();
  
  void deleteBadPoints();
  void EmptyTrash();
  
  /// List of so-far good mappoints
  std::vector<std::shared_ptr<MapPoint> > vpPoints;
  
  // This is a list of points that were excluded from the map
  std::vector<std::shared_ptr<MapPoint> > vpPointsTrash;
  
  // list of keyframes
  std::vector<std::shared_ptr<KeyFrame> > vpKeyFrames;

  
  
  bool bGood;
};




#endif

