// -*- c++ -*-
// Copyright 2008 Isis Innovation Limited
//
// This header declares the Map class.
// This is pretty light-weight: All it contains is
// a vector of MapPoints and a vector of KeyFrames.
//
// N.b. since I don't do proper thread safety,
// everything is stored as lists of pointers,
// and map points are not erased if they are bad:
// they are moved to the trash list. That way
// old pointers which other threads are using are not 
// invalidated!

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
  
  void MoveBadPointsToTrash();
  //void EmptyTrash();
  
  
  /// List of so-far good mappoints
  std::vector<std::shared_ptr<MapPoint> > vpPoints;
  // This is a list of points that were excluded from the map
  // This probably is now USELESS!!!
  // TO BE RMEOVED! TO BE REMOVED!!
  //std::vector<std::shared_ptr<MapPoint> > vpPointsTrash;
  // list of keyframes
  std::vector<std::shared_ptr<KeyFrame> > vpKeyFrames;

  bool bGood;
};




#endif

