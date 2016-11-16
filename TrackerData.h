// -*- c++ -*-
// Copyright 2008 Isis Innovation Limited
#ifndef __TRACKERDATA_H
#define __TRACKERDATA_H

#include "PatchFinder.h"
#include "ATANCamera.h"

// This class contains all the intermediate results associated with
// a map-point that the tracker keeps up-to-date. TrackerData
// basically handles all the tracker's point-projection jobs,
// and also contains the PatchFinder which does the image search.
// It's very code-heavy for an h-file (it's a bunch of methods really)
// but it's only included from Tracker.cc!

struct  TrackerData
{
  
typedef std::shared_ptr<TrackerData> Ptr;
  
TrackerData(MapPoint::Ptr pMapPoint) : Point(pMapPoint) {};
  
  MapPoint::Ptr Point;
  PatchFinder Finder;
  
  // Projection itermediates:
  cv::Vec<float, 3> v3Cam;        // 3D Coordinatess in current camera frame
  cv::Vec<float, 2> v2ImPlane;    // Euclidean Coordinates in current cam z=1 plane
  cv::Vec<float, 2> v2Image;      // Pixel coords in LEVEL0
  cv::Matx<float, 2, 2> m2CamDerivs;  // 2x2 Camera projection derivatives
  bool bInImage;        
  bool bPotentiallyVisible;
  
  int nSearchLevel;
  bool bSearched;
  bool bFound;
  bool bDidSubPix;
  cv::Vec<float, 2> v2Found;      // Pixel coords of found patch (L0)
  double dSqrtInvNoise;   // Only depends on search level..
  
  
  // Stuff for pose update:
  cv::Vec<float, 2> v2Error_CovScaled;
  cv::Matx<float, 2, 6> m26Jacobian;   // 2x6 Jacobian wrt camera position
  
  // Project point into image given certain pose and camera.
  // This can bail out at several stages if the point
  // will not be properly in the image.
  inline void Project(const SE3<> &se3CFromW, ATANCamera &Cam) {
    
    // set the potentially visible flag to false, 
    // in case we return prematurely....
    bInImage = bPotentiallyVisible = false;
    // Get the coordinates of the mappoint
    // in the camera coordinate frame
    v3Cam = se3CFromW * Point->v3WorldPos;
    // leave of depth is vanishing
    if(v3Cam[2] < 0.001) return;
    
    
    // Get the Euclidean normalized projection
    v2ImPlane = CvUtils::pproject(v3Cam);
    
    // Check if the estimated projection is in the visible part of of the Euclidean image plane
    if( cv::norm(v2ImPlane ) > Cam.LargestRadiusInImage() ) {
     
      //cout <<"DEBUG: Projection of "<<v3Cam<<"on the plane is : "<<v2ImPlane<<endl;
      //cout <<"DEBUG: While the largest radius is : "<<Cam.LargestRadiusInImage()<<" and the cam size is : "<<Cam.GetImageSize()<<endl;
      return;
    }
    
    // Now project on the image
    v2Image = Cam.Project(v2ImPlane);
    
     
    // If out-of-larger-radius (NOTE: This does NOT NECESSARILLY mean out-of-bounds!), exit.
    if(Cam.Invalid()) return;
    
    // Now check if we are out-of image bounds...
    if(v2Image[0] < 0 || v2Image[1] < 0 || v2Image[0] > irImageSize.width-1 || v2Image[1] > irImageSize.height-1 ) return;
    
    // If we are still here, then projection coordinates are valid...
    bInImage = true;
    
    
  }
  
  // Get the projection derivatives (depend only on the camera.)
  // This is called Unsafe because it depends on the camera caching 
  // results from the previous projection:
  // Only do this right after the same point has been projected!
  inline void GetDerivsUnsafe(ATANCamera &Cam) 
  {
    m2CamDerivs = Cam.GetProjectionDerivs();
  }
  
  // Does projection and gets camera derivs all in one.
  inline void ProjectAndDerivs(SE3<> &se3, ATANCamera &Cam)
  {
    Project(se3, Cam);
    
    if(bFound)  GetDerivsUnsafe(Cam);
      
    
  }
  
  // Jacobian of projection W.R.T. the camera position
  // I.e. if  p_cam = SE3Old * p_world, 
  //         SE3New = SE3Motion * SE3Old
  inline void CalcJacobian()
  {
    double inverseDepth = 1.0 / v3Cam[2];
    double Y = v3Cam[1];
    double X = v3Cam[0];
    
    for(int m = 0; m < 6; m++) {
       
	const cv::Vec<float, 4> v4Motion = SE3<>::generator_field(m, CvUtils::backproject( v3Cam) );
	cv::Vec<float, 2> v2CamFrameMotion( (v4Motion[0] - X * v4Motion[2] * inverseDepth) * inverseDepth, 
				            (v4Motion[1] - Y * v4Motion[2] * inverseDepth) * inverseDepth
				          );
	m26Jacobian(0, m) = m2CamDerivs(0, 0) * v2CamFrameMotion[0] + m2CamDerivs(0, 1) * v2CamFrameMotion[1];
	m26Jacobian(1, m) = m2CamDerivs(1, 0) * v2CamFrameMotion[0] + m2CamDerivs(1, 1) * v2CamFrameMotion[1];
	
      }
  }
  
  // Sometimes in tracker instead of reprojecting, just update the error linearly!
  inline void LinearUpdate(const cv::Vec<float, 6> &v6)
  {
    
    v2Image += m26Jacobian * v6;
  }
  
  // This static member is filled in by the tracker and allows in-image checks in this class above.
  static cv::Size2i irImageSize;
};






#endif




