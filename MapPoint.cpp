// George Terzakis 2016
//
// University of Portsmouth
//
// Code based on PTAM by Klein and Murray (Copyright 2008 Isis Innovation Limited)


#include "MapPoint.h"
#include "KeyFrame.h"

void MapPoint::RefreshPixelVectors() {
  // Get the KF in which the patch was originally found
  // (i.e., a "home" KF )
  KeyFrame::Ptr k = pPatchSourceKF;
  
  // And now compute the 3D coordinates of the mappoint 
  // in the coordinate frame of the home/source KF 
  cv::Vec3f v3PlanePoint_C = k->se3CfromW * v3WorldPos;
  
  // Now, we compute the DEPTH of the 3D location above its own projection in the Euclidean plane Z = 1.
  // NOTE!!! NOTE!!! This computation is valid ONLY if we assume that the normal is the same direction as the optical axis (Z)
  //              *** NOTE AGAIN!! I just HAD TO rename this variable from dCamHeight to dCamDepth ... ********
  //      because IT SIMPLY IS DEPTH and NOT height!!!
  float dCamDepth = fabs( v3PlanePoint_C.dot( v3Normal_NEC ) );

  // NOTE!!! Again, the following variable names are misleading unfortunately... But they actually, in some way, tell the truth!!!! 
  //          So I kept them...
  //
  //         The following dot products yield the angles (cosines in particular) from the Z axis (recall normal is the -Z direction):
  //     
  //         a) "dPixelRate" is effectively the angle of the feature's projection ray from the normal (also optical axis if n = (0, 0,-1) ).
  //          
  //         n) "dOneRightRate" is the angle cosine of the right pixel's Euclidean projection ray from the normal (also optical axis if n = (0, 0,-1) ).
  // 
  //         c) "dOneDownRate" is the angle cosine of the down pixel's Euclidean projection ray from the normal (also optical axis if n = (0, 0,-1) ).
  // 
  float dPixelRate = fabs( v3Center_NEC.dot( v3Normal_NEC ) );
  float dOneRightRate = fabs( v3OneRightFromCenter_NEC.dot( v3Normal_NEC ) );
  float dOneDownRate = fabs( v3OneDownFromCenter_NEC.dot(  v3Normal_NEC ) );
  
  // Now we can actually work out backprojection ray-lengths into the real world that these locations (right, down, center)
  // by simply using a bit of trigonometry 
  //
  // i. "v3CenterOnPlane_C" is the scaled vector of the real-world location from its normalized Euclidean projection along the y-axis
  cv::Vec3f v3CenterOnPlane_C = ( dCamDepth / dPixelRate ) * v3Center_NEC ; 
  
  
  
  // ii. "v3OneRightOnPlane_C" is the scaled vector of the real-world location from the projection ray of the "right pixel" along the x-axis.
  cv::Vec3f v3OneRightOnPlane_C = ( dCamDepth / dOneRightRate ) * v3OneRightFromCenter_NEC;
  
  // iii. "v3OneDownOnPlane_C" is the scaled vector of the real-world location from the projection ray of the "down pixel" along the y-axis.
  cv::Vec3f v3OneDownOnPlane_C = ( dCamDepth / dOneDownRate ) * v3OneDownFromCenter_NEC;
  
  //************* INTUITION: v3OneDownOnPlane_C and v3OneRightOnPlane_C are in fact, the BAT SIGNAL of the patch in the 3D sky!!!! *********
  
  // Express the difference vectors of these SCALED projection vectors wrt World frame orientation.
  SO3<> Rt = k->se3CfromW.get_rotation().inverse();
  v3PixelGoRight_W = Rt * (v3OneRightOnPlane_C - v3CenterOnPlane_C);
  v3PixelGoDown_W = Rt * (v3OneDownOnPlane_C - v3CenterOnPlane_C);
}  
