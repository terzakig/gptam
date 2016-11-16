// **************** George Terzakis 2016 **********************
// 
// ***************University of Portsmouth ****************
//
// Based on PTAM by Klein and Murray

#include "ATANCamera.h"

#include "GCVD/Operators.h"

#include <iostream>
#include "Persistence/instances.h" 


using namespace std;
using namespace Persistence;

// Quick description of the parameters:
// Params[0] - fx / IMG_WIDTH  : The scaled focal length in x
// Params[1] - fy / IMG_HEIGHT : The scaled focal length in y
// Params[2] - cx / IMG_WIDTH  : The scaled x-coord of the center of the image.
// Params[3] - cy / IMG_HEIGHT : The scaled y-coord of the center of the image.
// Params[4] - w               : The distortion coefficient according to the FOV model of Devernay and Faugeras (more comments later in the code)
cv::Vec<float, NUMTRACKERCAMPARAMETERS> ATANCamera::mvDefaultParams;



ATANCamera::ATANCamera(string sName, const cv::Size2i imgsize  )
{
  // creating default parameters: Assuming that at 5 m in the z axis, the frustum section is 10 m x 10m 
  // (which is roughly a standard webcamera field of view)
  // We also assume that cx = WIDTH / 2 and cy = HEIGHT / 2
  // The devernay-faugeras FOX parameter is 0.07 for fun...
  ATANCamera::mvDefaultParams = cv::Vec<float, NUMTRACKERCAMPARAMETERS>( 0.5,
						   4.0/5.0  ,
						    0.5 ,
						    0.5 ,
						    0.07 );
  
  // The camera name is used to find the camera's parameters in a GVar.
  msName = sName;
  // Need to do a "get" in order to put the tag msName+".Parameters" in the list. value is loaded either from the file, or from the defaults
  PV3::get<cv::Vec<float, NUMTRACKERCAMPARAMETERS> >(msName+".Parameters", ATANCamera::mvDefaultParams, Persistence::SILENT);
  PV3.Register(mpvvCameraParams, sName + ".Parameters", mvDefaultParams, HIDDEN | FATAL_IF_NOT_DEFINED);
  // assingining default image size. 
  // But the Calibrator or GTAM should be assigning the correct dimensions upon initialization
  mvImageSize.width = imgsize.width; 
  mvImageSize.height = imgsize.height;
  
  RefreshParams();
}







void ATANCamera::RefreshParams() 
{
  // This updates internal member variables according to the current camera parameters,
  // and the currently selected target image size.
  //
  
  // First: Focal length and image center in pixel coordinates
  mvFocal[0] = mvImageSize.width * (*mpvvCameraParams)[0];         // fx = p[0] * IMG_WIDTH
  mvFocal[1] = mvImageSize.height * (*mpvvCameraParams)[1];         // fy = p[1] * IMG_HEIGHT
  mvCenter[0] = mvImageSize.width * (*mpvvCameraParams)[2] - 0.5;  // cx = IMG_WIDTH * p[2]  (NOTE!!! the 0.5 is just an OpenGL related arbitrary offset;  
								// it's sole purpose is to simply force border pixels into visible OpenGL canvas;
								// of course, there is no justification of it being there - ESPECIALLY in the Project
								// function, so unfortunately my imagination ran wild before I managed to work this out......
  mvCenter[1] = mvImageSize.height * (*mpvvCameraParams)[3] - 0.5;  // cy = IMG_HEIGHT * p[3] (again, 0.5 is just an arbitrary offfset...)
  
  // One over focal length
  mvInvFocal[0] = 1.0 / mvFocal[0];   // inverse "horizontal: focal length: 1 / fx
  mvInvFocal[1] = 1.0 / mvFocal[1];   // inverse "vertical" focal length: 1 / fy

  // Some radial distortion parameters..
  mdW =  (*mpvvCameraParams)[4];    // The Devernay-Faugeras radial distortion FOV model parameter (AKA "angle")
  if(mdW != 0.0) {
      // This is just the denominator in the fraction that yields the UNDISTORTED radius (ru)
      md2Tan = 2.0 * tan(mdW / 2.0);
      // This is the inverse of the above denominator
      mdOneOver2Tan = 1.0 / md2Tan;
      // The inverse DFevernay - Faugeras "angle" 
      mdWinv = 1.0 / mdW;
      // Just a factor/flag that enables radial distortion compensation
      mdDistortionEnabled = 1.0;
  }
  else {
      // set distortion/undistortion parameter to zero
      mdWinv = 0.0;
      // denominator of the UNDISTORTED radius to zero (unused anyway)
      md2Tan = 0.0;
      // set distortion compensation flag/factor to zero.
      mdDistortionEnabled = 0.0;
    }
  
  
  // We need to compute the largest radius in image.
  // This is taken as the point in the two diagonals that lies the farthest from (cx, cy),
  // which will obviously be one of the four distant image corners.
  // Point is found using [0, 1]x[0, 1] image coordinate space since camera parameters are stored scaled by the dimensions of the image 
  // (hence there is no need to to work with pixels).
  cv::Vec2f v2( std::max( (*mpvvCameraParams)[2], 1.0f - (*mpvvCameraParams)[2] ) / (*mpvvCameraParams)[0],
		std::max( (*mpvvCameraParams)[3], 1.0f - (*mpvvCameraParams)[3] ) / (*mpvvCameraParams)[1] );
  // Retrieve the undistorted radius of the point. Invrtrans returns exactly this: tan(r*mdW) / (2 * tan(mdW/2))
  // which is the undistorted radius of a point with distorted radius r.
  mdLargestRadius = invrtrans( cv::norm(v2) );
  
  // At what stage does the model become invalid? (GK)
  // George: So Klein chooses 1.5 x <largest radius> to be the boundary of model validity
  mdMaxR = 1.5 * mdLargestRadius; // (pretty arbitrary)

  // work out world radius of one pixel (GK). 
  // (This only really makes sense for square-ish pixels) (GK).
  // George: OK, here's a more comperehensive description of the above:
  // Klein is trying to obtain (roughly) the actual distance of 1 pixel in the normalized Euclidean world. 
  // So he takes the middle pixel and the pixel at +(1, 1) offset from the middle, he back-projects them 
  // onto the normalized Euclidean plane and takes the norm of the difference (divided by two). 
  {
    cv::Vec2f v2Center = UnProject(0.5 * mvImageSize.width, 0.5 * mvImageSize.height);
    cv::Vec2f v2RootTwoAway = UnProject(0.5 * mvImageSize.width + 1.0,  0.5 * mvImageSize.height + 1.0);
    cv::Vec2f v2Diff = v2Center - v2RootTwoAway;
    mdOnePixelDist = cv::norm(v2Diff) / sqrt(2.0);
  }
  
  // Work out the linear projection values for the UFB
  {
    // First: Find out how big the linear bounding rectangle must be
    vector<cv::Vec2f > vv2Verts;
    vv2Verts.push_back(UnProject( -0.5, -0.5) ); // Backproject the central pixel (Again, what is implied here is that (0,0) is 
						// the left-lower corner, but recall from above that +-0.5 is a necessary offset for display reasons...)
    vv2Verts.push_back( UnProject(mvImageSize.width - 0.5,           -0.5) ); 		// Backproject Right lower - left corner to Euclidean norm. plane
    vv2Verts.push_back( UnProject(mvImageSize.width - 0.5, mvImageSize.height - 0.5) );  // Backproject Right-Upper corner to norm. Euclidean plabe.
    vv2Verts.push_back( UnProject(              -0.5     , mvImageSize.height - 0.5) );                // Backproject Left-Upper corner to norm. Euclidean plane.
    cv::Vec2f v2Min = vv2Verts[0];                                            
    cv::Vec2f v2Max = vv2Verts[0];
    // working out the two furthest points in the normalized Euclidean plane (most likely the 1st or 2nd diagonal)
    for(int i=0; i<4; i++)
      for(int j=0; j<2; j++) {
	
	  if(vv2Verts[i][j] < v2Min[j]) v2Min[j] = vv2Verts[i][j];
	  if(vv2Verts[i][j] > v2Max[j]) v2Max[j] = vv2Verts[i][j];
	}
    mvImplaneTL = v2Min; // Upper/Top-Left 
    mvImplaneBR = v2Max; // Bottom-Right
    
    // Store projection parameters to fill this bounding box
    cv::Vec2f v2Range = v2Max - v2Min;
    mvUFBLinearInvFocal = v2Range;
    mvUFBLinearFocal[0] = 1.0 / mvUFBLinearInvFocal[0];
    mvUFBLinearFocal[1] = 1.0 / mvUFBLinearInvFocal[1];
    mvUFBLinearCenter[0] = -1.0 * v2Min[0] * mvUFBLinearFocal[0];
    mvUFBLinearCenter[1] = -1.0 * v2Min[1] * mvUFBLinearFocal[1];
  }
  
}







// Utility function for easy drawing with OpenGL
// C.f. comment in top of ATANCamera.h
// George:  Creation of frustum matrix to indulge the idiosychracies of OpenGL,
// which demand that the frsutum be mapped onto the unit cube centered at the origin.
// GK wants Z+ is in front of the camera (Right-Handed frame)
cv::Matx<float, 4, 4> ATANCamera::MakeUFBLinearFrustumMatrix(float near, float far)
{
  cv::Matx<float, 4, 4> m4 = cv::Matx<float,4, 4>::zeros();
  
  double left = mvImplaneTL[0] * near;
  double right = mvImplaneBR[0] * near;
  double top = mvImplaneTL[1] * near;
  double bottom = mvImplaneBR[1] * near;
  
  // The openGhelL frustum manpage is A PACK OF LIES!!
  // Two of the elements are NOT what the manpage says they should be.
  // Anyway, below code makes a frustum projection matrix
  // Which projects a RHS-coord frame with +z in front of the camera
  // Which is what I usually want, instead of glFrustum's LHS, -z idea.
  m4(0, 0) = (2 * near) / (right - left);
  m4(1, 1) = (2 * near) / (top - bottom);
  
  m4(0, 2) = (right + left) / (left - right);
  m4(1, 2) = (top + bottom) / (bottom - top);
  m4(2, 2) = (far + near) / (far - near);
  m4(3, 2) = 1;
  
  m4(2, 3) = 2*near*far / (near - far);

  return m4;
};


// Compute the derivatives of the projection model wrt the NORMALIZED EUCLIDEAN COORINDATES of the point (i.e., [xe; ye; 1] = [ X/Z ; Y/Z; 1] )
// the comments below by GK confirm, although could be more helpful to anyone who reads...
// Interestingly, this is a far-from-simple jacobian, owed to the distortion compensation containing 
// trigonometric expressions of the norm (aka undistorted radius) of [xe ; ye]
cv::Matx<float, 2, 2> ATANCamera::GetProjectionDerivs()
{
  // get the derivative of image frame wrt camera z=1 frame at the last computed projection
  // in the form (d im1/d cam1, d im1/d cam2)
  //             (d im2/d cam1, d im2/d cam2)
  // **** Please NOTE that this is a PROJECTION, hence the fraction is:
  // ****  frac =  rd/ru = 1/w*atan(2*ru*tan(w/2)) / ru 
  // ****   where ru = sqrt(xe^2+ye^2)
  float dFracBydx;
  float dFracBydy;
  // Now obtaining the cached 2*tan(mdW/s) denominator of the  
  float &k = md2Tan; // k = 2 * tan(w / 2)
  // We are reusing the ready-made image coordinates of the previous projection!
  float &x = mvLastCam[0]; 
  float &y = mvLastCam[1]; 
  // And the ready-made UNDISTORTED radius 
  float ru = mdLastR * mdDistortionEnabled; // the result here is mdLastR or 0.0, ru = sqrt(xe^2 + ye^2)
  
  // if radius small, then the derivatives fo the correction fraction r'/r (Devernay - Faugeras) has zeros derivatives in terms of [xe; ye]
  if(ru < 0.01)  {
      dFracBydx = 0.0;
      dFracBydy = 0.0;
  }
  // otherwise, things get tough .... we need to get the derivative of the fraction in terms of xe and ye inside square->square root -> tangent....
  else {
      dFracBydx =  ( mdWinv *  k / (1 + k*k*ru*ru) - mdLastFactor ) * x / (ru*ru); 
      
      dFracBydy =  ( mdWinv *  k / (1 + k*k*ru*ru) - mdLastFactor ) * y / (ru*ru); 
      
      
    }
  
  cv::Matx<float,2 , 2> m2Derivs;
  
  m2Derivs(0, 0) = mvFocal[0] * (dFracBydx * x + mdLastFactor);  
  m2Derivs(1, 0) = mvFocal[1] * (dFracBydx * y);  
  m2Derivs(0, 1) = mvFocal[0] * (dFracBydy * x);  
  m2Derivs(1, 1) = mvFocal[1] * (dFracBydy * y + mdLastFactor); 
  
  return m2Derivs;
}

cv::Matx<float, 2, NUMTRACKERCAMPARAMETERS> ATANCamera::GetCameraParameterDerivs()
{
  // Differentials wrt to the camera parameters
  // Use these to calibrate the camera
  // No need for this to be quick, so do them numerically
  
  cv::Matx<float, 2, NUMTRACKERCAMPARAMETERS> m2NNumDerivs;
  cv::Vec<float, NUMTRACKERCAMPARAMETERS> vNNormal = *mpvvCameraParams; 
  cv::Vec2f v2Cam = mvLastCam;
  cv::Vec2f v2Out = Project(v2Cam);
  for(int i=0; i<NUMTRACKERCAMPARAMETERS; i++) {
    
      // skip the computation of the radial distortion derivative if ignored (mdW = 0)
      if(i == NUMTRACKERCAMPARAMETERS-1 && mdW == 0.0) continue;
      
      // create a vector "vNUpdate" to perturb the i-th camera parameter
      cv::Vec<float, NUMTRACKERCAMPARAMETERS> vNUpdate = cv::Vec<float, NUMTRACKERCAMPARAMETERS>::all(0);
      vNUpdate[i] += 0.001;
      // perturb the i-th camera parameter by 0.001.
      UpdateParams(vNUpdate); 
      // Now project v2Can on the image again
      cv::Vec2f v2Out_B = Project(v2Cam);
      // get the difference of the new projection from the original and divide it by the perturbation step
      // to get the approximation to the ith derivative
      cv::Vec2f DparamsByDpi = (v2Out_B - v2Out) / 0.001;
      // And store it in the Jacobian vector
      m2NNumDerivs(0, i) = DparamsByDpi[0];
      m2NNumDerivs(1, i) = DparamsByDpi[1];
      
      *mpvvCameraParams = vNNormal;
      RefreshParams();
    }
    // zero the derivatives of the radial distortion parameter if disabled
  if(mdW == 0.0) {
    m2NNumDerivs(0, NUMTRACKERCAMPARAMETERS-1) = 0;
    m2NNumDerivs(1, NUMTRACKERCAMPARAMETERS-1) = 0;
  }
  return m2NNumDerivs;
}


cv::Matx<float, 2, NUMTRACKERCAMPARAMETERS> ATANCamera::GetCamParamAnalyticalDerivs()
{
  // Differentials wrt to the camera parameters
  // Use these to calibrate the camera
  // No need for this to be quick, so do them numerically
  
  cv::Matx<float, 2, NUMTRACKERCAMPARAMETERS> m2nJ;
  
  cv::Vec2f v2Cam = mvLastCam;
  double ru = cv::norm(v2Cam); 	// undistorted radius
  double xe = v2Cam[0], ye = v2Cam[1]; // Euclidean normalized coordinates
  
  double imwidth = mvImageSize.width;
  double imheight = mvImageSize.height;
  
  double fx = imwidth * (*mpvvCameraParams)[0];
  //double cx = imwidth * (*mpvvCameraParams)[1];
  double fy = imheight * (*mpvvCameraParams)[2];
  //double cy = imheight * (*mpvvCameraParams)[3];
  double distort = 1.0;
  double rd;
  double w = (*mpvvCameraParams)[4];
  
  double DrdDw = 0; // the derivative of the distorted radius wrt w
  // **** Note that:
  //
  //   rd = atan(2 * ru * tan(w/2) ) / w
  //
  if (w != 0 && mdW != 0) {
    double cc = 2*ru*tan(w/2);
    rd = atan( cc ) / w;
    distort = rd / ru;    
  
    DrdDw = ( ru / ( cos(w/2)*cos(w/2) * (1 + cc*cc) ) - rd ) / w;
  } 
  
  // 1. Derivatives wrt sx = fx / w
  m2nJ(0, 0) =  imwidth * xe * distort;
  m2nJ(1, 0) =  0;
  
  // 2. Derivatives wrt kx = cx / w
  m2nJ(0, 2) = imwidth;
  m2nJ(1, 2) =  0;
  
  // 3. Derivatives wrt sy = fy / h
  m2nJ(0, 1) = 0;
  m2nJ(1, 1) = imheight * ye * distort;
  
  // 4. Derivatives wrt ky = cy / h
  m2nJ(0, 3) = 0;
  m2nJ(1, 3) = imheight;
  
  // 5. Derivatives wrt w
  m2nJ(0, 4) = fx * xe * DrdDw / ru;
  m2nJ(1, 4) = fy * ye * DrdDw / ru;
  
  
    
  
  return m2nJ;
}


// Just perturb the vector of camera parameters by a vector "vUpdate"
void ATANCamera::UpdateParams(cv::Vec<float, NUMTRACKERCAMPARAMETERS> vUpdate)
{ 
  // Update the camera parameters; use this as part of camera calibration.
  (*mpvvCameraParams) = (*mpvvCameraParams) + vUpdate;
  
  RefreshParams();
}

void ATANCamera::DisableRadialDistortion()
{
  // Set the radial distortion parameter to zero
  // This disables radial distortion and also disables its differentials
  (*mpvvCameraParams)[NUMTRACKERCAMPARAMETERS-1] = 0.0;
  RefreshParams();
}


/// Project a 2D point on the normalized EUclidean plane onto the OpenGL frustum near plane.
/// In other words, instead of image coordinates, we use [-1, 1] x [-1, 1]
// Here we simply need to use the normalized intrinsic parameters directly.
// Other than that, this the exact same projection function with "Project"
cv::Vec2f ATANCamera::UFBProject(const cv::Matx<float, 2, 1> &vCam)
{
  // Project from camera z=1 plane to UFB, storing intermediate calc results.
  mvLastCam[0] = vCam(0, 0);
  mvLastCam[1] = vCam(1, 0);
  mdLastR = cv::norm(vCam);
  mbInvalid = (mdLastR > mdMaxR);
  mdLastFactor = rtrans_factor(mdLastR);
  mdLastDistR = mdLastFactor * mdLastR;
  mvLastDistCam = mdLastFactor * mvLastCam;
  
  mvLastIm[0] = (*mpvvCameraParams)[2]  + (*mpvvCameraParams)[0] * mvLastDistCam[0];
  mvLastIm[1] = (*mpvvCameraParams)[3]  + (*mpvvCameraParams)[1] * mvLastDistCam[1];
  return mvLastIm;
}

/// Unproject from the openGL "near" plane onto the normalized Euclidean plane.
// This is exactly the same as "Unproject", only instead of image coordinates we have [-1, 1] x [-1, 1] on
// the "near" plane of the openGL frustum (hence the use of normalized intrinsics).
cv::Vec2f ATANCamera::UFBUnProject(const cv::Matx<float, 2, 1> &v2Im)
{
  mvLastIm[0] = v2Im(0, 0);
  mvLastIm[1] = v2Im(1, 0);
  
  mvLastDistCam[0] = (mvLastIm[0] - (*mpvvCameraParams)[2]) / (*mpvvCameraParams)[0];
  mvLastDistCam[1] = (mvLastIm[1] - (*mpvvCameraParams)[3]) / (*mpvvCameraParams)[1];
  mdLastDistR = sqrt(mvLastDistCam[0] * mvLastDistCam[0] + mvLastDistCam[1] * mvLastDistCam[1]);
  mdLastR = invrtrans(mdLastDistR);
  double dFactor;
  if(mdLastDistR > 0.01)
    dFactor =  mdLastR / mdLastDistR;
  else
    dFactor = 1.0;
  mdLastFactor = 1.0 / dFactor;
  mvLastCam = dFactor * mvLastDistCam;
  return mvLastCam;
}


