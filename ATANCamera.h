// George Terzakis 2016
//
// University of Portsmouth
//
// Code based on PTAM by Klein and Murray (Copyright 2008 Isis Innovation Limited)


// This one uses the ``FOV'' distortion model of
// Deverneay and Faugeras, Straight lines have to be straight, 2001
//
// BEWARE: This camera model caches intermediate results in member variables
// Some functions therefore depend on being called in order: i.e.
// GetProjectionDerivs() uses data stored from the last Project() or UnProject()
// THIS MEANS YOU MUST BE CAREFUL WITH MULTIPLE THREADS
// Best bet is to give each thread its own version of the camera!
//
// Camera parameters are stored in a GVar, but changing the gvar has no effect
// until the next call to RefreshParams() or SetImageSize().
//
// Pixel conventions are as follows:
// For Project() and Unproject(),
// round pixel values - i.e. (0.0, 0.0) - refer to pixel centers
// I.e. the top left pixel in the image covers is centered on (0,0)
// and covers the area (-.5, -.5) to (.5, .5)
//
// Be aware that this is not the same as what opengl uses but makes sense
// for acessing pixels using ImageRef, especially ir_rounded.
//
// What is the UFB?
// This is for projecting the visible image area
// to a unit square coordinate system, with the top-left at 0,0,
// and the bottom-right at 1,1
// This is useful for rendering into textures! The top-left pixel is NOT
// centered at 0,0, rather the top-left corner of the top-left pixel is at 
// 0,0!!! This is the way OpenGL thinks of pixel coords.
// There's the Linear and the Distorting version - 
// For the linear version, can use 
// glMatrixMode(GL_PROJECTION); glLoadIdentity();
// glMultMatrix(Camera.MakeUFBLinearFrustumMatrix(near,far));
// To render un-distorted geometry with full frame coverage.
//

#ifndef __ATAN_CAMERA_H
#define __ATAN_CAMERA_H


#define DEFAULT_IMG_HEIGHT 480
#define DEFAULT_IMG_WIDTH 640


#include <cmath>
#include "Persistence/PVars.h"


#include "OpenCV.h"

#define NUMTRACKERCAMPARAMETERS 5


class CameraCalibrator;
class CalibImage;

// The parameters are:
// 0 - normalized x focal length
// 1 - normalized y focal length
// 2 - normalized x offset
// 3 - normalized y offset
// 4 - w (distortion parameter)

class ATANCamera {
 public:
   // Default camera parameters
  static cv::Vec<float, NUMTRACKERCAMPARAMETERS> mvDefaultParams;
  
  ATANCamera(std::string sName, const cv::Size2i imgsize = cv::Size2i(DEFAULT_IMG_WIDTH, DEFAULT_IMG_HEIGHT) );
  
  // Image size get/set: updates the internal projection params to that target image size.
  inline void SetImageSize(const cv::Size2i &vImageSize) { mvImageSize = vImageSize; RefreshParams(); }

  
  cv::Size2i GetImageSize() {return mvImageSize;};
  void RefreshParams();
  
  // ************ Various projection functions (I moved the projection/backprojection functions here because ther are INLINE ********
  
  // Project from the normalized EUCLIDEAN camera plane (z=1) to image pixels (radian distortion compensation takes place in Euclidean coordinates)
  // while storing intermediate calculation results in member variables.
  // In terms of radial distortion, we MUST distort the Euclidean coordinates before we dump them on the image. Thuse, we premultiply the coporindates
  // with the factor f =  1/w * atan(2*ru*atan(w/2)) / ru (wehere ru = sqrt(xe^2 + ye^2) ) and then project on the image pinhole-style....
  inline cv::Vec2f Project(const cv::Matx<float, 2, 1> &vNormEuc) {
    mvLastCam[0] = vNormEuc(0, 0);
    mvLastCam[1] = vNormEuc(1, 0);
    // get the distance from the origin in the Euclidean projection plane n mdLastR
    mdLastR = cv::norm(mvLastCam); // This is the undistorted (presumably) radius of the normalized Euclidean coordinates
    mbInvalid = (mdLastR > mdMaxR); // We cant have a radius beyond the maximum radius 
				  // (as estimated from image border back-projections/un-projections in refreshparams()
    mdLastFactor = rtrans_factor(mdLastR); // so rtrans_factor is the DISTORTION factor function 
    mdLastDistR = mdLastFactor * mdLastR;  // Get the distorted radius and chache it for potential use in Jacobian computations
    mvLastDistCam = mdLastFactor * mvLastCam; // Now get the distorted coordinates
  
    // having the distorted normalized Euclidean coordinates, we can now project on the image (and chache the result in mvLastIm)...
    mvLastIm[0] = mvCenter[0] + mvFocal[0] * mvLastDistCam[0];
    mvLastIm[1] = mvCenter[1] + mvFocal[1] * mvLastDistCam[1];
  
    return mvLastIm;
  }

  inline cv::Vec2f Project(float xe, float ye) {
    mvLastCam = cv::Vec2f(xe, ye);
    // get the distance from the origin in the Euclidean projection plane n mdLastR
    mdLastR = cv::norm(mvLastCam); // This is the undistorted (presumably) radius of the normalized Euclidean coordinates
    mbInvalid = (mdLastR > mdMaxR); // We cant have a radius beyond the maximum radius 
				  // (as estimated from image border back-projections/un-projections in refreshparams()
    mdLastFactor = rtrans_factor(mdLastR); // so rtrans_factor is the DISTORTION factor function 
    mdLastDistR = mdLastFactor * mdLastR;  // Get the distorted radius and chache it for potential use in Jacobian computations
    mvLastDistCam = mdLastFactor * mvLastCam; // Now get the distorted coordinates
  
    // having the distorted normalized Euclidean coordinates, we can now project on the image (and chache the result in mvLastIm)...
    mvLastIm[0] = mvCenter[0] + mvFocal[0] * mvLastDistCam[0];
    mvLastIm[1] = mvCenter[1] + mvFocal[1] * mvLastDistCam[1];
  
    return mvLastIm;
  }

  // Un-project from image pixel coords to the  normalized Euclidean (z=1) camera  plane
  // while storing intermediate calculation results in member variables
  inline cv::Vec2f UnProject(const cv::Matx<float, 2, 1> &v2Im) {
    
    // store image location
    mvLastIm[0] = v2Im(0, 0); 
    mvLastIm[1] = v2Im(1, 0); 
    // Now unproject to a distorted Euclidean space
    mvLastDistCam[0] = (mvLastIm[0] - mvCenter[0]) * mvInvFocal[0];
    mvLastDistCam[1] = (mvLastIm[1] - mvCenter[1]) * mvInvFocal[1];
    // Now, mvLastDistCam contains the DISTORTED Euclidean coordinates of the imaged point
  
    // So now we compensate for radial distortion. Store the distorted radius in mdLastDistR .
    mdLastDistR = cv::norm(mvLastDistCam);
    // mdLastR now becomes the undistorted radius
    mdLastR = invrtrans(mdLastDistR);  // tan(rd * w) / (2 * tan(w/2))
    double dFactor; // the undistortion factor it should be ru/rd = mdLastR / mdLastDistR
    if(mdLastDistR > 0.01) // if very far from the center (hence distortion is probably heavy)
      dFactor =  mdLastR / mdLastDistR;
    else
      dFactor = 1.0;
    // storing the inverse undistortion factor (Yeah I know... Variable names couldn't get any worse....)
    mdLastFactor = 1.0 / dFactor;
    // storing the undistorted Euclidean coordinates
    mvLastCam = dFactor * mvLastDistCam;
  
    // return undistorted normalized Euclidean coordinates
    return mvLastCam;
  }


  inline cv::Vec2f UnProject(float x, float y) {
    
    // store image location
    mvLastIm = cv::Vec2f(x, y); 
    // Now unproject to a distorted Euclidean space
    mvLastDistCam[0] = (mvLastIm[0] - mvCenter[0]) * mvInvFocal[0];
    mvLastDistCam[1] = (mvLastIm[1] - mvCenter[1]) * mvInvFocal[1];
    // Now, mvLastDistCam contains the DISTORTED Euclidean coordinates of the imaged point
  
    // So now we compensate for radial distortion. Store the distorted radius in mdLastDistR .
    mdLastDistR = cv::norm(mvLastDistCam);
    // mdLastR now becomes the undistorted radius
    mdLastR = invrtrans(mdLastDistR);  // tan(rd * w) / (2 * tan(w/2))
    double dFactor; // the undistortion factor it should be ru/rd = mdLastR / mdLastDistR
    if(mdLastDistR > 0.01) // if very far from the center (hence distortion is probably heavy)
      dFactor =  mdLastR / mdLastDistR;
    else
      dFactor = 1.0;
    // storing the inverse undistortion factor (Yeah I know... Variable names couldn't get any worse....)
    mdLastFactor = 1.0 / dFactor;
    // storing the undistorted Euclidean coordinates
    mvLastCam = dFactor * mvLastDistCam;
  
    // return undistorted normalized Euclidean coordinates
    return mvLastCam;
  }
  
  cv::Vec2f UFBProject(const cv::Matx<float, 2, 1> &camframe);
  cv::Vec2f UFBUnProject(const cv::Matx<float, 2, 1> &camframe);
  inline cv::Vec2f UFBLinearProject(const cv::Matx<float, 2, 1> &camframe);
  inline cv::Vec2f UFBLinearUnProject(const cv::Matx<float, 2, 1> &fbframe);
  
  cv::Matx<float, 2, 2> GetProjectionDerivs(); // 2x2 Projection jacobian
  
  inline bool Invalid() {  return mbInvalid;}
  inline double LargestRadiusInImage() {  return mdLargestRadius; }
  inline double OnePixelDist() { return mdOnePixelDist; }
  
  // The z=1 plane bounding box of what the camera can see
  inline cv::Vec2f ImplaneTL(); 
  inline cv::Vec2f ImplaneBR(); 

  // OpenGL helper function
  cv::Matx<float, 4, 4> MakeUFBLinearFrustumMatrix(float near, float far); // Returns A 4x4 matrix
  
  // Feedback for Camera Calibrator
  double PixelAspectRatio() { return mvFocal[1] / mvFocal[0];}
  
  
  
  
  
 protected:
  Persistence::pvar3<cv::Vec<float, NUMTRACKERCAMPARAMETERS> > mpvvCameraParams; // The actual camera parameters
  
  cv::Matx<float, 2, NUMTRACKERCAMPARAMETERS> GetCamParamAnalyticalDerivs();
  cv::Matx<float, 2, NUMTRACKERCAMPARAMETERS> GetCameraParameterDerivs(); // 2x NUMTRACKERCAMPARAMETERS
  void UpdateParams(cv::Vec<float, NUMTRACKERCAMPARAMETERS> vUpdate);
  void DisableRadialDistortion();
  
  // Cached from the last project/unproject:
  cv::Vec2f mvLastCam;      // Last z=1 coord
  cv::Vec2f mvLastIm;       // Last image/UFB coord
  cv::Vec2f mvLastDistCam;  // Last distorted z=1 coord
  double mdLastR;           // Last z=1 radius
  double mdLastDistR;       // Last z=1 distorted radius
  double mdLastFactor;      // Last ratio of z=1 radii
  bool mbInvalid;           // Was the last projection invalid?
  
  // Cached from last RefreshParams:
  float mdLargestRadius; // Largest R in the image
  float mdMaxR;          // Largest R for which we consider projection valid
  float mdOnePixelDist;  // z=1 distance covered by a single pixel offset (a rough estimate!)
  float md2Tan;          // distortion model coeff
  float mdOneOver2Tan;   // distortion model coeff
  float mdW;             // distortion model coeff
  float mdWinv;          // distortion model coeff
  float mdDistortionEnabled; // One or zero depending on if distortion is on or off.
  cv::Vec2f mvCenter;     // Pixel projection center
  cv::Vec2f mvFocal;      // Pixel focal length
  cv::Vec2f mvInvFocal;   // Inverse pixel focal length
  cv::Size2i mvImageSize;  
  cv::Vec2f mvUFBLinearFocal;
  cv::Vec2f mvUFBLinearInvFocal;
  cv::Vec2f mvUFBLinearCenter;
  cv::Vec2f mvImplaneTL;   
  cv::Vec2f mvImplaneBR;
  
  // Radial distortion transformation factor: returns ratio of distorted / undistorted radius.
  // George: This IS the correction factor in the projection model: 
  // You need to multiply the Euclidean normalized coordinates by this factor BEFORE you project them onto the image
  // reason being, we have to "distort" the coordinates before we send them to the image
  /// Returns the distorted radius on the normalized Euclidean plane divided by the undistorted one
  /// This factor can be used verbatoc for projection to the image
  inline float rtrans_factor(float r)
  {
    if(r < 0.001 || mdW == 0.0) return 1.0;
    else 
      return (mdWinv* atan(r * md2Tan) / r); // 1/w * atan(2*ru*tan(w/2)) / ru 
  };

  // Inverse radial distortion: returns un-distorted radius from distorted.
  inline float invrtrans(float r)
  {
    if(mdW == 0.0) return r;
    return(tan(r * mdW) * mdOneOver2Tan); // mdOneOver2Tan is a radial distortion coefficient (see the paper by Devernay - Faugeras)
  };
  
  std::string msName;

  friend class CameraCalibrator;   // friend declarations allow access to calibration jacobian and camera update function.
  friend class CalibImage;
};

// Some inline projection functions:
inline cv::Vec2f ATANCamera::UFBLinearProject(const cv::Matx<float, 2, 1> &camframe)
{
  cv::Vec2f v2Res;
  v2Res[0] = camframe(0, 0) * mvUFBLinearFocal[0] + mvUFBLinearCenter[0];
  v2Res[1] = camframe(1, 0) * mvUFBLinearFocal[1] + mvUFBLinearCenter[1];
  return v2Res;
}

inline cv::Vec2f ATANCamera::UFBLinearUnProject(const cv::Matx<float, 2, 1> &fbframe)
{
  cv::Vec2f v2Res;
  v2Res[0] = (fbframe(0, 0) - mvUFBLinearCenter[0]) * mvUFBLinearInvFocal[0];
  v2Res[1] = (fbframe(1, 0) - mvUFBLinearCenter[1]) * mvUFBLinearInvFocal[1];
  return v2Res;
}


#endif

