// George Terzakis 2016
//
// University of Portsmouth
//
// Code based on PTAM by Klein and Murray (Copyright 2008 Isis Innovation Limited)

#ifndef __CALIB_IMAGE_H
#define __CALIB_IMAGE_H
#include "ATANCamera.h"
#include "CalibCornerPatch.h"
#include <vector>
#include "GCVD/SE3.h"
#include "GCVD/Addedutils.h"

#include "OpenCV.h"

const int N_NOT_TRIED=-1;
const int N_FAILED=-2;

struct CalibGridCorner
{
  struct NeighborState
  {
    NeighborState() {val = N_NOT_TRIED;}
    int val;
  };
  
  CalibCornerPatch::Params Params;
  cv::Point2i irGridPos;
  NeighborState aNeighborStates[4];
  
  cv::Mat_<float> GetSteps(std::vector<CalibGridCorner> &vgc); //2x2 matrix
  cv::Mat_<float> mInheritedSteps;
  
  void Draw();
  
  double ExpansionPotential();
};

class CalibImage
{
public:
  
  bool MakeFromImage(cv::Mat_<uchar> &im, cv::Mat &cim);
  RigidTransforms::SE3<> mse3CamFromWorld;
  void DrawImageGrid();
  void Draw3DGrid(ATANCamera &Camera, bool bDrawErrors);
  void GuessInitialPose(ATANCamera &Camera);

  struct ErrorAndJacobians
  {
    cv::Vec2f v2Error;
    cv::Matx<double, 2, 6> m26PoseJac; // 2x6 Jacobian!
    cv::Matx<double, 2, NUMTRACKERCAMPARAMETERS> m2NCameraJac; // 2 x NUMTRACKERCAMPARAMETERS !
    
    // EAJ definitely needs a constructor now...
    ErrorAndJacobians() : m26PoseJac( cv::Matx<double, 2, 6>() ), m2NCameraJac( cv::Matx<double, 2, NUMTRACKERCAMPARAMETERS>() ){}
  };

  std::vector<ErrorAndJacobians> Project(ATANCamera &Camera);

  cv::Mat_<uchar> mim;  // grayscale
  cv::Mat rgbmim;       // BGR
  
protected:
  std::vector<cv::Point2i> mvCorners;
  std::vector<CalibGridCorner> mvGridCorners;
  
  
  bool ExpandByAngle(int nSrc, int nDirn);
  int NextToExpand();
  void ExpandByStep(int n);
  cv::Point2i IR_from_dirn(int nDirn);
 
};




#endif

