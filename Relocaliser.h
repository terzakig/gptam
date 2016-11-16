// ************** George Terzakis 2016 **********
//
//        University of Portsmouth
//
// Code based on PTAM by klein and Murray ( Copyright 2008 Isis Innovation Limited )
//
//
// SmallBlurryImage-based relocaliser
// 
// Each KF stores a small, blurred version of itself;
// Just compare a small, blurred version of the input frame to all the KFs,
// choose the closest match, and then estimate a camera rotation by direct image
// minimisation.

#ifndef __RELOCALISER_H
#define __RELOCALISER_H

#include "ATANCamera.h"
#include "SmallBlurryImage.h"

#include "Map.h"


class Relocaliser
{
public:
  Relocaliser(Map &map, ATANCamera &camera);
  bool AttemptRecovery(KeyFrame::Ptr pKF);
  SE3<> BestPose();
  
protected:
  void ScoreKFs(KeyFrame::Ptr pKFCurrentF);
  Map &mMap;
  ATANCamera mCamera;
  int mnBest;
  double mdBestScore;
  SE2<> mse2;
  SE3<> mse3Best;

};
#endif









