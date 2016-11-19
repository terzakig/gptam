// George Terzakis 2016
//
// University of Portsmouth
//
// Code based on PTAM by Klein and Murray (Copyright 2008 Isis Innovation Limited)

#include "Relocaliser.h"

#include "SmallBlurryImage.h"

#include "Persistence/instances.h"

using namespace std;
using namespace Persistence;

Relocaliser::Relocaliser(Map &map, ATANCamera &camera) : mMap(map), mCamera(camera) { }

SE3<> Relocaliser::BestPose()
{
  return mse3Best;
}

bool Relocaliser::AttemptRecovery(KeyFrame::Ptr pKFCurrent) {
  cout <<"DEBUG: ========================================== Attempting Recovery!!!!"<<endl;
  // Ensure the incoming frame has a SmallBlurryImage attached
  if(!pKFCurrent->pSBI)
    pKFCurrent->pSBI = new SmallBlurryImage(*pKFCurrent);
  else
    pKFCurrent->pSBI->MakeFromKF(*pKFCurrent);
  
  // Find the best ZMSSD match from all keyframes in map
  ScoreKFs(pKFCurrent);

  // And estimate a camera rotation from a 3DOF image alignment
  pair<SE2<>, double> result_pair = pKFCurrent->pSBI->IteratePosRelToTarget( *(mMap.vpKeyFrames[mnBest]->pSBI), 6);
  mse2 = result_pair.first;
  double dScore = result_pair.second;
  
  SE3<> se3KeyFramePos = mMap.vpKeyFrames[mnBest]->se3CfromW;
  mse3Best = SmallBlurryImage::SE3fromSE2(mse2, mCamera) * se3KeyFramePos;
  
  if(dScore < PV3::get<double>("Reloc2.MaxScore", 9e6, SILENT))
    return true;
  else 
    return false;
}

// Compare current KF to all KFs stored in map by
// Zero-mean SSD
void Relocaliser::ScoreKFs(KeyFrame::Ptr pKFCurrent)
{
  mdBestScore = 99999999999999.9;
  mnBest = -1;
  
  for(unsigned int i=0; i<mMap.vpKeyFrames.size(); i++)
    {
      double dSSD = pKFCurrent->pSBI->ZMSSD( *(mMap.vpKeyFrames[i]->pSBI) );
      if(dSSD < mdBestScore) {
	
	  mdBestScore = dSSD;
	  mnBest = i;
      }
    }
}

