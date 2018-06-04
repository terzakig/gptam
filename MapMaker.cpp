//        George Terzakis 2016
//
//       University of Portsmouth
//
//
// Cide based on PTAM by Klein and Murray ( Copyright 2008 Isis Innovation Limited )

#include "MapMaker.h"
#include "MapPoint.h"
#include "Bundle.h"

#include "GCVD/image_interpolate.h"
#include "Persistence/instances.h"

#include <fstream>
#include <algorithm>

#include <iostream>
#include <thread>

#include <functional>

#ifdef WIN32
#define WIN32_LEAN_AND_MEAN // exaggeration...
#include <windows.h>
#endif


using namespace std;
using namespace Persistence;

// Constructor sets up internal reference variable to Map.
// Most of the intialisation is done by Reset()..
MapMaker::MapMaker(Map &m, const ATANCamera &cam) : mMap(m), mCamera(cam)
{
  pthread = NULL; // must be NULL the first time it is invoked...
  mbResetRequested = false; // no reset yet from the tracker....
  flag_IsStopped = true; // stopped for now....
  Reset();
  
  start(); // This class USED TO BE a CVD::thread (that inherited Runnable, etc. etc) 
	  // Now start() simply instantiates a thread object and sets the appropriate flags
  GUI.RegisterCommand("SaveMap", GUICommandCallBack, this);
  PV3::Register(mpvdWiggleScale, "MapMaker.WiggleScale", 0.05 , SILENT); // Default to 10cm between keyframes
}

// the start function
void MapMaker::start() {
  
  cout << "attempting to start the mapmaker ..." <<endl;
  if (!flag_IsStopped) return;
  
  // start the trhead...
  //if (pthread != NULL) delete pthread;
  pthread.reset( new std::thread( &MapMaker::run, this) );
  
  
  flag_StopRequest = false; // resume/start the main loop
  
}



void MapMaker::Reset()
{
  
  // This is only called from within the mapmaker thread...
  mMap.Reset(); // purge the map from all mappoints (good and bad...)
  mvFailureQueue.clear(); // clear the list of keyframes and failed observations
  
  while(!mqNewQueue.empty()) mqNewQueue.pop(); // clearing the queue of newly detected points
  
  mMap.vpKeyFrames.clear(); // TODO: actually erase old keyframes - we'll see... no need for now...
  
  mvpKeyFrameQueue.clear(); // TODO: actually erase old keyframes - Noneed...
  
  mbBundleRunning = false;
  mbBundleConverged_Full = true;
  mbBundleConverged_Recent = true;
  mbResetDone = true;
  mbResetRequested = false;
  mbBundleAbortRequested = false;
  
}

// just raise the stop request flag
void MapMaker::stop()
{
  flag_StopRequest = true;
}



// CHECK_RESET is a handy macro which makes the mapmaker thread stop
// what it's doing and reset, if required.
#define CHECK_RESET if(mbResetRequested) {Reset(); continue;};

void MapMaker::run()
{
flag_IsStopped = false; // just entered the main loop function
#ifdef WIN32
  // For some reason, I get tracker thread starvation on Win32 when
   //adding key-frames. Perhaps this will help:
  SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_LOWEST);
#endif

  while(!flag_StopRequest) { // ShouldStop is a CVD::Thread func which return true if the thread is told to exit.
    
      CHECK_RESET;
      
      // Handle any GUI commands encountered..
      while(!mvQueuedCommands.empty()) {
	
	  GUICommandHandler(mvQueuedCommands.begin()->sCommand, mvQueuedCommands.begin()->sParams);
	  mvQueuedCommands.erase(mvQueuedCommands.begin());
	}
      
      // Nothing to do if there is no map yet!
      if(!mMap.IsGood()) continue;
      
      // From here on, mapmaker does various map-maintenance jobs in a certain priority
      // Hierarchy. For example, if there's a new key-frame to be added (QueueSize() is >0)
      // then that takes high priority.
      
      CHECK_RESET;
      
      cout<<"DEBUG: Adjusting recent "<<endl;
      // Should we run local bundle adjustment?
      if(!mbBundleConverged_Recent && QueueSize() == 0)   {
	BundleAdjustRecent();   
	//HandleBadPoints();
      }
	
      
      CHECK_RESET;
      cout <<"DEBUG: Attempting to refind newlymade"<<endl;
      // Are there any newly-made map points which need more s from older key-frames?
      if(mbBundleConverged_Recent && QueueSize() == 0) ReFindNewlyMade();  
      
      CHECK_RESET;
      cout <<"DEBUG: Now Bundle adjusting ALL."<<endl;
      // Run global bundle adjustment?
      if(mbBundleConverged_Recent && !mbBundleConverged_Full && QueueSize() == 0) { 
	BundleAdjustAll();
	//HandleBadPoints();
      }
      
      CHECK_RESET;
      cout<<"DEBUG: Refinding from Failure Queue. "<<endl;
      // Very low priorty: re-find measurements marked as outliers
      if(mbBundleConverged_Recent && mbBundleConverged_Full && rand()%20 == 0 && QueueSize() == 0)
	ReFindFromFailureQueue();
      //cout <<"DEBUG: handling bad points."<<endl;
      CHECK_RESET;
      cout <<"DEBUG: Handling bad points again...."<<endl;
      HandleBadPoints();
      
      CHECK_RESET;
      cout<<"DEBUG: Adding Keyframe from top of queue."<<endl;
      // Any new key-frames to be added?
      if(QueueSize() > 0) AddKeyFrameFromTopOfQueue(); // Integrate into map data struct, and process
      
      
    }
    
    flag_IsStopped = true;// leaving the main loop function
    
}


// Tracker calls this to demand a reset
void MapMaker::RequestReset()
{
  mbResetDone = false;
  mbResetRequested = true;
}

bool MapMaker::ResetDone()
{
  return mbResetDone;
}

// HandleBadPoints() Does some heuristic checks on all points in the map to see if 
// they should be flagged as bad, based on tracker feedback.
void MapMaker::HandleBadPoints()
{
  if (mMap.vpPoints.size() == 0) return;
  //cout <<"DEBUG: handling bad points ..."<<endl;
  vector<MapPoint::Ptr> vGoodPoints;
  for(unsigned int i = 0; i < mMap.vpPoints.size(); i++) {
    
      //if (mMap.vpPoints[i].use_count() > 0) continue; 
	
      MapPoint::Ptr pMP = mMap.vpPoints[i];
      
      
      // Did the tracker see this point as an outlier more often than as an inlier?
      if( pMP->nMEstimatorOutlierCount > 20 && pMP->nMEstimatorOutlierCount > pMP->nMEstimatorInlierCount) {
	pMP->bBad = true;
	
      }
      // put the bad point in the trash bin
      if (pMP->bBad) 
	//vBadPoints.push_back(pMP);
	mMap.vpPointsTrash.push_back(pMP);
      else 
	vGoodPoints.push_back(pMP);
      
  }
  //if (DEBUGBadPoints > 0) cout  <<"Bad point handler found "<<DEBUGBadPoints<<"Bad points "<<endl;
  // All points marked as bad will be erased - erase all records of them
  // from keyframes in which they might have been measured.
  for(unsigned int i=0; i < mMap.vpPointsTrash.size(); i++) {
    
    // you never know... It never hurts to check...
    if(mMap.vpPointsTrash[i].use_count() == 0) continue;

    MapPoint::Ptr pMP = mMap.vpPointsTrash[i];
	
    // disengage keyframe links through measurements
    for(unsigned int j = 0; j < mMap.vpKeyFrames.size(); j++) {
	  
      KeyFrame::Ptr pKF = mMap.vpKeyFrames[j];
      // I dont know if it is possible to have a dereferenced Keyframe::Ptr 
      // but it doesnt hurt to check...
      if ( pKF.use_count() > 0)  
	if ( pKF->mMeasurements.count(pMP) ) pKF->mMeasurements.erase(pMP); // just erase the bad point from the map! 
									        // Dont do the trash bin!!!!!
    }
  
    
  }
  
  //mMap.deleteBadPoints();
  // just assign the good points to the current mappoint vector of the map
  // This way we avoid deleting entries in the point vector in a loop 
  mMap.vpPoints = vGoodPoints;
  // here's a check of a very likely scenario:
  if (mMap.vpPoints.size() < 6) RequestReset();
}

MapMaker::~MapMaker()
{
  mbBundleAbortRequested = true;
  if (!flag_IsStopped) {
    stop(); // requests a stop
    cout << "Waiting for mapmaker to exit the main loop.." << endl;
    pthread->join();
  }
  cout << "Mapmaker destroyed..." << endl;
  
  //delete pthread; 
}


/// Find the 3D coordinates of a point in 2 views from 
// its normalized Euclidean projections and the given relative pose
cv::Vec<float, 3> MapMaker::ReconstructPoint2Views(SE3<float> se3AfromB,    // relative pose in m2 = R * m2 + t fashion
						   const cv::Vec<float, 2> &v2A,    // Normalized Euclidean coordinates in the first view
						   const cv::Vec<float, 2> &v2B)    // Normalized Euclidean coordinates in the second view
{

  // get the rotation and translation separately
  cv::Matx<float, 3, 3> R = se3AfromB.get_rotation().get_matrix();
  cv::Vec<float, 3> t = se3AfromB.get_translation();
  // NOTE!!!! As implied in PTAM, [R|t] transforms the world point M1 to M2 as follows: M2 = R*M1 + t1
  //          This means that the second camera unit orientation vectors are the ROWs of R1
  //          and the position b of the camera in world coordinates is obtained as : b = -R * t
  
  // To find the world point M, we formulate the following cost function 
  //  based on algebraic error (to avoid divisions with depth):
  //
  //    f = ( (1z'*M)*m1 - M )^2 + ( (1z'*(R*M + t)*m2 - (R*M1+t) )^2 
  //
  // where m1 = [x1;y1;1] and m2 = [x2;y2;1] are the normalized Euclidean projections 
  // of the correspondences and 1z = [0;0;1]
  //
  // The solution of the above cost function is:
  //		
  //			M = -inv(W1 + R'*W2*R)*R'*W2*t
  //
  // where W1 = [1, 0, -x1;0, 1, -y1; -x1, -y1, x1^2 + y1^2 - 1]
  //
  //  and, W2 = [1, 0, -x2;0, 1, -y2; -x2, -y2, x2^2 + y2^2 - 1]
  //
  
  // giving coordinates better names...
  float x1 = v2A[0], y1 = v2A[1], x2 = v2B[0], y2 = v2B[1];
  
  float mag2sq = x2*x2 + y2*y2, mag1sq = x1*x1 + y1*y1;
  
  float r00 = R(0, 0), r01 = R(0, 1), r02 = R(0, 2);
  float r10 = R(1, 0), r11 = R(1, 1), r12 = R(1, 2);
  float r20 = R(2, 0), r21 = R(2, 1), r22 = R(2, 2);
	
	
  // 1. R'*W2 (we need the entire matrix...)
  // n.b.: R'*W2 = [Rrow1 - x2*Rrow3, Rrow2 - y2*Rrow3, mag2sq * Rrow3 - x2 * Rrow1 - y2 * Rrow1]
  float RtW2[3][3];
  // Column #1                  Column#2					 
  RtW2[0][0] = r00 - x2 * r20;  RtW2[0][1] = r10 - y2 * r20; 
  RtW2[1][0] = r01 - x2 * r21;  RtW2[1][1] = r11 - y2 * r21;
  RtW2[2][0] = r02 - x2 * r22;  RtW2[2][1] = r12 - y2 * r22;

  // Column #3
  RtW2[0][2] = mag2sq * r20 - x2 * r00 - y2 * r10;
  RtW2[1][2] = mag2sq * r21 - x2 * r01 - y2 * r11;
  RtW2[2][2] = mag2sq * r22 - x2 * r02 - y2 * r12;
	
   
  // 2. Q = R'*W2*R + W1 (this is a PSD matrix, only the upper triangle...)
  float Q[6] = { RtW2[0][0] * r00 + RtW2[0][1] * r10 + RtW2[0][2] * r20 + 1,
		 RtW2[0][0] * r01 + RtW2[0][1] * r11 + RtW2[0][2] * r21 + 0,
		 RtW2[0][0] * r02 + RtW2[0][1] * r12 + RtW2[0][2] * r22 - x1,
			      
			      RtW2[1][0] * r01 + RtW2[1][1] * r11 + RtW2[1][2] * r21 + 1, 
			      RtW2[1][0] * r02 + RtW2[1][1] * r12 + RtW2[1][2] * r22 - y1,
					  
					    RtW2[2][0] * r02 + RtW2[2][1] * r12 + RtW2[2][2] * r22 + mag1sq};
  
    // 3. ksi = -R'*W2*t
  float ksi[3] = {   -( RtW2[0][0] * t[0] + RtW2[0][1] * t[1] + RtW2[0][2] * t[2] ),
		     -( RtW2[1][0] * t[0] + RtW2[1][1] * t[1] + RtW2[1][2] * t[2] ),
		     -( RtW2[2][0] * t[0] + RtW2[2][1] * t[1] + RtW2[2][2] * t[2] )
		 };
  // ***************** inverting Q (just inlining the inversion...) ********************
  float det = +Q[0] * ( Q[3] * Q[5]  - Q[4] * Q[4] )  
	      -Q[1] * ( Q[1] * Q[5]  - Q[4] * Q[2] )  
	      +Q[2] * ( Q[1] * Q[4]  - Q[3] * Q[2] );
  if (fabs(det) < 10E-6) {
	      
    cout << "Found degenerate/ambiguous correspondence!" <<endl;
    
    return cv::Vec3f(0, 0, -1); // return negative depth so that the mapmaker drops the point
  }
  
  // it follows that the inverse of Q will be PSD - do the upper triangle only 
  float Qinv[6];
  Qinv[0] =  ( Q[3] * Q[5] - Q[4] * Q[4] ) / det;
  Qinv[1] = -( Q[1] * Q[5] - Q[2] * Q[4] ) / det;
  Qinv[2] =  ( Q[1] * Q[4] - Q[2] * Q[3] ) / det;
 
  Qinv[3] =  ( Q[0] * Q[5] - Q[2] * Q[2] ) / det;
  Qinv[4] = -( Q[0] * Q[4] - Q[2] * Q[1] ) / det;
  
  Qinv[5] =  ( Q[0] * Q[3] - Q[1] * Q[1] ) / det;

  
  // ***************************** INVERSION DONE HERE !!! ****************************
  
  // FINALLY, obtaining world point M!
  
  cv::Vec<float, 3>  v3M( Qinv[0] * ksi[0] + Qinv[1] * ksi[1] + Qinv[2] * ksi[2],
			  Qinv[1] * ksi[0] + Qinv[3] * ksi[1] + Qinv[4] * ksi[2],
		   	  Qinv[2] * ksi[0] + Qinv[4] * ksi[1] + Qinv[5] * ksi[2]
			  );
  
  return v3M;
  
}

//  		The original PTAM reconstruction method based on Hartley-Zissermann 
// 	       DLT based method (i.e., use cross product to eliminate the divisions by depth)
/*cv::Vec<float, 3> MapMaker::ReconstructPoint2Views(SE3<float> se3AfromB,    // relative pose in m2 = R * m1 + t fashion
					   const cv::Vec2f &v2A,    // Normalized Euclidean coordinates in the first view
					   const cv::Vec2f &v2B)    // Normalized Euclidean coordinates in the second view
{

  cv::Vec2f pB = v2A;
  cv::Vec2f pA = v2B; // oh lord...........................
  
  
  cv::Matx<float, 3, 4> PDash;
  
  cv::Matx<float, 3, 3> R = se3AfromB .get_rotation().get_matrix();
  cv::Vec3f t = se3AfromB.get_translation();
  //PDash.slice<0,0,3,3>() = se3AfromB.get_rotation().get_matrix();
  PDash(0, 0) = R(0, 0); PDash(0, 1) = R(0, 1); PDash(0, 2) = R(0, 2);
  PDash(1, 0) = R(1, 0); PDash(1, 1) = R(1, 1); PDash(1, 2) = R(1, 2);
  PDash(2, 0) = R(2, 0); PDash(2, 1) = R(2, 1); PDash(2, 2) = R(2, 2);
  
  //PDash.slice<0,3,3,1>() = se3AfromB.get_translation().as_col();
  PDash(0, 3) = t[0]; PDash(1, 3) = t[1]; PDash(2, 3) = t[2];
  
  cv::Matx<float, 4, 4> A;
  //A[0][0] = -1.0; A[0][1] =  0.0; A[0][2] = v2B[0]; A[0][3] = 0.0;
  //A[1][0] =  0.0; A[1][1] = -1.0; A[1][2] = v2B[1]; A[1][3] = 0.0;
  A(0, 0) = -1.0; A(0, 1) =  0.0; A(0, 2) = pB[0]; A(0, 3) = 0.0;
  A(1, 0) =  0.0; A(1, 1) = -1.0; A(1, 2) = pB[1]; A(1, 3) = 0.0;
  //A[2] = v2A[0] * PDash[2] - PDash[0];
  //A[3] = v2A[1] * PDash[2] - PDash[1];
  A(2, 0) = pA[0] * PDash(2, 0) - PDash(0, 0);
  A(2, 1) = pA[0] * PDash(2, 1) - PDash(0, 1);
  A(2, 2) = pA[0] * PDash(2, 2) - PDash(0, 2);
  A(2, 3) = pA[0] * PDash(2, 3) - PDash(0, 3);
  
  
  A(3, 0) = pA[1] * PDash(2, 0) - PDash(1, 0);
  A(3, 1) = pA[1] * PDash(2, 1) - PDash(1, 1);
  A(3, 2) = pA[1] * PDash(2, 2) - PDash(1, 2);
  A(3, 3) = pA[1] * PDash(2, 3) - PDash(1, 3);
  
  //SVD<4,4> svd(A);
  cv::Matx<float, 4, 4> U, Vt;
  cv::Vec<float, 4> w;
  cv::SVD::compute(A, w, U, Vt);
  cv::Vec<float, 4> v4Smallest( Vt(3, 0),
				Vt(3, 1),
				Vt(3, 2),
				Vt(3, 3) );
  
  if(v4Smallest[3] == 0.0) v4Smallest[3] = 0.00001;
  return cv::Vec3f( v4Smallest[0] / v4Smallest[3], 
		    v4Smallest[1] / v4Smallest[3],
		    v4Smallest[2] / v4Smallest[3] );
		    
} */




// InitFromStereo() generates the initial match from two keyframes
// and a vector of image correspondences. Uses the 
/*bool MapMaker::InitFromStereo(KeyFrame::Ptr pkF,     // First KF
			      KeyFrame::Ptr pkS,     // Second KF
			      vector<pair<cv::Point2i, cv::Point2i> > &vTrailMatches, // image correspondences
			      SE3<> &se3TrackerPose  // This is the second camera pose referenced... 
			     )
{
  // Map scale. This is the norm of the baseline in the initial map.
  mdWiggleScale = *mpvdWiggleScale; // Cache this for the new map.

  // Recall that we need to restore original camera size at the end!!!!
  //cv::Size2i orgCamSize = mCamera.GetImageSize();
  // In all possibility, there shouldn't be a change in size...
  // There are however some versions of PTAM (such as the eth ptam) that may do trailtracking in different levels, 
  // so saving camera size maybe necessary.
  mCamera.SetImageSize( pkF->aLevels[0].im.size() );

  // A homography match is essentially a record for normalized Euclidean plane correpsondences
  // The following loop creates a list of Euclidean plane correspondence and caches projection derivatives
  std::vector<HomographyMatch> vMatches; 
  for(unsigned int i=0; i<vTrailMatches.size(); i++) {
    
      HomographyMatch m;
      // store image points. They will be useful for homography refinement...
      m.imPtFirst = vTrailMatches[i].first;
      m.imPtSecond = vTrailMatches[i].second;
      // store normalized Euclidean projections in the source and second view (n.b., the names are misleading!)
      m.v2EucPlaneFirst = mCamera.UnProject( vTrailMatches[i].first.x, vTrailMatches[i].first.y  );
      m.v2EucPlaneSecond = mCamera.UnProject( vTrailMatches[i].second.x, vTrailMatches[i].second.y );
      // cache the derivatives of image projections wrt the Euclidean coords
      // NOTE!!!: The caching of the image projection derivatives is NOT necessary with the new iterative refinement.
      //          The reason I left them there is for backward-compatibility with the original PTAM homography refinement.
      m.m2PixelProjectionGrad = mCamera.GetProjectionDerivs();  
      // and put the entry in the list
      vMatches.push_back(m);
    
  }

  // again, sparing us the trouble if few homography matches
  if (vMatches.size() < 4 ) return false;
  
  SE3<> se3;
  bool bGood;
  HomographyInit HomographyInit(&mCamera); // let HomographuInit to use a referenced camera 
					   // so that the MapMaker stays in control of things...
  
  bGood = HomographyInit.Compute(vMatches, 5.0, se3); // set the error threshgold to 5.0 pixels. 
						      // Arbitrary, but better than finding variance...
  
  if(!bGood) {
    
      cout << "  Could not init from stereo pair, try again." << endl;
      return false;
  }
  
  // Check that the initialiser estimated a non-zero baseline
  double dTransMagn = cv::norm( se3.get_translation() );
  
  if(dTransMagn == 0) {
    
      cout << "  Estimated zero baseline from stereo pair, try again." << endl;
      return false;
  }
  // change the scale of the map so the second camera is wiggleScale away from the first
  se3.get_translation() *= mdWiggleScale/dTransMagn;

  
  KeyFrame::Ptr pkFirst = pkF; // no data copying necessary here...
  KeyFrame::Ptr pkSecond = pkS; // no data cpying necessary
  
  // fix the first frame so that bundle adjustment doesnt roam into deep space...
  pkFirst->bFixed = true;
  // set first camera frame to world frame
  pkFirst->se3CfromW = SE3<>();
  // second is not fixed (by default, but just incase, we do the assignment)
  pkSecond->bFixed = false;
  // set rigid transformation to the relative pose we 've just worked
  pkSecond->se3CfromW = se3;
  
  // *** Constructing map from the stereo matches *******.
  //
  // Btw, we need a PatchFinder to refine (or possiblty reject a few points)
  // to sub-pixel accuracy.
  PatchFinder finder;

  for(unsigned int i=0; i<vMatches.size(); i++) {
      // NOTE!!! "_NEC" postfix stands for : "Normalized Euclidean Coordinates"
      // 	 Mappoint members with the postfix "_NEC" are taken as Euclidean vectors 
    // 		 and the points are always lying in the plane Z = 1.
      MapPoint::Ptr p( new MapPoint() );
      
      // Patch source stuff:
      p->pPatchSourceKF = pkFirst;
      p->nSourceLevel = 0;
      // v3Normal_NEC is the normal of the patch of a feature in the Z = 1 Euclidean plane.
      // It is basically pointing towards the negative side and along the Z (optical ) axis.
      // All normals will be (0, 0, -1) for a newly insrted map-point. 
      p->v3Normal_NEC = cv::Vec<float, 3>( 0,0,-1);
      // "irCenter" is the actual image location of the feature,
      p->irCenter = vTrailMatches[i].first; 
      
      // The respective normalized Euclidean projection (has postfix "_NEC")
      p->v3Center_NEC = CvUtils::backproject( mCamera.UnProject(p->irCenter.x, p->irCenter.y) );
      
      // The Euclidean normalized projection of the pixel eactly below the feature. 
      p->v3OneDownFromCenter_NEC = CvUtils::backproject( mCamera.UnProject( p->irCenter.x, p->irCenter.y + 1.0 ) );
      
      // The Euclidean normalized projection of the pixel exactly to the right of the feature.
      p->v3OneRightFromCenter_NEC = CvUtils::backproject( mCamera.UnProject( p->irCenter.x + 1.0,  p->irCenter.y ) );
      
      // Now turn all Eudlidean projections into unit (Direction) vectors in 3D:
      p->v3Center_NEC = CvUtils::normalize( p->v3Center_NEC );
      p->v3OneDownFromCenter_NEC = CvUtils::normalize(p->v3OneDownFromCenter_NEC);
      p->v3OneRightFromCenter_NEC = CvUtils::normalize(p->v3OneRightFromCenter_NEC);
      p->RefreshPixelVectors();

      // ************ The following does subpixel refinement ofthe feature location ****************
      // 
      // Create a template without warping (i.e., simply copy the image patch into the template image)
      // NOTE: Template is done based on the SOURCE keyframe (already a pointer in p)
      finder.MakeTemplateCoarseNoWarp(p);
      // Prepare the patch pixel gradient cumulative gram-matrix and invert it
      finder.prepSubPixGNStep();
      // Now set the starting position ("v2SubPixPos") in the patch finder 
      // to be the recovered/tracked feature location in the SECOND image at 0-level
      finder.SetSubPixPos( cv::Vec2f(vTrailMatches[i].second.x, vTrailMatches[i].second.y) );
      // And now take the second view /keyframe 
      // and run G-N step for until convergence or 10 iterations maximum...
      // The itereation can fail if the resulting perturbation norm is greater than 0.03 
      // (it would make sense that the subpixel correection is small).
      bool bGood = finder.IterateSubPixToConvergence(pkSecond,10);
      if(!bGood) {
	  p->bBad = true; // just set it to true. Who knows?
	  //cout <<" Bad point in subpix recovery. "<<endl;
	  //delete p; // just do nothing! 
	  continue;
      }
      //cout <<"A good subpix point. Wow..."<<endl;
      // ------------------------------ Subpixel refinement done ------------------------
      
      // ***************** Triangulate the feature now ********************
      // Finally, the tracked subpixel coordinates of the feature found by the PatchFinder
      // in the second image!!!
      cv::Vec2f v2SecondPos = finder.GetSubPixPos(); 
      // Finally, Compute world coordinates!!!
      
      // NEW-NEW!!! Now the reconstruction method requires the argumnets in the normal order!!!!
      //         In other words, first point goes first and second point goes second!!!
      //         This means that all calls to previously "ReprojectPoint" must also have their argument 
      //         ordcering swapped back to normal!!! 
      p->v3WorldPos = ReconstructPoint2Views(se3, vMatches[i].v2EucPlaneFirst, mCamera.UnProject(v2SecondPos) );
      
      // delete the point if the depth is negative
      if(p->v3WorldPos[2] < 0.0) {
 	  //delete p; 
	p->bBad = true; // set the bad property  just in case...
	continue;
      }
      //cout <<"Reconstruction succesfull! "<<p->v3WorldPos<<endl;
      // -------------------------------- World point found and accepted -------------------
      
      //  Add to map.
      p->pMMData.reset( new MapMakerData() );
      
      mMap.vpPoints.push_back(p);
      
      // Must create Measurement Entry and add them 
      // to the respective Keyframe (1st and 2nd) measurement lists.
      // a) First measurement entry
      KFMeasurement mFirst;
      mFirst.nLevel = 0;
      // This is an initialization mappoint (SRC_ROOT)
      mFirst.Source = KFMeasurement::SRC_ROOT; 
      // store the image position of the feature at the home/source/root KF
      mFirst.v2RootPos = cv::Vec2f(vTrailMatches[i].first.x, vTrailMatches[i].first.y);
      // Set the subpix flag to true. Feature has been refined in the corresponding (second/base) KF
      mFirst.bSubPix = true;
      // now update the first keyframe's measurements with mFirst
      pkFirst->mMeasurements[p] = mFirst;
      // Also do a loop-back reference by adding a pointer to the keyframe in the 
      // mamaker data of the mappoint.. So now its is constant time search from mappoint to KF-measurement and vice versa...
      p->pMMData->sMeasurementKFs.insert(pkFirst);
      
      // b) Second measurement entry and links... Same stuff basically...
      KFMeasurement mSecond;
      mSecond.nLevel = 0;
      // Now this is a base KF measurement (SRC_TRAIL)
      mSecond.Source = KFMeasurement::SRC_TRAIL; 
      // Store the subpixel position
      mSecond.v2RootPos = finder.GetSubPixPos(); // recall that the last subpixel iteration was on the second KF
      // and set the respective flag
      mSecond.bSubPix = true;
      // Store the measurement inside the base (second) KF's list of measurements (INDEXED by p itself-awesome!)
      pkSecond->mMeasurements[p] = mSecond;
      // Finally, do the loop-back connection from mappoint to the keyframe (and measurement) through the mapmaker data structure
      p->pMMData->sMeasurementKFs.insert(pkSecond);
  }
  
  // I agree with Weiss at this point:
  if(mMap.vpPoints.size()<4) {
    
    cout << "Too few map points to init."<<endl; 
    RequestReset();
    return false;
  }
  
  // store the keyframes in the map list
  mMap.vpKeyFrames.push_back(pkFirst);
  mMap.vpKeyFrames.push_back(pkSecond);
  
  // ******* Now need to further process the features!!!! ****************
  pkFirst->MakeKeyFrame_Rest();
  pkSecond->MakeKeyFrame_Rest();
  
  for(int i=0; i<3; i++) {
   BundleAdjustAll(); // Maybe 3 times would be nice...
   if (mbResetRequested) return false;
  }
  // Estimate the feature depth distribution in the first two key-frames
  // (Needed for epipolar search)
  RefreshSceneDepth(pkFirst);
  RefreshSceneDepth(pkSecond);
  // SLAM scale divided by the mean scene depth (somehow this makes sense as a heuristic - see later comments)
  mdWiggleScaleDepthNormalized = mdWiggleScale / pkFirst->dSceneDepthMean;

  // add points from the 1st pyramid level (epipolar search)
  AddSomeMapPoints(0);
  // add points from the 4th pyramid level (epipolar search)
  AddSomeMapPoints(3);
  // add points from the 3d pyramid level (epipolar search)
  AddSomeMapPoints(1);
  // add points from the 2nd pyramid level (epipolar search)
  AddSomeMapPoints(2);
  
  
  mbBundleConverged_Full = false;
  mbBundleConverged_Recent = false;
  
  // some more bundle adjustment...
  while(!mbBundleConverged_Full) {
    
      BundleAdjustAll();
      if(mbResetRequested) return false;
  }
  // Rotate and translate the map so the dominant plane is at z=0:
  ApplyGlobalTransformationToMap(CalcPlaneAligner());
  mMap.bGood = true;
  se3TrackerPose = pkSecond->se3CfromW;
  
  // restoring camera size!!!!
  //mCamera.SetImageSize(orgCamSize);
  
  cout << "  MapMaker: made initial map with " << mMap.vpPoints.size() << " points." << endl;
  return true; 
} */

// ThinCandidates() Thins out a key-frame's candidate list.
// Candidates are those salient corners where the mapmaker will attempt 
// to make a new map point by epipolar search. We don't want to make new points
// where there are already existing map points, this routine erases such candidates.
// Operates on a single level of a keyframe.
void MapMaker::ThinCandidates(KeyFrame::Ptr pKF, int nLevel)
{
  vector<Candidate> &vCSrc = pKF->aLevels[nLevel].vCandidates;
  vector<Candidate> vCGood;
  vector<cv::Point2i> irBusyLevelPos;
  // Make a list of `busy' image locations, which already have features at the same level
  // or at one level higher.
  meas_it ipKF_Meas;
  for(ipKF_Meas = pKF->mMeasurements.begin(); ipKF_Meas != pKF->mMeasurements.end(); ipKF_Meas++) {
     // we are only interested in measureements the chosen level (nlevel) or the next one (nlevel+1)
      if(!(ipKF_Meas->second.nLevel == nLevel || ipKF_Meas->second.nLevel == nLevel + 1)) continue;
      // Now register the merasurement location by scaling its root coordinates to match the chosen level scale
      irBusyLevelPos.push_back( CvUtils::roundIL( ( 1.0 / LevelScale(nLevel) ) * ipKF_Meas->second.v2RootPos  ) );
  }
  
  // Only keep those candidates further than 10 pixels away from busy positions.
  int nMinMagSquared = 10*10;
  // Going through each candidate in the chosen level (nlevel)
  for(unsigned int candidateIndex = 0; candidateIndex < vCSrc.size(); candidateIndex++) {
    
      cv::Point2i irC = vCSrc[candidateIndex].irLevelPos;
      bool bGood = true;
      // now browing the measurements 
      for(unsigned int j=0; j<irBusyLevelPos.size(); j++) {
	
	  cv::Point2i irB = irBusyLevelPos[j];
	  // reject if distance is more than "nMinMagSquared"
	  if( (irB.x - irC.x) * (irB.x - irC.x) + (irB.y - irC.y) * (irB.y - irC.y) < nMinMagSquared) {
	    
	      bGood = false;
	      break;
	  }
      }
      if(bGood) vCGood.push_back( vCSrc[candidateIndex] );
    } 
  // Now vDSrc contains the "thinned" candidates (i.e., the candidates who are close to actual measurements)
  vCSrc = vCGood; 
}

// Adds map points by epipolar search to the last-added key-frame, at a single
// specified pyramid level. Does epipolar search in the target keyframe as closest by
// the ClosestKeyFrame function.
void MapMaker::AddSomeMapPoints(int nLevel)
{
  KeyFrame::Ptr pkSrc = mMap.vpKeyFrames[mMap.vpKeyFrames.size() - 1]; // The new keyframe
  KeyFrame::Ptr pkTarget = ClosestKeyFrame(pkSrc);   
  Level &l = pkSrc->aLevels[nLevel];

  ThinCandidates(pkSrc, nLevel);
  
  for(unsigned int i = 0; i<l.vCandidates.size(); i++)
    AddPointEpipolar(pkSrc, pkTarget, nLevel, i);
}

// Rotates/translates the whole map and all keyframes
void MapMaker::ApplyGlobalTransformationToMap(SE3<> se3NewFromOld)
{
  // Upgrading Keyframes...
  for(unsigned int i=0; i < mMap.vpKeyFrames.size(); i++)
    mMap.vpKeyFrames[i]->se3CfromW = mMap.vpKeyFrames[i]->se3CfromW * se3NewFromOld.inverse();
  
  // Now updating the map...
  for(unsigned int i=0; i < mMap.vpPoints.size(); i++) {
    
      mMap.vpPoints[i]->v3WorldPos = se3NewFromOld * mMap.vpPoints[i]->v3WorldPos;
      mMap.vpPoints[i]->RefreshPixelVectors();
  }
}

// Applies a global scale factor to the map
void MapMaker::ApplyGlobalScaleToMap(double dScale)
{
  for(unsigned int i=0; i<mMap.vpKeyFrames.size(); i++)
    mMap.vpKeyFrames[i]->se3CfromW.get_translation() *= dScale;
  
  for(unsigned int i=0; i<mMap.vpPoints.size(); i++)
    {
      mMap.vpPoints[i]->v3WorldPos *= dScale;
      mMap.vpPoints[i]->v3PixelGoRight_W *= dScale;
      mMap.vpPoints[i]->v3PixelGoDown_W *= dScale;
      mMap.vpPoints[i]->RefreshPixelVectors();
    }
}

// The tracker entry point for adding a new keyframe;
// the tracker thread doesn't want to hang about, so 
// just dumps it on the top of the mapmaker's queue to 
// be dealt with later, and return.
void MapMaker::AddKeyFrame(KeyFrame::Ptr pKF)
{
  pKF->pSBI = NULL; // Mapmaker uses a different SBI than the tracker, so will re-gen its own
  mvpKeyFrameQueue.push_back(pKF);
  if(mbBundleRunning)   // Tell the mapmaker to stop doing low-priority stuff and concentrate on this KF first.
    mbBundleAbortRequested = true;
}

// Mapmaker's code to handle incoming key-frames.
void MapMaker::AddKeyFrameFromTopOfQueue()
{
  cout <<"DEBUG: Adding KF from Top of Queue"<<endl;
  if(mvpKeyFrameQueue.size() == 0) return;
  
  KeyFrame::Ptr pKF = mvpKeyFrameQueue[0];
  mvpKeyFrameQueue.erase(mvpKeyFrameQueue.begin());
  pKF->MakeKeyFrame_Rest();
  mMap.vpKeyFrames.push_back(pKF);
  // Any measurements? Update the relevant point's measurement counter status map
  for(meas_it ipMP_Meas = pKF->mMeasurements.begin(); ipMP_Meas != pKF->mMeasurements.end(); ipMP_Meas++) {
    
      ipMP_Meas->first->pMMData->sMeasurementKFs.insert(pKF);
      ipMP_Meas->second.Source = KFMeasurement::SRC_TRACKER;
  }
  
  // And maybe we missed some - this now adds to the map itself, too.
  ReFindInSingleKeyFrame(pKF);
  
  AddSomeMapPoints(3);       // .. and add more map points by epipolar search.
  AddSomeMapPoints(0);
  AddSomeMapPoints(1);
  AddSomeMapPoints(2);
  
  mbBundleConverged_Full = false;
  mbBundleConverged_Recent = false;

  cout <<"DEBUG: Added KF from Top of Queue!"<<endl;
  
}

// Tries to make a new map point out of a single candidate point
// by searching for that point in another keyframe, and triangulating
// if a match is found.
bool MapMaker::AddPointEpipolar(KeyFrame::Ptr pKFSrc, 
				KeyFrame::Ptr pKFTarget, 
				int nLevel,
				int nCandidate)
{
  static cv::Mat_<cv::Vec<float, 2> > imUnProj; // the normalized Euclidean projections 
					        // of a frame cached in an image
  static bool bMadeCache = false;
  // Caching image normalized Euclidean backprojections (for ever and ever)
  // just in case, setting the right resolution in the camera
  //cv::Size2i orgCamSize = mCamera.GetImageSize();
  
  if(!bMadeCache) {
    
      imUnProj.create(pKFSrc->aLevels[0].im.rows, pKFSrc->aLevels[0].im.cols);
      //mCamera.SetImageSize(imUnProj.size());
      int x, y;
      for (y = 0; y < imUnProj.rows; y++)
	for (x = 0; x < imUnProj.cols; x++) imUnProj(y, x) = mCamera.UnProject(x, y); // very weird syntax. It makes sense in TooN notation though...
	
      bMadeCache = true;
      // restore original camera size
      //mCamera.SetImageSize(orgCamSize);
  }
  
  
  
  int nLevelScale = LevelScale(nLevel);
  Candidate &candidate = pKFSrc->aLevels[nLevel].vCandidates[nCandidate];
  // position of the feature in its opriginal pyramid level
  cv::Point2i irLevelPos = candidate.irLevelPos;
  // obtaining the position of the feature in the 0-th level
  cv::Vec<float, 2> v2RootPos = LevelZeroPos(irLevelPos, nLevel);
  
  // Get the normalized Euclidean projection of the point in the SOURCE keyframe
  cv::Vec<float, 3> v3Ray_SC = CvUtils::backproject( mCamera.UnProject(v2RootPos) );
  // And normalize it...
  v3Ray_SC = CvUtils::normalize(v3Ray_SC);
  // Finally, express this direction vector in the TARGET keyframe coordibnates
  cv::Vec<float, 3> v3LineDirn_TC = pKFTarget->se3CfromW.get_rotation() * ( pKFSrc->se3CfromW.get_rotation().inverse() * v3Ray_SC );

  
  // Restrict epipolar search to a relatively narrow depth range
  // to increase reliability
  double dMean = pKFSrc->dSceneDepthMean;
  double dSigma = pKFSrc->dSceneDepthSigma;
  
  double dStartDepth = max(mdWiggleScale, dMean - dSigma);
  double dEndDepth = min(40 * mdWiggleScale, dMean + dSigma);
  // The following "tighter" bounds by Weiss in the ETH version simply yield terrible results,
  // because they clearly force the tracker to have limited scope in its search
  
  //double dStartDepth = max(0.0, dMean - dSigma); 
  //double dEndDepth = dMean + dSigma;           
  
  
  // Now obtain the source camera center in the TARGET KF coordinate frame 
  // (We need a baseline vector to define the epipolar plane)
  cv::Vec<float, 3> v3CamCenter_TC = pKFTarget->se3CfromW * pKFSrc->se3CfromW.inverse().get_translation(); 
  
  // NOTE: Now, "v3CamCenter_TC" and "v3LineDirn_TC" are a basis of the epipolar plane in 
  //       the TARGET K coordina frame!!!
  
  
  // NOTE: ************* How the search is done  ************
  //       PTAM cleverly searches on the epipolar line by creating direction vectors 
  //       as linear combinations of the the baseline vector ( "v3CamCenter_TC" ) 
  //       and the direction vector in target KF coorindates ( "v3LineDirn_TC" ).
  //       These combinations always point to (and through) the epipolar line.
  //       *** It is just a question of setting upper and lower bounds in these directions for a reasonable search.
  //       The best idea is to use map-based statistics on the depth.
  //       Thus, the search is between the following:
  //
  //       a. "Lower/start point" : b + ( <depth mean> - sigma) * u
  //       b. "Upper/end point" : a + ( <depth mean> + sigma) * u
  // So now we start wondering along the baseline by creating vectors
  
  // TODO-TODO-TODO: Make this search a more meaningful and robust!!!
  
  cv::Vec3f v3RayStart_TC = v3CamCenter_TC + ( dStartDepth * v3LineDirn_TC );                               
  cv::Vec3f v3RayEnd_TC = v3CamCenter_TC + ( dEndDepth * v3LineDirn_TC );                               

   
  // Discard the case in which the depth of the end ray is smaller than the starting ray
  if(v3RayEnd_TC[2] <= v3RayStart_TC[2]) return false;
  // Same if negative end point is obtained
  if(v3RayEnd_TC[2] <= 0.0 )  return false;
  // But if only the depth of the starting point is negative, then 
  // we simply add a quantity that will make the depth 1/100 of the depth of "v3LineDirn_TC" 
  // NOTE: This is UGLY....
  if(v3RayStart_TC[2] <= 0.0)
    v3RayStart_TC += (0.001 - v3RayStart_TC[2] / v3LineDirn_TC[2]) * v3LineDirn_TC;
  
   
   
  // **** Taking matters on the normalized Euclidean projection plane now...
  // Normalized Euclidean projection of the starting direction 
  cv::Vec2f v2A = CvUtils::pproject(v3RayStart_TC);
  // Normalized Euclidean projection of the end direction
  cv::Vec2f v2B = CvUtils::pproject(v3RayEnd_TC);
  // Ge the epoipolar line direction!
  cv::Vec2f v2AlongProjectedLine = v2A-v2B;
  
  if( v2AlongProjectedLine.dot( v2AlongProjectedLine ) < 0.00000001 ) {
    
      cout << "v2AlongProjectedLine too small." << endl;
      return false;
   }
   // Now we need a normal in order to search up and down
  v2AlongProjectedLine = CvUtils::normalize(v2AlongProjectedLine);
  // Here's the normal of the epipolar line
  cv::Vec<float, 2> v2Normal( v2AlongProjectedLine[1], 
			     -v2AlongProjectedLine[0] );
  
  // Get the projection of the start point in the normal
  double dNormDist = v2A.dot( v2Normal );
  // if it is toop big, then get out of here...
  if(fabs(dNormDist) > mCamera.LargestRadiusInImage() ) return false;
  
  double dMinLen = min(v2AlongProjectedLine.dot( v2A ), v2AlongProjectedLine.dot( v2B ) ) - 0.05;
  double dMaxLen = max(v2AlongProjectedLine.dot( v2A ), v2AlongProjectedLine.dot( v2B ) ) + 0.05;
  
  if(dMinLen < -2.0)  dMinLen = -2.0;
  if(dMaxLen < -2.0)  dMaxLen = -2.0;
  if(dMinLen > 2.0)   dMinLen = 2.0;
  if(dMaxLen > 2.0)   dMaxLen = 2.0;

  // Find current-frame corners which might match this
  PatchFinder Finder;
  Finder.MakeTemplateCoarseNoWarp(pKFSrc, nLevel, irLevelPos);
  if(Finder.TemplateBad())  return false;
  
  
  vector<cv::Vec<float, 2> > &vv2Corners = pKFTarget->aLevels[nLevel].vImplaneCorners;
  vector<cv::Point2i> &vIR = pKFTarget->aLevels[nLevel].vCorners;
  
  
  // storing the new target corners as normalized Euclidean vectors 
  // (using the "imUnProj" static coordinate image to do things quickly...
  if(!pKFTarget->aLevels[nLevel].bImplaneCornersCached) {
    
      for(unsigned int i=0; i<vIR.size(); i++) {  // over all corners in target img..
	//vv2Corners.push_back(imUnProj[ir(LevelZeroPos(vIR[i], nLevel))]);
	cv::Point2i p2LevelPos = CvUtils::IL( LevelZeroPos(vIR[i], nLevel) ); 
	vv2Corners.push_back(imUnProj( p2LevelPos.y, p2LevelPos.x ) ); // n.b. "ir" is simply integer casting in libCVD....
      }
      pKFTarget->aLevels[nLevel].bImplaneCornersCached = true;
  }
  
  
  int nBest = -1;
  int nBestZMSSD = Finder.mnMaxSSD + 1;
  double dMaxDistDiff = mCamera.OnePixelDist() * (4.0 + 1.0 * nLevelScale);
  double dMaxDistSq = dMaxDistDiff * dMaxDistDiff;
  
  for(unsigned int i=0; i<vv2Corners.size(); i++) {  // over all corners in target img..
    
      cv::Vec<float, 2> v2Im = vv2Corners[i];
      double dDistDiff = dNormDist - v2Im .dot( v2Normal );
      if(dDistDiff * dDistDiff  > dMaxDistSq)	continue; // skip if not along epi line
      if(v2Im.dot( v2AlongProjectedLine ) < dMinLen)	continue; // skip if not far enough along line
      if(v2Im.dot( v2AlongProjectedLine ) > dMaxLen)	continue; // or too far
      int nZMSSD = Finder.ZMSSDAtPoint(pKFTarget->aLevels[nLevel].im, vIR[i]);
      
      if(nZMSSD < nBestZMSSD) {
	
	  nBest = i;
	  nBestZMSSD = nZMSSD;
	}
    } 
  
  if(nBest == -1)   return false;   // Nothing found.
  
  //  Found a likely candidate along epipolar ray!!!!
  
  // So first do subpixel iteration and if it converges, reconstruct...
  Finder.prepSubPixGNStep();
  Finder.SetSubPixPos(LevelZeroPos(vIR[nBest], nLevel));
  bool bSubPixConverges = Finder.IterateSubPixToConvergence(pKFTarget,10);
  if(!bSubPixConverges) return false;
  
  // Reconstruct the point (n.b. function arguments are now in correct order)
  // 
  // A brief explanation about arrangememnt of transformations below...
  // 
  // NOTE #1: The point reconstruction is done in the target reference frame(!) 
  //           and NOT in the source's reference (as any sane person would expect).
  //
  // NOTE #2: The reconstructed point is thereafter taken to world coordinates 
  //          by multiplying with the inverse (target keyframe) transform.
  //
  // nb.: Reconstructing wrt source KF would require exactly the same operations. 
  //      My guess is that GK tried to compensate for the weird "reproject" function argument ordering...
  //      But since I changed that back to something more intuitive, the whole thing now looks like weird arrangement again...
  //      Anyway...
  cv::Vec<float, 3> v3New_TC =  ReconstructPoint2Views(  pKFSrc->se3CfromW * pKFTarget->se3CfromW.inverse(), 
										       mCamera.UnProject(Finder.GetSubPixPos()), 
										       mCamera.UnProject(v2RootPos) 
										    );
  
  // Checking if the reconstructed point came with negative depth (It happens all the time!)		    
  if (v3New_TC[2] < 0.0) return false;  
 
  // ******** Now doing a familiar process... Registering a new mappoint! *********
  // Create the mappoint
  MapPoint::Ptr pNew( new MapPoint );
  // assign world coordinates
  pNew->v3WorldPos = pKFTarget->se3CfromW.inverse() * v3New_TC;
  // Create the MapMakerData structure 
  // (essentially contains the measurements in a per KF fashion)
  pNew->pMMData.reset( new MapMakerData() );
  
  // Register the source KF in the mappoint
  pNew->pPatchSourceKF = pKFSrc;
  // Pyramid level in which the source feature was found
  pNew->nSourceLevel = nLevel;
  // The normal of the patch in the normalized Euclidean plane (always [0, 0, -1] )
  pNew->v3Normal_NEC = cv::Vec<float, 3>( 0,0,-1);
  // Coordinates of the feature in the respective pyramid level
  pNew->irCenter = irLevelPos;
  // The normalized Euclidean coordinates of the point (in the world).
  pNew->v3Center_NEC = CvUtils::backproject( mCamera.UnProject(v2RootPos) );
  // Normalized Euclidean coordinates of the +1 pixel to the right (in the first level ofcourse)
  pNew->v3OneRightFromCenter_NEC = CvUtils::backproject( mCamera.UnProject( v2RootPos[0] + nLevelScale, v2RootPos[1] ) );
  // Normalized Euclidean coordinates of the +1 pixel down. 
  pNew->v3OneDownFromCenter_NEC = CvUtils::backproject( mCamera.UnProject( v2RootPos[0], v2RootPos[1] + nLevelScale ) );
  
  // Normalize the "batsignal" rays...
  pNew->v3Center_NEC = CvUtils::normalize( pNew->v3Center_NEC );
  pNew->v3OneDownFromCenter_NEC = CvUtils::normalize(pNew->v3OneDownFromCenter_NEC);
  pNew->v3OneRightFromCenter_NEC = CvUtils::normalize(pNew->v3OneRightFromCenter_NEC);
  
  // Now compute the back-projection of the Bat-Signal in the map.
  // and create two vectors : "v3PixelGoRight_W" and "pNew->v3PixelGoDown_W"
  // which are the translations from the world point to the respective 
  // backprojections of the "right" and "down" pixels in the world.
  // These translation will be necessary to work-out local affine distortion in a subsequent 
  // tracked location of the feature in another KF.
  pNew->RefreshPixelVectors();
  
  // register the mappoint in the map  
  mMap.vpPoints.push_back(pNew);
  // amd register the mappoint also in the queue of newly made points
  mqNewQueue.push(pNew);
  // FINALLY, we need to register the two measurements (source KF and the second KF)
  // in the MapMakerData structure of the mappoint
  // NOTE!!! In the code that follows, we register TWO DISTINCT measurements with just one KFMeasurement structure (m)
  //         The assignment operator will create distinct copies in the map of <KeyFrame*, KFMeasurement> pairs...
  KFMeasurement m;
  // Root measurement
  m.Source = KFMeasurement::SRC_ROOT; // This is a Root feature (but from a higher pyramidal level most likely...) 
  m.v2RootPos = v2RootPos;
  m.nLevel = nLevel;
  m.bSubPix = true;
  pKFSrc->mMeasurements[pNew] = m;

  // the tracked measurement
  m.Source = KFMeasurement::SRC_EPIPOLAR; // This feature was found with epipolar search
  m.v2RootPos = Finder.GetSubPixPos();
  pKFTarget->mMeasurements[pNew] = m;
  
  // Don't forget to register the source and second/target KF in the 
  // respective list in Mapmaker data...
  pNew->pMMData->sMeasurementKFs.insert(pKFSrc);
  pNew->pMMData->sMeasurementKFs.insert(pKFTarget);
  
  // All done!
  return true;
}

/// Returns the norm of the difference between KF positions in the WORLD frame
double MapMaker::BaselineLength(KeyFrame::Ptr pk1, KeyFrame::Ptr pk2)
{
  cv::Vec<float, 3> v3WCamPos1 = pk1->se3CfromW.inverse().get_translation();
  cv::Vec<float, 3> v3WCamPos2 = pk2->se3CfromW.inverse().get_translation();
  cv::Vec<float, 3> b = v3WCamPos2 - v3WCamPos1;
  
  return cv::norm(b);
}

/// Returns the N-closest Keyframes in terms of BASELINE
vector<std::shared_ptr<KeyFrame> > MapMaker::NClosestKeyFrames(KeyFrame::Ptr pk, unsigned int N) {
  
  vector<pair<double, KeyFrame::Ptr > > vKFandScores;
  for(unsigned int i=0; i<mMap.vpKeyFrames.size(); i++) {
    
      if(mMap.vpKeyFrames[i] == pk) continue;
      // get baseline distance of the current KF from k
      double dDist = BaselineLength(pk, mMap.vpKeyFrames[i]);
      // store the distance
      vKFandScores.push_back( make_pair(dDist, mMap.vpKeyFrames[i]) );
  }
  
  if(N > vKFandScores.size())  N = vKFandScores.size();
  // just sort partially upot to N smallest elements
  partial_sort(vKFandScores.begin(), vKFandScores.begin() + N, vKFandScores.end());
  // copy the first N-keyframes in the sorted list 
  // into a new list of keyframes 
  vector<KeyFrame::Ptr> vResult;
  for(unsigned int i=0; i<N; i++)
    vResult.push_back(vKFandScores[i].second);
  // return the list
  return vResult;
}

/// Returns the keyframe with the shortest baseline from the keyframe passed 
KeyFrame::Ptr MapMaker::ClosestKeyFrame(KeyFrame::Ptr pk) {
  
  double dClosestDist = 9999999999.9;
  int nClosest = -1;
  for(unsigned int i=0; i<mMap.vpKeyFrames.size(); i++) {
    
      if(mMap.vpKeyFrames[i] == pk) continue;
      double dDist = BaselineLength(pk, mMap.vpKeyFrames[i]);
      if(dDist < dClosestDist) {
	  dClosestDist = dDist;
	  nClosest = i;
      }
  }
  assert(nClosest != -1 && "For some reason I ended up with negative closest distance! CHECK YOUR CODE!!!!");
  return mMap.vpKeyFrames[nClosest];
}

/// Returns the both the keyframe and the respective shortest baseline from the keyframe passed 
KeyFrame::Ptr MapMaker::ClosestKeyFrame(KeyFrame::Ptr pk, double &dClosestDist) {
  
  dClosestDist = 9999999999.9;
  int nClosest = -1;
  for(unsigned int i=0; i<mMap.vpKeyFrames.size(); i++) {
    
      if(mMap.vpKeyFrames[i] == pk) continue;
      double dDist = BaselineLength(pk, mMap.vpKeyFrames[i]);
      if(dDist < dClosestDist) {
	  dClosestDist = dDist;
	  nClosest = i;
      }
  }
  
  assert(nClosest != -1 && "For some reason I ended up with negative closest distance! CHECK YOUR CODE!!!!");
  
  return mMap.vpKeyFrames[nClosest];
}



/// This function asseses the current KF distance from the rest of the sMeasurementKFs and 
/// decides on whether a new KF is needed/
bool MapMaker::NeedNewKeyFrame(KeyFrame::Ptr pKFCurrent)
{
  // get the closest keyframe and distance
  double dist;
  KeyFrame::Ptr closestKF = ClosestKeyFrame(pKFCurrent, dist);
  
  //double dDist = BaselineLength(kCurrent, *pClosest);
  dist *= (1.0 / pKFCurrent->dSceneDepthMean);
  //
  // Weiss doesd something here. TODO: Assess the idea..
  // CHECK WEISS's Addition - CHECK WEISS's Addition - CHECK WEISS's Addition
  // CHECK WEISS's Addition - CHECK WEISS's Addition - CHECK WEISS's Addition
  
  // if the standardize (by the mean) distance is greater than the standardized baseline than return true
  if(dist > PV3::get<double>("MapMaker.MaxKFDistWiggleMult",1.0,SILENT) * mdWiggleScaleDepthNormalized) return true;
  
  return false;
}

// Perform bundle adjustment on all keyframes, all map points
void MapMaker::BundleAdjustAll()
{
  // construct the sets of kfs/points to be adjusted:
  // in this case, all of them
  set<KeyFrame::Ptr> sKFs2Adjust;
  set<KeyFrame::Ptr> sKFsFixed;
  for(unsigned int i=0; i<mMap.vpKeyFrames.size(); i++) {
    
    //if (mMap.vpKeyFrames[i].use_count() > 0)// you never know...
      if(mMap.vpKeyFrames[i]->bFixed) sKFsFixed.insert(mMap.vpKeyFrames[i]);
      else
	sKFs2Adjust.insert(mMap.vpKeyFrames[i]);
  }
  
  set<MapPoint::Ptr> sMapPoints;
  for(unsigned int i=0; i<mMap.vpPoints.size();i++) 
    //if (mMap.vpPoints[i].use_count()>0) // just in case...
    if (!mMap.vpPoints[i]->bBad) // there could actually be bad points, so i added the check
      sMapPoints.insert(mMap.vpPoints[i]);
  
  // It is likely that points may not be enough...
  if (sMapPoints.size() < 6) {
    cout <<"DEBUG: Too few Mappoints to bundle adjust ALL: "<<sMapPoints.size()<<endl;
    RequestReset();
    return; 
  }    
  if (sKFs2Adjust.size() == 0) {
   cout <<"DEBUG: Too few KFs to bundle adjust ALL: "<<sKFs2Adjust.size()<<endl;
    RequestReset();
    return;
  }
  
  BundleAdjust(sKFs2Adjust, sKFsFixed, sMapPoints, false);
}

// Peform a local bundle adjustment which only adjusts
// recently added key-frames
void MapMaker::BundleAdjustRecent()
{
  if(mMap.vpKeyFrames.size() < 8)  
    { // Ignore this unless map is big enough
      mbBundleConverged_Recent = true;
      return;
    }

  // First, make a list of the keyframes we want adjusted in the adjuster.
  // This will be the last keyframe inserted, and its four nearest neighbors
  set<KeyFrame::Ptr> sKFs2Adjust; // list of Keyframes to adjust
  // 1. Get the most recent keyframe from the map
  KeyFrame::Ptr pKFNewest = mMap.vpKeyFrames.back();
  //  ... and store it in the list to adjust
  sKFs2Adjust.insert(pKFNewest);
  // 2. Now, go get the 4 closest keyframes to the last one
  vector<KeyFrame::Ptr> vClosest = NClosestKeyFrames(pKFNewest, 4);
  // .. and add them to the adjustment list
  for(int i=0; i<4; i++)
    if(vClosest[i]->bFixed == false) sKFs2Adjust.insert(vClosest[i]);
  
  // 3. Now we register the mappoints (for adjustment) corresponding to the measurements
  // of the listed Keyframes
  set<MapPoint::Ptr> sMPs2Adjust;
  set<KeyFrame::Ptr>::iterator siKF;
  for(siKF = sKFs2Adjust.begin(); siKF != sKFs2Adjust.end(); siKF++) {
     // Get the list of measurements for this particular keyframe
      map<MapPoint::Ptr, KFMeasurement> &mKFMeasurements = (*siKF)->mMeasurements;
      meas_it ipMP_Meas;
      for(ipMP_Meas = mKFMeasurements.begin(); ipMP_Meas != mKFMeasurements.end(); ipMP_Meas++)
	if (!ipMP_Meas->first->bBad)
	  sMPs2Adjust.insert(ipMP_Meas->first);
	
  }
  
  // 4. Finally, add all fixed keyframes which observed ONLY the above points 
  set<KeyFrame::Ptr> sKFsFixed;
  vector<KeyFrame::Ptr>::iterator viKF;
  for( viKF = mMap.vpKeyFrames.begin(); viKF != mMap.vpKeyFrames.end(); viKF++) {
    
    // if the KF is already in the list, then skip the rest...
      if(sKFs2Adjust.count(*viKF)) continue;
      
      //bool bInclude = false; // why on earth do we need to use this???
      meas_it ipMP_Meas;
      for( ipMP_Meas = (*viKF)->mMeasurements.begin(); ipMP_Meas != (*viKF)->mMeasurements.end(); ipMP_Meas++)
	
	if(sMPs2Adjust.count(ipMP_Meas->first)) {
	  
	    sKFsFixed.insert(*viKF);
	    //bInclude = true;
	    break;
	}
      //if(bInclude) sKFsFixed.insert(*viKF);
  }
  
  BundleAdjust(sKFs2Adjust, sKFsFixed, sMPs2Adjust, true);
}

// Common bundle adjustment code. This creates a bundle-adjust instance, populates it, and runs it.
void MapMaker::BundleAdjust(set<KeyFrame::Ptr> sAdjustSet, set<KeyFrame::Ptr> sFixedSet, set<MapPoint::Ptr> sMapPoints, bool bRecent)
{
  Bundle ba(mCamera);   // Our bundle adjuster
  mbBundleRunning = true;
  mbBundleRunningIsRecent = bRecent;
  
  // The bundle adjuster does different accounting of keyframes and map points;
  // Translation maps are stored:
  map<MapPoint::Ptr, int> mPoint_BundleID;
  map<int, MapPoint::Ptr> mBundleID_Point;
  map<KeyFrame::Ptr, int> mView_BundleID;
  map<int, KeyFrame::Ptr> mBundleID_View;
  
  // Add the keyframes' poses to the bundle adjuster. Two parts: first nonfixed, then fixed.
  set<KeyFrame::Ptr>::iterator ipKF;
  for(ipKF = sAdjustSet.begin(); ipKF!= sAdjustSet.end(); ipKF++) {
    
      int nBundleID = ba.AddCamera((*ipKF)->se3CfromW, (*ipKF)->bFixed);
      // now we must create two-way Look-Up entries 
      // between the bundle adjusted Keyframes(Camera poses) and the 
      // "BundleID" of the pose entries in the bundle adjuster! 
      // This is how the mapmaker tracks the changes in the poses.
      mView_BundleID[*ipKF] = nBundleID;
      mBundleID_View[nBundleID] = *ipKF;
  }
  // fixed camera poses:
  for(ipKF = sFixedSet.begin(); ipKF!= sFixedSet.end(); ipKF++) {
    
      int BundleCamIND = ba.AddCamera((*ipKF)->se3CfromW, true);
      // now we must create two-way Look-Up entries 
      // between the bundle adjusted Keyframes(Camera poses) and the 
      // "BundleID" of the pose entries in the bundle adjuster! 
      // This is how the mapmaker tracks the changes in the poses.
      // NOTE: Both fixed and non-fixed poses get a 2-way entry.
      mView_BundleID[*ipKF] = BundleCamIND;
      mBundleID_View[BundleCamIND] = *ipKF;
  }
  
  // Add 3D points
  set<MapPoint::Ptr>::iterator ipMP;
  for(ipMP = sMapPoints.begin(); ipMP != sMapPoints.end(); ipMP++) {
    
      int BundlePtIND = ba.AddPoint((*ipMP)->v3WorldPos);
      // now we must create two-way Look-Up entries 
      // between the bundle adjusted MapPoints and the 
      // "BundleID" of the mappoint's entries! 
      // This is how the mapmaker tracks the changes in these variables
      mPoint_BundleID[*ipMP] = BundlePtIND ;
      mBundleID_Point[BundlePtIND] = *ipMP;
  }
  
  // Add the relevant point-in-keyframe measurements
  // We scan ALL keyframes in the map
  for(unsigned int KF_Index = 0; KF_Index < mMap.vpKeyFrames.size(); KF_Index++) {
    
      // if there is no bundle camera index for this keyframe, skip...
      if(mView_BundleID.count(mMap.vpKeyFrames[KF_Index]) == 0) continue;
      // But if we HAVE a camera index under this keyframe, then store it.
      int nKF_BundleID = mView_BundleID[ mMap.vpKeyFrames[KF_Index] ];
      
      // Now go through all the measurements of the Keyframe
      for(meas_it ipMP_Meas = mMap.vpKeyFrames[KF_Index]->mMeasurements.begin();
	  ipMP_Meas != mMap.vpKeyFrames[KF_Index]->mMeasurements.end(); ipMP_Meas++ )
	{
	  // if there is NO bundle entry for this point, skip (a *MapPoint stored in "first" of the measurement pair)
	  if(mPoint_BundleID.count(ipMP_Meas->first) == 0) continue;
	  // get the bundle ID if the mappoint
	  int nMP_BundleID = mPoint_BundleID[ipMP_Meas->first];
	  // add the measurement to the adjuster's list
	  double sigma = LevelScale(ipMP_Meas->second.nLevel); // using level scale as standard deviation
	  ba.AddMeasurement(nKF_BundleID, 
			    nMP_BundleID, 
			    ipMP_Meas->second.v2RootPos, 
			    sigma*sigma);
	}
    }
  
  // Run the bundle adjuster. This returns the number of successful iterations
  cout<<"DEBUG: Attempting BA with "<<mMap.vpPoints.size()<<" points in the map..."<<endl;
  int nAccepted = ba.Compute(&mbBundleAbortRequested);
  
  if(nAccepted < 0) {
    
      // Crap: - LM Ran into a serious problem!
      // This is probably because the initial stereo was messed up.
      // Get rid of this map and start again! 
      cout << "!! MapMaker: Cholesky failure in bundle adjust. " << endl
	   << "   The map is probably corrupt: Ditching the map. " << endl;
      mbResetRequested = true;
      return;
  }

  // Bundle adjustment did some updates, apply these to the map
  if(nAccepted > 0) {
      
    cout <<"DEBUG: Updating keyframes and points after BA ..."<<endl;
    map<MapPoint::Ptr,int>::iterator ipMP_ID; // point-index post-BA iterator
    
    // update points
    for( ipMP_ID = mPoint_BundleID.begin() ; ipMP_ID != mPoint_BundleID.end(); ipMP_ID++) {
      
	ipMP_ID->first->v3WorldPos = ba.GetPointCoords(ipMP_ID->second);
    }
    
    // update keyframe coord. frames
    map<KeyFrame::Ptr,int>::iterator ipKF_ID;    // cam-index post-BA iterator
    for( ipKF_ID = mView_BundleID.begin(); ipKF_ID != mView_BundleID.end(); ipKF_ID++) {
      
	ipKF_ID->first->se3CfromW = ba.GetCameraPose(ipKF_ID->second);
    }
    
    if(bRecent) mbBundleConverged_Recent = false;
      
    mbBundleConverged_Full = false;
  }
  // update Converged flags...
  if(ba.Converged()) {
    
      mbBundleConverged_Recent = true;
      
      if(!bRecent) mbBundleConverged_Full = true;
      
      //cout <<"DEBUG: Recent BA Converged!"<<endl;
  }
  //else
  //cout <<"DEBUG: BA did not converge!"<<endl;
  
  mbBundleRunning = false;
  mbBundleAbortRequested = false;
  
  // Handle outlier measurements as pairs of bundle IDs : <Mappoint Bundle ID, KF Bundle ID>:
  vector<pair<int,int> > vOutliers = ba.GetOutlierMeasurements();
  cout <<"DEBUG: BA left "<<vOutliers.size() << " outliers !"<<endl;
  
  for(unsigned int pairIndex = 0; pairIndex < vOutliers.size(); pairIndex++) {
    
      // get the mappoint of the outlier entry from 2-way lookup list
      MapPoint::Ptr pMP = mBundleID_Point[vOutliers[pairIndex].first];
      // get the keyframe of the outlier from the 2-way lookup list
      KeyFrame::Ptr pKF = mBundleID_View[vOutliers[pairIndex].second];
      // now simply get the mesurement indexes by the MP inside the KF
      KFMeasurement &m = pKF->mMeasurements[pMP];
      
      // Is the original source kf considered an outlier? That's bad.
      if(pMP->pMMData->GoodMeasCount() <= 2 || m.Source == KFMeasurement::SRC_ROOT)  pMP->bBad = true;
      else {
	
	  // Do we retry it? Depends where it came from!!
	  if(m.Source == KFMeasurement::SRC_TRACKER || m.Source == KFMeasurement::SRC_EPIPOLAR)
	    mvFailureQueue.push_back( pair<KeyFrame::Ptr, MapPoint::Ptr>(pKF, pMP) );
	  else
	    pMP->pMMData->sNeverRetryKFs.insert(pKF); 
	  
	  pKF->mMeasurements.erase(pMP);
	  pMP->pMMData->sMeasurementKFs.erase(pKF);
	}
    }
    
  
}

// Mapmaker's try-to-find-a-point-in-a-keyframe code. This is used to update
// data association if a bad measurement was detected, or if a point
// was never searched for in a keyframe in the first place. This operates
// much like the tracker! So most of the code looks just like in 
// TrackerData.h.
bool MapMaker::ReFind_Common(KeyFrame::Ptr pKF, MapPoint::Ptr pMP)
{
  //cout <<"DEBUG: ******************** Refinding common ..."<<endl;
  // abort if either a measurement is already in the map, or we've
  // decided that this point-kf combo is beyond redemption
  if(pMP->pMMData->sMeasurementKFs.count(pKF) || pMP->pMMData->sNeverRetryKFs.count(pKF) )
    return false;
  
  static PatchFinder Finder;
  // get the Map point in the KFs camera coodinate frame (this was delivered in a silver platter by the tracker)
  cv::Vec<float, 3> v3Cam = pKF->se3CfromW * pMP->v3WorldPos;
  
  // if 0 depth then dont ever try again the KF with this point
  if(v3Cam[2] < 0.001) {
    
      pMP->pMMData->sNeverRetryKFs.insert(pKF);
      return false;
  }
  // Now, if the depth is acceptable, get the normalized Euclidean projection
  cv::Vec<float, 2> v2EucPlane = CvUtils::pproject(v3Cam);
  // If the projection is out of rhe largest allowable radius on the Euclidean plane,
  // dont ever try the current KF with this point
  if(cv::norm(v2EucPlane) > mCamera.LargestRadiusInImage() ) {
    
      pMP->pMMData->sNeverRetryKFs.insert(pKF);
      return false;
  }
  // If the Euclidean projection is acceptable, get the image projection
  cv::Vec<float, 2> v2Image = mCamera.Project(v2EucPlane);
  // Once again, if projection beyond image radius, dont try the KF with this point again...
  if( mCamera.Invalid() ) {
      pMP->pMMData->sNeverRetryKFs.insert(pKF);
      return false;
  }

  // Finally, check if out-of-bounds in the image
  if(v2Image[0] < 0 || v2Image[1] < 0 || v2Image[0] > pKF->aLevels[0].im.cols - 1 || v2Image[1] > pKF->aLevels[0].im.rows - 1)
  {
      pMP->pMMData->sNeverRetryKFs.insert(pKF);
      return false;
  }
  
  //cout <<"DEBUG: ****************************** About to use the patch finder! "<<endl;
  
  // All being well, we reached this point where we have a valid image projection of the mappoint on the KF
  cv::Matx<float, 2, 2> m2CamDerivs = mCamera.GetProjectionDerivs();
  // The following does two things:
  // a) Works out a warp matrix for the loca feature patch (using the "batsignal").
  // b) Creates a coarse template based on the source KF of the mappoint.
  Finder.MakeTemplateCoarse(pMP, pKF->se3CfromW, m2CamDerivs);
  
  // if making the template failed, dont ever try this KF on the current point
  if(Finder.TemplateBad()) {
    
      pMP->pMMData->sNeverRetryKFs.insert(pKF);
      return false;
  }
  // Now find this putative patch in the target KF
  bool bFound = Finder.FindPatchCoarse(v2Image, pKF, 4);  // Very tight search radius!
  
  
  
  // Again, if not found, dont ever try this KF on the current point
  if(!bFound) {
      pMP->pMMData->sNeverRetryKFs.insert(pKF);
      return false;
  }
  
  // If we found something, generate a measurement struct 
  KFMeasurement measurement;
  measurement.nLevel = Finder.GetLevel();
  measurement.Source = KFMeasurement::SRC_REFIND;
  
  // if search at higher pyramid levels, do subpix refinement
  // and return the Level - 0 scaled subpixel position
  if(Finder.GetLevel() > 0) {
    
      Finder.prepSubPixGNStep();
      Finder.IterateSubPixToConvergence(pKF,8);
      measurement.v2RootPos = Finder.GetSubPixPos();
      measurement.bSubPix = true;
  }
  // Otherwise , if search ing level-0, just return the FAST position
  else {
      measurement.v2RootPos = Finder.GetCoarsePosAsVector();
      measurement.bSubPix = false;
  }
  
  // The following snould NOT happen (but keep the code in case something horrible went wring). 
  // We just cant be doing all of the above
  // when the KF ALREADY contains a measurement on this point...
  if( pKF->mMeasurements.count(pMP) ) {
    
      assert(0 && "You are trying to track a mappoint that has ALREADY been processed!"); // This should never happen, we checked for this at the start.
  }
  
  // FINALLY, register:
  // a) the measurement in the KF's list of measured mappoints
  // and,
  // b) the KF in the Mappoint's respective list of KFs
  pKF->mMeasurements[pMP] = measurement;
  pMP->pMMData->sMeasurementKFs.insert(pKF);
  
  return true;
}

// A general data-association update for a single keyframe
// Do this on a new key-frame when it's passed in by the tracker
int MapMaker::ReFindInSingleKeyFrame(KeyFrame::Ptr pKF) {
  
  // There's gotta be a reason that we wanna copy the map list (probably other threads???)
  // But its a list of pointers ANYWAY...
  // THREAD SAFETY perhaps????????? Need to look into this very seriously....
  vector<MapPoint::Ptr> vToFind;
  for(unsigned int i=0; i<mMap.vpPoints.size(); i++)
     vToFind.push_back(mMap.vpPoints[i]);
  // TODO - TODO - TODO - TODO - TODO - TODO - TODO - TODO - TODO
  // TODO - TODO - TODO - TODO - TODO - TODO - TODO - TODO - TODO
  // Make this look better (and safer...)
  
  
  int nFoundNow = 0;
  for(unsigned int i=0; i<vToFind.size(); i++)
    if( ReFind_Common( pKF, vToFind[i] ) ) nFoundNow++;
  
  return nFoundNow;
}

// When new map points are generated, they're only created from a stereo pair
// this tries to make additional measurements in other KFs which they might
// be in.
void MapMaker::ReFindNewlyMade()
{
  if(mqNewQueue.empty()) return;
  
  int nFound = 0;
  int nBad = 0;
 
  while(!mqNewQueue.empty() && mvpKeyFrameQueue.size() == 0) {
    
      MapPoint::Ptr pNew = mqNewQueue.front();
      mqNewQueue.pop();
      
      if(pNew->bBad) {
	
	  nBad++;
	  continue;
      }
      
      for(unsigned int i=0; i<mMap.vpKeyFrames.size(); i++)
	if( ReFind_Common( mMap.vpKeyFrames[i], pNew ) ) nFound++;
    }
}

// Dud measurements get a second chance.
void MapMaker::ReFindFromFailureQueue()
{
  if(mvFailureQueue.size() == 0) return;
  cout<<"DEBUG: *************************************** Failure Queue Size : "<<mvFailureQueue.size()<<endl;
  sort(mvFailureQueue.begin(), mvFailureQueue.end());
  //vector<pair<KeyFrame::Ptr, MapPoint::Ptr> >::iterator iKF_MP;
  int nFound=0;
  
  for (unsigned int i = 0; i < mvFailureQueue.size(); i++) {
  //for(iKF_MP = mvFailureQueue.begin(); iKF_MP != mvFailureQueue.end(); iKF_MP++)
    pair<KeyFrame::Ptr, MapPoint::Ptr> KF_MP = mvFailureQueue[i];
    if (KF_MP.first.use_count()>0)
      if (KF_MP.second.use_count()>0)
	if( ReFind_Common( KF_MP.first, KF_MP.second ) ) nFound++;
  }

  cout <<"DEBUG: **************************** Erasing failure to the end! "<<endl;
  //mvFailureQueue.erase(mvFailureQueue.begin(), iKF_MP);
  mvFailureQueue.clear();
  
}

// Is the tracker's camera pose in cloud-cuckoo land?
bool MapMaker::IsDistanceToNearestKeyFrameExcessive(KeyFrame::Ptr pKFCurrent)
{
  double dist2Nearest;
  ClosestKeyFrame(pKFCurrent, dist2Nearest);
  
  return dist2Nearest > mdWiggleScale * 10.0;
}

// Find a dominant plane in the map, find an SE3<> to put it as the z=0 plane
SE3<> MapMaker::CalcPlaneAligner() {
  
  unsigned int nPoints = mMap.vpPoints.size();
  if(nPoints < 10) {
      cout << "  MapMaker: CalcPlane: too few points to calc plane." << endl;
      return SE3<>();
  }
  
  int nRansacs = PV3::get<int>("MapMaker.PlaneAlignerRansacs", 100, HIDDEN|SILENT);
  cv::Vec<float, 3> v3BestMean(0, 0, 0);
  cv::Vec<float, 3> v3BestNormal(0, 0, 0);
  double dBestDistSquared = 9999999999999999.9;
  
  for(int i=0; i<nRansacs; i++) {
    
      int A_ind = rand() % nPoints;
      int B_ind = A_ind;
      int C_ind = A_ind;
      
      while(B_ind == A_ind) B_ind = rand() % nPoints;
      
      while(C_ind == A_ind || C_ind == B_ind) C_ind = rand() % nPoints;
      // Pk, we now have random three (possibly non collinear) points
      // randomly chosen, indexed by A_ind, B_ind, C_ind.
      // We get teh barycenter of the triangle to begin with:
      cv::Vec<float, 3> v3BaryCenter = 0.33333333 * ( mMap.vpPoints[A_ind]->v3WorldPos + 
						      mMap.vpPoints[B_ind]->v3WorldPos + 
						      mMap.vpPoints[C_ind]->v3WorldPos);
      
      cv::Vec<float, 3> v3AC = mMap.vpPoints[C_ind]->v3WorldPos  - mMap.vpPoints[A_ind]->v3WorldPos;
      cv::Vec<float, 3> v3AB = mMap.vpPoints[B_ind]->v3WorldPos  - mMap.vpPoints[A_ind]->v3WorldPos;
      cv::Vec<float, 3> v3Normal = v3AC ^ v3AB;
      float dNormalmag = cv::norm(v3Normal);
      if(dNormalmag == 0) continue;
      v3Normal = (1 / dNormalmag) * v3Normal; 
      
      double dSumError = 0.0;
      // Now finding out how good this normal is for the reset of the mappoints
      for(unsigned int i=0; i<nPoints; i++) {
	
	  cv::Vec<float, 3> v3Diff = mMap.vpPoints[i]->v3WorldPos - v3BaryCenter;
	  double dDistSq = v3Diff.dot( v3Diff );
	  if(dDistSq == 0.0) continue;
	  
	  double dNormDist = fabs( v3Diff.dot( v3Normal ) );
	  
	  if(dNormDist > 0.05) dNormDist = 0.05; // arbitrarilly chosen "robust" cut-off bound
	  dSumError += dNormDist;
      }
      if(dSumError < dBestDistSquared) {
	
	  dBestDistSquared = dSumError;
	  v3BestMean = v3BaryCenter;
	  v3BestNormal = v3Normal;
      }
  }
  
  // Done the ransacs, now collect the supposed inlier set
  vector<cv::Vec<float, 3> > vv3Inliers;
  for(unsigned int i=0; i<nPoints; i++) {
    
      cv::Vec<float, 3> v3Diff = mMap.vpPoints[i]->v3WorldPos - v3BestMean;
      
      
      double dDistSq = v3Diff.dot( v3Diff );
      if(dDistSq == 0.0) continue;
      
      double dNormDist = fabs( v3Diff.dot( v3BestNormal ) );
     
      if(dNormDist < 0.05) vv3Inliers.push_back(mMap.vpPoints[i]->v3WorldPos);
  }
  
  // With these inliers, calculate mean and cov
  cv::Vec3f v3MeanOfInliers(0, 0, 0);
  
  
  for(unsigned int i = 0; i < vv3Inliers.size(); i++) 
    v3MeanOfInliers += vv3Inliers[i];
   
  v3MeanOfInliers = ( 1.0 / vv3Inliers.size() ) * v3MeanOfInliers;
  
  cv::Matx<float, 3, 3> m3Cov = cv::Matx<float, 3, 3>::zeros();
  cv::Vec<float, 3> v3Diff; 
  // computing the covariance matrix of the inliers...
  for(unsigned int i=0; i<vv3Inliers.size(); i++) {
    
      v3Diff = vv3Inliers[i] - v3MeanOfInliers;
      
      m3Cov += v3Diff * v3Diff.t();
  }
  
  // get the normal of the plane as the eigenvector
  // with smallest eigenvalue (row #2 of Vt)
  cv::Matx<float, 3, 3> U, Vt;
  cv::Matx<float, 3, 1> w;
  cv::SVD::compute(m3Cov, w, U, Vt);
  cv::Vec<float, 3> v3Normal( Vt(2, 0),
			      Vt(2, 1),
			      Vt(2, 2) );
  
  // Use the version of the normal which points towards the cam center
  if(v3Normal[2] > 0) v3Normal = -v3Normal;
  
  
  // ******* Now we construct a rotation that will align the dominant plane with camera x-z plane
  
  cv::Matx<float, 3, 3> m3Rot = cv::Matx<float, 3, 3>::eye();
  // 1. We want the z-axis of the transformed map to be the normal..
  m3Rot(2, 0) = v3Normal[0]; m3Rot(2, 1) = v3Normal[1]; m3Rot(2, 2) = v3Normal[2];
  
  // 2. We want the new x-axis to be the orthogonalized version wrt to the normal
  //    (i.e., totated a little bit in order to be orthogonal to the normal, but 
  //     still close to its original position... )
  float dot13 = v3Normal[0]; // recall that m3Rot[0] = [1 0 0 ]...
  m3Rot(0, 0) = m3Rot(0, 0) - dot13 * v3Normal[0]; m3Rot(0, 1) = m3Rot(0, 1) - dot13 * v3Normal[1]; m3Rot(0, 2) = m3Rot(0, 2) - dot13 * v3Normal[2];
  // and normalize of course...
  float norm1 = cv::norm(m3Rot.row(0));
  m3Rot(0, 0) /= norm1; m3Rot(0, 1) /= norm1; m3Rot(0, 2) /= norm1;
  
  // 3. Finally, the y-axis has to be orthogonal to the other two (cross-product row3 x row1)
  m3Rot(1, 0) = -v3Normal[2] * m3Rot(0, 1) + v3Normal[1] * m3Rot(0, 2);
  m3Rot(1, 1) =  v3Normal[2] * m3Rot(0, 0) - v3Normal[0] * m3Rot(0, 2);
  m3Rot(1, 2) = -v3Normal[1] * m3Rot(0, 0) + v3Normal[0] * m3Rot(0, 1);
  
   
  SE3<> se3Aligner;
  se3Aligner.get_rotation().get_matrix() = m3Rot;
  
  cv::Vec<float, 3> v3RMean = se3Aligner * v3MeanOfInliers;
  se3Aligner.get_translation() = -v3RMean;
  
return se3Aligner;
}

// Calculates the depth(z-) distribution of map points visible in a keyframe
// This function is only used for the first two keyframes - all others
// get this filled in by the tracker
void MapMaker::RefreshSceneDepth(KeyFrame::Ptr pKF) {
  
  double dSumDepth = 0.0;
  double dSumDepthSquared = 0.0;
  int numMeasurements = 0;
  meas_it ipMP_Meas;
  for(ipMP_Meas = pKF->mMeasurements.begin(); ipMP_Meas!=pKF->mMeasurements.end(); ipMP_Meas++) {
    
      MapPoint::Ptr point = ipMP_Meas->first;
      // point coordinates in the local KF camera frame
      cv::Vec<float, 3> v3MPinKF = pKF->se3CfromW * point->v3WorldPos; 
      dSumDepth += v3MPinKF[2];
      dSumDepthSquared += v3MPinKF[2] * v3MPinKF[2];
      numMeasurements++;
  }
 
  assert(numMeasurements > 2); // If not then something is seriously wrong with this KF!!
  pKF->dSceneDepthMean = dSumDepth / numMeasurements;
  pKF->dSceneDepthSigma = sqrt( (dSumDepthSquared / numMeasurements) - (pKF->dSceneDepthMean) * (pKF->dSceneDepthMean) );
}

void MapMaker::GUICommandCallBack(void* ptr, string sCommand, string sParams)
{
  Command c;
  c.sCommand = sCommand;
  c.sParams = sParams;
  ((MapMaker*) ptr)->mvQueuedCommands.push_back(c);
}

void MapMaker::GUICommandHandler(string sCommand, string sParams)  // Called by the callback func..
{
  if(sCommand=="SaveMap")
    {
      cout << "  MapMaker: Saving the map.... " << endl;
      ofstream ofs("map.dump");
      for(unsigned int i=0; i<mMap.vpPoints.size(); i++)
	{
	  ofs << mMap.vpPoints[i]->v3WorldPos << "  ";
	  ofs << mMap.vpPoints[i]->nSourceLevel << endl;
	}
      ofs.close();
      
      for(unsigned int i=0; i<mMap.vpKeyFrames.size(); i++)
	{
	  ostringstream ost1;
	  ost1 << "keyframes/" << i << ".jpg";
//	  img_save(mMap.vpKeyFrames[i]->aLevels[0].im, ost1.str());
	  
	  ostringstream ost2;
	  ost2 << "keyframes/" << i << ".info";
	  ofstream ofs2;
	  ofs2.open(ost2.str().c_str());
	  ofs2 << mMap.vpKeyFrames[i]->se3CfromW << endl;
	  ofs2.close();
	}
      cout << "  ... done saving map." << endl;
      return;
    }
  
  cout << "! MapMaker::GUICommandHandler: unhandled command "<< sCommand << endl;
  exit(1);
} 












