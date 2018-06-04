//   George Terzakis 2016
//
//     University of Portsmouth
//

// Code based on PTAM by Klein and Murray (Copyright 2008 Isis Innovation Limited)


#ifndef __MAPMAKER_H
#define __MAPMAKER_H

#include "OpenCV.h"

#include "PatchFinder.h"
#include "HomographyInit.h"


#include "Map.h"
#include "KeyFrame.h"
#include "ATANCamera.h"

#include <queue>

#include <thread>
//#include <chrono> 

// Each MapPoint has an associated MapMakerData class
// Where the mapmaker can store extra information
 
struct MapMakerData
{
public: // think itys public by default...
  typedef std::shared_ptr<MapMakerData> Ptr;
  
  std::set<KeyFrame::Ptr> sMeasurementKFs;   // Which keyframes has this map point got measurements in?
  std::set<KeyFrame::Ptr> sNeverRetryKFs;    // Which keyframes have measurements failed enough so I should never retry?
  inline int GoodMeasCount() {  return sMeasurementKFs.size(); }
};

// MapMaker now is just a class that contains its thread (good old styd::thread)
class MapMaker {

public:
  MapMaker(Map &m, const ATANCamera &cam);
  ~MapMaker();
  
  // Make a map from scratch. Called by the tracker. 
  // NOTE: This function is now templated so that we can use a generic initializer.
  //       Recommended: Use the Essential Matrix initializer instead of the homography decomposition.
  template<class SLAMInitializer>
  bool InitFromStereo(KeyFrame::Ptr kFirst, KeyFrame::Ptr kSecond, 
		      std::vector<std::pair<cv::Point2i, cv::Point2i> > &vMatches,
		      SE3<> &se3CameraPos);

  bool InitFromStereo_OLD(KeyFrame::Ptr kFirst, KeyFrame::Ptr kSecond,  // EXPERIMENTAL HACK
		      std::vector<std::pair<cv::Point2i, cv::Point2i> > &vMatches,
		      SE3<> &se3CameraPos);
  
  
  void AddKeyFrame(KeyFrame::Ptr k);   // Add a key-frame to the map. Called by the tracker.
  void RequestReset();   // Request that the we reset. Called by the tracker.
  bool ResetDone();      // Returns true if the has been done.
  int  QueueSize() { return mvpKeyFrameQueue.size() ;} // How many KFs in the queue waiting to be added?
  bool NeedNewKeyFrame(KeyFrame::Ptr kCurrent);            // Is it a good camera pose to add another KeyFrame?
  bool IsDistanceToNearestKeyFrameExcessive(KeyFrame::Ptr kCurrent);  // Is the camera far away from the nearest KeyFrame (i.e. maybe lost?)
  
  // start the thread (could be protected in this context I suppose... Anyways, it won't harm anyone as public...)
  void start();
  // External request to stop the thread
  void stop();
  
protected:
  // The thread for our business (replaces the GCD thread)
  std::shared_ptr<std::thread> pthread;
  bool flag_StopRequest; // this flag tells the thread to stop
  bool flag_IsStopped;  // indicates whether the mapmaker is running pr not
  
  Map &mMap;               // The map
  ATANCamera mCamera;      // Same as the tracker's camera: N.B. not a reference variable!
  virtual void run();      // The MapMaker thread code lives here

  // Functions for starting the map from scratch:
  SE3<> CalcPlaneAligner();
  void ApplyGlobalTransformationToMap(SE3<> se3NewFromOld);
  void ApplyGlobalScaleToMap(double dScale);
  
  // Map expansion functions:
  void AddKeyFrameFromTopOfQueue();  
  void ThinCandidates(KeyFrame::Ptr k, int nLevel);
  void AddSomeMapPoints(int nLevel);
  bool AddPointEpipolar(KeyFrame::Ptr kSrc, KeyFrame::Ptr kTarget, int nLevel, int nCandidate);
  // Returns point in ref frame B
  cv::Vec<float, 3> ReconstructPoint2Views(SE3<> se3AfromB, const cv::Vec2f &v2A, const cv::Vec2f &v2B);
  
  // Bundle adjustment functions:
  void BundleAdjust(std::set<KeyFrame::Ptr>, std::set<KeyFrame::Ptr>, std::set<std::shared_ptr<MapPoint> >, bool);
  void BundleAdjustAll();
  void BundleAdjustRecent();

  // Data association functions:
  int ReFindInSingleKeyFrame(KeyFrame::Ptr k);
  void ReFindFromFailureQueue();
  void ReFindNewlyMade();
  void ReFindAll();
  bool ReFind_Common(KeyFrame::Ptr k, std::shared_ptr<MapPoint> p);
  void SubPixelRefineMatches(KeyFrame::Ptr k, int nLevel);
  
  // General Maintenance/Utility:
  void Reset();
  void HandleBadPoints();
  //double DistToNearestKeyFrame(KeyFrame &kCurrent);
  double BaselineLength(KeyFrame::Ptr k1, KeyFrame::Ptr k2);
  KeyFrame::Ptr ClosestKeyFrame(KeyFrame::Ptr k);
  KeyFrame::Ptr ClosestKeyFrame(KeyFrame::Ptr k, double &dist);
  std::vector<KeyFrame::Ptr> NClosestKeyFrames(KeyFrame::Ptr k, unsigned int N);
  void RefreshSceneDepth(KeyFrame::Ptr pKF);
  

  // GUI Interface:
  void GUICommandHandler(std::string sCommand, std::string sParams);
  static void GUICommandCallBack(void* ptr, std::string sCommand, std::string sParams);
  struct Command {std::string sCommand; std::string sParams; };
  std::vector<Command> mvQueuedCommands;
  

  // Member variables:
  std::vector<KeyFrame::Ptr> mvpKeyFrameQueue;  // Queue of keyframes from the tracker waiting to be processed
  std::vector< std::pair<KeyFrame::Ptr, std::shared_ptr<MapPoint> > > mvFailureQueue; // Queue of failed observations to re-find
  std::queue<std::shared_ptr<MapPoint> > mqNewQueue;   // Queue of newly-made map points to re-find in other KeyFrames
  
  double mdWiggleScale;  // Metric distance between the first two KeyFrames (copied from GVar)
                         // This sets the scale of the map
  Persistence::pvar3<double> mpvdWiggleScale;   // GVar for above
  double mdWiggleScaleDepthNormalized;  // The above normalized against scene depth, 
                                        // this controls keyframe separation
  
  bool mbBundleConverged_Full;    // Has global bundle adjustment converged?
  bool mbBundleConverged_Recent;  // Has local bundle adjustment converged?
  
  // Thread interaction signalling stuff
  bool mbResetRequested;   // A reset has been requested
  bool mbResetDone;        // The reset was done.
  bool mbBundleAbortRequested;      // We should stop bundle adjustment
  bool mbBundleRunning;             // Bundle adjustment is running
  bool mbBundleRunningIsRecent;     //    ... and it's a local bundle adjustment.

  
};




// Generates the initial match from two keyframes
// and a vector of image correspondences. 
// NOTE: The definition of this function needs to be in the header file because of the template.
template<class SLAMInitializer>
bool MapMaker::InitFromStereo(KeyFrame::Ptr pkF,     // First KF
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
  std::vector<InitializerMatch> vMatches; 
  for(unsigned int i=0; i<vTrailMatches.size(); i++) {
    
      InitializerMatch m;
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
  
  
  SLAMInitializer initializer(&mCamera);
  						      
  bGood = initializer.Compute(vMatches, SLAMInitializer::RANSAC_DEFAULT_THRESHOLD_BOUND, se3); 
  if(!bGood) {
    
      cout << "  Could not init from stereo pair, try again." <<endl;
      return false;
  }
  
  // Check that the initialiser estimated a non-zero baseline
  double dTransMagn = cv::norm( se3.get_translation() );
  
  if(dTransMagn == 0) {
    
      cout << "  Estimated zero baseline from stereo pair, try again." << se3 << endl;
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
  // Btw, we need a PatchFinder to refine (or possibly reject a few points)
  // to sub-pixel accuracy.
  PatchFinder finder;

  for(unsigned int i = 0; i < vMatches.size(); i++) {
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
	p->bBad = true; // set the bad property!!!
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
}


#endif


















