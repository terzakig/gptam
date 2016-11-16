//   George Terzakis 2016
//
//     University of Portsmouth
//

// Code based on PTAM by Klein and Murray (Copyright 2008 Isis Innovation Limited)


#ifndef __MAPMAKER_H
#define __MAPMAKER_H

#include "OpenCV.h"

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

#endif


















