// George Terzakis 2016
//
// University of Portsmouth
//
// Code based on PTAM by Klein and Murray (Copyright 2008 Isis Innovation Limited)
// 


#ifndef __TRACKER_H
#define __TRACKER_H

#include "MapMaker.h"
#include "ATANCamera.h"
#include "MiniPatch.h"
#include "Relocaliser.h"

#include "GCVD/GLHelpers.h"

#include <sstream>
#include <vector>
#include <list>


class TrackerData;

struct Trail    // This struct is used for initial correspondences of the first stereo pair.
{
  MiniPatch mPatch;
  cv::Point2i irCurrentPos;
  cv::Point2i irInitialPos;
};

class Tracker
{
public:
  Tracker(cv::Size2i irVideoSize, const ATANCamera &c, Map &m, MapMaker &mm);
  
  // TrackFrame is the main working part of the tracker: call this every frame.
  void TrackFrame(cv::Mat_<uchar> &imFrame, bool bDraw, cv::Mat &rgbFrame); 

  inline SE3<> GetCurrentPose() { return mse3CamFromWorld;}
  
  // Gets messages to be printed on-screen for the user.
  std::string GetMessageForUser();
  
protected:
  KeyFrame::Ptr pCurrentKF;            // The current KF as a managed pointer
  
  // The major components to which the tracker needs access:
  Map &mMap;                      // The map, consisting of points and keyframes
  MapMaker &mMapMaker;            // The class which maintains the map
  ATANCamera mCamera;             // Projection model
  Relocaliser mRelocaliser;       // Relocalisation module

  cv::Size2i mirSize;          // Image size of whole image
  
  void Reset();                   // Restart from scratch. Also tells the mapmaker to reset itself.
  void RenderGrid();              // Draws the reference grid

  // The following members are used for initial map tracking (to get the first stereo pair and correspondences):
  void TrackForInitialMap();      // This is called by TrackFrame if there is not a map yet.
  enum {TRAIL_TRACKING_NOT_STARTED, 
	TRAIL_TRACKING_STARTED, 
	TRAIL_TRACKING_COMPLETE} mnInitialStage;  // How far are we towards making the initial map?
  void TrailTracking_Start();     // First frame of initial trail tracking. Called by TrackForInitialMap.
  int  TrailTracking_Advance();   // Steady-state of initial trail tracking. Called by TrackForInitialMap.
  std::list<Trail> mlTrails;      // Used by trail tracking
  KeyFrame::Ptr pFirstKF;              // First of the stereo pair
  KeyFrame::Ptr pPreviousFrameKF;      // Used by trail tracking to check married matches
  
  // a helper function for corner comparisons (passed to sort() )
  bool pairCompare(const std::pair<double, cv::Point2i> &left, const std::pair<double, cv::Point2i> &right);

  
  // Methods for tracking the map once it has been made:
  void TrackMap();                // Called by TrackFrame if there is a map.
  void AssessTrackingQuality();   // Heuristics to choose between good, poor, bad.
  void ApplyMotionModel();        // Decaying velocity motion model applied prior to TrackMap
  void UpdateMotionModel();       // Motion model is updated after TrackMap
  int SearchForPoints(std::vector<std::shared_ptr<TrackerData> > &vTD, 
		      int nRange, 
		      int nFineIts);  // Finds points in the image
  cv::Vec<float, 6> CalcPoseUpdate(std::vector<std::shared_ptr<TrackerData> > vTD, 
			   double dOverrideSigma = 0.0, 
			   bool bMarkOutliers = false); // Updates pose from found points.
  SE3<> mse3CamFromWorld;           // Camera pose: this is what the tracker updates every frame.
  SE3<> mse3StartPos;               // What the camera pose was at the start of the frame.
  cv::Vec<float, 6> mv6CameraVelocity;    // Motion model
  double mdVelocityMagnitude;     // Used to decide on coarse tracking 
  double mdMSDScaledVelocityMagnitude; // Velocity magnitude scaled by relative scene depth.
  bool mbDidCoarse;               // Did tracking use the coarse tracking stage?
  
  bool mbDraw;                    // Should the tracker draw anything to OpenGL?
  
  // Interface with map maker:
  int mnFrame;                    // Frames processed since last reset
  int mnLastKeyFrameDropped;      // Counter of last keyframe inserted.
  void AddNewKeyFrame();          // Gives the current frame to the mapmaker to use as a keyframe
  
  // Tracking quality control:
  int manMeasAttempted[LEVELS];
  int manMeasFound[LEVELS];
  enum {BAD, DODGY, GOOD} mTrackingQuality;
  int mnLostFrames;
  
  // Relocalisation functions:
  bool AttemptRecovery();         // Called by TrackFrame if tracking is lost.
  bool mbJustRecoveredSoUseCoarse;// Always use coarse tracking after recovery!

  // Frame-to-frame motion init:
  SmallBlurryImage *mpSBILastFrame;
  SmallBlurryImage *mpSBIThisFrame;
  void CalcSBIRotation();
  cv::Vec<float, 6> mv6SBIRot;
  bool mbUseSBIInit;
  
  // User interaction for initial tracking:
  bool mbUserPressedSpacebar;
  std::ostringstream mMessageForUser;
  
  // GUI interface:
  void GUICommandHandler(std::string sCommand, std::string sParams);
  static void GUICommandCallBack(void* ptr, std::string sCommand, std::string sParams);
  struct Command {std::string sCommand; std::string sParams; };
  std::vector<Command> mvQueuedCommands;
};

#endif






