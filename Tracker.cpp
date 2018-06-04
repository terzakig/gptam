//
//
// ************ George Terzakis 2016 ***********
//
//           U&niversity of Portsmouth 
//
// Code based on PTAM by Klein and Murray

#include "OpenGL.h"
#include "Tracker.h"
#include "MEstimator.h"
#include "ShiTomasi.h"

#include "PatchFinder.h"
#include "TrackerData.h"

// The two possible map initializers:
#include "HomographyInit.h"   // homography initializer 
#include "EssentialInit.h"    // essential matrix initializer (used below, but could be changed)

#include "OpenCV.h"
#include "OpenGL.h"

#include "Persistence/instances.h"
#include "Persistence/GStringUtil.h"


#include <fstream>
#include <fcntl.h>

#include <unistd.h>

using namespace std;
using namespace Persistence;


// The constructor mostly sets up internal reference variables
// to the other classes..
Tracker::Tracker(cv::Size2i irVideoSize, const ATANCamera &c, Map &m, MapMaker &mm) : 
  mMap(m),
  mMapMaker(mm),
  mCamera(c),
  mRelocaliser(mMap, mCamera),
  mirSize(irVideoSize)
{
  
  //mCurrentKF.bFixed = false;
  pCurrentKF.reset(new KeyFrame);
  
  
  GUI.RegisterCommand("Reset", GUICommandCallBack, this);
  GUI.RegisterCommand("KeyPress", GUICommandCallBack, this);
  GUI.RegisterCommand("PokeTracker", GUICommandCallBack, this);
  TrackerData::irImageSize = mirSize;

  mpSBILastFrame = NULL;
  mpSBIThisFrame = NULL;
 
  // Most of the initialisation is done in Reset()
  Reset();
  
 
}

// Resets the tracker, wipes the map.
// This is the main Reset-handler-entry-point of the program! Other classes' resets propagate from here.
// It's always called in the Tracker's thread, often as a GUI command.
void Tracker::Reset()
{
  mbDidCoarse = false;
  mbUserPressedSpacebar = false;
  mTrackingQuality = GOOD;
  mnLostFrames = 0;
  mdMSDScaledVelocityMagnitude = 0;
  
  pCurrentKF->dSceneDepthMean = 1.0;
  pCurrentKF->dSceneDepthSigma = 1.0;
  pCurrentKF->mMeasurements.clear();
  
  mnInitialStage = TRAIL_TRACKING_NOT_STARTED;
  mlTrails.clear();
  mCamera.SetImageSize(mirSize); // just in case...
  
  
  mnLastKeyFrameDropped = -20;
  mnFrame=0;
  mv6CameraVelocity = cv::Vec<float, 6>(0, 0, 0, 0, 0, 0); // could use all() here, but its just 6 zeros....
  mbJustRecoveredSoUseCoarse = false;
  
  // Tell the MapMaker to reset itself.. 
  // this may take some time, since the mapmaker thread may have to wait
  // for an abort-check during calculation, so sleep while waiting.
  // MapMaker will also clear the map.
  mMapMaker.RequestReset();
  cout <<"Waiting for mapmaker to reset ..."<<endl; 
  
  while(!mMapMaker.ResetDone())
#ifndef WIN32
	  usleep(10);
#else
	  Sleep(1);
#endif
}

// TrackFrame is called by System.cpp with each incoming video frame.
// It figures out what state the tracker is in, and calls appropriate internal tracking
// functions. bDraw tells the tracker wether it should output any GL graphics
// or not (it should not draw, for example, when AR stuff is being shown.)
void Tracker::TrackFrame(cv::Mat_<uchar> &imFrame, bool bDraw, cv::Mat &rgbFrame) {
  mbDraw = bDraw;
  mMessageForUser.str("");   // Wipe the user message clean
  
  // allocate a new managed Keyframe
  pCurrentKF.reset( new KeyFrame() );
  
  // clear the measurement list (not very much necessary, but just in case...)
  pCurrentKF->mMeasurements.clear();
  
  // MakeKeyFrame_Lite does the following:
  // a) Create a pyramid of successively decimated images (4-levels).
  // b) Initial detection of FAST corners in each level of the pyramid (WITHOUT second-pass cherry-picking/non-max suppession). 
  pCurrentKF->MakeKeyFrame_Lite(imFrame);

  // Update the small images for the rotation estimator
  static pvar3<double> gvdSBIBlur("Tracker.RotationEstimatorBlur", 0.75, SILENT);
  static pvar3<int> gvnUseSBI("Tracker.UseRotationEstimator", 1, SILENT);
  mbUseSBIInit = *gvnUseSBI; // this flag indicates whether to use an initial estimate 
			     // of rotation between small blurry images (see more comments below).
  
  // mpSBIframeThisFrame is the SBI cache of the tracker.
  // At this point, if SLAM is starting, mpSBIThisFrame should be NULL;
  // in which case, we store the SBI of the current KF in both the last and current SBI cache
  if(!mpSBIThisFrame) {
    
      mpSBIThisFrame = new SmallBlurryImage(*pCurrentKF, *gvdSBIBlur);
      mpSBILastFrame = new SmallBlurryImage(*pCurrentKF, *gvdSBIBlur);
  }
  // But if SLAM has been going on for a while, then the current SBI cache (i.e., mpSBIThisFrame) is not NULL;
  // in fact, it should contain the SBI of the previous keyframe.
  // So we copy the mpSBIframeThisFrame to mpSBIBLastFrame and make a new SBI.
  else {
    
      delete  mpSBILastFrame;
      mpSBILastFrame = mpSBIThisFrame;
      mpSBIThisFrame = new SmallBlurryImage(*pCurrentKF, *gvdSBIBlur);
  }
  
  // From now on we only use the keyframe struct!
  mnFrame++; // increase number of processed frames
  
  // display image and draw FAST corners
  if(mbDraw) {
      
     glRasterPos2i(0, 0); 
     // draw the image
     GLXInterface::glDrawPixelsBGR(rgbFrame);
     // draw FAST free lying corners
     if(PV3::get<int>("Tracker.DrawFASTCorners",1, SILENT)) {
	
	  glColor3f(1,0,1);  glPointSize(1); glBegin(GL_POINTS);
	  for(unsigned int i=0; i<pCurrentKF->aLevels[0].vCorners.size(); i++) 
	   GLXInterface::glVertex(pCurrentKF->aLevels[0].vCorners[i]);
	  glEnd();
      }
  }
  
  // Decide what to do - if there is a map, try to track the map ...
  // More comments on what "good" means coming up!
  
  
  if(mMap.IsGood()) {
    // .. but only if we're not lost!
    if(mnLostFrames < 3) { 
	// the calculation below is simply an optical flow
        // estimation routine applied to a very small, blurred version of the entire frame.
        // The result is questionable, but its there, so whjy not use it for starters?
	if(mbUseSBIInit) CalcSBIRotation();
	
	// Now we need to predict the camera pose
	// based on its previous motion. A bit of help...
	ApplyMotionModel();       // 
	// 
	//cout <<"DEBUG: Tracking map !!!!"<<endl;
	//cout <<"DEBUG: The map currently has "<<mMap.vpPoints.size()<<" points "<<endl;
	TrackMap();               //  These three lines do the main tracking work.
	//cout <<"DEBUG: Exiting trackmap"<<endl;
	UpdateMotionModel();      // 
	  
	AssessTrackingQuality();  //  Check if we're lost or if tracking is poor.
	  
	{ // Provide some feedback for the user: cute..
	  mMessageForUser << "Tracking Map, quality ";
	  if(mTrackingQuality == GOOD)  mMessageForUser << "good.";
	  if(mTrackingQuality == DODGY) mMessageForUser << "poor.";
	  if(mTrackingQuality == BAD)   mMessageForUser << "bad.";
	  mMessageForUser << " Found:";
	  for(int i=0; i<LEVELS; i++) mMessageForUser << " " << manMeasFound[i] << "/" << manMeasAttempted[i];
	    //	    mMessageForUser << " Found " << mnMeasFound << " of " << mnMeasAttempted <<". (";
	    mMessageForUser << " Map: " << mMap.vpPoints.size() << "P, " << mMap.vpKeyFrames.size() << "KF";
	    
	}
	  
	// Heuristics to check if a key-frame should be added to the map:
	// A lot of papers have been written on how to to this......................
	if(  mTrackingQuality == GOOD &&
	     mMapMaker.NeedNewKeyFrame(pCurrentKF) &&
	     mnFrame - mnLastKeyFrameDropped > 20  &&
	     mMapMaker.QueueSize() < 3
	  ) {
	      //mMessageForUser << " Adding key-frame.";
	      //cout <<"Adding keyframe! "<<endl;
	      AddNewKeyFrame();
	    }
    }
    // (lost frames > 3): many frames where discarded. need to recover!!!!!
     else { 
	
	  mMessageForUser << "** Attempting recovery **.";
	  if(AttemptRecovery()) {
	    
	      TrackMap();
	      AssessTrackingQuality();
	    }
     }
     if(mbDraw) RenderGrid();
  } 
  // If there is no map, try to make one.
  else TrackForInitialMap(); 
  
  // GUI interface
  while(!mvQueuedCommands.empty()) {
    
      GUICommandHandler(mvQueuedCommands.begin()->sCommand, mvQueuedCommands.begin()->sParams);
      mvQueuedCommands.erase(mvQueuedCommands.begin());
  }
}

// Try to relocalise in case tracking was lost.
// Returns success or failure as a bool.
// Actually, the SBI relocaliser will almost always return true, even if
// it has no idea where it is, so graphics will go a bit 
// crazy when lost. Could use a tighter SSD threshold and return more false,
// but the way it is now gives a snappier response and I prefer it.
bool Tracker::AttemptRecovery()
{
  bool bRelocGood = mRelocaliser.AttemptRecovery(pCurrentKF);
  
  if(!bRelocGood) return false;
  
  SE3<> se3Best = mRelocaliser.BestPose();
  mse3CamFromWorld = mse3StartPos = se3Best;
  mv6CameraVelocity = cv::Vec<float, 6>(0, 0, 0, 0, 0, 0); // avoiding the ::all(0) static method
  mbJustRecoveredSoUseCoarse = true;
  
  return true;
}

// Draw the reference grid to give the user an idea of wether tracking is OK or not.
void Tracker::RenderGrid() {
  
  // The colour of the ref grid shows if the coarse stage of tracking was used
  // (it's turned off when the camera is sitting still to reduce jitter.)
  if(mbDidCoarse) glColor4f(.0, 0.5, .0, 0.6);
  else
    glColor4f(0,0,0,0.6);
  
  // The grid is projected manually, i.e. GL receives projected 2D coords to draw.
  int nHalfCells = 8;
  int nTot = nHalfCells * 2 + 1;
  cv::Mat_<cv::Vec<float, 2> >  imVertices( nTot, nTot );
  for(int i=0; i<nTot; i++)
    for(int j=0; j<nTot; j++) {
      
	cv::Vec<float, 3> v3 ( (i - nHalfCells) * 0.1, 
			       (j - nHalfCells) * 0.1, 
		                                  0.0 );
	cv::Vec<float, 3> v3Cam = mse3CamFromWorld * v3;
	
	if (v3Cam[2] < 0.001) v3Cam[2] = 0.001;
	// projecting on normalized Euclidean plane (could be done with CvUtils::pproject)
	imVertices(i, j) = mCamera.Project(CvUtils::pproject( v3Cam ) );
    }
  glEnable(GL_LINE_SMOOTH);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glLineWidth(2);
  for(int i=0; i<nTot; i++) {
    
      glBegin(GL_LINE_STRIP);
      for(int j=0; j<nTot; j++)
	glVertex2f( imVertices(i, j)[0], imVertices(i, j)[1] );
      glEnd();
      
      glBegin(GL_LINE_STRIP);
      for(int j=0; j<nTot; j++)
	glVertex2f( imVertices(j, i)[0], imVertices(j, i)[1] );
      glEnd();
    }
  
  glLineWidth(1);
  glColor3f(1,0,0);
}

// GUI interface. Stuff commands onto the back of a queue so the tracker handles
// them in its own thread at the end of each frame. Note the charming lack of
// any thread safety (no lock on mvQueuedCommands).
void Tracker::GUICommandCallBack(void* ptr, string sCommand, string sParams)
{
  Command c;
  c.sCommand = sCommand;
  c.sParams = sParams;
  ((Tracker*) ptr)->mvQueuedCommands.push_back(c);
}

// This is called in the tracker's own thread.
void Tracker::GUICommandHandler(string sCommand, string sParams) { // Called by the callback func.. 
  
  if(sCommand=="Reset") {
      Reset();
      return;
    }

  // KeyPress commands are issued by GLWindow
  if(sCommand=="KeyPress") {
    
    if(sParams == "Space") {
	  
      mbUserPressedSpacebar = true;
    }
    else if(sParams == "r") {
	  
      Reset();
    }
    else if(sParams == "q" || sParams == "Escape") {
	  
      GUI.ParseLine("quit");
    }
    return;
  }
  
  if((sCommand=="PokeTracker")) {
      
    mbUserPressedSpacebar = true;
    return;
  }
    
  
  cout << "! Tracker::GUICommandHandler: unhandled command "<< sCommand << endl;
  exit(1);
}; 

// Routine for establishing the initial map. This requires two spacebar presses from the user
// to define the first two key-frames. Salient points are tracked between the two keyframes
// using cheap frame-to-frame tracking (which is very brittle - quick camera motion will
// break it.) The salient points are stored in a list of `Trail' data structures.
// What action TrackForInitialMap() takes depends on the mnInitialStage enum variable..
void Tracker::TrackForInitialMap()
{
  // MiniPatch tracking threshhold.
  static pvar3<int> gvnMaxSSD("Tracker.MiniPatchMaxSSD", 100000, SILENT);
  MiniPatch::mnMaxSSD = *gvnMaxSSD;
  
  // What stage of initial tracking are we at?
  if(mnInitialStage == TRAIL_TRACKING_NOT_STARTED)  {
    
    // First spacebar = this is the first keyframe
      if(mbUserPressedSpacebar)  {
	
	  mbUserPressedSpacebar = false;
	  TrailTracking_Start();
	  mnInitialStage = TRAIL_TRACKING_STARTED; // switch state to TRAIL_TRACKING_STARTED 
						   // (which implies that we are expecting the second KF anytime soon...)
      }
      else
	mMessageForUser << "Point camera at planar scene and press spacebar to start tracking for initial map." << endl;
      return;
  } // end the case of TRAIL_TRACKING_NOT_STARTED
  
  // Here we handle the case in which we aree waiting for the second KF
  if(mnInitialStage == TRAIL_TRACKING_STARTED) {
    
      int nGoodTrails = TrailTracking_Advance();  // This call actually tracks the trails
      // if less than 10 features were good, then
      // do it all over again.
      
      if(nGoodTrails < 10) { 
	  cout << " Less than 10 correspondences found. Resetting... "<<endl;
	  Reset();
	  return;
      }
      
      
      // If the user pressed spacebar here, 
      // use the list of surviving Trail Matches for 20-view Reconstruction...
      if(mbUserPressedSpacebar) {
	
	  mbUserPressedSpacebar = false;
	  // Unfortunately the mapmaker wont process trail structs, so we need to copy the matches into 
	  // a "smaller" package (i.e. pair<Point2i, Point2i> )
	  vector<pair<cv::Point2i, cv::Point2i> > vMatches;   // This is the format the mapmaker wants for the stereo pairs
	  for(list<Trail>::iterator i = mlTrails.begin(); i!=mlTrails.end(); i++)
	    vMatches.push_back(pair<cv::Point2i, cv::Point2i>(i->irInitialPos, i->irCurrentPos));
	  // Alright! Here comes the hot stuff: 2-view Camera pose and structure estimation...
	  //
	  // NOTE: Using the new templated InitFronStereo with the Essential Matrix Initializer
	  ///      Feel free to change back to homography, but it really works better!
	  //
	  mMapMaker.InitFromStereo<EssentialInit>(pFirstKF, pCurrentKF, vMatches, mse3CamFromWorld);  
	  mnInitialStage = TRAIL_TRACKING_COMPLETE;
	  //cout <<"completed trail tracking and initialized from stereo! " <<endl;
      }
      else
	mMessageForUser << "Translate the camera slowly sideways, and press spacebar again to perform stereo init." << endl;
       
    
  }
}

// Need to provide a comparison function for sort because there is no comparison operator for cv::Point2i 
bool cornerCompare(const std::pair<double, cv::Point2i> &left, const std::pair<double, cv::Point2i>& right) {
  
  return left.first < right.first; // ??????????????????? What's going on here????????????????????????

}

// The current frame is to be the first keyframe!
void Tracker::TrailTracking_Start()
{
  // MakeKeyFrame_rest() populates the candidates (i.e., the featurs in the first frame)
  // which are essentially maximally suppressed FAST features, followed by a Shi-Tomasi 
  // threshold-based (default is 70.0) short-listing.
  pCurrentKF->MakeKeyFrame_Rest();  
  // Store the KF as "first"
  pFirstKF = pCurrentKF; 
  vector<pair<double, cv::Point2i> > vCornersAndSTScores;
  // Copy candidates into a trivially sortable vector
  // so that we can choose the image corners with max ST score
  for(unsigned int i=0; i<pCurrentKF->aLevels[0].vCandidates.size(); i++)  {  
                                                                             
      Candidate &c = pCurrentKF->aLevels[0].vCandidates[i];
      //if(!mCurrentKF.aLevels[0].im.in_image_with_border(c.irLevelPos, MiniPatch::mnHalfPatchSize))
      if ( !CvUtils::in_image_with_border( c.irLevelPos.y, 
					   c.irLevelPos.x,
					   pCurrentKF->aLevels[0].im, 
					   MiniPatch::mnHalfPatchSize, 
					   MiniPatch::mnHalfPatchSize 
					) 
	 ) continue;
      
      vCornersAndSTScores.push_back( pair<double, cv::Point2i>(-c.dSTScore, c.irLevelPos) ); // negative so highest score first in sorted list
    }
  // George: Had to define an order function, possibly because there is no way to break ties in terms of second element (size)in the pair
  sort(vCornersAndSTScores.begin(), vCornersAndSTScores.end(), cornerCompare );  // Sort according to Shi-Tomasi score
  // maximum 1000 (by default) points to add (we will be decreasing upon insertion)
  int nToAdd = PV3::get<int>("MaxInitialTrails", 1000, SILENT);
  for(unsigned int i = 0; i<vCornersAndSTScores.size() && nToAdd > 0; i++) {
    
    //cout <<"DEBUG: "<<i<<" Corner score : "<<vCornersAndSTScores[i].first<<endl;
      if ( !CvUtils::in_image_with_border(vCornersAndSTScores[i].second.y, 
					  vCornersAndSTScores[i].second.x,
					  pCurrentKF->aLevels[0].im, 
					  MiniPatch::mnHalfPatchSize, 
					  MiniPatch::mnHalfPatchSize ) 
	 ) continue;
      // A trail struct is simply an initializing correspondence (first 2-views of the SLAM).
      // It stores the MiniPatch of the original point and the respective locatrions in the first and second KF 
      Trail t;
      // copy the patch around the feature and store in the Trail's MiniPatch member
      t.mPatch.SampleFromImage(vCornersAndSTScores[i].second, pCurrentKF->aLevels[0].im);
      // store the initial position of the feature in the irInitialPos member
      t.irInitialPos = vCornersAndSTScores[i].second;
     // Also make the irCurrentPos equal to the original. For now....
      t.irCurrentPos = t.irInitialPos;
      // Store the TrailMatch for now...
      mlTrails.push_back(t);
      // decrease the number of remaining features to be added
      nToAdd--;
  }
  pPreviousFrameKF = pFirstKF;  // Cache the curfrent KF as "mPreviousFrameKF". It will be useful in any case...
  
}

// This function does the matching by generating a list of "Trail" matches
// which correspond to the views that initiate the map (and SLAM). 
// Backward matching filters-out a lot false matches here...
int Tracker::TrailTracking_Advance()
{
  int nGoodTrails = 0;
  // Setup OpenGL for the plotting of good correspondences
  if(mbDraw) {
    
      glPointSize(5);
      glLineWidth(2);
      glEnable(GL_POINT_SMOOTH);
      glEnable(GL_LINE_SMOOTH);
      glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
      glEnable(GL_BLEND);
      glBegin(GL_LINES);
  }
  
  MiniPatch BackwardsPatch;
  Level &lCurrentFrame = pCurrentKF->aLevels[0]; // The eth version has parametrized this 0-level...
  Level &lPreviousFrame = pPreviousFrameKF->aLevels[0];
  //cout <<" Initial number of trail matches : "<<mlTrails.size()<<endl;
  for(list<Trail>::iterator i = mlTrails.begin(); i!=mlTrails.end(); ) {
      
      list<Trail>::iterator next = i; next++; // we need a "next" pointer for some reason
      
      Trail &trail = *i; // get a reference for the current trailmatch
      cv::Point2i irStart = trail.irCurrentPos; // the original position
      cv::Point2i irEnd = irStart;              // same as original; a good place to start looking!
      
      // This is a very simple half-way brute-force search method in a 15 x 15 neighborhood
      // of the feature for a matching FAST corner in the new image
      //cout <<"irEnd before the find : "<<irEnd<<endl;
      bool bFound = trail.mPatch.FindPatch(irEnd, lCurrentFrame.im, 10, lCurrentFrame.vCorners);
      
	
      // great! a match was found!
      if(bFound) {
	// Now we do the backward matching to see if we actually get the same match!
	// So we sample the original image around the coordinates of the newly found match in the second ("irEnd"). 
	BackwardsPatch.SampleFromImage(irEnd, lCurrentFrame.im);
	cv::Point2i irBackWardsFound = irEnd;
	// And search for a match again
	bFound = BackwardsPatch.FindPatch(irBackWardsFound, lPreviousFrame.im, 10, lPreviousFrame.vCorners);
	// if the match is not the same (more than 2 pixels apart), then set bFound to false.
	if( (irBackWardsFound.x - irStart.x) * (irBackWardsFound.x - irStart.x) +  
	    (irBackWardsFound.y - irStart.y) * (irBackWardsFound.y - irStart.y) > 2) bFound = false;
	else {
	  trail.irCurrentPos = irEnd;
	  nGoodTrails++;
	}
      }
      // Draw the matches with nice colorings for bad ones
      
      if(mbDraw) {
	
	  if(!bFound) glColor3f(0,1,1); // Failed trails are light blue with a bit of purple at the end.
	  else
	    glColor3f(1,1,0); // start with yellow and end with red if its a good (bFound == true) 
			      // correspondence
	  
	  glVertex2i(trail.irInitialPos.x, trail.irInitialPos.y);
	  
	  if(bFound) glColor3f(1,0,0); // so, finish with red if good, otherwise stay purple and die...
	  
	  glVertex2i(trail.irCurrentPos.x, trail.irCurrentPos.y);
	}
      // Erase from list of trails if not found this frame.
      if(!bFound)  mlTrails.erase(i);
      i = next;
  } // end trails for-loop!
  // end the OpenGL drawing clause
  if(mbDraw) glEnd();

  // shift KF cache
  pPreviousFrameKF = pCurrentKF;
  
  // Return the trailing matches
  return nGoodTrails;
}

// TrackMap is the main purpose of the Tracker.
// It first projects all map points into the image to find a potentially-visible-set (PVS);
// Then it tries to find some points of the PVS in the image;
// Then it updates camera pose according to any points found.
// Above may happen twice if a coarse tracking stage is performed.
// Finally it updates the tracker's current-frame-KeyFrame struct with any
// measurements made.
// A lot of low-level functionality is split into helper classes:
// class TrackerData handles the projection of a MapPoint and stores intermediate results;
// class PatchFinder finds a projected MapPoint in the current-frame-KeyFrame.
void Tracker::TrackMap() {
  
  // Some accounting which will be used for tracking quality assessment:
  for(int i=0; i<LEVELS; i++)
    manMeasAttempted[i] = manMeasFound[i] = 0;
  
  // The Potentially-Visible-Set (PVS) is split into pyramid levels.
  vector<TrackerData::Ptr> avPVS[LEVELS]; 
  for(int i=0; i<LEVELS; i++) 
    avPVS[i].reserve(500); // preallocating - reserve 500 bytes for each trackerdata entry per level

   //cout <<"DEBUG: Scanning mappoints ... Map size : "<<mMap.vpPoints.size()<<endl;
   //cout <<"DEBUG: trashed mappoints: "<<mMap.vpPointsTrash.size()<<endl;
   // For each point in the map...
  //std::vector<MapPoint>::iterator ipMP;
  for(unsigned int pointIndex = 0; pointIndex < mMap.vpPoints.size(); pointIndex++) {
  //for (ipMP = mMap.vpPoints.begin(); ipMP != mMap.vpPoints.end(); ++ipMP)
      // Every mappoint should have a TrackerData member.
      // We want to allocate and populate this member...
      MapPoint::Ptr pMP = mMap.vpPoints[pointIndex]; 
      // Ensure that this map point has an associated TrackerData struct.
      // The TrackerData structure constructor simply assigns the mappoint pointer internally. 
      // The rest of the data fields are populated in due time...
      //cout <<"DEBUG: Mappoint TrackerData structure : "<<p->pTData<<endl;
      
      if(!pMP->pTData) pMP->pTData.reset( new TrackerData(pMP) );   
      TrackerData::Ptr pTData = pMP->pTData;// just store a reference to avoid long names
      
      // Project according to current view
      pTData->Project(mse3CamFromWorld, mCamera); 
      // if out of the image (out-of-bounds OR beyond the maximum image radius 
      // in the Euclidean z = 1 plane), skip to next point.
      if(!pTData->bInImage) continue;   
      
      // Calculate camera projection derivatives of this point.
      // This is simply the derivatives of the point provided by the camera.
      // The reason G.K calls it "Unsafe" is probably because the camera frame 
      // is very unreliable at this stage (we just got it from "ApplyMotionModel" which is a very "noisy" prediction so to speak...)
      pTData->GetDerivsUnsafe(mCamera);

      // And check what the PatchFinder (included in TrackerData) makes of the mappoint in this view..
      pTData->nSearchLevel = pTData->Finder.CalcSearchLevelAndWarpMatrix(pTData->Point, mse3CamFromWorld, pTData->m2CamDerivs);
      
      // a negative search pyramid level indicates an inappropriate warp for this view, so skip.
      if(pTData->nSearchLevel == -1) continue;   
      //cout <<"nSearchLevel in trackmap loop for PVS: "<<pTData->nSearchLevel<<endl;
      
      // Otherwise, this point is suitable to be searched in the current image! Add to the PVS.
      pTData->bSearched = false;
      pTData->bFound = false;
      avPVS[pTData->nSearchLevel].push_back(pTData);
      
    }
    
    
  // Next: A large degree of faffing about and deciding which points are going to be measured!
  // First, randomly shuffle the individual levels of the PVS.
  for(int i=0; i<LEVELS; i++) {
    random_shuffle(avPVS[i].begin(), avPVS[i].end());
    //cout <<"PVS size in level "<<i<<" is :" << avPVS[i].size() << endl;
  }
  
  // ******************************** 1. COARSE TRACKING STAGE *************************************************
  
  // The next two data structs contain the list of points which will next 
  // be searched for in the image, and then used in pose update.
  vector<TrackerData::Ptr> vNextToSearch;
  vector<TrackerData::Ptr> vIterationSet;
  
  // Tunable parameters to do with the coarse tracking stage:
  static pvar3<unsigned int> gvnCoarseMin("Tracker.CoarseMin", 20, SILENT);   // Min number of large-scale features for coarse stage
  static pvar3<unsigned int> gvnCoarseMax("Tracker.CoarseMax", 60, SILENT);   // Max number of large-scale features for coarse stage
  static pvar3<unsigned int> gvnCoarseRange("Tracker.CoarseRange", 30, SILENT);       // Pixel search radius for coarse features
  static pvar3<int> gvnCoarseSubPixIts("Tracker.CoarseSubPixIts", 8, SILENT); // Max sub-pixel iterations for coarse features
  static pvar3<int> gvnCoarseDisabled("Tracker.DisableCoarse", 0, SILENT);    // Set this to 1 to disable coarse stage (except after recovery)
  static pvar3<double> gvdCoarseMinVel("Tracker.CoarseMinVelocity", 0.006, SILENT);  // Speed above which coarse stage is used.
  
  unsigned int nCoarseMax = *gvnCoarseMax;
  unsigned int nCoarseRange = *gvnCoarseRange;
  
  mbDidCoarse = false;

  // Set of heuristics to check if we should do a coarse tracking stage.
  bool bTryCoarse = true;
  if(*gvnCoarseDisabled || 
      mdMSDScaledVelocityMagnitude < *gvdCoarseMinVel  ||
      nCoarseMax == 0)  
    bTryCoarse = false;
  
  if(mbJustRecoveredSoUseCoarse) {
      
      bTryCoarse = true;
      nCoarseMax *=2;
      nCoarseRange *=2;
      mbJustRecoveredSoUseCoarse = false;
    }
   
  // If we do want to do a coarse stage, also check that there's enough high-level 
  // PV map points. We use the lowest-res two pyramid levels (LEVELS-1 and LEVELS-2),
  // with preference to LEVELS-1.
  //cout <<"DEBUG: Potentially Visible set at level-1 + PVS at level-2 size: "<< avPVS[LEVELS-1].size() + avPVS[LEVELS-2].size()<<endl;
  // This wil not happen unless we have potentially visible features in levels 2 or 3 
  // (probably not going to happen shortly after initialization)
  if(bTryCoarse && avPVS[LEVELS-1].size() + avPVS[LEVELS-2].size() > *gvnCoarseMin ) {
    
      //cout <<"DEBUG: Entering Coarse Tracking section..."<<endl;
    
      // Now, fill the vNextToSearch struct with an appropriate number of 
      // TrackerDatas corresponding to coarse map points! This depends on how many
      // there are in different pyramid levels compared to CoarseMin and CoarseMax.
      
      if(avPVS[LEVELS-1].size() <= nCoarseMax) {
	 // Fewer than CoarseMax in LEVELS-1? then take all of them, and remove them from the PVS list.
	  vNextToSearch = avPVS[LEVELS-1];
	  avPVS[LEVELS-1].clear();
      }
      else {
	 // ..otherwise choose nCoarseMax at random, again removing from the PVS list.
	  for(unsigned int i=0; i<nCoarseMax; i++)
	    vNextToSearch.push_back(avPVS[LEVELS-1][i]);
	  avPVS[LEVELS-1].erase(avPVS[LEVELS-1].begin(), avPVS[LEVELS-1].begin() + nCoarseMax);
      }
      
      // If didn't source enough from LEVELS-1, get some from LEVELS-2... same as above.
      if(vNextToSearch.size() < nCoarseMax) {
	
	  unsigned int nMoreCoarseNeeded = nCoarseMax - vNextToSearch.size();
	  if(avPVS[LEVELS-2].size() <= nMoreCoarseNeeded) {
	    
	      vNextToSearch = avPVS[LEVELS-2];
	      avPVS[LEVELS-2].clear();
	  }
	  else {
	    
	      for(unsigned int trackDataIndex = 0; trackDataIndex < nMoreCoarseNeeded; trackDataIndex++)
		vNextToSearch.push_back( avPVS[LEVELS-2][trackDataIndex] );
	      
	      avPVS[LEVELS-2].erase(avPVS[LEVELS-2].begin(), avPVS[LEVELS-2].begin() + nMoreCoarseNeeded);
	  }
      }
      // Now go and attempt to find these points in the image!
      //cout <<"Searching for "<<vNextToSearch.size()<< " points!"<<endl;
      unsigned int nFound = SearchForPoints(vNextToSearch, nCoarseRange, *gvnCoarseSubPixIts);
      vIterationSet = vNextToSearch;  // Copy over into the to-be-optimised list.
      //cout <<"DEBUG: Size of iteration set " <<vIterationSet.size()<<" found... "<<endl;
      // Were enough found to do any meaningful optimisation?
      if(nFound >= *gvnCoarseMin)   {
	
	  mbDidCoarse = true;
	  // If so: do ten Gauss-Newton pose updates iterations.
	  for(int iter = 0; iter<10; iter++) {
	    
	      if(iter != 0) { 
		// Re-project the points on all but the first iteration.
		  for(unsigned int trackDataIndex = 0; trackDataIndex < vIterationSet.size(); trackDataIndex++)
		    if(vIterationSet[trackDataIndex]->bFound)  
		      vIterationSet[trackDataIndex]->ProjectAndDerivs(mse3CamFromWorld, mCamera);
	      }
	      
	      for(unsigned int trackDataIndex = 0; trackDataIndex < vIterationSet.size(); trackDataIndex++)
		if(vIterationSet[trackDataIndex]->bFound)
		  vIterationSet[trackDataIndex]->CalcJacobian();
	      
		double dOverrideSigma = 0.0;
	      // Hack: force the MEstimator to be pretty brutal 
	      // with outliers beyond the fifth iteration.
	      if(iter > 5)
		dOverrideSigma = 1.0;
	      
	      // Calculate and apply the pose update...
	      cv::Vec<float, 6> v6Update = CalcPoseUpdate(vIterationSet, dOverrideSigma /*, true*/);
	      mse3CamFromWorld = SE3<>::exp(v6Update) * mse3CamFromWorld;
	    }
	}
    }
  
  // ********************************** 2. FINE TRACKING STAGE ***************************************************
  
  //cout <<"DEBUG: Entering Fine Tracking section..."<<endl;
  // So, at this stage, we may or may not have done a coarse tracking stage.
  // Now do the fine tracking stage. This needs many more points!
  
  int nFineRange = 10;  // Pixel search range for the fine stage. 
  // Can use a tighter search if the coarse stage was already done.
  if(mbDidCoarse) nFineRange = 5;
  
  // What patches shall we use this time? The high-level ones are quite important,
  // so do all of these, with sub-pixel refinement.
  {
    int levelIndex = LEVELS - 1;
    
    // for each TrackerData entry in the Level indexed by "levelIndex"
    // project and prepare derivatives onto the (VERY) coarsely estimated pose of the latest keyframe...
    for(unsigned int TrackerDataIndex = 0 ; TrackerDataIndex  < avPVS[levelIndex].size(); TrackerDataIndex++)
      avPVS[levelIndex][TrackerDataIndex]->ProjectAndDerivs(mse3CamFromWorld, mCamera);
    // Now Search for these points
    SearchForPoints(avPVS[levelIndex], nFineRange, 8);
   
    // After the search, pick ALL the tracker data entries in the potentialy visible list
    // non linear iteration
    for(unsigned int TrackerDataIndex = 0; TrackerDataIndex < avPVS[levelIndex].size(); TrackerDataIndex++)
      vIterationSet.push_back(avPVS[levelIndex][TrackerDataIndex]);  // Again, plonk all searched points onto the (maybe already populate) vIterationSet.
  }
  //cout <<"DEBUG: VIteration set size for level 3: "<<vIterationSet.size()<<endl;
  // All the others levels: Initially, put all remaining potentially visible patches onto vNextToSearch.
  vNextToSearch.clear();
  for(int levelIndex = LEVELS - 2; levelIndex >= 0; levelIndex--)
    for(unsigned int TrackerDataIndex = 0; TrackerDataIndex < avPVS[levelIndex].size(); TrackerDataIndex++)
      vNextToSearch.push_back(avPVS[levelIndex][TrackerDataIndex]);
  
  // But we haven't got CPU to track _all_ patches in the map - arbitrarily limit 
  // ourselves to 1000, and choose these randomly.
  static pvar3<int> gvnMaxPatchesPerFrame("Tracker.MaxPatchesPerFrame", 1000, SILENT);
  int nFinePatchesToUse = *gvnMaxPatchesPerFrame - vIterationSet.size();
  
  if(nFinePatchesToUse < 0) nFinePatchesToUse = 0;
  
  // If we have more than we bargained for, random shuffle and pick the maximum allowable number
  if((int) vNextToSearch.size() > nFinePatchesToUse) {
    
      random_shuffle(vNextToSearch.begin(), vNextToSearch.end());
      vNextToSearch.resize(nFinePatchesToUse); // Chop!
  }
  
  // If we did a coarse tracking stage: re-project and find derivs of fine points
  if(mbDidCoarse)
    for(unsigned int TrackerDataIndex=0; TrackerDataIndex < vNextToSearch.size(); TrackerDataIndex++)
      vNextToSearch[TrackerDataIndex]->ProjectAndDerivs(mse3CamFromWorld, mCamera);
  
  // Find fine points in image:
  SearchForPoints(vNextToSearch, nFineRange, 0);
  // And attach them all to the end of the optimisation-set.
  for(unsigned int TrackerDataIndex = 0; TrackerDataIndex < vNextToSearch.size(); TrackerDataIndex++)
    vIterationSet.push_back( vNextToSearch[TrackerDataIndex] );
  //cout <<"DEBUG: vIterationSet size before optimization : "<<vIterationSet.size()<<endl;
  // Again, ten gauss-newton pose update iterations.
  cv::Vec<float, 6> v6LastUpdate(0, 0, 0, 0, 0, 0);
  
  // **************************** Iterative Tracking Now! *****************************************
  
  for(int iter = 0; iter<10; iter++) {
    // For a bit of time-saving: don't do full nonlinear
    // reprojection at every iteration - it really isn't necessary!
    bool bNonLinearIteration; 
                                
    // Even this is probably overkill, the reason we do many
    // iterations is for M-Estimator convergence rather than
    // linearization effects
    bNonLinearIteration  = (iter == 0 || iter == 4 || iter == 9); 
	
      // Either way: first iteration doesn't need projection update.
      if(iter != 0) {
	
	  if (bNonLinearIteration) {
	    
	      for(unsigned int TrackerDataIndex = 0; TrackerDataIndex < vIterationSet.size(); TrackerDataIndex++)
		if(vIterationSet[TrackerDataIndex]->bFound) 
		  vIterationSet[TrackerDataIndex]->ProjectAndDerivs(mse3CamFromWorld, mCamera);
	  }
	  else {
	    
	      for(unsigned int i=0; i<vIterationSet.size(); i++)
		if(vIterationSet[i]->bFound)
		  vIterationSet[i]->LinearUpdate(v6LastUpdate);
	  }
      }
      
      if (bNonLinearIteration)
	for(unsigned int TrackerDataIndex = 0; TrackerDataIndex < vIterationSet.size(); TrackerDataIndex++)
	  if(vIterationSet[TrackerDataIndex]->bFound)
	    vIterationSet[TrackerDataIndex]->CalcJacobian();

      // Again, an M-Estimator hack beyond the fifth iteration.
      double dOverrideSigma = 0.0;
      
      if(iter > 5) dOverrideSigma = 16.0;
      
      // Calculate and update pose; also store update vector for linear iteration updates.
      cv::Vec<float, 6> v6Update = CalcPoseUpdate(vIterationSet, dOverrideSigma, iter==9);
      
      mse3CamFromWorld = SE3<>::exp(v6Update) * mse3CamFromWorld;
      
      v6LastUpdate = v6Update;
  }
  
  if(mbDraw) {
    
      glPointSize(6);
      glEnable(GL_BLEND);
      glEnable(GL_POINT_SMOOTH);
      glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
      glBegin(GL_POINTS);
      for(vector<TrackerData::Ptr>::reverse_iterator it = vIterationSet.rbegin();
	  it!= vIterationSet.rend(); 
	  it++)
	{
	  if(! (*it)->bFound)
	    continue;
	  GLXInterface::glColor(gavLevelColors[(*it)->nSearchLevel]);
	  GLXInterface::glVertex((*it)->v2Image);
	}
      glEnd();
      glDisable(GL_BLEND);
    }
  
  // Update the current keyframe with info on what was found in the frame.
  // Strictly speaking this is unnecessary to do every frame, it'll only be
  // needed if the KF gets added to MapMaker. Do it anyway.
  // Export pose to current keyframe:
  pCurrentKF->se3CfromW = mse3CamFromWorld;
  
  // Record successful measurements. Use the KeyFrame-Measurement struct for this.
  // and find mean scene depth
  pCurrentKF->mMeasurements.clear();
  double dSum = 0;
  double dSumSq = 0;
  int nNum = 0;
    
  vector<TrackerData::Ptr>::iterator ipTData;
  for( ipTData = vIterationSet.begin(); ipTData != vIterationSet.end(); ipTData++) {
      
    if(! (*ipTData)->bFound) continue;
    KFMeasurement m;
    m.v2RootPos = (*ipTData)->v2Found;
    m.nLevel = (*ipTData)->nSearchLevel;
    m.bSubPix = (*ipTData)->bDidSubPix; 
    pCurrentKF->mMeasurements[(*ipTData)->Point] = m;
    
    // necessary map stats (depth mean and deviation in the local coordinate frame)
    double z = (*ipTData)->v3Cam[2];
    dSum+= z;
    dSumSq+= z*z;
    nNum++;

  }
  
   
 if(nNum > 20) {
  pCurrentKF->dSceneDepthMean = dSum/nNum;
  pCurrentKF->dSceneDepthSigma = sqrt((dSumSq / nNum) - (pCurrentKF->dSceneDepthMean) * (pCurrentKF->dSceneDepthMean));
 }
  
}

// Find points in the image. Uses the PatchFiner struct stored in TrackerData
int Tracker::SearchForPoints(vector<TrackerData::Ptr> &vTD, int nRange, int nSubPixIts)
{
  int nFound = 0;
  for(unsigned int i=0; i<vTD.size(); i++) {  // for each point..
    
      // First, attempt a search at pixel locations which are FAST corners.
      // (PatchFinder::FindPatchCoarse)
      TrackerData::Ptr pTD = vTD[i];
      PatchFinder &Finder = pTD->Finder;
      Finder.MakeTemplateCoarseCont(pTD->Point);
      if(Finder.TemplateBad()) {
	
	  pTD->bInImage = pTD->bPotentiallyVisible = pTD->bFound = false;
	  
	  continue;
      }
      
      manMeasAttempted[Finder.GetLevel()]++;  // Stats for tracking quality assessmenta
      
      //bool bFound =  Finder.FindPatchCoarse(CvUtils::IL(pTD->v2Image), pCurrentKF, nRange);
      bool bFound =  Finder.FindPatchCoarse(pTD->v2Image, pCurrentKF, nRange);
      
      pTD->bSearched = true;
      
      if(!bFound) {
	  pTD->bFound = false;
	  continue;
      }
      
      pTD->bFound = true;
      pTD->dSqrtInvNoise = (1.0 / Finder.GetLevelScale());
      
      nFound++;
      manMeasFound[Finder.GetLevel()]++;
      
      // Found the patch in coarse search - are Sub-pixel iterations wanted too?
      if(nSubPixIts > 0) {
	
	  pTD->bDidSubPix = true;
	  Finder.prepSubPixGNStep();
	  bool bSubPixConverges=Finder.IterateSubPixToConvergence(pCurrentKF, nSubPixIts);
	  // If subpix doesn't converge, the patch location is probably very dubious!
	  if(!bSubPixConverges) {
	     
	      pTD->bFound = false;
	      nFound--;
	      manMeasFound[Finder.GetLevel()]--;
	      continue;
	  }
	  pTD->v2Found = Finder.GetSubPixPos();
      }
      else {
	
	  pTD->v2Found = Finder.GetCoarsePosAsVector();
	  pTD->bDidSubPix = false;
	
      }
  }
  //cout <<"DEBUG: Number of points found : "<<nFound<<endl;
  return nFound;
}

//Calculate a pose update 6-vector from a bunch of image measurements.
//User-selectable M-Estimator.
//Normally this robustly estimates a sigma-squared for all the measurements
//to reduce outlier influence, but this can be overridden if
//dOverrideSigma is positive. Also, bMarkOutliers set to true
//records any instances of a point being marked an outlier measurement
//by the Tukey MEstimator.
cv::Vec<float, 6> Tracker::CalcPoseUpdate(vector<TrackerData::Ptr> vTD, double dOverrideSigma, bool bMarkOutliers) {
  
  // Which M-estimator are we using?
  int nEstimator = 0;
  static pvar3<string> pvsEstimator("TrackerMEstimator", "Tukey", SILENT);
  
  if(*pvsEstimator == "Tukey") nEstimator = 0;
  else 
    if(*pvsEstimator == "Cauchy") nEstimator = 1;
  else 
    if(*pvsEstimator == "Huber") nEstimator = 2;
  else  {
    
    cout << "Invalid TrackerMEstimator, choices are Tukey, Cauchy, Huber" << endl;
    nEstimator = 0;
    *pvsEstimator = "Tukey";
  }
  
  // Find the covariance-scaled reprojection error for each measurement.
  // Also, store the square of these quantities for M-Estimator sigma squared estimation.
  vector<double> vdErrorSquared;
  for(unsigned int TrackerDataIndex = 0; TrackerDataIndex < vTD.size(); TrackerDataIndex++) {
    
      TrackerData::Ptr pTD = vTD[TrackerDataIndex];
      if(!pTD->bFound) continue;
      
      pTD->v2Error_CovScaled = pTD->dSqrtInvNoise * (pTD->v2Found - pTD->v2Image);
      vdErrorSquared.push_back(pTD->v2Error_CovScaled.dot( pTD->v2Error_CovScaled ) );
  }
  //cout <<"DEBUG: For this iteration, "<<vdErrorSquared.size()<< " squared errors will be used"<<endl;
  
  // No valid measurements? Return null update.
  //if(vdErrorSquared.size() == 0) return cv::Vec<float, 6>( 0,0,0,0,0,0);
  if(vdErrorSquared.size() <6) return cv::Vec<float, 6>( 0,0,0,0,0,0);
  
  // What is the distribution of errors?
  double dSigmaSquared;
  if(dOverrideSigma > 0) dSigmaSquared = dOverrideSigma; // Bit of a waste having stored the vector of square errors in this case!
  else {
    
      if (nEstimator == 0)
	dSigmaSquared = Tukey::FindSigmaSquared(vdErrorSquared);
      else if(nEstimator == 1)
	dSigmaSquared = Cauchy::FindSigmaSquared(vdErrorSquared);
      else 
	dSigmaSquared = Huber::FindSigmaSquared(vdErrorSquared);
  }
  
// nonlinear LS step
  cv::Matx<float, 6, 6> m6Omega = 100.0 * cv::Matx<float, 6, 6>::eye(); // make the prior very confident...
  cv::Vec<float, 6> v6ksi(0, 0, 0, 0, 0, 0);                           // zero the information matrix (we solve for the perturbation Dx)
  
  for(unsigned int TrackerDataIndex = 0; TrackerDataIndex < vTD.size(); TrackerDataIndex++) {
    
      TrackerData::Ptr pTD = vTD[TrackerDataIndex];
      
      if(!pTD->bFound) {
	
	continue;
      }
      cv::Vec<float, 2> &v2 = pTD->v2Error_CovScaled;
      double dErrorSq = v2[0] * v2[0] + v2[1] * v2[1]; // avoid overloads and inline code whenever u can...
      double dWeight;
      
     if(nEstimator == 0) dWeight= Tukey::Weight(dErrorSq, dSigmaSquared);	
     else 
       if(nEstimator == 1) dWeight= Cauchy::Weight(dErrorSq, dSigmaSquared);
      else
	dWeight= Huber::Weight(dErrorSq, dSigmaSquared);
      
      // Inlier/outlier accounting, only really works for cut-off estimators such as Tukey.
      // George: Or with a manual threshold... But we already have RANSAC for this...
      if(dWeight < 10e-3) {
	
	  if(bMarkOutliers) pTD->Point->nMEstimatorOutlierCount++;
	  
	  continue;
      }
      else
	if(bMarkOutliers) pTD->Point->nMEstimatorInlierCount++;
      
      //wls.add_mJ(v2[0], TD.dSqrtInvNoise * m26Jac[0], dWeight); // These two lines are currently
      //wls.add_mJ(v2[1], TD.dSqrtInvNoise * m26Jac[1], dWeight); // the slowest bit of poseits (NOT anymore...)
      
      //wls.add_mJ(mv2, TD.dSqrtInvNoise * m26Jac, dWeight * I2);
      
      m6Omega += dWeight * (pTD->dSqrtInvNoise * pTD->dSqrtInvNoise) * (pTD->m26Jacobian.t() * pTD->m26Jacobian);
      v6ksi += dWeight * ( ( pTD->dSqrtInvNoise * pTD->m26Jacobian.t() ) * v2 );
      
      //cout <<"DEBUG: Robust weight in cal pose : "<<dWeight<<endl;
      //cout <<"DEBUG: Tracker data Jacobian : "<<m26Jac<<endl;
      //cout <<"Previous Error : "<<v2<<endl;
  }
  
  cv::Vec<float, 6> v6Update;
  //cout <<"DEBUG: Pose Update Omega : "<<m6Omega<<endl<<"And ksi:"<<v6ksi <<endl;
  cv::solve(m6Omega, v6ksi, v6Update, cv::DECOMP_CHOLESKY);

  return v6Update;
}



// Just add the current velocity to the current pose.
// N.b. this doesn't actually use time in any way, i.e. it assumes
// a one-frame-per-second camera. Skipped frames etc
// are not handled properly here.
void Tracker::ApplyMotionModel() {
  // Start from our last known pose...
  mse3StartPos = mse3CamFromWorld;
  // store cvamera velocity 
  cv::Vec<float, 6> v6Velocity = mv6CameraVelocity;
  // if we have enabled the use of SBIs,
  // then start from that rotation estimate and assume zero translational velocity
  if(mbUseSBIInit) {
    
      v6Velocity[3] = mv6SBIRot[3];
      v6Velocity[4] = mv6SBIRot[4];
      v6Velocity[5] = mv6SBIRot[5];
      
      v6Velocity[0] = 0.0;
      v6Velocity[1] = 0.0;
  }
  // predicting new camera position
  mse3CamFromWorld = SE3<>::exp(v6Velocity) * mse3StartPos;

  // and that's all folks!
  
}


// The motion model is entirely the tracker's, and is kept as a decaying
// constant velocity model.
void Tracker::UpdateMotionModel()
{
  SE3<> se3NewFromOld = mse3CamFromWorld * mse3StartPos.inverse();
  cv::Vec<float, 6> v6Motion = SE3<>::ln(se3NewFromOld);
  cv::Vec<float, 6> v6OldVel = mv6CameraVelocity;
  
  mv6CameraVelocity = 0.9 * (0.5 * v6Motion + 0.5 * v6OldVel);
  mdVelocityMagnitude = cv::norm(mv6CameraVelocity);
  
  // Also make an estimate of this which has been scaled by the mean scene depth.
  // This is used to decide if we should use a coarse tracking stage.
  // We can tolerate more translational vel when far away from scene!
  cv::Vec<float, 6> v6 = mv6CameraVelocity;
  
  v6[0] *= 1.0 / pCurrentKF->dSceneDepthMean;
  v6[1] *= 1.0 / pCurrentKF->dSceneDepthMean;
  v6[2] *= 1.0 / pCurrentKF->dSceneDepthMean;
  
  mdMSDScaledVelocityMagnitude = cv::norm(v6);
}

// Time to add a new keyframe? The MapMaker handles most of this.
void Tracker::AddNewKeyFrame()
{
  
  mMapMaker.AddKeyFrame(pCurrentKF);
  mnLastKeyFrameDropped = mnFrame;
}

// Some heuristics to decide if tracking is any good, for this frame.
// This influences decisions to add key-frames, and eventually
// causes the tracker to attempt relocalisation.
void Tracker::AssessTrackingQuality()
{
  int nTotalAttempted = 0;
  int nTotalFound = 0;
  int nLargeAttempted = 0;
  int nLargeFound = 0;
  
  for(int i=0; i<LEVELS; i++) {
    
      nTotalAttempted += manMeasAttempted[i];
      nTotalFound += manMeasFound[i];
      if(i>=2) nLargeAttempted += manMeasAttempted[i];
      if(i>=2) nLargeFound += manMeasFound[i];
    }
  
  if(nTotalFound == 0 || nTotalAttempted == 0) mTrackingQuality = BAD;
  else {
    
      double dTotalFracFound = (double) nTotalFound / nTotalAttempted;
      double dLargeFracFound;
      
      if(nLargeAttempted > 10)
	dLargeFracFound = (double) nLargeFound / nLargeAttempted;
      else
	dLargeFracFound = dTotalFracFound;

      static pvar3<double> gvdQualityGood("Tracker.TrackingQualityGood", 0.3, SILENT);
      static pvar3<double> gvdQualityLost("Tracker.TrackingQualityLost", 0.13, SILENT);
      
      
      if(dTotalFracFound > *gvdQualityGood) mTrackingQuality = GOOD; 
      else 
	if(dLargeFracFound < *gvdQualityLost) mTrackingQuality = BAD;
      else
	mTrackingQuality = DODGY;
  }
  
  if(mTrackingQuality == DODGY) {
    
    // Further heuristics to see if it's actually bad, not just dodgy...
    // If the camera pose estimate has run miles away, it's probably bad.
    if (mMapMaker.IsDistanceToNearestKeyFrameExcessive(pCurrentKF) ) mTrackingQuality = BAD;
  }
  
  if(mTrackingQuality==BAD) mnLostFrames++;
  else
    mnLostFrames = 0;
}

string Tracker::GetMessageForUser()
{
  return mMessageForUser.str();
}

void Tracker::CalcSBIRotation() {
  // Just compute a simple (1st order different spatial gradinet of the image) - btw, the 1/2 is missing...
  // These gradients are part of the G-N Jacobian
  mpSBILastFrame->MakeGradients();
  
  pair<SE2<>, double> result_pair;
  // Now this function returns a 2D rigidtransformation that translates and rotates the
  // center of the source SBI ("mpSBILatsFrame") in order to match the target ("MmpSBIThisFrame").
  result_pair = mpSBIThisFrame->IteratePosRelToTarget(*mpSBILastFrame, 6);
  // The following tries to find a 3D rotation that takes the patch from the source to the 
  // target SBI according to the recovered 2D transformation. Again, in my opinion,the whole thing is extremely dodgy...
  SE3<> se3Adjust = SmallBlurryImage::SE3fromSE2(result_pair.first, mCamera);
  // Just return the axis-angle parameters of the recovered 3D rotation
  mv6SBIRot = se3Adjust.ln();
}

cv::Size2i TrackerData::irImageSize;  // Static member of TrackerData lives here








