// George Terzakis 2016
//
// University of Portsmouth
//
// Code based on PTAM by Klein and Murray (Copyright 2008 Isis Innovation Limited)

#include "KeyFrame.h"
#include "ShiTomasi.h"
#include "SmallBlurryImage.h"
#include "FAST/prototypes.h"
#include "FAST/fast_corner.h"

#include "GCVD/Addedutils.h"
#include "OpenCV.h"

using namespace std;
using namespace Persistence;
using namespace FAST;

void KeyFrame::MakeKeyFrame_Lite(cv::Mat_<uchar> &im)
{
  // Perpares a Keyframe from an image. Generates pyramid levels, does FAST detection, etc.
  // Does not fully populate the keyframe struct, but only does the bits needed for the tracker;
  // e.g. does not perform FAST nonmax suppression. Things like that which are needed by the 
  // mapmaker but not the tracker go in MakeKeyFrame_Rest();
  
  // First, copy out the image data to the pyramid's zero level 
  // (destination image data are automatically allocated).
  im.copyTo(aLevels[0].im);
  
  // Now we are generating a pyramid by simply decimating (smaller image is not blurred)
  // with interpolation at the higher (lower resolution) level
  for(int i=0; i<LEVELS; i++) {
      
    Level &lev = aLevels[i];
    // Now obtaining the new level image as the decimated image of the previous level.
    // Note that he resizing should work even if the next level image data have not been allocated yet...
    if(i!=0) CvUtils::halfSample(aLevels[i-1].im, lev.im); // ALWAYS do this a la Rosten!!!!
      // DERECATED: 
      /*cv::resize(aLevels[i - 1].im, lev.im, 
      		 cv::Size2i(aLevels[i - 1].im.cols / 2, aLevels[i - 1].im.rows / 2)
		); */
	
      
    // Now detecting FAST corners for the current level (i-th).
    // G.K. uses different threshold for each level. TODO: I don't know if that can be somehow improved..  
    // The aim is to balance the different levels' relative feature densities.
    lev.vCorners.clear();
    lev.vCandidates.clear();
    lev.vMaxCorners.clear();
    int b; // The FAST threshold
    switch(i) {
      case 0: b = 10; break;
      case 1: b = 15; break;
      case 2: b = 15; break;
      default: b = 10; // 10 for every level above-equal to 3 (if any that is...)
    }
    
    // detect corners at this level and store in the respective corner list 
    FAST::fast_corner_detect_plain_10(lev.im, lev.vCorners, b);
      
    // Generate row look-up-table for the FAST corner points: this speeds up 
    // finding close-by corner points later on.
    // Given that FAST corners are scanned row-wise, I am not sure what the following code accomplishes...
    // It appears that corners are ordered in terms of their row (but that is anticipated given the way FAST works...)
    unsigned int v=0;
    lev.vCornerRowLUT.clear();
    for(int r=0; r<lev.im.rows; r++) {
   
	while(v < lev.vCorners.size() && r > lev.vCorners[v].y) v++; 	
	lev.vCornerRowLUT.push_back(v);
    }
    
  }
 
}

void KeyFrame::MakeKeyFrame_Rest()
{
  // Fills the rest of the keyframe structure needed by the mapmaker:
  // FAST nonmax suppression, generation of the list of candidates for further map points,
  // creation of the relocaliser's SmallBlurryImage.
  static pvar3<double> pvdCandidateMinSTScore("MapMaker.CandidateMinShiTomasiScore", 70, SILENT);
  
  // Now look into all levels for maximal FAST corners
  // that can be "Candidate" mappoints
  for(int l=0; l<LEVELS; l++) {
    
      Level &lev = aLevels[l];
      // .. find those FAST corners which are maximal..
      fast_nonmax(lev.im, lev.vCorners, 10, lev.vMaxCorners);
      // .. and then calculate the Shi-Tomasi scores of those, and keep the ones with
      // a suitably high score as Candidates, i.e. points which the mapmaker will attempt
      // to make new map points out of.
      
      vector<cv::Point2i>::iterator iCorner;
      for( iCorner = lev.vMaxCorners.begin(); iCorner != lev.vMaxCorners.end(); iCorner++) {
	
	if( !CvUtils::in_image_with_border(iCorner->y, iCorner->x, lev.im, 10, 10) ) continue;
	// find Shi-Tomasi score in small patches of 6 pixels (-3 to +3)
	double dSTScore = FindShiTomasiScoreAtPoint(lev.im, 3, *iCorner);
	  
	// if the score is above minimum (70),
	// we need to consider this mappoint as candidate.
	// Hence, put in the list of candidates in this level..
	if(dSTScore > *pvdCandidateMinSTScore) {
	    
	  Candidate c;
	  c.irLevelPos = *iCorner;
	  c.dSTScore = dSTScore;
	  lev.vCandidates.push_back(c);
	  
	  
	}
	//cout <<"DEBUG: "<<lev.vCandidates.size()<<" pushed back in make_keyframe_Rest!"<<endl;
	   
      }
      
      
  }
  
  
  // Also, make a SmallBlurryImage of the keyframe: The relocaliser uses these.
  pSBI = new SmallBlurryImage(*this);  
  
  // Relocaliser also wants the jacobians..
  pSBI->MakeGradients();
  
}

// The keyframe struct is quite happy with default operator=, but Level needs its own
Level& Level::operator =(const Level &rhs)
{
  // Operator= should physically copy pixels:
  
  rhs.im.copyTo(im);
  
  vCorners = rhs.vCorners;
  vMaxCorners = rhs.vMaxCorners;
  vCornerRowLUT = rhs.vCornerRowLUT;
  
  return *this;
}

// -------------------------------------------------------------
// Some useful globals defined in LevelHelpers.h live here:
cv::Vec3f gavLevelColors[LEVELS];

// These globals are filled in here. A single static instance of this struct is run before main()
struct LevelHelpersFiller // Code which should be initialised on init goes here; this runs before main()
{
  LevelHelpersFiller()
  {
    for(int i=0; i<LEVELS; i++)
      switch(i) {
	case 0:  gavLevelColors[i] = cv::Vec3f( 1.0, 0.0, 0.0); break;
	case 1:  gavLevelColors[i] = cv::Vec3f( 1.0, 1.0, 0.0); break;
	case 2:  gavLevelColors[i] = cv::Vec3f( 0.0, 1.0, 0.0); break;
	case 3:  gavLevelColors[i] = cv::Vec3f( 0.0, 0.0, 0.7); break;
	default:  gavLevelColors[i] =  cv::Vec3f( 1.0, 1.0, 0.7); // In case I ever run with LEVELS > 4
      }
  }
};

static LevelHelpersFiller foo;







