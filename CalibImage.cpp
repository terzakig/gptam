// George Terzakis 2016
//
// University of Portsmouth
//
// Code based on PTAM by Klein and Murray (Copyright 2008 Isis Innovation Limited)

#include "OpenGL.h"
#include "CalibImage.h"
#include <stdlib.h>

#include "FAST/fast_corner.h"
#include "GCVD/image_interpolate.h"

#include "Persistence/instances.h"


using namespace std;

using namespace FAST;
using namespace CvUtils;
using namespace RigidTransforms;


inline bool IsCorner(cv::Mat_<uchar> &im, int row, int col, int nGate)
{ // Does a quick check to see if a point in an image could be a grid corner.
  // Does this by going around a 16-pixel ring, and checking that there's four
  // transitions (black - white- black - white - )
  // Also checks that the central pixel is blurred.

  // Find the mean intensity of the pixel ring...
  int nSum = 0;
  static int abPixels[16];
  for(int i=0; i<16; i++)
    {
      abPixels[i] = im(row + fast_pixel_ring[i].y, col + fast_pixel_ring[i].x) ;
      nSum += abPixels[i];
    };
  int nMean = nSum / 16;
  int nHiThresh = nMean + nGate;
  int nLoThresh = nMean - nGate;

  // If the center pixel is roughly the same as the mean, this isn't a corner.
  int nCenter = im(row, col);
  if(nCenter <= nLoThresh || nCenter >= nHiThresh) return false;
  
  // Count transitions around the ring... there should be four!
  bool bState = (abPixels[15] > nMean);
  int nSwaps = 0;
  for(int i=0; i<16; i++)
    {
      uchar bValNow = abPixels[i];
      if(bState)
	{
	  if(bValNow < nLoThresh)
	    {
	      bState = false;
	      nSwaps++;
	    }
	}
      else
	if(bValNow > nHiThresh)
	  {
	    bState = true;
	    nSwaps++;
	  };
    }
  return (nSwaps == 4);
};



// This function does the following hack (which may/may not work):
// It rotates a thin "strip" of size 6 x 0.2 (i.e. -3 to 3 on the horizontal and -0.1 to 0.1 on the vertical)
// about the patch point and tries to find where pixel differences from one quadrant to the mirrored quadrant (about the horizontal)
// sum up to a large absolute value (which would suggest that, black pixels are over the horizontal on one side of the vertical 
// and below the horizontal on the other side of the vertical (or the other way around).
// I am adding multiple critera along the 2D "cones" centerd at v2Pos which should robustify the response

cv::Vec2f GuessInitialAngles(cv::Mat_<uchar> &im, cv::Point2i irCenter)
{
  // ****** Original comments contained in PTAM
  // The iterative patch-finder works better if the initial guess
  // is roughly aligned! Find one of the line-axes by searching round 
  // the circle for the strongest gradient, and use that and +90deg as the
  // initial guesses for patch angle.
  //
  // Yes, this is a very poor estimate, but it's generally (hopefully?) 
  // enough for the iterative finder to converge.
  
  image_interpolate<Interpolate::Bilinear, uchar> imInterp(im);
  double dBestAngle = 0;
  double dBestGradMag = 0;
  double dGradAtBest = 0;
  cv::Vec2d irCenterv(irCenter.x, irCenter.y);
  for(double dAngle = 0.0; dAngle < M_PI; dAngle += 0.001) {
    
      
      cv::Vec2d v2Dirn( cos(dAngle)     ,      sin(dAngle) ); // horizontal principal axis
      cv::Vec2d v2Perp( v2Dirn[1]       ,      -v2Dirn[0]  ); // vertical principal axis
      
      // computing a resposne for a rotated narrow strip. This might help.......
      double response = 0;
      for (int k = 0; k < 10; k++) {
      // first criterion
      cv::Vec4d dG4 =    imInterp[irCenterv + (k + 1.0) * v2Dirn + k * sin(M_PI * 10.0 / 180.0)  * v2Perp] - 
			 imInterp[irCenterv + (k + 1.0) * v2Dirn - k * sin(M_PI * 10.0 / 180.0)  * v2Perp] +
			 imInterp[irCenterv - (k + 1.0) * v2Dirn - k * sin(M_PI * 10.0 / 180.0)  * v2Perp] - 
			 imInterp[irCenterv - (k + 1.0) * v2Dirn + k * sin(M_PI * 10.0 / 180.0)  * v2Perp];
      
      response += dG4[0];
      }
      
      if(fabs(response) > dBestGradMag)
	{
	  dBestGradMag = fabs(response);
	  dGradAtBest = response;
	  dBestAngle = dAngle;
	};
    }
  
  cv::Vec2f v2Ret(dGradAtBest < 0 ? dBestAngle : dBestAngle - M_PI / 2.0, 
		  dGradAtBest < 0 ? dBestAngle + M_PI / 2.0 : dBestAngle );
  /*if(dGradAtBest < 0)
    {   v2Ret[0] = dBestAngle; v2Ret[1] = dBestAngle + M_PI / 2.0;    }
  else
    {   v2Ret[1] = dBestAngle; v2Ret[0] = dBestAngle - M_PI / 2.0;    } */
  return v2Ret;
}


bool CalibImage::MakeFromImage(cv::Mat_<uchar> &im, cv::Mat &cim)
{
  static Persistence::pvar3<int> gvnCornerPatchSize("CameraCalibrator.CornerPatchPixelSize", 20, Persistence::SILENT);
  mvCorners.clear();
  mvGridCorners.clear();
  
  im.copyTo(mim);
  
  rgbmim = cim;
  
  // Find potential corners..
  // This works better on a blurred image, so make a blurred copy
  // and run the corner finding on that.
  cv::Vec2f baryCenter(0, 0); // destined to be the barycenter of the detected points (if any that is...)
  {
    cv::Mat_<uchar> imBlurred;
    
    double dBlurSigma = Persistence::PV3::get<double>("CameraCalibrator.BlurSigma", 2.0, Persistence::SILENT);
   
    int gkerSize = (int)ceil(dBlurSigma*3.0); // where 3.0 is the default "sigmas" parameter in libCVD
    gkerSize += (gkerSize % 2 == 0) ? 1 : 0;
    
    cv::GaussianBlur(mim, imBlurred, cv::Size(gkerSize, gkerSize), dBlurSigma );
    
    cv::Point2i irTopLeft(5,5);
    cv::Point2i irBotRight(mim.cols - irTopLeft.x, mim.rows - irTopLeft.y);
    // Ok, now drawing points
    glPointSize(4);
    glColor3f(1,0,0);
    glBegin(GL_POINTS);
    
    // So, this "nGate" is a threshold parameter. The larger it is, the fewer corners are to be expected
    // In effect, it is an acceptance boundary for the corner patch mean intensity in terms of its own center intensity
    // (if within the boundary, then it gets discarded, thus larger values suggest a tighter criterion)
    int nGate = Persistence::PV3.get<int>("CameraCalibrator.MeanGate", 20, Persistence::SILENT); // 10 is a good value for some cameras, 
								     // but 20 may work bertter for others
    
    // Now cherry-picking the corners and drawing them as red points in the image
    for (int r = irTopLeft.y; r < irBotRight.y; r++)
      for (int c = irTopLeft.x; c < irBotRight.x; c++) {
	
	if(IsCorner(imBlurred, r, c, nGate)) {
	
	    mvCorners.push_back( cv::Point2i(c, r) );
	    
	    baryCenter[0] += c; baryCenter[1] += r; 
	    
	    glVertex2i(c, r);
	 
	  }
      }
      
    glEnd();
  }
  // NOTE: The above appears to be working ok... Portential corners are drawn on the image with thik red dots
  
  
  // If there's not enough corners, i.e. camera pointing somewhere random, abort.
  //if((int) mvCorners.size() < GV2.GetInt("CameraCalibrator.MinCornersForGrabbedImage", 20, SILENT))
  //return false;
 

  if((int) mvCorners.size() < Persistence::PV3.get<int>("CameraCalibrator.MinCornersForGrabbedImage", 20, Persistence::SILENT)) return false;
  
  // normalizing the baryCenter
  baryCenter[0] /= mvCorners.size(); baryCenter[1] /= mvCorners.size();
  
  // Pick the barycenter now, instead of a central corner point in the original PTAM calibrator...
  cv::Point2i irBestCenterPos;
  unsigned int nBestDistSquared = 99999999;
  for(unsigned int i=0; i<mvCorners.size(); i++)
    {
      
      unsigned int nDist = (mvCorners[i].x - baryCenter[0]) * (mvCorners[i].x - baryCenter[0]) + 
			    (mvCorners[i].y - baryCenter[1]) * (mvCorners[i].y - baryCenter[1]);
      
      if(nDist < nBestDistSquared)
	{
	  nBestDistSquared = nDist;
	  irBestCenterPos = mvCorners[i];
	}
    }
  
  // So now, the above loop has selected as "irBestCenterPos", 
  // the point that is closest to the barycenter of the corners. 
  // All being well, this should be roughly the center of the visdible part of the grid
  // ... and now, we try to fit a corner-patch to it.
  
  // 1 . Create a corner patch object sized gvnCornerPatchSize x gvnCornerPatchSize
  CalibCornerPatch Patch(*gvnCornerPatchSize);
  // 2. Now create the parameters struct associated with the actual position (v2Pos), orientation (v2Angles)
  //    and appearance (dMean and dGain) of the patch in the image
  CalibCornerPatch::Params Params;
  // Setting the position in the parameters (v2Pos) to be the position of the best corner found above
  Params.v2Pos = cv::Vec2f(irBestCenterPos.x, irBestCenterPos.y);
  // obtaining initial angles of the principal axes of the corner (of course, at first they should be perpendicular) 
  Params.v2Angles = GuessInitialAngles(mim, irBestCenterPos); 
  // setting defaults for dMean and dGain (NO MORE COMMENTS ON THOSE TWO FOR NOW: See my comments in iterate() for details...)
  Params.dGain = 80.0;
  Params.dMean = 120.0;
  
  // 3. Now try to optimize the parameters with a 20-step G-N method run
  if(!Patch.IterateOnImageWithDrawing(Params, mim)) return false;
  
  
  // The first found corner patch becomes the origin of the detected grid.
  // NOTE-NOTE!!! Now creating a GRID corner object (CalibGridCorner)! We got promoted from free-lying corner to GRID corner! 
  CalibGridCorner cFirst; 
  cFirst.Params = Params; // copy the parameters given to us by "IterateOnImageWithDrawing" in a silver platter
  mvGridCorners.push_back(cFirst); // Oh my! My first GRID corner in the sack!!!! Now going fishing!!!!
  // Draw a nice green cross at the location of the corner.
  cFirst.Draw();
  
  // Next, search horizontally and vertically  (in botth directions - negative and positive for each case)
  // to find a new corner from all those free corners lying around the image
  if( !( ExpandByAngle(0,0) || ExpandByAngle(0,2) ) ) return false; // if nothing on the left AND nothing on the right, false
  if( !( ExpandByAngle(0,1) || ExpandByAngle(0,3) ) ) return false; // if nothing above AND nothing below, false
  
  // So we now we should have TWO new fresh corners in our list for each of the two directions.
  // Ok, I know what you are thinking: WHAT ON EARTH is mInheritedSteps and what is "GetSteps"??????? Good questions!!! And names dont help....
  // ANSWER: Well, mInheritedSteps is a 2x2 matrix that stores (in row-wise fashion, strangely) some weird average direction vectors ROUGHLY along the two principal  
  // directions (horizontal and vertical) that connect the corner's EXISTING neighboring corners in the grid. If these corners however DONT exist (a likelihood), 
  // then mInderitedSteps is actually inherited - hence the name - from another node. What do you know... Of course, if we have reached this far in the code, 
  // then we know that the first node in the grid has non-trivial mInherited vectors. 
  // ***** In short, GetStep is not about discrete steps (as anyone would be misled to think),
  // ***** but rather an average DISPLACEMENT vector 'approximately' (because the search is upper-lower boudned) 
  // ***** along the prinicpal directions of the grid corner.
  mvGridCorners[1].mInheritedSteps = mvGridCorners[2].mInheritedSteps = mvGridCorners[0].GetSteps(mvGridCorners);
   
  // The three initial grid elements are enough to find the rest of the grid 
  int nNext;
  int nSanityCounter = 0; // Stop it getting stuck in an infinite loop...
  const int nSanityCounterLimit = 500;
  while((nNext = NextToExpand()) >= 0 && nSanityCounter < nSanityCounterLimit ) {
    
      ExpandByStep(nNext);
      nSanityCounter++;
    }
  if(nSanityCounter == nSanityCounterLimit)
    return false;
  
  DrawImageGrid();
  
  // need more than 8 grid corners to make a decent optimization of grid pose!!!! 
  unsigned int minGridCorners = Persistence::PV3::get<int>("CameraCalibrator.MinimumGridCorners4Pose", 8, Persistence::SILENT);
  cout << " minimum allowable corners per calibration image, " <<minGridCorners<<" and only " <<mvGridCorners.size() << " found ... "<<endl;
  if (mvGridCorners.size() < minGridCorners) return false;
  
  return true;
}

/// @nSrc The index of the current corner in the GRID corner list
/// @nDirn The INDEX of the direction to search for a corner (0 - horizontal, 1 - vertical).
/// CAUTION - CAUTION!!! Values of nDirn ABOVE/EQUAL to 2 are perceived as NEGATIVE DIRECTIONS
bool CalibImage::ExpandByAngle(int nSrc, int nDirn)
{
  // obtaining patch size in pixels. Default, 20.
  //static Persistence::pvar3<int> gvnCornerPatchSize("CameraCalibrator.CornerPatchPixelSize", 20, Persistence::SILENT);
  static Persistence::pvar3<int> gvnCornerPatchSize("CameraCalibrator.CornerPatchPixelSize", 20, Persistence::SILENT);
  // Get the GridCorner object (struct)
  CalibGridCorner &gSrc = mvGridCorners[nSrc];
  
  // Our best point cache
  cv::Point2i irBest;
  // distance cache for comparisons
  double dBestDist = 99999;
  // caching the distortion matrix (i.e., the two direction vectors - horizontal and vertical)
  cv::Mat_<float> m2Warp = gSrc.Params.m2Warp(); 
  
  // storing the direction (unit vector) to expand in v2TargetDirn:
  cv::Vec2f v2TargetDirn( m2Warp(0, nDirn % 2) , m2Warp(1, nDirn % 2) );
  
  // Taking the negative direction if nDirn is greater/equal to 2
  if(nDirn >= 2) v2TargetDirn *= -1;
  
  // just a flag to indicate that at least one corner was selected
  bool freeCornerFound = false;
  
  // Looping all the registered Corners (NOT the grid corners of course!)
  unsigned int i;
  for(i=0; i<mvCorners.size(); i++) {
      // get the distance of the i-th "free" corner from the our grid corner
      cv::Vec2f v2Diff( mvCorners[i].x - gSrc.Params.v2Pos[0], mvCorners[i].y - gSrc.Params.v2Pos[1] );
      
      // if the distance is below 7 pixels (I made this less than 10 because it generally improved the number of detected grid corners), then skip the current free corner and go to the next
      if(v2Diff[0] * v2Diff[0] + v2Diff[1] * v2Diff[1] < 100) continue;
      // if the distance is beyond "dBestDist" (the maximum allowable distance so to speak... this will become clear below) go to the next free corner
      if(v2Diff[0] * v2Diff[0] + v2Diff[1] * v2Diff[1] > dBestDist * dBestDist) continue;
      // ok, we are still here, so we might consider this distance. We store it as a unit vector v2Dirn
      cv::Vec2f v2Dirn = cv::normalize(v2Diff);
      // Now, if the angle of the recovered direction in v2Dirn with the nDirn-th direction in v2TargetDirn is above 30 degrees,
      // then skip to the next corner
      double angularMargin = Persistence::PV3::get("CameraCalibrator.CornerSearchAngMargin", 30.0, Persistence::SILENT);
      if( v2Dirn[0] * v2TargetDirn[0] + v2Dirn[1] * v2TargetDirn[1]   < cos(M_PI * angularMargin/ 180.0) ) continue;
      
      // Hurrah! We found a free corner in the direction of v2TargetDirn!!!!!
      // Save its distance as "best distance". The next corner in that direction should be closer. Otherwise, its just this or bust!
      dBestDist = cv::norm(v2Diff);
      // Store the coordinates of the free corner
      irBest = cv::Point2i(mvCorners[i].x, mvCorners[i].y);
      // raise the found-free-corner flag
      freeCornerFound = true;
    }
  // this loop can always return nothing. I am adding a return statement here to save further processing
  if (!freeCornerFound) {
    gSrc.aNeighborStates[nDirn].val = N_FAILED;
    return false;
   }
    
  // Ok, so here we are! A corner has been selected, so we need to create a bew GridCorner object....  
  CalibGridCorner gTarget;
  // in theory, the new grid corner should share the same orientation parameters with the source grid corner
  // so we copy the entire parameter set and the alter the rest...
  gTarget.Params.v2Angles = cv::Vec2f(gSrc.Params.v2Angles[0], gSrc.Params.v2Angles[1]);
  gTarget.Params.dMean = gSrc.Params.dMean;
  gTarget.Params.dGain = gSrc.Params.dGain;
  
  // set position to the newly-found corner's position
  gTarget.Params.v2Pos = cv::Vec2f(irBest.x, irBest.y);
  // and negate the gain for whatever reason...
  gTarget.Params.dGain *= -1;
  
  // We have the new Grid corner, now let's create a patch in order to iterate
  CalibCornerPatch Patch(*gvnCornerPatchSize);
  // now we iterate on the image but for the new grid corner. God bless....
  if(!Patch.IterateOnImageWithDrawing(gTarget.Params, mim)) {
    
      // if we couldn't converge with the new corner, mark this direction as "FAILED" in the source grid corner
      gSrc.aNeighborStates[nDirn].val = N_FAILED;
      return false;
  }

  
  // ok, so now we add the new grid corner and update the source's neighborhood states
  gTarget.irGridPos = gSrc.irGridPos;
  cv::Vec2i virGridPos( gTarget.irGridPos.x, gTarget.irGridPos.y ); 
  
  if(nDirn < 2)  virGridPos[nDirn]++;
  else 
    virGridPos[nDirn%2]--;
  
  // storing grid position
  gTarget.irGridPos.x = virGridPos[0]; gTarget.irGridPos.y = virGridPos[1];
  
  // Update connection states:
  
  // 1. push the new grid corner in the list
  mvGridCorners.push_back(gTarget); // n.b. This invalidates gSrc! (interesting...)
  
  // 2. now update the neighbor of the new grid corner to be the source
  mvGridCorners.back().aNeighborStates[(nDirn + 2) % 4].val = nSrc;
  
  // 3. And now update the source's neighbor index (not using gSrc anymore after the recent insertion)
  mvGridCorners[nSrc].aNeighborStates[nDirn].val = mvGridCorners.size() - 1;
  
  // 4. Finally, draw a cross for our new pride in the grid corner list!
  mvGridCorners.back().Draw();
  
  return true;
}

// This function draws a 15-pixel long GREEN cross dot in the image to indicate a GRID corner 
void CalibGridCorner::Draw()
{
  glLineWidth(4);
  glColor3f(0,1,0); 
  glEnable(GL_LINE_SMOOTH);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  
  glBegin(GL_LINES);
  
  // right vertex
  cv::Vec2d vertex1( Params.v2Pos[0] + Params.m2Warp()(0, 0) * 15 + Params.m2Warp()(0, 1) * 0.0, 
		     Params.v2Pos[1] + Params.m2Warp()(1, 0) * 15 + Params.m2Warp()(1, 1) * 0.0); 
  // left vertex
  cv::Vec2d vertex2( Params.v2Pos[0] + Params.m2Warp()(0, 0) * (-15) + Params.m2Warp()(0, 1) * 0.0, 
		     Params.v2Pos[1] + Params.m2Warp()(1, 0) * (-15) + Params.m2Warp()(1, 1) * 0.0 );
  // upper vertex
  cv::Vec2d vertex3( Params.v2Pos[0] + Params.m2Warp()(0, 0) * 0.0 + Params.m2Warp()(0, 1) * 15  , 
		     Params.v2Pos[1] + Params.m2Warp()(1, 0) * 0.0 + Params.m2Warp()(1, 1) * 15 );
  // lower vertex
  cv::Vec2d vertex4( Params.v2Pos[0] + Params.m2Warp()(0, 0) * 0.0 + Params.m2Warp()(0, 1) * (-15) , 
		     Params.v2Pos[1] + Params.m2Warp()(1, 0) * 0.0 + Params.m2Warp()(1, 1) * (-15) );
  
  
  // 'horizontal' line 
  glVertex2d(vertex1[0], vertex1[1]);
  glVertex2d(vertex2[0], vertex2[1]);
  // 'vertical' line
  glVertex2d(vertex3[0], vertex3[1]);
  glVertex2d(vertex4[0], vertex4[1]);
  
  glEnd();
}

// For a change, original comments give the idea (see below)...
double CalibGridCorner::ExpansionPotential()
{
  // Scoring function. How good would this grid corner be at finding a neighbor?
  // The best case is if it's already surrounded by three neighbors and only needs
  // to find the last one (because it'll have the most accurate guess for where
  // the last one should be) and so on.
  int nMissing = 0;
  for(int i=0; i<4; i++)
    if(aNeighborStates[i].val == N_NOT_TRIED) nMissing++;

  if(nMissing == 0) return 0.0;
  
  if(nMissing == 1) return 100.0;
  
  if(nMissing == 3) return 1.0;

  if(nMissing == 2) {
    
      int nFirst = 0;
      while(aNeighborStates[nFirst].val != N_NOT_TRIED) nFirst++;
      
      if(aNeighborStates[(nFirst + 2) % 4].val == N_NOT_TRIED) return 10.0;
      else
	return 20.0;
    }
  
    
    assert(0 && " Yes, we got t a point that the code is not supposed to get!!!!! Got FOUR UNTRIED neighboring states!"); // should never get here. Oh yes you get...
   
    return 0.0;
};


// This function computes the AVERAGE difference vector in each of the principal dirctions (i.e, horizontal and vertical)
// and stores it in a matrix row-wise (so 1st row is the average in the vertical and the second contains the average in the vertical).
cv::Mat_<float> CalibGridCorner::GetSteps(vector<CalibGridCorner> &vgc)
{
  
  cv::Mat_<float> m2Steps(2, 2); // 2x2
  for(int dirn=0; dirn<2; dirn++) {
    
      cv::Vec2f v2Dirn(0,0);
      int nFound = 0;
      
      // if there is a neighbor in the positive direction indexed by "dirn" 
      // then add the respective direction vector to V2Dirn and increase the number of found neighbors
      if(aNeighborStates[dirn].val >=0) {
	
	  v2Dirn += vgc[aNeighborStates[dirn].val].Params.v2Pos - Params.v2Pos;
	  nFound++;
      }
      
      // Similarly, if there is a neighbor in the negative direction indexed by "dirn" (in which case it is ndirn + 2),
      // then subtract the respective unnormalized direction vector from v2Dirn.
      if(aNeighborStates[dirn+2].val >=0)
	{
	  v2Dirn -= vgc[aNeighborStates[dirn+2].val].Params.v2Pos - Params.v2Pos;
	  nFound++;
	}
      // if no neighbor was found, then assign the "inherited" steps for this particular direction (horizontal or vertical)
      // The inherited steps SHOULD NOT be uninitialized, given the way things evolve. This is because, the first time ever that
      // this function gets called, the respective grid corner has at least one positive neighboring value in each principal direction.
      if(nFound == 0) {
	m2Steps(dirn, 0) = mInheritedSteps(dirn, 0);
	m2Steps(dirn, 1) = mInheritedSteps(dirn, 1);
      }
      else { // this is what we want: The AVERAGE of the unnormalized direction vectors to neighboring grid corners...
	m2Steps(dirn, 0) = v2Dirn[0] / nFound;
	m2Steps(dirn, 1) = v2Dirn[1] / nFound;
      }
    }
   // So now m2steps contains an AVERAGE direction vectorin each row corresponding to the horizontal (row 1) and vertical (2)
  return m2Steps;
}

int CalibImage::NextToExpand()
{
  int nBest = -1;
  double dBest = 0.0;
  
  for(unsigned int i=0; i<mvGridCorners.size(); i++)
    {
      double d = mvGridCorners[i].ExpansionPotential();
      if(d > dBest)
	{
	  nBest = i;
	  dBest = d;
	}
    }
  return nBest;
}

void CalibImage::ExpandByStep(int n)
{
  static Persistence::pvar3<double> gvdMaxStepDistFraction("CameraCalibrator.ExpandByStepMaxDistFrac", 0.4, Persistence::SILENT);
  static Persistence::pvar3<int> gvnCornerPatchSize("CameraCalibrator.CornerPatchPixelSize", 20, Persistence::SILENT);
  
  CalibGridCorner &gSrc = mvGridCorners[n];
  
  // First, choose which direction to expand in...
  // Ideally, choose a dirn for which the Step calc is good!
  int nDirn = -10;
  for(int i=0; nDirn == -10 && i<4; i++)
    {
      if(gSrc.aNeighborStates[i].val == N_NOT_TRIED &&
	 gSrc.aNeighborStates[(i+2) % 4].val >= 0)
	nDirn = i;
    }
  if(nDirn == -10)
  for(int i=0; nDirn == -10 && i<4; i++)
    {
      if(gSrc.aNeighborStates[i].val == N_NOT_TRIED)
	nDirn = i;
    }
  assert((nDirn != -10) && "Couldn't find a direction to expand when we were actually supposed to (by virtue of the results of 'ExpansionPotential()' ) !!!");

  // The function below decodes a direction index (0 - 3) into 
  // a grid direction tuple [0, 1], [1, 0], [0, -1], [-1, 0]
  cv::Point2i irGridStep = IR_from_dirn(nDirn);
  // Now, depending on irGridStep, we wanna move either along the 1st row vector direction (horizontal)
  // or the second row vector (vertical), hence the multiplication of M' * irGridStep which will result in wither the first row or the second
  cv::Mat_<float> M = gSrc.GetSteps(mvGridCorners);
  cv::Vec2f v2Step(M(0, 0) * irGridStep.x + M(1, 0) * irGridStep.y, 
		   M(0, 1) * irGridStep.x + M(1, 1) * irGridStep.y );
  
  // now the position to look for a new corner is the current grid corner position plus the (spatial) offset found in v2Step
  cv::Vec2f v2SearchPos = gSrc.Params.v2Pos + v2Step;
  
  // Before the search: pre-fill the failure result for easy returns.
  gSrc.aNeighborStates[nDirn].val = N_FAILED;
  
  // here's our best point again. Will try to find something close to the search location (stored in "searchPos")
  cv::Point2i irBest;
  double dBestDist = 99999;
  for(unsigned int i=0; i<mvCorners.size(); i++) {
      cv::Vec2f v2Diff( mvCorners[i].x - v2SearchPos[0], mvCorners[i].y - v2SearchPos[1] );
      // Again, if we have a free corner that is closer to the search location, skip to the next corner
      double v2DiffNorm = cv::norm(v2Diff);
      if( v2DiffNorm  > dBestDist  ) continue;
      // so this is the closest corner that we got so far. Store it.
      dBestDist = v2DiffNorm;
      irBest = mvCorners[i];
    }
  // let dSetpDist be the norm of the step
  double dStepDist = cv::norm(v2Step);
  // Then if the best distance is from the search point is greater then some percentage (less than 50%)
  // of the distance from the current grip corner to the search point, then quit this search effort.
  // This is yet another hackey criterion...
  if(dBestDist > *gvdMaxStepDistFraction * dStepDist) return;
  
  // Ok, we are about to add one more corner in the gridcorner list!
  CalibGridCorner gTarget;
  // again, copy the source parameters because its easy (distortion(angles) should be the same)
  gTarget.Params = gSrc.Params;
  // storing position
  gTarget.Params.v2Pos = cv::Vec2f(irBest.x, irBest.y);
  // negating gain (for whatever reason... My guess that this is done provisionally - in case someone needs to identify a recently created grid corner )
  //gTarget.Params.dGain *= -1;
  // storing position in the grid
  gTarget.irGridPos = cv::Point2i(gSrc.irGridPos.x + irGridStep.x , gSrc.irGridPos.y + irGridStep.y);
  // work out the  inheritedsteps (again, for reminders, GetSteps() DOES NOT REALLY RETURN STEPS! It merely gives an average displacement vector
  // along the two principal directions returns them stored (row-wise fashion) in a 2x2 matrix.
  gTarget.mInheritedSteps = gSrc.GetSteps(mvGridCorners);
  // Now create a [patch object in order to refine its parameters with iterate
  CalibCornerPatch Patch(*gvnCornerPatchSize);
  // Run iteration for position and parameters
  if(!Patch.IterateOnImageWithDrawing(gTarget.Params, mim)) return;
  
  // So now, having passed the iterative refinement stage, we have a brand new GRID corner and we need:
  // a) Add it to the list of Grid Corners,
  // b) Update the list entries of grid corners that are connected to it.
  // Update connection states:
  int nTargetNum = mvGridCorners.size(); // the index of the new entry to be
  for(int dirn = 0; dirn<4; dirn++) {
      // the grid step ([0 1], [1 0], [-1 0], [0 -1]) from the index (dirn =  0, 1, 2, 3)
      cv::Point2i irFromDirn = IR_from_dirn(dirn);
      // We are taking a step (in the grid-world) to the nearest grod corner along the direction defined by "dirn"
      cv::Point2i irSearch(gTarget.irGridPos.x + irFromDirn.x,
			   gTarget.irGridPos.y + irFromDirn.y); // not using overload just for peace of mind...
      // loop over the registered grid corners. If one of them is "irSearch", then update their neighbors in the appropriate direction
      for(unsigned int i=0; i<mvGridCorners.size(); i++)
	if(mvGridCorners[i].irGridPos == irSearch)
	  {
	    gTarget.aNeighborStates[dirn].val = i;
	    mvGridCorners[i].aNeighborStates[(dirn + 2) % 4].val = nTargetNum; // recall that nTargetNum is the index of the new corner to be
	  }
   }
   // all done, grid corner entries updated! Now we simply add the new grid corner entry
  mvGridCorners.push_back(gTarget);
  // draw a cross at the new corner location
  mvGridCorners.back().Draw();
}

// just draw the detected grid on the image (blue color)
void CalibImage::DrawImageGrid() 
{
  glLineWidth(4);
  glColor3f(0,0,1);
  glEnable(GL_LINE_SMOOTH);
  glEnable(GL_BLEND);
  glBegin(GL_LINES);
  // specify linear segments from each grid corner to its neighbors...
  for(int i=0; i< (int) mvGridCorners.size(); i++)
    {
      for(int dirn=0; dirn<4; dirn++)
	if(mvGridCorners[i].aNeighborStates[dirn].val > i)
	  {
	    glVertex2f(mvGridCorners[i].Params.v2Pos[0], mvGridCorners[i].Params.v2Pos[1]);
	    
	    glVertex2f(mvGridCorners[mvGridCorners[i].aNeighborStates[dirn].val].Params.v2Pos[0], 
		       mvGridCorners[mvGridCorners[i].aNeighborStates[dirn].val].Params.v2Pos[1]);
	  }
    }
  glEnd();
  
  glPointSize(5);
  glEnable(GL_POINT_SMOOTH);
  glColor3f(1,1,0);
  
  // and draw points for each grid corner...
  glBegin(GL_POINTS);
  
  for(unsigned int i=0; i<mvGridCorners.size(); i++)
    glVertex2f(mvGridCorners[i].Params.v2Pos[0], mvGridCorners[i].Params.v2Pos[1]);
  
  glEnd();
};


// This method draws a cool 3D projection grid from the detected grid corner locations
void CalibImage::Draw3DGrid(ATANCamera &Camera, bool bDrawErrors)
{
  glLineWidth(3);
  glColor3f(1,0,0); // red
  glEnable(GL_LINE_SMOOTH);
  glEnable(GL_BLEND);
  glBegin(GL_LINES);
  
  // go through the registered grid corners
  for(int i=0; i< (int) mvGridCorners.size(); i++)
    {
      // now, foreach or the four grid directions , 
      for(int dirn=0; dirn<4; dirn++)
	if(mvGridCorners[i].aNeighborStates[dirn].val > i)
	  {
	    // store the grid coordinates in a 3D vector 
	    cv::Vec3f v3( mvGridCorners[i].irGridPos.x,
			  mvGridCorners[i].irGridPos.y,
			  0.0 );
	    // Now, transform these coordinates into the 3D world
	    cv::Vec3f cvec = mse3CamFromWorld * v3;// Hopefully this overload works well (havent yet tested SE3, SO3, SO2, and SE2 for bugs)...
	    // Acquiring the 2D Euclidean projection of cvec into cvec_proj
	    cv::Vec2f cvec_proj = cv::Vec2f(cvec[0] / cvec[2], cvec[1] / cvec[2]);;
	    // Now turn the Euclidean projection into image projection (funny, we don't have the camera intrinsics!)
	    cv::Vec2f m = Camera.Project(cvec_proj);
	    // set the vertex
	    glVertex2d(m[0], m[1]);
	    
	    // Now taking the grid position of the neighbor of the current current grid corner (in the direction "dirn")
	    // and we do the same!
	    v3[0] = mvGridCorners[mvGridCorners[i].aNeighborStates[dirn].val].irGridPos.x;
	    v3[1] = mvGridCorners[mvGridCorners[i].aNeighborStates[dirn].val].irGridPos.y;
	    // World location
	    cvec = mse3CamFromWorld * v3; // Hope the overload is done without mistakes...
	    // Euclidean projection
	    cvec_proj = cv::Vec2f(cvec[0] / cvec[2], cvec[1] / cvec[2]);
	    // image projection
	    m = Camera.Project(cvec_proj);
	    // set the vertex
	    glVertex2d(m[0], m[1]);
	  }
    }
    
  glEnd();

  // draw errors
  if(bDrawErrors) {
    
      glColor3f(1,1,0);
      glLineWidth(1);
      glBegin(GL_LINES);
      // go over the grid corners again
      for(int i=0; i< (int) mvGridCorners.size(); i++)
	{
	  // again, transform the grid location into the 3D world and then back-project it on the image (with made-up/default intrinsics)
	  cv::Vec3f v3( mvGridCorners[i].irGridPos.x, 
			mvGridCorners[i].irGridPos.y,
			0.0 );
	  
	  cv::Vec3f cvec = mse3CamFromWorld * v3;// Once again, SE3, SO3, SE2, SO2 classes and operators have not been fully tested for bugs.
						 // The same goes for arithmetic operators (including ^ for cross product), so handle with care...
	
	  cv::Vec2f cvec_proj(cvec[0] / cvec[2], cvec[1] / cvec[2]);
	  cv::Vec2f m = Camera.Project(cvec_proj);
	 
	  
	  cv::Vec2f v2pixBackProjection = Camera.Project(m);
	  // now the error vector is simply the difference between the v2Pos measured in the image and back-projection (aka "v2PixelsBackProjection")
	  cv::Vec2f v2Error = mvGridCorners[i].Params.v2Pos - v2pixBackProjection;
	  // set vertex at the backProjection point
	  glVertex2f(v2pixBackProjection[0], v2pixBackProjection[1]);
	  // set second vertex 10 pixels at the direction of the error away from the back-projection.
	  glVertex2f(v2pixBackProjection[0] + 10.0 * v2Error[0], v2pixBackProjection[1] + 10.0 * v2Error[1]);
	}
	
      glEnd();
    }
};


// returns a displacement in grid terms from a direction index.
// i.e., 0 = [1 0]  ,  1 = [0 1], 2 = [-1 0], 3 = [0 -1]
cv::Point2i CalibImage::IR_from_dirn(int nDirn)
{
  cv::Vec2i ir(0, 0);
  ir[nDirn%2] = (nDirn < 2) ? 1: -1;
  
  return cv::Point2i(ir[0], ir[1]);
}


// Extracts camera pose from the homography that maps the detected grid from its plane in space 
// on to the screen/image plane 
void CalibImage::GuessInitialPose(ATANCamera &Camera)
{
  
  // number of registered grid points
  int nPoints = mvGridCorners.size();
  // Doing the 9x9 gram-matrix accumulator instead of the data matrix. Its better.
  cv::Matx<double, 9, 9> m9D = cv::Matx<double, 9, 9>::zeros();
  for(int n=0; n<nPoints; n++) {
    // First, beck-project the image locations of the recovered grid corners onto the normalized Euclidean plane (z = 1)
    cv::Vec2f v2UnProj = Camera.UnProject(mvGridCorners[n].Params.v2Pos);  
    double x2 = v2UnProj[0];
    double y2 = v2UnProj[1];
    // So, now u and v are 2D Euclidean coordinates on the projection ray for z = 1.
      
    // Then fill in the matrix..
    double x1 = mvGridCorners[n].irGridPos.x; // corner x-location in the grid! (assuming unit length in the grid!)
    double y1 = mvGridCorners[n].irGridPos.y; // corner y-locatin in the grid!  
    
    // filling ONLY the Upper triangle of m9D...
    //
    // 1. Filling the upper triangle of the upper 3x3 block (which is equal to  the second 3x3 diagonal block)
    m9D(0, 0) += x1*x1;  m9D(0, 1) += x1*y1; m9D(0, 2) += x1;
			 m9D(1, 1) += y1*y1; m9D(1, 2) += y1;
					     m9D(2, 2) += 1.0;

      // 2. Now filling the 3 columns from 7 to 8 down-to and including the diagonal:
      m9D(0, 6) += -x1*x1*x2;             m9D(0, 7) += -x1*x2*y1;              m9D(0, 8) += -x1*x2;
      m9D(1, 6) += -x1*x2*y1;             m9D(1, 7) += -x2*y1*y1;              m9D(1, 8) += -x2*y1;
      m9D(2, 6) += -x1*x2;                m9D(2, 7) += -x2*y1;                 m9D(2, 8) += -x2;
      m9D(3, 6) += -x1*x1*y2;             m9D(3, 7) += -x1*y1*y2;              m9D(3, 8) += -x1*y2;
      m9D(4, 6) += -x1*y1*y2;             m9D(4, 7) += -y1*y1*y2;              m9D(4, 8) += -y1*y2;
      m9D(5, 6) += -x1*y2;                m9D(5, 7) += -y1*y2;                 m9D(5, 8) += -y2;
      m9D(6, 6) += x1*x1*(x2*x2 + y2*y2); m9D(6, 7) += x1*y1*(x2*x2 + y2*y2);  m9D(6, 8) += x1*(x2*x2 + y2*y2);
					  m9D(7, 7) += y1*y1*(x2*x2 + y2*y2);  m9D(7, 8) += y1*(x2*x2 + y2*y2);
									       m9D(8, 8) += x2*x2 + y2*y2;
      
  }
  // Sow no filling-in the gaps (left-out due to symmetry):
  // 1. Filling the missing lower partof the upper 3x3 diagonal block and copying tho the second diagonal 3x3 block
  m9D(1, 0) = m9D(4, 3) = m9D(3, 4) = m9D(0, 1); 
  m9D(2, 0) = m9D(5, 3) = m9D(3, 5) = m9D(0, 2);   m9D(2, 1) = m9D(5, 4) = m9D(4, 5) = m9D(1, 2);
  // and the diagonalelements from 3 - 5 are the same ones from 0-2:
  m9D(3, 3) = m9D(0, 0); m9D(4, 4) = m9D(1, 1); m9D(5, 5) = m9D(2, 2);
  
  // 2. Now copying the last 3 columns (down-to and exluding the diagonal) to the last 3 rows...
  m9D(6, 0) = m9D(0, 6); m9D(6, 1) = m9D(1, 6); m9D(6, 2) = m9D(2, 6); m9D(6, 3) = m9D(3, 6); m9D(6, 4) = m9D(4, 6); m9D(6, 5) = m9D(5, 6);
  m9D(7, 0) = m9D(0, 7); m9D(7, 1) = m9D(1, 7); m9D(7, 2) = m9D(2, 7); m9D(7, 3) = m9D(3, 7); m9D(7, 4) = m9D(4, 7); m9D(7, 5) = m9D(5, 7); m9D(7, 6) = m9D(6, 7);
  m9D(8, 0) = m9D(0, 8); m9D(8, 1) = m9D(1, 8); m9D(8, 2) = m9D(2, 8); m9D(8, 3) = m9D(3, 8); m9D(8, 4) = m9D(4, 8); m9D(8, 5) = m9D(5, 8); m9D(8, 6) = m9D(6, 8); m9D(8, 7) = m9D(7, 8); 
  
  
  
  // In any case (null-space or smallest singular value), we need the last row of V^t
  // The right null-space or th last eigenvector of m3D9 gives the homography...
  cv::Matx<double, 9, 9> U, Vt;
  cv::Matx<double, 9, 1> w;
  cv::SVD::compute(m9D, w, U, Vt);
  
  cv::Matx<double, 3, 3> m3Homography;
  m3Homography(0, 0) = Vt(8, 0); m3Homography(0, 1) = Vt(8, 1); m3Homography(0, 2) = Vt(8, 2);
  m3Homography(1, 0) = Vt(8, 3); m3Homography(1, 1) = Vt(8, 4); m3Homography(1, 2) = Vt(8, 5);
  m3Homography(2, 0) = Vt(8, 6); m3Homography(2, 1) = Vt(8, 7); m3Homography(2, 2) = Vt(8, 8);
  

  // Fix up possibly poorly conditioned bits of the homography
  // This appears to be essentially the scaling-down of the homography by the largest singular values of its upper- left 2x2 block
 
  cv::Matx<double, 2, 2> Htl = m3Homography.get_minor<2, 2>(0, 0);
    
  cv::Matx<double, 2, 1> v2Diagonal;
  cv::Matx<double, 2, 2> v2Vt;
  cv::Matx<double, 2 , 2> v2U;
    
  cv::SVD::compute(Htl, v2Diagonal, v2U, v2Vt); 
  double smax = v2Diagonal(0,0);
    
     
  // scaling down the entire homography by the largest singular value of H11
  m3Homography = (1 / smax) * m3Homography ;
    

  // OK, now turn homography into something 3D ...simple gram-schmidt ortho-norm
  // Take 3x3 matrix H with column: abt
  // And add a new 3rd column: abct
  cv::Matx<float, 3, 3> mRotation(3, 3);
  cv::Vec3f vTranslation;
  //double dMag1 = sqrt(m3Homography.T()[0] * m3Homography.T()[0]);
  double dMag1 = sqrt( m3Homography(0, 0) * m3Homography(0, 0)  + 
		       m3Homography(1, 0) * m3Homography(1, 0) + 
		       m3Homography(2, 0) * m3Homography(2, 0) );
  // 1. Scale the entire homography (it could have been just the first column)
  m3Homography = (1/ dMag1) * m3Homography ;
  
  // Store the first column of the homography in the 1st column of the rotation
  //mRotation.T()[0] = m3Homography.T()[0];
  mRotation(0, 0) = m3Homography(0, 0); 
  mRotation(1, 0) = m3Homography(1, 0); 
  mRotation(2, 0) = m3Homography(2, 0);
  
  // 2. Now subtract the projection of the second column onto the first from the second. Store result in the second column of the rotation matrix
  double dot12 = m3Homography(0, 0) * m3Homography(0,1)  + m3Homography(1,0) * m3Homography(1, 1) + m3Homography(2,0) * m3Homography(2, 1);
  mRotation(0, 1) = m3Homography(0, 1) - dot12 * m3Homography(0, 0); 
  mRotation(1, 1) = m3Homography(1, 1) - dot12 * m3Homography(1, 0); 
  mRotation(2, 1) = m3Homography(2, 1) - dot12 * m3Homography(2, 0); 
  
  // 3. Normalize the second column of the rotation matrix...
  double norm2 = sqrt(mRotation(0, 1) * mRotation(0, 1) + mRotation(1, 1) * mRotation(1, 1) + mRotation(2, 1) * mRotation(2, 1)); 
  mRotation(0, 1) /= norm2; mRotation(1, 1) /= norm2; mRotation(2, 1) /= norm2;
  
  // 3. Store the cross product of the first and second column of the rotation matrix in the third,
  // Although i have an overload read ("^"), I 'd rather play it safe and embed the operation below...
  mRotation(0, 2) = -mRotation(2, 0) * mRotation(1, 1) + mRotation(1, 0) * mRotation(2, 1);
  mRotation(1, 2) =  mRotation(2, 0) * mRotation(0, 1) - mRotation(0, 0) * mRotation(2, 1);
  mRotation(2, 2) = -mRotation(1, 0) * mRotation(0, 1) + mRotation(0, 0) * mRotation(1, 1);
  
  // Obtaining the translation from the 3d column of the homography
  //vTranslation = m3Homography.T()[2];
  vTranslation[0] =  m3Homography(0, 2);
  vTranslation[1] =  m3Homography(1, 2);
  vTranslation[2] =  m3Homography(2, 2);

  int negative_depth_counter = 0, 
  positive_depth_counter = 0;
  for(int n=0; n<nPoints; n++) 
  {

    // Then fill in the matrix..
    double X = mvGridCorners[n].irGridPos.x; // corner x-location in the grid! (assuming unit length in the grid!)
    double Y = mvGridCorners[n].irGridPos.y; // corner y-locatin in the grid!  

    cv::Vec<double, 3> Mc = m3Homography * cv::Vec<double, 3>(X, Y, 0);
  	if (Mc[2] < 0) 
  	{
  	  negative_depth_counter++;
  	}
  	else
  	{
  	  positive_depth_counter++;
  	}
  }

  if (negative_depth_counter > positive_depth_counter)
  {
  	// Negate first and second column of rotation
  	mRotation(0, 0) *=-1; mRotation(0, 1) *=-1;
  	mRotation(1, 0) *=-1; mRotation(1, 1) *=-1;
  	mRotation(2, 0) *=-1; mRotation(2, 1) *=-1;

  	// And negate the translation
  	vTranslation = -vTranslation;
  }

  // Finally, store everything in the SE3 object the takes world points to the camera
  mse3CamFromWorld.get_rotation().get_matrix() = mRotation;
  mse3CamFromWorld.get_translation() = vTranslation;


}





// This function essentially fills-in the derivatives per calibration image
vector<CalibImage::ErrorAndJacobians> CalibImage::Project(ATANCamera &Camera) {
  
  vector<CalibImage::ErrorAndJacobians> vResult;
  for(unsigned int n=0; n < mvGridCorners.size(); n++) {
      
      ErrorAndJacobians EAJ; 
      
      // The coordinates of the point in the grid are simply its (unit-size) grid coordinates
      cv::Vec3f v3World(mvGridCorners[n].irGridPos.x,  mvGridCorners[n].irGridPos.y, 0.0);
      
      // In the camera frame, the coordinates are:
      cv::Vec3f v3Cam = mse3CamFromWorld * v3World;
      
      // drop the point if the depth is very small
      if(v3Cam[2] <= 0.001) continue;
      
     
      // Now project the point on the image
      cv::Vec2f v2Image = Camera.Project( CvUtils::pproject(v3Cam) );
      // if the prjection falls out-of-bounds (or off-maximum radius) 
      // then drop the point
      if(Camera.Invalid()) continue;
      // Error is the measured (tracked) coordinates on the image 
      // minus the projected one based on current parameters for camera pose and 
      // intrinsics.
      EAJ.v2Error = mvGridCorners[n].Params.v2Pos - v2Image;
      
      // Now find motion jacobian..
      double invZ = 1.0 / v3Cam[2], X = v3Cam[0], Y = v3Cam[1];
      
      cv::Matx<float, 2, 2> m2CamDerivs = Camera.GetProjectionDerivs();
      
      for(int dof=0; dof<6; dof++) {
	  
	  // Get the3 derivative of the camera pose in the following order of argument in "generator_field":
	  // 0: tx    ,   1:ty   ,    2:tz     , 3-5: axis-0angle vector u
	  const cv::Vec4f v4Motion = SE3<>::generator_field(dof, CvUtils::backproject(v3Cam) );
	  
	  cv::Vec2f v2CamFrameMotion( (v4Motion[0] - X * v4Motion[2] * invZ) * invZ, 
				      (v4Motion[1] - Y * v4Motion[2] * invZ) * invZ);
	  
	  //EAJ.m26PoseJac.T()[dof] = m2CamDerivs * v2CamFrameMotion;
	  EAJ.m26PoseJac(0, dof) = m2CamDerivs(0, 0) * v2CamFrameMotion[0] + m2CamDerivs(0, 1) * v2CamFrameMotion[1]; 
	  EAJ.m26PoseJac(1, dof) = m2CamDerivs(1, 0) * v2CamFrameMotion[0] + m2CamDerivs(1, 1) * v2CamFrameMotion[1]; 
	  
      }

      // Finally, the derivatives of the projection wrt camera parameters.
      // Note the derivatives are numerically estimated
      //EAJ.m2NCameraJac = Camera.GetCameraParameterDerivs();
      EAJ.m2NCameraJac = Camera.GetCamParamAnalyticalDerivs();
      vResult.push_back(EAJ);
      
    }
  return vResult;
}







