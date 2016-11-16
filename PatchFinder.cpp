// George Terzakis 2016
//
// University of Portsmouth
//
// Code based on PTAM by Klein and Murray (Copyright 2008 Isis Innovation Limited)

#include "PatchFinder.h"
#include "KeyFrame.h"

#include "GCVD/Addedutils.h"
#include "GCVD/image_interpolate.h"

// This stuff is ghenerally available these days. GNU has it, I guess MSVC has it as well...
#if USE_XMMINTRIN
#include <xmmintrin.h>
#endif

using namespace std;
// Note that the patchfinder sets-up an 8x8 patch by default which is a brand new empty thing....
PatchFinder::PatchFinder(int nPatchSize) : mimTemplate( cv::Mat_<uchar>(nPatchSize, nPatchSize) ) {
  
  mnPatchSize = nPatchSize;
  // Just for the aid of anyone trying to decipher the code,
  // REMEMBER! mirCenter is simply HALF the size the of the Patch!!!!!
  mirCenter = cv::Point2i(nPatchSize/2, nPatchSize/2);
  int nMaxSSDPerPixel = 500; // TODO: Find nominal values and make it persistent!
  mnMaxSSD = mnPatchSize * mnPatchSize * nMaxSSDPerPixel;
  // Initialize sthe warping matrix cache.
  mm2LastWarpMatrix = 9999.9 * cv::Matx<float, 2, 2>::eye();
  mpLastTemplateMapPoint = NULL;
}


// Find the warping matrix and search level
int PatchFinder::CalcSearchLevelAndWarpMatrix(MapPoint::Ptr pMP, SE3<> se3CFromW, cv::Matx<float, 2, 2> &m2CamDerivs) {
  // Calc point pos in new view camera frame
  // Slightly dumb that we re-calculate this here when the tracker's already done this!
  cv::Vec<float, 3> v3Cam = se3CFromW * pMP->v3WorldPos;
  double invDepth = 1.0 / v3Cam[2];
  
  
  // ************************** READ BELOW - READ BELOW ***********************************
  // NOTE: So this why we need to store the Euclidean images of the up and right pixels!!!
  //       We can (not so) simply use them to obtain the 2D warping matrix! Briliant! Briliant!
  //
  // Rotate the "PixelGoDown" and "pixelGoRight"  Euclidean translation vectors
  // (RECALL the are eactually the translations from the 3D point to the backprojections of the right and down pixels
  // in the map!!! Think of them as a BAT-SIGNAL in the sky...) by R.
  cv::Vec<float, 3> v3MotionRight = se3CFromW.get_rotation() * pMP->v3PixelGoRight_W; 
  cv::Vec<float, 3> v3MotionDown = se3CFromW.get_rotation() * pMP->v3PixelGoDown_W;   
  double invDepthRight = 1.0 / ( v3Cam[2] + v3MotionRight[2] );
  double invDepthDown = 1.0 / ( v3Cam[2] + v3MotionDown[2] );
  
  // So now we have the 3D point (v3Cam) and we have the translation vectors to up and right in the world!
  // All we need to do is project these three points on the plane, find the respective translations on the Euclidean plane,
  // and that should give us the warp we are looking for!!! Briliant!!!
  
  // NOTE!!!! Original PTAM comments once ORTHOGONALLY misleading (possibly a third person ?) !!! 
  //          " Calculate in-image derivatives of source image pixel motions:"????? Absolutely nothing like that unfortunately...
  //
  // ************************** HOW THE WARPING MATRIX IS COMPUTED ************************************
  //
  // Having the BAT-SIGNAL of the patch in the sky, we want to project it onto the new camera Euclidean plane Z = 1,
  // and subsequently onto the new image, so that we get an idea on how the image is deformed LOCALLY in a region
  // CENTERED around the point (thus we are taking local coordinates and NOT image coordinates).
  //
  // To do this, we have to project the BATSIGNAL using the camera object. Problem is, the projection model is no longer linear,
  // due to the presence of the radial distortion compensation term. So what do we do???? Of copurse we linearize (hey, its 
  // practically a linear relationship for the most part anyway)...
  //
  // -----> So, we linearize the image projection in the area of the new point, say p0. Thus, suppose that m0, mr, md are the
  //        normalized Euclidean projections of the respective map locations (in local new camera frame), then:
  //
  //           m0 = M/Z        , md = (M + d) / ( Z+dz )    and   mr = (M + r) / (Z + rz)
  //
  //  Thus, on the Euclidean plane Z = 1, the differences will be:
  //
  //          md - m0 = ( d - dz * M / Z ) / ( Z + dz )   and mr - m0 = ( r - rz * M / Z ) / ( Z + rz )
  // 
  // The creators of PTAM make one more approximation: Z + dz ~= Z + rz = Z and that's how we obtain the differences in the image:
  //
  //                   pd - p0 ~= G * (d - dz * M / Z) / Z     and,   pl - p0 ~= G * (r - rz * M / Z ) / Z
  //
  // So, finally, knowing that the original points where 1 pixel apart, we can easily create a warping matrix (in point centered coordinates)
  //     as follows:
  // 
  //                             M = [ G * (r - rz * X / Z) / (Z + rz)    ,    G * (d - dz * X / Z) / (Z + dz) ] 
  //
  // Note here that I choose to keep the Z + rz and Z+dz as global denominators just to see what happens...                                                
  //
  //    			So the formula in the original code was:
  //
  //				M = [ G * (r - rz * X / Z)    ,    G * (d - dz * X / Z) ] / Z 
  
  // 1st Column of the warp matrix
  mm2WarpInverse(0, 0) = ( m2CamDerivs(0, 0) * (v3MotionRight[0] - v3MotionRight[2] * v3Cam[0] * invDepth) +
			   m2CamDerivs(0, 1) * (v3MotionRight[1] - v3MotionRight[2] * v3Cam[1] * invDepth) ) * invDepthRight; //invDepth;
  mm2WarpInverse(1, 0) = ( m2CamDerivs(1, 0) * (v3MotionRight[0] - v3MotionRight[2] * v3Cam[0] * invDepth) +
			   m2CamDerivs(1, 1) * (v3MotionRight[1] - v3MotionRight[2] * v3Cam[1] * invDepth) ) * invDepthRight; // invDepth;
  
  // 2nd column of the warp matrix
  mm2WarpInverse(0, 1) = ( m2CamDerivs(0, 0) * (v3MotionDown[0] - v3MotionDown[2] * v3Cam[0] * invDepth) +
			   m2CamDerivs(0, 1) * (v3MotionDown[1] - v3MotionDown[2] * v3Cam[1] * invDepth) ) * invDepthDown; // invDepth;
  mm2WarpInverse(1, 1) = ( m2CamDerivs(1, 0) * (v3MotionDown[0] - v3MotionDown[2] * v3Cam[0] * invDepth) +
			   m2CamDerivs(1, 1) * (v3MotionDown[1] - v3MotionDown[2] * v3Cam[1] * invDepth) ) * invDepthDown; //invDepth;
  
  double dDet = mm2WarpInverse(0, 0) * mm2WarpInverse(1, 1) - mm2WarpInverse(1, 0) * mm2WarpInverse(0, 1);
  mnSearchLevel = 0;
  
  // This warp matrix is likely not appropriate for finding at level zero, which is 
  // the level at which it has been calculated. Vary the search level until the 
  // at that level would be appropriate (does not actually modify the matrix.)
  while(dDet > 3 && mnSearchLevel < LEVELS-1) {
    
      mnSearchLevel++;
      dDet *= 0.25;
  }
  
  // Some warps are inappropriate, e.g. too near the camera, too far, or reflected, 
  // or zero area.. reject these!
  if(dDet > 3 || dDet < 0.25) {
      mbTemplateBad = true;
      return -1;
  }
  else
    return mnSearchLevel;
}

// This is just a convenience function wich caluclates the warp matrix and generates
// the template all in one call.
void PatchFinder::MakeTemplateCoarse(MapPoint::Ptr pMP, SE3<> se3CFromW, cv::Matx<float, 2, 2> &m2CamDerivs /*2x2*/) {
  
  CalcSearchLevelAndWarpMatrix(pMP, se3CFromW, m2CamDerivs);
  // Go to the MakeTemplateCoarse() stage  ONLY if the warping matrix
  // is NON-SINGULAR! Otherwise you will get a fat exception for the inversion!
  if (!mbTemplateBad) MakeTemplateCoarseCont(pMP);
}

// This function generates the warped search template.
// NOTE! Should NOT be called if the inverse Warping matrix is singular (i.e., mbTemplateBad == true) !! 
void PatchFinder::MakeTemplateCoarseCont(MapPoint::Ptr pMP)
{
  // Get the warping matrix appropriate for use with CVD::transform...
    
    cv::Matx<float, 2, 2> m2 = LevelScale(mnSearchLevel) * CvUtils::M2Inverse(mm2WarpInverse); 
  // m2 now represents the number of pixels in the source image for one 
  // pixel of template image
  
  // Optimisation: Don't re-gen the coarse template if it's going to be substantially the 
  // same as was made last time. This saves time when the camera is not moving. For this, 
  // check that (a) this patchfinder is still working on the same map point and (b) the 
  // warping matrix has not changed much.
  
  bool bNeedToRefreshTemplate = false;
  
  if(pMP != mpLastTemplateMapPoint) bNeedToRefreshTemplate = true;
  
  // Still the same map point? Then compare warping matrix..
  for(int i=0; !bNeedToRefreshTemplate && i<2; i++) {
    
      //Vector<2> v2Diff = m2.T()[i] - mm2LastWarpMatrix.T()[i];
      cv::Vec<float, 2> v2Diff( m2(0, i) - mm2LastWarpMatrix(0, i) , 
				m2(1, i) - mm2LastWarpMatrix(1, i) 
			      );
      const double dRefreshLimit = 0.07;  // Sort of works out as half a pixel displacement in src img
      if ( v2Diff[0] * v2Diff[0] + v2Diff[1] * v2Diff[1]  > dRefreshLimit * dRefreshLimit) 
	bNeedToRefreshTemplate = true;
  }
  
  // Need to regen template? Then go ahead.
  if(bNeedToRefreshTemplate) {
    
      int nOutside;  // Use the transform to warp the patch according the the warping matrix m2
                     // This returns the number of pixels outside the source image hit, which should be zero.
      nOutside = CvUtils::transform(pMP->pPatchSourceKF->aLevels[pMP->nSourceLevel].im, 
				    mimTemplate, 
				    m2,
				    cv::Vec<float, 2>(pMP->irCenter.x, pMP->irCenter.y),
				    cv::Vec<float, 2>(mirCenter.x , mirCenter.y)
				    ); 
      
      if(nOutside) mbTemplateBad = true;
      else
	mbTemplateBad = false;
      
      MakeTemplateSums();
      
      // Store the parameters which allow us to determine if we need to re-calculate
      // the patch next time round.
      mpLastTemplateMapPoint = pMP;
      mm2LastWarpMatrix = m2;
    }
}

// This makes a template without warping. 
// It basically copies the mage patch around the feature position
// into the template image ("mimTemplate").
void PatchFinder::MakeTemplateCoarseNoWarp(KeyFrame::Ptr pKF, int nLevel, cv::Point2i irLevelPos) {
  
  mnSearchLevel = nLevel;
  cv::Mat_<uchar> im = pKF->aLevels[nLevel].im; // n.b. This is a reference assignment ...
  if ( !CvUtils::in_image_with_border(irLevelPos.y, irLevelPos.x, im, mnPatchSize / 2 + 1, mnPatchSize / 2 + 1)  ) {
     
    mbTemplateBad = true;
    return;
  }
  mbTemplateBad = false;
  
  
  // So now copying im from (irLevelPose - mirCenter) onto mimTemplate 
  // which should give us the lower-right quadrant of im inside mimTemplate
  int nOffsetRow = irLevelPos.y - mirCenter.y,
      nOffsetCol = irLevelPos.x - mirCenter.x;
  assert(nOffsetRow >=0 && nOffsetCol >= 0 && "irLevelPos - mirCenter DID not give positive stuff back! Look me up in MakeTemplateCoarseNoWarp");
  
  im( cv::Range(nOffsetRow, nOffsetRow + mimTemplate.rows),
      cv::Range(nOffsetCol, nOffsetCol + mimTemplate.cols) ).copyTo(mimTemplate); 
  
  // just compute sum and squared sum...
  MakeTemplateSums();
}

// Convenient wrapper for the above
void PatchFinder::MakeTemplateCoarseNoWarp(MapPoint::Ptr pMP)
{
  MakeTemplateCoarseNoWarp(pMP->pPatchSourceKF, pMP->nSourceLevel,  pMP->irCenter);
}

// Finds the sum, and sum-squared, of template pixels. These sums are used
// to calculate the ZMSSD.
inline void PatchFinder::MakeTemplateSums()
{
  int nSum = 0;
  int nSumSq = 0;
  int r, c;
  for (r = 0; r < mimTemplate.rows; r++) {
    
    uchar* pTempImRow = mimTemplate.ptr<uchar>(r);
    
    for (c = 0; c < mimTemplate.cols; c++)
    {
      int imbyte = pTempImRow[c];
      nSum += imbyte;
      nSumSq += imbyte * imbyte;
    }      
  }
  mnTemplateSum = nSum;
  mnTemplateSumSq = nSumSq;
}

// One of the main functions of the class! Looks at the appropriate level of 
// the target keyframe to try and find the template. Looks only at FAST corner points
// which are within radius nRange of the center. (Params are supplied in Level0
// coords.) Returns true on patch found.

bool PatchFinder::FindPatchCoarse(cv::Vec<float, 2> v2Pos, KeyFrame::Ptr pKF, unsigned int nRange) {
  
  mbFound = false;
  
  // Scale Level - 0 coordinates to the search level coordinates
  int nLevelScale = LevelScale(mnSearchLevel);
  mv2PredictedPos = v2Pos;
  cv::Point2i irLevelPredPos( (int)mv2PredictedPos[0] / nLevelScale,
			      (int)mv2PredictedPos[1] / nLevelScale
			     );
  // same with range
  nRange = (nRange + nLevelScale - 1) / nLevelScale;
  
  // Bounding box of search circle
  int nTop = irLevelPredPos.y - nRange;
  int nBottomPlusOne = irLevelPredPos.y + nRange + 1;
  int nLeft = irLevelPredPos.x - nRange;
  int nRight = irLevelPredPos.x + nRange;
  
  // Get a reference for the search level
  Level &L = pKF->aLevels[mnSearchLevel];
  
  // Some bounds checks on the bounding box..
  if(nTop < 0) nTop = 0;
  
  if(nTop > L.im.rows - 1) return false;
  if(nBottomPlusOne <= 0) return false;
  
  // The next section finds all the FAST corners in the target level which 
  // are near enough the search center. It's a bit optimised to use 
  // a corner row look-up-table, since otherwise the routine
  // would spend a long time trawling throught the whole list of FAST corners!
  vector<cv::Point2i>::iterator iCorner;
  vector<cv::Point2i>::iterator iLastCorner;
  // start from the index that the LU tables gives for "nTop" row index 
  iCorner = L.vCorners.begin() + L.vCornerRowLUT[nTop];
  // and finish either at the end of the corner list,
  if(nBottomPlusOne >= L.im.rows ) iLastCorner = L.vCorners.end();
  else 
    // or at the bottom row (inclusive - hence , +1) in "nBottomPlusOne"/ 
    iLastCorner = L.vCorners.begin() + L.vCornerRowLUT[nBottomPlusOne];
  
  cv::Point2i irBest;             // Best match so far
  int nBestSSD = mnMaxSSD + 1; // Best score so far is beyond the max allowed
  
  for(; iCorner < iLastCorner; iCorner++) {         // For each corner ...
                            
       // if the corner falls out of horizontal bounds, skip...
      if( iCorner->x < nLeft || iCorner->x > nRight) continue;
      
      // If the corner is out of radius (nRange) skip...
      if( (irLevelPredPos.x - iCorner->x)*(irLevelPredPos.x - iCorner->x) +
	  (irLevelPredPos.y - iCorner->y)*(irLevelPredPos.y - iCorner->y) > nRange * nRange) continue;              
      
      // Great! Corner is good so far...
      int nSSD;                
      // Now find tghe zero mean Sum of Squared Differences
      nSSD = ZMSSDAtPoint(L.im, *iCorner);
      //cout <<"DEBUG: Patchfinder distance of "<<*iCorner<<" from level image of size "<< L.im.size()<<" is : "<<nSSD<<endl;
      if(nSSD < nBestSSD) {     // Best yet?
	
	  irBest = *iCorner;
	  nBestSSD = nSSD;
      }
  } // done looping over corners
  
  if(nBestSSD < mnMaxSSD) {     // Found a valid match?
    
      mv2CoarsePos= LevelZeroPos(irBest, mnSearchLevel);
      mbFound = true;
  }
  else
    mbFound = false;
  
  return mbFound;
}

// The following function computes the 3x3 jacobian for 
// a G-N step in the 3D alignment of a putative patch with a patch 
// in some given pyramid image. The LS vector is [tx; ty; theta; offset]
// whete [tx, ty] is the 2D translation, theta is the rotation angle and offset (dubbed mean in the code) is
// a DC offset variable aimed at making the fitting more adaptive.
void PatchFinder::prepSubPixGNStep() {  
 
  mimJacs.create(mimTemplate.rows - 2, mimTemplate.cols - 2);
  
  float s = 1.0; // global LS scaler. I am setting it to 1.0 for now to avoid further entanglements...
	
  cv::Matx<float, 3, 3> m3JtJ = cv::Matx<float, 3, 3>::zeros(); // Information matrix / Cumulative Gram-matrix of the pixel gradients.
  int r, c;
  for(r = 1; r < mnPatchSize - 1; r++) {
    // pointers to current, previous and next row of mimTemplate
    uchar* pTempImRow0 = mimTemplate.ptr<uchar>(r);
    uchar* pTempImRow_1 = mimTemplate.ptr<uchar>(r - 1);
    uchar* pTempImRow1 = mimTemplate.ptr<uchar>(r + 1);
    // pointer to the r-1 row of the gradient image
    cv::Vec<float, 3>* gRowptr = mimJacs.ptr<cv::Vec<float, 3> >(r - 1);
    
    for(c = 1; c < mnPatchSize - 1; c++) {
	
	// standard difference image gradient...
        cv::Vec3f v3Grad( 0.5 * ( pTempImRow0[c+1] - pTempImRow0[c-1] ),
			  0.5 * ( pTempImRow1[c]   - pTempImRow_1[c] ),
			  1.0
			);
	
	
	// populate upper triagle of m3JtJ with the gradient gram-matrix
	m3JtJ(0, 0) += v3Grad[0] * v3Grad[0] / s; m3JtJ(0, 1) += v3Grad[0] * v3Grad[1] / s; m3JtJ(0, 2) += v3Grad[0] * v3Grad[2] / s;
					          m3JtJ(1, 1) += v3Grad[1] * v3Grad[1] / s; m3JtJ(1, 2) += v3Grad[1] * v3Grad[2] / s;
											    m3JtJ(2, 2) += v3Grad[2] * v3Grad[2] / s;
       gRowptr[c-1] = (1 / s) * v3Grad; // store the scaled gradient
			
       									
      
    }					
  }
  // filling-in the lower triangle of m3JtJ
  m3JtJ(1, 0) = m3JtJ(0, 1);
  m3JtJ(2, 0) = m3JtJ(0, 2); m3JtJ(2, 1) = m3JtJ(1, 2);
  
 float det =  CvUtils::M3Det(m3JtJ);
  if( std::fabs( det ) < 10e-8 ) {
    // Make a note that low rank system was obtained...
    std::cout << "Low rank materials in makeSubPixTemplate!!!!" << endl; // difficult to occur. It contradicts feature selection...
    
    cout << "JtJ : "<<m3JtJ<<endl;
    cout << " The computed determinant : "<<det <<endl;
    // invert the Jacobian gram matrix using the SVD
   cv::Vec<float, 3> w; 
   cv::Matx<float, 3, 3> U; 
   cv::Matx<float, 3, 3> Vt; 
   cv::Matx<float, 3, 3> S;
   cv::SVD::compute(m3JtJ, w, U, Vt);
   for (int i = 0; i<3; i++) {
     S(i, 0) = S(i, 1) = S(i, 2) = 0;
     if (w[i] != 0) S(i, i) = 1/w[i];
   }
    
    mm3JtJInv = Vt.t() * S * U.t(); // pseudo-inverse
  }
  else {
    mm3JtJInv = CvUtils::M3Inverse(m3JtJ); // TODO: Make sure there's no mistake in this!!!!
    //cv::invert(m3JtJ, mm3JtJInv, cv::DECOMP_CHOLESKY);
  }
     
  mv2SubPixPos = mv2CoarsePos; // Start the sub-pixel search at the result of the coarse search..
  mdMeanDiff = 0.0;
}

// Run G-N position refinement until converrgence (or timeout). Since it should never have 
// to travel more than a pixel's distance, set a max number of iterations; 
// if this is exceeded, consider the IC to have failed.
bool PatchFinder::IterateSubPixToConvergence(KeyFrame::Ptr pKF, int nMaxIts)
{
  const double dConvLimit = 0.03;
  bool bConverged = false;
  int nIts;
  for(nIts = 0; nIts < nMaxIts && !bConverged; nIts++) {
      
    double dUpdateSquared = IterateSubPix(pKF);
    
    if(dUpdateSquared < 0)  return false; // should not happen
    
    if(dUpdateSquared < dConvLimit*dConvLimit) return true; // unacceptable perturbation
    
  }
  
  return false;
}


// ******************** Gauss-Newton iteration for the patch position (and a DC offset in image differences...) ***********************
//
// The  function with the following quadratic terms (over each pixel in the patch):
//
//		mimTemplate(p0x+deltaxi, p0y+deltayi) - im(px + deltaxi, py + deltayi) - d0 -  G * ([px; py; d] - [p0x; p0y; d0]) 
// 
// where: a) (deltaxi, deltayi) is simply patch-relative sub-pixel coordindates and i is a pixel index.
//	  b) p = (px, py) is the subpixel IMAGE coordinates of the Top-Left corner of the patch
//        c) d is a "DC offset" quantity (see similar method in the calibrator's CalibrCornerPatch.cpp file).
//        d) G is simply the image gradient at (px0 + deltaxi, py0 + deltayi)
//
// NOTE!!! The 3x3 matrix JtJ is caluclated (and inverted) SEPARATELY in function "prepSubPixGNStep" 
// 	   which is the former "MakeSubPixTemplate" (probably the most MISLEADING function name 
// 	   EVER conceived ).

// So, IterateSubpix simply does a single G-N step in adjusting the TL corner of the patch
// Furthermore, it updates the respective estimate in the 0-th level.
double PatchFinder::IterateSubPix(KeyFrame::Ptr pKF)
{
  // Store the position of the feature at the current search level.
  // in orther words, "LevelNPose" simply returns the coordinates of an image point 
  // at the pyramidal level specified.
  // Note here, "mv2SubPixPos" is theposition of the patch in level 0.
  cv::Vec2f v2Center = LevelNPos(mv2SubPixPos, mnSearchLevel);
  cv::Mat_<uchar> im = pKF->aLevels[mnSearchLevel].im;   // NOTE: OpenCV makes this a reference assignment
  
  // NOTE!!! We will be working on a patch size of 8 pixels (default)
  //         on the currenet level image (so this patch size has nothing 
  //         to do with the putative patchin level 0).	     
  if (!CvUtils::in_image_with_border(std::round(v2Center[1]), // row
				     std::round(v2Center[0]), // col 
				     im, 
				     mnPatchSize / 2 + 1,     // width
				     mnPatchSize / 2 + 1 )    // height  
     )
   return -1.0;       // Negative return value indicates off edge of image 
  
  
  // NOTE: 
  //       a) v2Center : Is the position of the point in the current search level (computed above using "LevelNPos()".
  //       b) mirCenter: IS the CENTER of the Patch, i.e. (mnPatchSize /2 , mnPatchSize/2)!
  //
  // So, v2Base ios essentially subpixel coordinates for the UPPER-LEFT corner of the patch !!!!
  cv::Vec2f v2Base(v2Center[0] - mirCenter.x, 
		   v2Center[1] - mirCenter.y
		  );
  
  // information vector as J'*error accumulator.
  cv::Vec<float, 3> v3Accum(0, 0, 0);
  
  
  // Each template pixel will be compared to an (bilinearly) interpolated target pixel
  // The target value is made using bilinear interpolation as the weighted sum
  // of four target image pixels. Calculate mixing fractions:
  //
  // Since we are only interested in translation here, the same differences should apply for the pixels of the entire patch.
  // Thus, they are calculated only once
  
  cv::Point2i floorBase (std::floor(v2Base[0]), 
		         std::floor(v2Base[1]) );
  
  double dX = v2Base[0]-floorBase.x; // Distances from pixel center of TL pixel
  double dY = v2Base[1]-floorBase.y;
  // interpolation coefficients...
  float fMixTL = (1.0 - dX) * (1.0 - dY);
  float fMixTR = (dX)       * (1.0 - dY);
  float fMixBL = (1.0 - dX) * (dY);
  float fMixBR = (dX)       * (dY);
  
  cv::Point2i i2Base ( (int)v2Base[0],
		       (int)v2Base[1]
		     );
  
  // Loop over template image
  //unsigned long nRowOffset = kf.aLevels[mnSearchLevel].im.ptr<uchar>(1,0) - kf.aLevels[mnSearchLevel].im.ptr<uchar>(0,0);
  //unsigned long int nRowOffset = (unsigned long int)kf.aLevels[mnSearchLevel].im.step; 
  int c, r;
  uchar *pImPatchRow, *pImPatchRow1,       // level patch current and next row pointers for the in the Image 
	*pTempImRow; 			    // template image row pointer
  cv::Vec<float, 3> *pGradRow;		    // gradient image row pointer
  // ok, loop
  for(r = 1; r < mnPatchSize - 1; r++) {
    
      //pTopLeftPixel = &im[::ir(v2Base) + ImageRef(1,ir.y)]; // n.b. the x=1 offset, as with y
      
    pImPatchRow = im.ptr<uchar>(i2Base.y + r, i2Base.x );
    pImPatchRow1 = im.ptr<uchar>(i2Base.y + r + 1, i2Base.x );
      
    pGradRow = mimJacs.ptr<cv::Vec<float, 3> >(r - 1);
      
    pTempImRow = mimTemplate.ptr<uchar>(r);
      
    for(c = 1; c < mnPatchSize - 1; c++) {
	
      // Interpolate the pixel as a weighted average: fmixTL * p(r, c)   +   fmixTR * p(r, c+1) +
      //                                             fmixLL * p(r+1, c) +   fmixLR * p(r + 1, c + 1)
      float fPixel =   fMixTL * pImPatchRow[c]      + 	  fMixTR * pImPatchRow[c + 1] + 
			fMixBL * pImPatchRow1[c]    + 	  fMixBR * pImPatchRow1[c + 1];
	  
	  
      // Update JT*d now...
      double error = fPixel - pTempImRow[c] + mdMeanDiff;
      v3Accum += error * pGradRow[c - 1];
	 
    }
  }
  
  // Compute the petrurbation in the TL corner coorindates
  // (not forgetting that we also obtain a perturbation in the DC offset as well)
  cv::Vec<float, 3> v3Update = mm3JtJInv * v3Accum; 
  // Now also update the position in the 0-th level,
  // of course accounting for the scale (hence the LevelScale multiplcation)
  mv2SubPixPos -=  LevelScale(mnSearchLevel) * cv::Vec<float, 2>( v3Update[0],
								  v3Update[1] 
								 );
  mdMeanDiff -= v3Update[2];
  
  // we need to check the norm of the perturbation...
  return v3Update[0] * v3Update[0] + v3Update[1] * v3Update[1];
}

/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////
// 
//
//              ZMSSDatpoint, which is SSE optimised, follows
//
// The top version is the SSE version for 8x8 patches. It is compiled
// only if CVD_HAVE_XMMINTRIN is true, also you need to give your 
// compiler the appropriate flags (e.g. -march=core2 -msse3 for g++.)
// The standard c++ version, which is about half as quick (not a disaster
// by any means) is below.
//
// The 8x8 SSE version looks long because it has been unrolled, 
// it just does the same thing eight times. Both versions are one-pass
// and need pre-calculated template sums and sum-squares.
//
/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////

#if USE_XMMINTRIN
// Horizontal sum of uint16s stored in an XMM register
inline int SumXMM_16(__m128i &target)
{
  unsigned short int sums_store[8];    
  _mm_storeu_si128((__m128i*)sums_store, target);
  return sums_store[0] + sums_store[1] + sums_store[2] + sums_store[3] +
    sums_store[4] + sums_store[5] + sums_store[6] + sums_store[7];
}
// Horizontal sum of uint32s stored in an XMM register
inline int SumXMM_32(__m128i &target)
{
  unsigned int sums_store[4];    
  _mm_storeu_si128((__m128i*)sums_store, target);
  return sums_store[0] + sums_store[1] + sums_store[2] + sums_store[3];
}
#endif



// Calculate the Zero-mean SSD of the coarse patch and a target imate at a specific 
// point.
int PatchFinder::ZMSSDAtPoint(const cv::Mat_<uchar> &im, const cv::Point2i &ir) {
  
  if (!CvUtils::in_image_with_border(ir.y, ir.x, im, mirCenter.x, mirCenter.y ) ) return mnMaxSSD + 1;
  
  cv::Point2i irImgBase(ir.x - mirCenter.x, 
			ir.y - mirCenter.y); // just the TL corner in the image
  uchar *imagepointer;
  uchar *templatepointer;
  
  int nImageSumSq = 0;
  int nImageSum = 0;
  int nCrossSum = 0;
  
#if USE_XMMINTRIN
  if(mnPatchSize == 8)
    {
      long unsigned int imagepointerincrement;

      __m128i xImageAsEightBytes;
      __m128i xImageAsWords;
      __m128i xTemplateAsEightBytes;
      __m128i xTemplateAsWords;
      __m128i xZero;
      __m128i xImageSums; // These sums are 8xuint16
      __m128i xImageSqSums; // These sums are 4xint32
      __m128i xCrossSums;   // These sums are 4xint32
      __m128i xProduct;

      
      xImageSums = _mm_setzero_si128();
      xImageSqSums = _mm_setzero_si128();
      xCrossSums = _mm_setzero_si128();
      xZero = _mm_setzero_si128();
      
      //imagepointer = &im[irImgBase + ImageRef(0,0)];
      imagepointer = im.ptr<uchar>(irImgBase.y , irImgBase.x);
      //templatepointer = &mimTemplate[ImageRef(0,0)];
      templatepointer = mimTemplate.ptr<uchar>(0, 0);
      //imagepointerincrement = &im[irImgBase + ImageRef(0,1)] - imagepointer;
      imagepointerincrement = im.ptr<uchar>(irImgBase.y + 1, irImgBase.x)  - imagepointer;
      
      xImageAsEightBytes=_mm_loadl_epi64((__m128i*) imagepointer);
      imagepointer += imagepointerincrement;
      xImageAsWords = _mm_unpacklo_epi8(xImageAsEightBytes,xZero);
      xImageSums = _mm_adds_epu16(xImageAsWords,xImageSums);
      xProduct = _mm_madd_epi16(xImageAsWords, xImageAsWords);
      xImageSqSums = _mm_add_epi32(xProduct, xImageSqSums);
      xTemplateAsEightBytes=_mm_load_si128((__m128i*) templatepointer);
      templatepointer += 16;
      xTemplateAsWords = _mm_unpacklo_epi8(xTemplateAsEightBytes,xZero);
      xProduct = _mm_madd_epi16(xImageAsWords, xTemplateAsWords);
      xCrossSums = _mm_add_epi32(xProduct, xCrossSums);
      xImageAsEightBytes=_mm_loadl_epi64((__m128i*) imagepointer);
      imagepointer += imagepointerincrement;
      xImageAsWords = _mm_unpacklo_epi8(xImageAsEightBytes,xZero);
      xImageSums = _mm_adds_epu16(xImageAsWords,xImageSums);
      xProduct = _mm_madd_epi16(xImageAsWords, xImageAsWords);
      xImageSqSums = _mm_add_epi32(xProduct, xImageSqSums);
      xTemplateAsWords = _mm_unpackhi_epi8(xTemplateAsEightBytes,xZero);
      xProduct = _mm_madd_epi16(xImageAsWords, xTemplateAsWords);
      xCrossSums = _mm_add_epi32(xProduct, xCrossSums);

      xImageAsEightBytes=_mm_loadl_epi64((__m128i*) imagepointer);
      imagepointer += imagepointerincrement;
      xImageAsWords = _mm_unpacklo_epi8(xImageAsEightBytes,xZero);
      xImageSums = _mm_adds_epu16(xImageAsWords,xImageSums);
      xProduct = _mm_madd_epi16(xImageAsWords, xImageAsWords);
      xImageSqSums = _mm_add_epi32(xProduct, xImageSqSums);
      xTemplateAsEightBytes=_mm_load_si128((__m128i*) templatepointer);
      templatepointer += 16;
      xTemplateAsWords = _mm_unpacklo_epi8(xTemplateAsEightBytes,xZero);
      xProduct = _mm_madd_epi16(xImageAsWords, xTemplateAsWords);
      xCrossSums = _mm_add_epi32(xProduct, xCrossSums);
      xImageAsEightBytes=_mm_loadl_epi64((__m128i*) imagepointer);
      imagepointer += imagepointerincrement;
      xImageAsWords = _mm_unpacklo_epi8(xImageAsEightBytes,xZero);
      xImageSums = _mm_adds_epu16(xImageAsWords,xImageSums);
      xProduct = _mm_madd_epi16(xImageAsWords, xImageAsWords);
      xImageSqSums = _mm_add_epi32(xProduct, xImageSqSums);
      xTemplateAsWords = _mm_unpackhi_epi8(xTemplateAsEightBytes,xZero);
      xProduct = _mm_madd_epi16(xImageAsWords, xTemplateAsWords);
      xCrossSums = _mm_add_epi32(xProduct, xCrossSums);

      xImageAsEightBytes=_mm_loadl_epi64((__m128i*) imagepointer);
      imagepointer += imagepointerincrement;
      xImageAsWords = _mm_unpacklo_epi8(xImageAsEightBytes,xZero);
      xImageSums = _mm_adds_epu16(xImageAsWords,xImageSums);
      xProduct = _mm_madd_epi16(xImageAsWords, xImageAsWords);
      xImageSqSums = _mm_add_epi32(xProduct, xImageSqSums);
      xTemplateAsEightBytes=_mm_load_si128((__m128i*) templatepointer);
      templatepointer += 16;
      xTemplateAsWords = _mm_unpacklo_epi8(xTemplateAsEightBytes,xZero);
      xProduct = _mm_madd_epi16(xImageAsWords, xTemplateAsWords);
      xCrossSums = _mm_add_epi32(xProduct, xCrossSums);
      xImageAsEightBytes=_mm_loadl_epi64((__m128i*) imagepointer);
      imagepointer += imagepointerincrement;
      xImageAsWords = _mm_unpacklo_epi8(xImageAsEightBytes,xZero);
      xImageSums = _mm_adds_epu16(xImageAsWords,xImageSums);
      xProduct = _mm_madd_epi16(xImageAsWords, xImageAsWords);
      xImageSqSums = _mm_add_epi32(xProduct, xImageSqSums);
      xTemplateAsWords = _mm_unpackhi_epi8(xTemplateAsEightBytes,xZero);
      xProduct = _mm_madd_epi16(xImageAsWords, xTemplateAsWords);
      xCrossSums = _mm_add_epi32(xProduct, xCrossSums);

      xImageAsEightBytes=_mm_loadl_epi64((__m128i*) imagepointer);
      imagepointer += imagepointerincrement;
      xImageAsWords = _mm_unpacklo_epi8(xImageAsEightBytes,xZero);
      xImageSums = _mm_adds_epu16(xImageAsWords,xImageSums);
      xProduct = _mm_madd_epi16(xImageAsWords, xImageAsWords);
      xImageSqSums = _mm_add_epi32(xProduct, xImageSqSums);
      xTemplateAsEightBytes=_mm_load_si128((__m128i*) templatepointer);
      templatepointer += 16;
      xTemplateAsWords = _mm_unpacklo_epi8(xTemplateAsEightBytes,xZero);
      xProduct = _mm_madd_epi16(xImageAsWords, xTemplateAsWords);
      xCrossSums = _mm_add_epi32(xProduct, xCrossSums);
      xImageAsEightBytes=_mm_loadl_epi64((__m128i*) imagepointer);
      xImageAsWords = _mm_unpacklo_epi8(xImageAsEightBytes,xZero);
      xImageSums = _mm_adds_epu16(xImageAsWords,xImageSums);
      xProduct = _mm_madd_epi16(xImageAsWords, xImageAsWords);
      xImageSqSums = _mm_add_epi32(xProduct, xImageSqSums);
      xTemplateAsWords = _mm_unpackhi_epi8(xTemplateAsEightBytes,xZero);
      xProduct = _mm_madd_epi16(xImageAsWords, xTemplateAsWords);
      xCrossSums = _mm_add_epi32(xProduct, xCrossSums);

      nImageSum = SumXMM_16(xImageSums);
      nCrossSum = SumXMM_32(xCrossSums);
      nImageSumSq = SumXMM_32(xImageSqSums);
    }
  else
#endif  
    {    
      for(int nRow = 0; nRow < mnPatchSize; nRow++) {
	
	  //imagepointer = &im[irImgBase + ImageRef(0,nRow)];
	  imagepointer = im.data + (irImgBase.y + nRow) * im.step +  irImgBase.x /* * elemSize() */;
	  //templatepointer = &mimTemplate[ImageRef(0,nRow)];
	  templatepointer = mimTemplate.data + nRow * mimTemplate.step;
	  for(int nCol = 0; nCol < mnPatchSize; nCol++) {
	    
	      int n = imagepointer[nCol];
	      nImageSum += n;
	      nImageSumSq += n*n;
	      nCrossSum += n * templatepointer[nCol];
	  }
      }
    }
  
  int SA = mnTemplateSum;
  int SB = nImageSum;
  
  int N = mnPatchSize * mnPatchSize;
  return ( (2*SA*SB - SA*SA - SB*SB)/N + nImageSumSq + mnTemplateSumSq - 2*nCrossSum );
}






