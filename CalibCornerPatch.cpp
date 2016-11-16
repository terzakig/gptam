// George Terzakis 2016
//
// University of Portsmouth
//
// Code based on PTAM by Klein and Murray (Copyright 2008 Isis Innovation Limited)

#include "CalibCornerPatch.h"
#include "OpenGL.h"

#include "GCVD/image_interpolate.h"
//#include "GCVD/Addedutils.h"
#include <GL/gl.h>
#include <GL/glut.h>


using namespace std;


cv::Mat_<float> CalibCornerPatch::mimSharedSourceTemplate;

// This is a constructor for a (calibration) corner object.
/// @nSideSize is the side size of the patch (by default, 20 pixels)
CalibCornerPatch::CalibCornerPatch(int nSideSize)
{
  
  mimTemplate.create(nSideSize, nSideSize);
  mimGradients.create(nSideSize, nSideSize);
  mimAngleJacs.create(nSideSize, nSideSize);
  
  MakeSharedTemplate();
  
}



// This function transforms the template image (aka mimSharedSourceTemplate)
// according to the currrent parameter values (position and orientation mainly ) stored in "Params"
void CalibCornerPatch::MakeTemplateWithCurrentParams()
{
  int r, c;
  double dBlurSigma = 2.0;
  //int nExtraPixels = (int) (dBlurSigma * 6.0) + 2; // Rosten's extra margin to account for blurring. I have to assume he is giving a full kernel margin
  // I am giving a full kernel extra margin.
  int nExtraPixels = (int)ceil(dBlurSigma * 3.0);
  
  //cv::Mat_<float> imToBlur(mimTemplate.rows + nExtraPixels, mimTemplate.cols + nExtraPixels); // may not be necessary to allocate here...
  cv::Mat_<float> imToBlur; // Not necessary to allocate the image here...
  int im2BlurRows = mimTemplate.rows + nExtraPixels;
  int im2BlurCols = mimTemplate.cols + nExtraPixels;
  
  cv::Mat_<float> imTwiceToBlur(im2BlurRows * 2, im2BlurCols * 2);
  
  // Make actual template:
  int nOffsetCol, nOffsetRow;
  {
    // obtaining the transformation that rotates and "distorts" the corner.
    cv::Mat_<float> m2Warp = mParams.m2Warp();
    // now, transform the purative template image into how it should look according to m2Warp and store in "imTwiceToBlue"...
    CvUtils::transform<>(mimSharedSourceTemplate, 
			 imTwiceToBlur,
			 CvUtils::M2Inverse(m2Warp),
			 0.5 * cv::Vec2f(mimSharedSourceTemplate.cols - 1, mimSharedSourceTemplate.rows - 1),
			 0.5 * cv::Vec2f(imTwiceToBlur.cols - 1, imTwiceToBlur.rows - 1)
			 );
    
    //cv::resize(imTwiceToBlur, imToBlur, cv::Size2i(mimTemplate.cols + nExtraPixels, mimTemplate.rows + nExtraPixels) );
    CvUtils::halfSample(imTwiceToBlur, imToBlur);
    // applying Gaussian blurring 
    cv::Mat temp(imToBlur.rows, imToBlur.cols, CV_8UC1);
    // using the formula Rosten uses in his "convolveGaussian" function...
    int gkerSize = (int)ceil(dBlurSigma*3.0); // where 3.0 is the default "sigmas" parameter in libCVD
    gkerSize += gkerSize % 2 == 0 ? 1 : 0;
    cv::GaussianBlur(imToBlur, temp, cv::Size(gkerSize, gkerSize), dBlurSigma);
    temp.copyTo(imToBlur);
    // Ok, now half-sampled and smoothed image stored in "imToBlur"
    
    // Now obtaining the center of the blurred image (which, by the way is x2 mimTemplate)
    nOffsetCol = (imToBlur.cols - mimTemplate.cols) / 2;
    nOffsetRow = (imToBlur.rows - mimTemplate.rows) / 2;
   
    
    
    //copy(imToBlur, mimTemplate, mimTemplate.size(), ImageRef(nOffset, nOffset));
    // So now copying imToBlur from (nOffset, nOffset) onto mimTemplate 
    // which should give us the lower-right quadrant of imToBlur inside mimTemplate
    imToBlur( cv::Range(nOffsetRow, nOffsetRow + mimTemplate.rows),
	      cv::Range(nOffsetCol, nOffsetCol + mimTemplate.cols) ).copyTo(mimTemplate); 
		
    
  };
  
  // Make numerical angle jac images:
  for(int dof=0; dof<2; dof++) {
    
      // creating the warp matrix corresponding to "mParams" here 
      // this could have been a call tp mParams.m2Warp(),
     // but we need to perturb by 0.01 the angle indexed by "dof" each time, so we are doing it inline-fashion
      cv::Mat_<float> m2Warp(2, 2); 
      for(int i=0; i<2; i++)
	{
	  double dAngle = mParams.v2Angles[i];
	  // NOTE HERE: Purterbing the angle of the dof-th axis
	  if(dof == i) dAngle += 0.01;
	  
	  m2Warp(0, i) = cos(dAngle); 
	  m2Warp(1, i) = sin(dAngle);
	};
       
      
      CvUtils::transform<>(mimSharedSourceTemplate, 
			 imTwiceToBlur,
			 CvUtils::M2Inverse(m2Warp),
			 0.5 * cv::Vec2f(mimSharedSourceTemplate.cols - 1, mimSharedSourceTemplate.rows - 1),
			 0.5 * cv::Vec2f(imTwiceToBlur.cols - 1, imTwiceToBlur.rows - 1)
			 );
    
      
      //cv::resize(imTwiceToBlur, imToBlur, imToBlur.size() );
      CvUtils::halfSample(imTwiceToBlur, imToBlur);
      // applying Gaussian blurring 
      cv::Mat temp;
      // again, using Rostens' formula (or something close to it) for the kernel size...
      int gkerSize = (int)ceil(dBlurSigma*3.0); // where 3.0 is the default "sigmas" parameter in libCVD
      gkerSize += gkerSize % 2 == 0 ? 1 : 0;
      cv::GaussianBlur(imToBlur, temp, cv::Size(gkerSize, gkerSize), dBlurSigma);
      temp.copyTo(imToBlur);
     
      
      
      // now filling in the Jacobian of imToBlur in terms of the two angles
      for (r = 0; r < mimTemplate.rows; r++) {
	cv::Vec<float, 2>* aRowPtr = mimAngleJacs.ptr<cv::Vec<float, 2>>(r);
	float* bRowPtr =  imToBlur.ptr<float>(r+ nOffsetRow);
	float* tRowPtr =  mimTemplate.ptr<float>(r);
	for (c = 0; c < mimTemplate.cols; c++)
	  aRowPtr[c][dof] = ( bRowPtr[c + nOffsetCol] - tRowPtr[c] ) / 0.01;
      } 
    } // end the dof-for
  
  // Make the image of image gradients here too (while we have the bigger template to work from)
  for (r = 0; r < mimGradients.rows; r++) {
    float* bRowPtr = imToBlur.ptr<float>(r + nOffsetRow);
    float* bRowPtr1 = imToBlur.ptr<float>(r + nOffsetRow + 1);
    float* bRowPtr_1 = imToBlur.ptr<float>(r + nOffsetRow - 1);
    cv::Vec<float, 2>* gRowPtr = mimGradients.ptr<cv::Vec<float, 2>>(r);
    for (c = 0; c < mimGradients.cols; c++)
    {
      gRowPtr[c][0] = 0.5 * ( bRowPtr[c + nOffsetCol + 1] - bRowPtr[c + nOffsetCol- 1] );
      gRowPtr[c][1] = 0.5 * ( bRowPtr1[c + nOffsetCol] - bRowPtr_1[c + nOffsetCol] );
    }
  }
  
} // and this concludes the creation of a template 


// This function updates the current corner parameters (position, angles, gain and mean)
// and then it draws the posnts according to the new estimate
bool CalibCornerPatch::IterateOnImageWithDrawing(CalibCornerPatch::Params &params, cv::Mat_<uchar> &im)
{
  // Run a Gauss-Newton iteration in order to a (potentially) better parameter estimate (position, distortion(angles))
  // for this corner
  bool bReturn = IterateOnImage(params, im);
  
  // if G-N gave acceptable results. then draw the new corner position as a thick(5) red dot.
  if(!bReturn)
    {
      glPointSize(5);
      glColor3f(0,0,1);
      glBegin(GL_POINTS);
      glVertex2f(params.v2Pos[0], params.v2Pos[1]);
      glEnd();
    }
  return bReturn;
}


// This function actually does the Gauss-Newton iteration over corner parameters (position, angles, gain, mean)
bool CalibCornerPatch::IterateOnImage(CalibCornerPatch::Params &params, cv::Mat_<uchar> &im)
{
  
  
  mParams = params;
  double dLastUpdate = 0.0;
  // run G-N a few times (20 is really enough)...
  for(int i=0; i<20; i++) {
    
      // generate an "impression" of how the putative corner should appear in the image
      // based on current position and distortion (angles) parameters 
      MakeTemplateWithCurrentParams();
      
      // Build and SOLVE the LS system for this G-N step
      // Unfortunately, function is called "iterate"; but this "iteration" is only over all pixels of the template
      dLastUpdate = Iterate(im); // store the norm of the last position update in "dLastUpdate"
      
      // if the norm is negative, something went horribly wrong... This actually not possible...
      if(dLastUpdate < 0) return false;
      
      // this means convergence (whether successful or not, we will have to see...). Break G-N iteration.
      if(dLastUpdate < 0.00001) break;
      
      
    }

   // cout << " The error in the las update is (must be below 0.001) : " << dLastUpdate<<endl;
  // cout <<"My gain was (must be above 20): "<<mParams.dGain<<endl;
  //cout <<"My LastError was (must be below 25): "<<mdLastError<<endl;
    
  // The iteration timed-out. Return false.
  if(dLastUpdate > 0.001) return false;
  
    
  // We need the two axes to have a relative reasonable angle between them (abovw 30 degrees), 
  // otherwise perspective distortion is going to be a bit two much for our purposes...
  if(fabs(sin(mParams.v2Angles[0] - mParams.v2Angles[1])) < sin(M_PI / 6.0)) return false;
    
  // Check the gain value. Must be big (> 20). This must be an empirical figure... I would ask Klein or Murray...
  if(fabs(mParams.dGain) < 20.0) return false;
  
  
  // Check average absolute error. Must be relatively small (<25). I would argue that it must be 1, but it seems that fitting a linear model works better...
  // Again, this must be an empirical figure. As klein or Murray...
  if(mdLastError > 25.0)  return false;
 
    
  // Hurrah! New parameters accepted! Store in mParams and leave!
  params = mParams;
  
  return true;
}


// This function does the hard work in terms of solving the actual position and orientation of the corner patch.
// At this stage, "mimTemplate" should represent hw the corner should appear based on the current position and orientation/distortion 
// parameters stored in "mParams"
// CAUTION-CAUTION!!! This function (contrary to its name) DOES NOT iterate in a Gauss-Newton fashion. "Iterate" referes to the template pixels....
// CAUTION-CAUTION!!! In other words, this a single step update in the Gauss-Newton iteration
float CalibCornerPatch::Iterate(cv::Mat_<uchar> &im)
{ 
  // Finding the Top Left (TL) corner of the patch in the image. Using doubles to represent locations just to boost accuracy a bit...
  cv::Vec2d v2TL = cv::Vec2d(mParams.v2Pos[0] - (mimTemplate.cols - 1) / 2.0, mParams.v2Pos[1] - (mimTemplate.rows - 1) / 2.0 );
  if(!(v2TL[0] >= 0.0 && v2TL[1] >= 0.0)) return -1.0;
  
  // And the Bottom-Right (BR) corner of the patch in the image
  cv::Vec2d v2BR = v2TL + cv::Vec2d(mimTemplate.cols - 1, mimTemplate.rows - 1);
  if( ( v2BR[0] >= (im.cols - 1.0) )  || ( v2BR[1] >= (im.rows - 1.0) ) ) return -1.0;
  
  // create an image interpolation object
  CvUtils::image_interpolate< CvUtils::Interpolate::Bilinear, uchar> imInterp(im);
  
  // normal equation matrices. Prepare for Least Squares!
  cv::Matx<double, 6, 6> m6JTJ = cv::Matx<double, 6, 6>::zeros(); // this is the J'*J (can be though of as information matrix)
  cv::Vec<double, 6> v6JTD ;       // J' * error (think information vector)
  
  
  double dSum = 0.0;
  // looping over each pixel in the current template (the putative corner transformed based on existing parameters)
  int r, c;
  for (r = 0; r < mimTemplate.rows; r++) {
    float* tRowPtr = mimTemplate.ptr<float>(r);
    cv::Vec<float, 2>* gRowPtr = mimGradients.ptr<cv::Vec<float, 2>>(r);
    cv::Vec<float, 2>* aRowPtr = mimAngleJacs.ptr<cv::Vec<float, 2>>(r);
    for (c = 0; c < mimTemplate.cols; c++)
    {
      // Switching to centralized coordinates. Now v2Pos will be the centralized coordinate in the template patch
      cv::Vec2d v2Pos_Template( c - (mimTemplate.cols - 1) / 2.0 , r - (mimTemplate.rows - 1) / 2.0 );
      // Accordingly, v2Pos_Image will now be the position in the actual image coordinates (of course, NOT centralized)
      cv::Vec2d v2Pos_Image = mParams.v2Pos + v2Pos_Template;
      
      // Ok, the following assignment needs a lot of explanations... Anyway I owed a description on "dGain" and "dMean" from earlier...
      // Klein is fitting a linear model A*x + B where A is some "gain" (AKA dGain) and B is some "DC offset" (AKA "dMean")
      // from the template image to the actual image. 
      // This adds flexibility to the process (or at least it appears so...)
      // now finding the difeerence between the (interpolated) brightness at v2Pos_Image and the actual template stores via the linear model
      double dDiff = imInterp[v2Pos_Image][0] - (mParams.dGain * tRowPtr[c] + mParams.dMean);
      
      // just a cumulative error measure over all pixels in the template
      dSum += fabs(dDiff);
      
      // Jac for center pos: Minus sign because +pos equates to sliding template
      // Ok, now notice that this is the Jacobian in terms of PARAMETERS  (hence the use of "mimGradients" and "mimAngleJacs" due to the chain rule)
      cv::Vec<double, 6> v6Jac;
      // First two derivatives are in terms of position
      v6Jac[0] = -1.0 * mParams.dGain * gRowPtr[c][0]; 
      v6Jac[1] = -1.0 * mParams.dGain * gRowPtr[c][1]; 
      // The next two derivatives are in terms of the angles
      v6Jac[2] = aRowPtr[c][0] * mParams.dGain; 
      v6Jac[3] = aRowPtr[c][1] * mParams.dGain,
      // That's the derivative of the constant (AKA dMean) I believe...
      v6Jac[4] = 1.0; 
      // And finally, the derivative of the coefficient (AKA dGain)
      v6Jac[5] = tRowPtr[c];
			    
      
      // populating the information matrix additively with the jacobian vector tensor product
      m6JTJ +=  v6Jac * v6Jac.t();
      // populating the information vector additively with the error-times-Jacobian vector
      v6JTD += dDiff* v6Jac;
    }
  }
   // Ok, solving now! I am using SVD here, but anything should work
   cv::Vec<double, 6> v6Update; // the 6x1 LS solution
   if (cv::determinant(m6JTJ) == 0) return 9999;
   cv::solve(m6JTJ, v6JTD, v6Update, cv::DECOMP_CHOLESKY);
   
   // 1. Updating position in "mParams"
   mParams.v2Pos += cv::Vec2f(v6Update[0], v6Update[1]);
   
   // 2. Updating anmgles in "mParams"
   mParams.v2Angles += cv::Vec2f(v6Update[2], v6Update[3] );
   // 3. Updating the "dMean" constant
   mParams.dMean += v6Update[4];
   // 4. Updating the "dGain" constant
   mParams.dGain += v6Update[5];
   // finding absolute error per pixel in the template
   mdLastError = dSum / mimTemplate.size().area();
  
   // return norm of position update
  return sqrt( v6Update[0] * v6Update[0] + v6Update[1] * v6Update[1] ); 
  
}




// The following function constructs a 100 x100 PUTATIVE corner
// and stores it in mimSharedSourceTemplate ( a float image with elements taking values 1.0 or -1.0 )
void CalibCornerPatch::MakeSharedTemplate()
{
  const int nSideSize = 100;
  const int nHalf = nSideSize / 2;
  
  mimSharedSourceTemplate.create(nSideSize, nSideSize);
  
  int r, c;
  for (r = 0; r < mimSharedSourceTemplate.rows; r++)
    for (c = 0; c < mimSharedSourceTemplate.cols; c++)
    {
      float fX = (c < nHalf) ? 1.0 : -1.0;
      float fY = (r < nHalf) ? 1.0 : -1.0;
      mimSharedSourceTemplate(r, c) = fX * fY;
    }
  
}

CalibCornerPatch::Params::Params()
{
  v2Angles[0] = 0.0;
  v2Angles[1] = M_PI / 2.0;
  dMean = 0.0;
  dGain = 1.0;
}



// This function generates an 2x2 transformation
// that rotates the horizontal and vertical axis
// to a (potentially) skewed aligniment according to vAngles[0] (horizontal) and vAngles[1] (Vertical).
cv::Mat_<float> CalibCornerPatch::Params::m2Warp() //2x2
{
  cv::Mat_<float> m2Warp(2, 2);
  for(int i=0; i<2; i++)
    {
      m2Warp(0, i) = cos(v2Angles[i]);
      m2Warp(1, i) = sin(v2Angles[i]);
    };
  return m2Warp;
}










