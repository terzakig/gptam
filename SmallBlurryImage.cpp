// George Terzakis 2016
//
// University of Portsmouth
//
// Code based on PTAM by Klein and Murray (Copyright 2008 Isis Innovation Limited)


#include "SmallBlurryImage.h"

#include "GCVD/GraphSLAM.h"
#include "GCVD/Addedutils.h"

using namespace RigidTransforms;
using namespace Optimization;
using namespace std;

cv::Size2i SmallBlurryImage::mirSize(-1,-1);

SmallBlurryImage::SmallBlurryImage(KeyFrame &kf, double dBlur) {
  
  mbMadeGradients = false; // no derivatives calculated yet
  // create the small, blurry image from the KF
  MakeFromKF(kf, dBlur);
}

SmallBlurryImage::SmallBlurryImage()
{
  mbMadeGradients = false;
}

// Make a SmallBlurryImage from a KeyFrame This fills in the mimSmall
// // image (Which is just a small un-blurred version of the KF) and
// mimTemplate (which is a floating-point, zero-mean blurred version
// of the above)
void SmallBlurryImage::MakeFromKF(KeyFrame &kf, double dBlur) {
  
  // size the SBI to half the size of the top image in the KF pyramid
  if (mirSize.width == -1)
    mirSize = cv::Size2i(kf.aLevels[3].im.cols / 2, kf.aLevels[3].im.rows / 2);
  
  // Image gradients will be computed for the need of the Benhimane-Malis LS formulation.
  // But we may skip this Optimization, so let's just save us the trouble for the time being.
  // So lower this flag...
  mbMadeGradients = false; 

  // allocate what needs to be allocated 
  // (in thiscase, only the template)
  mimTemplate.create(mirSize);

  
  // halfsampling the top level into the SBI now 
  // (Note that mirSize is now half the size of the top image)
  //DEPRECATED: cv::resize(kf.aLevels[3].im, mimSmall, mirSize);
  // Simple averaging does a lot better than openCV's resizing or pyrdown function!!!!
  CvUtils::halfSample(kf.aLevels[3].im, mimSmall);
  // get the mean of mimSmall:
  float fMean = cv::mean(mimSmall)[0];
  //float fMean = CvUtils::mavg(mimSmall);
 
  
  // subtracting the mean from the template
  int r, c;
  float* tRowPtr; // mimTemplate row pointer
  uchar* sRowPtr; // mimSmall row pointer
  for (r = 0; r < mimTemplate.rows; r++) {
    
    tRowPtr = mimTemplate.ptr<float>(r);
    sRowPtr = mimSmall.ptr<uchar>(r);
    
    for (c = 0; c < mimTemplate.cols; c++)
      tRowPtr[c] = sRowPtr[c] - fMean;
  }
  //cv::subtract(mimSmall, fMean, mimTemplate);
   
 
  // applying Gaussian blurring 
  cv::Mat_<float> imTemp; // no need to allocate!
  // using the formula Rosten uses in the "convolveGaussian" libCVD function...
  int gkerSize = (int)ceil(dBlur*3.0); // where 3.0 is the default "sigmas" parameter in libCVD
  gkerSize += gkerSize % 2 == 0 ? 1 : 0;
  cv::GaussianBlur(mimTemplate, imTemp, cv::Size(gkerSize, gkerSize), dBlur);
  //copy back to mimTemplate
  imTemp.copyTo(mimTemplate);
 
  
}

// Make the jacobians (actually, no more than a gradient image)
// of the blurred template
void SmallBlurryImage::MakeGradients() {
  
  mimImageGradients.create(mirSize);
  // Fill-in the gradient image
  int r, c;
  float* tRowPtr00, *tRowPtr10, *tRowPtr_10; 	    // template row pointers at rows  r, r+1, r-1
  
  cv::Vec<float, 2>* gRowPtr; // gradient row pointer
  for (r = 0; r < mirSize.height; r++) {
    
    tRowPtr00 = mimTemplate.ptr<float>(r);
    tRowPtr10 = mimTemplate.ptr<float>(r + 1);
    tRowPtr_10 = mimTemplate.ptr<float>(r - 1);
    
    gRowPtr = mimImageGradients.ptr<cv::Vec<float,2 > >(r);
    
    for (c = 0; c < mirSize.width; c++) 
      // probably all vectors are already zero, but let's be on the safe side
      if ( (r == 0) || (c ==0) || (r == mirSize.height - 1)  || (c == mirSize.width - 1) ) 
	gRowPtr[c][0]  = gRowPtr[c][1] = 0;
      //otherwise simply compute and store the gradient
      else {
	gRowPtr[c][0] = tRowPtr00[c + 1] - tRowPtr00[c - 1];
	gRowPtr[c][1] = tRowPtr10[c]     - tRowPtr_10[c];
	// N.b. missing 0.5 factor in above - will be added later.
      }
  }
  
  mbMadeGradients = true;
}

// Calculate the zero-mean SSD between one image and the next.
// Since both are zero mean already, just calculate the SSD...
double SmallBlurryImage::ZMSSD(SmallBlurryImage &other) {
  
  double dSSD = 0.0;
  int r, c;
  float *tRowPtr, *oRowPtr;
  for (r = 0; r < mirSize.height; r++) {
    
    tRowPtr = mimTemplate.ptr<float>(r);
    oRowPtr = other.mimTemplate.ptr<float>(r);
    
    for (c = 0; c < mirSize.width; c++) {
      
      double dDiff = tRowPtr[c] - oRowPtr[c];
      dSSD += dDiff * dDiff;
    }
  }
  
  return dSSD;
}


// Find an SE2 which best aligns an SBI to a target
// Do this by ESM-tracking a la Benhimane & Malis
// George: This means formulating a cost function that between image intensities. 
// This is actually a standard pyramidal optical flow estimation method 
// (see the revised LK tracker paper or Bouguet's pyramidal LK tracker report)
//
// ***************************** Method Overview ***********************************
//
//              	Optical flow estimation with rotation
//               Although this method is attributed to Benhimane & Malis,
//              I would defer the reader to Bouguet or the "Handbook of mathematical models for Computer Vision"
//               by Paragios, Chen et al. for more comperehensive reading...
//
//                                     OR... You can read right below...
//
//
//                   We formulate a cost function comprising the following terms:
//
//                               f(di) - ( g(R*di + t) + d )
//
// where g is the current SBI and f is the "other" (nb target). di are the CENTRALIZED coordinates of the i-th pixel in the patch,
//       R is a 2D rotation, t is a 2D translation from the center of the patch and d (aka "dMeanOffset") is a DC offset unknown 
//       (could be omitted, but PTAM authors appear to be very fond of it; I guess they know something about it..).
// - Linearizing g at R0*di+t0 yields the linearized Gauss-Newton factor:
//
//                    f(di) - f(R0*di+t0) - d0 - [ G * [dR/da , I] , I] * ([t;a;d] - [t0;a0;d0] )
//
// where G is the spatial gradient of G, a is the angle of R. In practice, instead of G we use the average (G + F) / 2
//       of both image gradients. The rest is G-N optimization... Note that t0, a0 and d0 are the current estimates 
//       of rotation,translation and DC offset.
//
// Now notice that the error is:                 error = -f(di)       + g(R0*di+t) +    d,          
// 
// which, in the code, corresponds to:  	 error = -otRowPtr[c] + here_      +  dMeanOffset;  
//
//                                 the rest can be wokred-out based on the above.... Enjoy!
//
pair<SE2<>,double> SmallBlurryImage::IteratePosRelToTarget(SmallBlurryImage &other, int nIterations)
{
  //  A few explanations for the below choice of transformations : "se2CtoC" and "se2WfromC"
  // G.K is looking for a single 2D rigid transformation (ala Benhimane style) which will transform pixels to pixels. 
  // HOWEVER:
  // He decides to decompose this transformation using two transformations (SE2) ,
  // possibly because he is actually looking for the transformation of some central point 
  // translation (and rotation), hence the "se2CToC"...
  // So, se2fromWToC is simply the FIXED translation that takes the upper-left corner of the image center!
  // In other words, the LS cost function is formulates in CENTRALIZED coordinates (hence the respective subtractions in the
  // rotation derivatives.
  
  
  
  // The unknown translation that takes central point to central point ([I | 0] initially of course...)
  SE2<> se2CtoC;
  // And the transformation that links the upper-left corner with the central point. it is FIXED.   
  SE2<> se2WfromC;
  // It makes sense that se2FromC is,
  cv::Point2i irCenter(mirSize.width / 2, mirSize.height / 2);
  // It would make sense to initialize se2WToC  to the center of the SBI
  se2WfromC.get_translation() = cv::Vec2f(irCenter.x, irCenter.y);

  
  pair<SE2<>, double> result_pair;
  if(!other.mbMadeGradients)
  {
    cerr << "Target SBI gradients not made!" << endl;
    assert(other.mbMadeGradients);
  }

  double dMeanOffset = 0.0;
  cv::Vec<double, 4> v4Accum; // accumulator

  cv::Mat_<float> imWarped(mirSize);

  double dFinalScore = 0.0;
  for(int it = 0; it<nIterations; it++) {
    
    

    SE2<> se2XForm = se2WfromC * se2CtoC * se2WfromC.inverse();

    // Warp the current image template based on our estimate:
    cv::Vec2f v2Zero(0, 0);
    CvUtils::transform<>(mimTemplate, 
		         imWarped,
		         se2XForm.get_rotation().get_matrix(), 
		         se2XForm.get_translation(), 
			 v2Zero, 
		         -9e20f);
    
    
    
    
    // Note here that the 3 first unknowns represent the SE2, 
    // but the 4th unknown is is simply a "DC offset" error in brightness (aka "dMeanOffset" in the code).
    
    
    float* iwRowPtr00, *iwRowPtr10, *iwRowPtr_10; // r, r+1, r-1 row pointers to warped image
    float* otRowPtr;                               // r row pointer of other.mimTemplate
    cv::Vec<float, 2>* ogRowPtr;                   // r row of other.mimImageGradient
    
    // use a scaler...
    double s = 1.0;   
    
    dFinalScore = 0.0;
    v4Accum = cv::Vec<float, 4>(0, 0, 0, 0);
    // We cache the UPPER triangle of sum(JTJ) in a 1D array
    double UT[10] = {0, 0, 0, 0, 0, 0, 0, 0 ,0 , 0}; 
    // cache vector for the (4D) Jacobian 
    cv::Vec<float, 4> v4Jac;
    v4Jac[3] = 1.0; // always one...
    
    int r, c;
    for (r =1; r < mirSize.height; r++) {
      
        iwRowPtr00 = imWarped.ptr<float>(r);      // r  row of imWarped
	iwRowPtr10 = imWarped.ptr<float>(r + 1);  // r+1 row of imWarped
	iwRowPtr_10 = imWarped.ptr<float>(r - 1);     // r-1 row of imWarped
      
	otRowPtr = other.mimTemplate.ptr<float>(r);                         // r row of other.mimTemplate
	ogRowPtr = other.mimImageGradients.ptr<cv::Vec<float, 2> >(r);      // r row of other.mimImageGradients
      
      for (c = 1; c < mirSize.width; c++) {
	//we skip the boundary pixels for obvious reasons....
	if ( (r == 0) || (c == 0) || (r == mirSize.height-1) || (c == mirSize.width - 1) ) continue;
	// now get the values of the left, right, up, down pixels in the warped image
	// we need warped coordinates in order to obtain the warped image gradient
	float left_,right_,up_,down_,here_;
	
	left_ = iwRowPtr00[c - 1];
	right_ = iwRowPtr00[c + 1];
	up_ = iwRowPtr_10[c];
	down_ = iwRowPtr10[c];
	here_ = iwRowPtr00[c];
	
	// just a test in case warped point falls out of the image; c.f. the -9e20f param to transform.
	if(left_ + right_ + up_ + down_ + here_ < -9999.9)  continue;

	//cout <<"DEBUG: l, r, u, d: "<<left_<<" , "<<right_<<" , "<<up_<<" , "<<down_<<endl;
	
	// now this is the gradient of TARGET image at the warped locations
	cv::Vec2f v2CurrentGrad( right_ - left_, // Missing 0.5 factor (read below about the notorious lost factor...) 
				 down_  - up_ );

	// Now this averaging business is just a RECOMMENDED hack as far as I know... Strictly speaking, ONLY the warped gradient 
	// should be taken into consideration.It turns-out that many use the average of both source 
	// and target image gradients at the warped coordinatesand have reported better results... 
	// Anyway, this whole business is dodgy if you ask me...
	cv::Vec2f v2GradAvg = 0.25 * (v2CurrentGrad  + ogRowPtr[c] ); // actualy, this is 0.5 * v2GradVg
								      // but in effect IT IS AN AVERAGE, so I renamed it...
	// Why 0.25? This is from missing 0.5 factors: One for
	// the fact we average two gradients, the other from
	// each gradient missing a 0.5 factor.

	// And now storing the gradient:
	// a) The first two components of the Jacobian are the derivatives in terms of translation.
	//    In effect, they simply turn-out to be the averaged directional gradients 
	//    of the source and target images at the warped locations. 
	// b) The derivatives of the warped image in terms of the angle of the SE2.
	//    They turn out to be: [Df/Dx , Df/Dy, 1] * [1 , -1 * y, 0; 1 * x, 1, 0; 0, 0, 1] * [x - cx; y - cy; 1]
	//    where Df/Dx and Df/Dy are the spatial gradients at the warped locations, phi is the rotation angle,
	//    and (cx, cy) is the center of the image (note that in the formula I used homogenous representations in 
	//    order to impress that the matrix in the middle is a 3x3 Lie infinitesimal rotation/generator associated with the z - axis).
	v4Jac[0] = v2GradAvg[0]; 
	v4Jac[1] = v2GradAvg[1]; 
	v4Jac[2] = -(r - irCenter.y) * v2GradAvg[0] + (c - irCenter.x) * v2GradAvg[1];
	//v4Jac[3] = 1.0; // already there...

	double error = here_ - otRowPtr[c] + dMeanOffset;
	dFinalScore += error * error;
	
	
	
	v4Accum += error * v4Jac / s;
	
	// speeding up access a bit...
	UT[0] += v4Jac[0] * v4Jac[0] / s; UT[1] += v4Jac[0] * v4Jac[1] / s; UT[2] += v4Jac[0] * v4Jac[2] / s; UT[3] += v4Jac[0] * v4Jac[3] / s; 
					  UT[4] += v4Jac[1] * v4Jac[1] / s; UT[5] += v4Jac[1] * v4Jac[2] / s; UT[6] += v4Jac[1] * v4Jac[3] / s; 
									    UT[7] += v4Jac[2] * v4Jac[2] / s; UT[8] += v4Jac[2] * v4Jac[3] / s;
													      UT[9] += v4Jac[3] * v4Jac[3] / s;
      }
    }
    
    // Filling the m4 accumulator matrix values now...
    // Jacobian Gram-matrix accumulator m4 (i.e., sum(JtJ) )
    cv::Matx<double, 4, 4> m4; 
    m4(0, 0) = UT[0];
    m4(1, 0) = m4(0, 1) = UT[1]; m4(1, 1) = UT[4]; m4(1, 2) = m4(2, 1) = UT[5]; m4(1, 3) = m4(3, 1) = UT[6];
    m4(2, 0) = m4(0, 2) = UT[2]; m4(2, 2) = UT[7]; m4(2, 3) = m4(3, 2) = UT[8];
    m4(3, 0) = m4(0, 3) = UT[3]; m4(3, 3) = UT[9];
   
    
    // m4 is PSD; solving with Cholesky preferrably...
    cv::Vec<double, 4> v4Update;

    cv::solve(m4, v4Accum, v4Update, cv::DECOMP_CHOLESKY);
    

    SE2<> se2Update;
    se2Update.get_translation() = cv::Vec2f(-v4Update[0], -v4Update[1]);
    se2Update.get_rotation() = SO2<>::exp(-v4Update[2]);
    se2CtoC = se2CtoC * se2Update;
    dMeanOffset -= v4Update[3];
  }

  result_pair.first = se2CtoC;
  result_pair.second = dFinalScore;
  
  //cout <<"DEBUG: The recovered SBI transformation : "<<se2CtoC<<endl;
  
  return result_pair;
}


// What is the 3D camera rotation (zero trans) SE3<> which causes an
// input image SO2 rotation?
SE3<> SmallBlurryImage::SE3fromSE2(SE2<> se2, ATANCamera camera) 
{
  // Do this by projecting two points, and then iterating the SE3<> (SO3
  // actually) until convergence. It might seem stupid doing this so
  // precisely when the whole SE2-finding is one big hack, but hey.

  // BACKUP ORIGINAL CAMERA SIZE!!! (dont forget to restore upon exit!)
  cv::Size2i orgCamSize = camera.GetImageSize();
  
  // Set the camera size to the size of the SBI 
  // in order to use it for projection and derivative computations...
  camera.SetImageSize(mirSize);

  // Our two warped points in pixels: a) center -(5, 0) and, b) center + (5, 0)
  cv::Vec<float, 2> av2Turned[2];   
  cv::Matx<float, 2, 2> m2R = se2.get_rotation().get_matrix();
  cv::Vec<float, 2> v2t = se2.get_translation();
  
  av2Turned[0] = cv::Vec<float, 2>( mirSize.width  / 2  + m2R(0, 0) * 5 + v2t[0], 
				    mirSize.height / 2  + m2R(1, 0) * 5 + v2t[1]);
  
  av2Turned[1] = cv::Vec<float, 2>( mirSize.width  / 2 + m2R(0, 0) * (-5) + v2t[0], 
				    mirSize.height / 2 + m2R(1, 0) * (-5) + v2t[1]);
  
  
  // And the  normalized Euclidean coordinates of the ORIGINAL points
  // (hence the absense of the 2D transformation from the expressions below)
  cv::Vec3f av3OrigPoints[2];   
  av3OrigPoints[0] = CvUtils::backproject( camera.UnProject(mirSize.width / 2 + 5, mirSize.height / 2) );
  av3OrigPoints[1] = CvUtils::backproject( camera.UnProject(mirSize.width / 2 - 5, mirSize.height / 2) );
  
 
  SO3<> so3; // start with the identity
  // Just 3 iterations... 
  for(int it = 0; it<3; it++) {
    
    // simply inline least squares here. No need to use WLS
    cv::Matx<float, 3, 3> m3Omega = 10.0 * cv::Matx<float, 3, 3>::eye(); // information matrix. initialize with 10*I3
    cv::Vec<float, 3> v3ksi(0, 0, 0); // information vector
    for(int i=0; i<2; i++) {
      
      // Project into the image to find error
      cv::Vec3f v3Cam = so3 * av3OrigPoints[i];
      cv::Vec2f v2Pixels = camera.Project( CvUtils::pproject(v3Cam) );
      cv::Vec2f v2Error = av2Turned[i] - v2Pixels;

      // NOTE!!!! Camera derivatives are cv::Mat objects. Ridiculous, but will have to do for the time being....
      cv::Matx<float, 2, 2> m2CamDerivs = camera.GetProjectionDerivs();
      cv::Matx<float, 2, 3> m23Jacobian;
      double dOneOverCameraZ = 1.0 / v3Cam[2];
      for(int m=0; m<3; m++) {
	
        const cv::Vec<float, 3> v3Motion = SO3<>::generator_field(m, v3Cam);
        cv::Vec<float, 2> v2CamFrameMotion( ( v3Motion[0] - v3Cam[0] * v3Motion[2] * dOneOverCameraZ ) * dOneOverCameraZ,
				            ( v3Motion[1] - v3Cam[1] * v3Motion[2] * dOneOverCameraZ ) * dOneOverCameraZ
					    );
        //m23Jacobian.T()[m] = m2CamDerivs * v2CamFrameMotion;
	m23Jacobian(0, m) = m2CamDerivs(0, 0) * v2CamFrameMotion[0] + m2CamDerivs(0, 1) * v2CamFrameMotion[1];
	m23Jacobian(1, m) = m2CamDerivs(1, 0) * v2CamFrameMotion[0] + m2CamDerivs(1, 1) * v2CamFrameMotion[1];
	
      }
      
      // 1. updating the information matrix (m3Omega)
      m3Omega += 1.0 * m23Jacobian.t() * m23Jacobian; 
      // 2. updating the information vector
      v3ksi += 1.0 * m23Jacobian.t() * v2Error;
     
    }

    // solve the linear system
    cv::Vec<float, 3> v3Res;
    cv::solve(m3Omega, v3ksi, v3Res, cv::DECOMP_CHOLESKY);
    so3 = SO3<>::exp(v3Res) * so3;
  }

  SE3<> se3Result;
  se3Result.get_rotation() = so3;
  
  // ********************** Restore camera size!!!! **********************
  //camera.SetImageSize(orgCamSize); // NOT?????
  
  // get outta here...
  return se3Result;
}










