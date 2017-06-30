// ************ George Terzakis 2016 *************
// ******* University of Portsmouth **************
//
// Current code is an adaptartion of original PTAM by Klein and Murrary (Copyright 2008 Isis Innovation Limited)

#include "Bundle.h"
#include "MEstimator.h"
#include "GCVD/Addedutils.h"
//#include "GCVD/GraphSLAM.h"

#include <fstream>
#include <iomanip>

#include "Persistence/instances.h"

//using namespace Optimization;
using namespace Persistence;
using namespace std;

#ifdef WIN32
inline bool isnan(double d) {return !(d==d);}
#endif

#define cout if(*mgvnBundleCout) cout

// Some inlines which replace standard matrix multiplications 
// with LL-triangle-only versions.
inline void BundleTriangle_UpdateM6U_LL(cv::Matx<float, 6, 6> &m6U,  // 6x6 matrix
					cv::Matx<float, 2, 6> &m26A)  // 2x6 matrix
{
  for(int r = 0; r < 6; r++)
    for(int c = 0; c <= r; c++)
      //m6U(r,c)+= m26A.T()(r,0)*m26A(0,c) + m26A.T()(r,1)*m26A(1,c);
      m6U(r,c) += m26A(0, r) * m26A(0,c) + m26A(1, r) * m26A(1,c);
}
inline void BundleTriangle_UpdateM3V_LL(cv::Matx<float, 3, 3> &m3V, // 3x3 matrix
					cv::Matx<float, 2, 3> &m23B) // 2x3 matrix
{
  for(int r = 0; r < 3; r++) 
    for(int c = 0; c <= r; c++) 
      //m3V(r,c)+= m23B.T()(r,0)*m23B(0,c) + m23B.T()(r,1)*m23B(1,c); 
      m3V(r,c) += m23B(0, r) * m23B(0,c) + m23B(1, r) * m23B(1, c); 
}

// Constructor copies MapMaker's camera parameters
Bundle::Bundle(const ATANCamera &TCam) : mCamera(TCam)
{
  mnCamsToUpdate = 0;
  mnStartRow = 0;
  PV3::Register(mgvnMaxIterations, "Bundle.MaxIterations", 20,  SILENT);
  PV3::Register(mgvdUpdateConvergenceLimit, "Bundle.UpdateSquaredConvergenceLimit", 1e-06, SILENT);
  PV3::Register(mgvnBundleCout, "Bundle.Cout", 0, SILENT);
}

// Add a camera to the system, 
// The function returns the bundle adjuster's ID of the camera
int Bundle::AddCamera(SE3<> se3CamFromWorld, bool bFixed)
{
  // to be the index of the newly inserted camera (if not fixed of course...)
  int n = mvCameras.size(); 
  BACamera c;
  c.bFixed = bFixed;
  c.se3CfW = se3CamFromWorld;
  if(!bFixed) {
    
      c.nStartRow = mnStartRow;
      mnStartRow += 6;
      mnCamsToUpdate++;
  }
  else
    c.nStartRow = -999999999; 
  
  mvCameras.push_back(c);
   
  return n;
}

// Add a map point to the system, return value is the bundle adjuster's ID for the point
int Bundle::AddPoint(cv::Vec<float, 3> v3Pos)
{
  // to be the index of the newly inserted point
  int n = mvPoints.size();
  BAPoint p;
  if( isnan( v3Pos.dot( v3Pos ) ) ) {
      cerr << " Trying to add point with NaN coordinates! " << v3Pos << endl;
      v3Pos = cv::Vec<float, 3>(0, 0, 0);
  }
  p.v3Pos = v3Pos;
  mvPoints.push_back(p);
  
  return n;
}

// Add a measurement of one point with one camera
void Bundle::AddMeasurement(int nCam, int nPoint, cv::Vec<float, 2> v2Pos, double dSigmaSquared) {
  
  assert(nCam < (int) mvCameras.size());
  assert(nPoint < (int) mvPoints.size());
  
  mvPoints[nPoint].nMeasurements++;
  mvPoints[nPoint].sCameras.insert(nCam);
  
  BAMeasurement m;
  m.pointIndex = nPoint;
  m.camIndex = nCam;
  m.v2Found = v2Pos;
  m.dSqrtInvNoise = sqrt(1.0 / dSigmaSquared);
  
  mMeasList.push_back(m);
}

// Zero temporary quantities stored in cameras and points
void Bundle::ClearAccumulators()
{
  for(size_t pointIndex = 0; pointIndex < mvPoints.size(); pointIndex++) {
    
      mvPoints[pointIndex].m3V = cv::Matx<float, 3, 3>::zeros();
      mvPoints[pointIndex].v3EpsilonB[0] = mvPoints[pointIndex].v3EpsilonB[1] = mvPoints[pointIndex].v3EpsilonB[2] = 0;
  }
  for(size_t camIndex = 0; camIndex < mvCameras.size(); camIndex++) {
    
      mvCameras[camIndex].m6U = cv::Matx<float, 6, 6>::zeros();
      
      mvCameras[camIndex].v6EpsilonA[0] = mvCameras[camIndex].v6EpsilonA[1] = mvCameras[camIndex].v6EpsilonA[2] = 0;
      mvCameras[camIndex].v6EpsilonA[3] = mvCameras[camIndex].v6EpsilonA[4] = mvCameras[camIndex].v6EpsilonA[5] = 0;
    
  }
}

// Perform bundle adjustment. The parameter points to a signal bool 
// which mapmaker will set to high if another keyframe is incoming
// and bundle adjustment needs to be aborted.
// Returns number of accepted iterations if all good, negative 
// value for big error.
int Bundle::Compute(bool *pbAbortSignal)
{
  mpbAbortSignal = pbAbortSignal;

  // Some speedup data structures
  GenerateMeasurementLUTs();
  GenerateOffDiagScripts();

  // Initially behave like gauss-newton
  mdLambda = 0.0001;
  mdLambdaFactor = 2.0;
  mbConverged = false;
  mbHitMaxIterations = false;
  mnCounter = 0;
  mnAccepted = 0;
  
  // What MEstimator are we using today?
  static pvar3<string> gvsMEstimator("BundleMEstimator", "Tukey", SILENT);
  
  while(!mbConverged  && !mbHitMaxIterations && !*pbAbortSignal) {
    
      bool bNoError;
      if(*gvsMEstimator == "Cauchy") bNoError = Do_LM_Step<Cauchy>(pbAbortSignal);  
      else 
	if(*gvsMEstimator == "Tukey") bNoError = Do_LM_Step<Tukey>(pbAbortSignal);
      else 
	if(*gvsMEstimator == "Huber") bNoError = Do_LM_Step<Huber>(pbAbortSignal);
      else {
	
	  cout << "Invalid BundleMEstimator selected !! " << endl;
	  cout << "Defaulting to Tukey." << endl;
	  *gvsMEstimator = "Tukey";
	  bNoError = Do_LM_Step<Tukey>(pbAbortSignal);
      }
      
      if(!bNoError) return -1;
  }
  
  if(mbHitMaxIterations) cout << "  Hit max iterations." << endl;
  
  cout << "Final Sigma Squared: " << mdSigmaSquared << " (= " << sqrt(mdSigmaSquared) / 4.685 << " pixels.)" << endl;
  
  return mnAccepted;
}

// Reproject a single measurement, find error
inline void Bundle::ProjectAndFindSquaredError(BAMeasurement &measurement)
{
  BACamera &cam = mvCameras[measurement.camIndex];
  
  BAPoint &point = mvPoints[measurement.pointIndex];
  
  // Project the point.
  measurement.v3Cam = cam.se3CfW * point.v3Pos;
  if(measurement.v3Cam[2] <= 0) {
    
      measurement.bBad = true;
      return;
  }
  measurement.bBad = false;
  
  cv::Vec2f v2EucPlane = CvUtils::pproject(measurement.v3Cam);
  cv::Vec2f v2Image   = mCamera.Project(v2EucPlane);
  
  measurement.m2CamDerivs = mCamera.GetProjectionDerivs();
  measurement.v2Epsilon = measurement.dSqrtInvNoise * (measurement.v2Found - v2Image);
  measurement.dErrorSquared = measurement.v2Epsilon.dot( measurement.v2Epsilon );
}

template<class MEstimator>
bool Bundle::Do_LM_Step(bool *pbAbortSignal)
{
  // Reset accumulators to zero
  ClearAccumulators();

  //  Do a LM step according to Hartley and Zisserman Algo A6.4 in MVG 2nd Edition
  //  Actual work starts a bit further down - first we have to work out the 
  //  projections and errors for each point, so we can do tukey reweighting
  vector<double> vdErrorSquared;
  list<BAMeasurement>::iterator iMeasurement;
  for(iMeasurement = mMeasList.begin(); iMeasurement != mMeasList.end(); iMeasurement++) {
    
      ProjectAndFindSquaredError(*iMeasurement);
      
      if(!iMeasurement->bBad) vdErrorSquared.push_back(iMeasurement->dErrorSquared);
  }
  
  // Projected all points and got vector of errors; find the median, 
  // And work out robust estimate of sigma, then scale this for the tukey
  // estimator
  mdSigmaSquared = MEstimator::FindSigmaSquared(vdErrorSquared);

  // Initially the median error might be very small - set a minimum
  // value so that good measurements don't get erased!
  static pvar3<double> gvdMinSigma("Bundle.MinTukeySigma", 0.4, SILENT);
  
  const double dMinSigmaSquared = (*gvdMinSigma) * (*gvdMinSigma);
  
  if(mdSigmaSquared < dMinSigmaSquared) mdSigmaSquared = dMinSigmaSquared;


  //  OK - good to go! weights can now be calced on second run through the loop.
  //  Back to Hartley and Zisserman....
  //  `` (i) Compute the derivative matrices Aij = [dxij/daj]
  //      and Bij = [dxij/dbi] and the error vectors eij''
  //
  //  Here we do this by looping over all measurements
  // 
  //  While we're here, might as well update the accumulators U, ea, B, eb
  //  from part (ii) as well, and let's calculate Wij while we're here as well.
  
  double dCurrentError = 0.0;
  for(iMeasurement = mMeasList.begin(); iMeasurement !=mMeasList.end(); iMeasurement++) {
    
      
      BACamera &cam = mvCameras[iMeasurement->camIndex];
      BAPoint &point = mvPoints[iMeasurement->pointIndex];
      
      // Project the point.
      // We've done this before - results are still cached in measurement.
      if(iMeasurement->bBad) {
	
	  dCurrentError += 1.0;
	  continue;
      }
      
      // What to do with the weights? The easiest option is to independently weight
      // The two jacobians, A and B, with sqrt of the tukey weight w;
      // And also weight the error vector v2Epsilon.
      // That makes everything else automatic.
      // Calc the square root of the tukey weight:
      double dWeight= MEstimator::SquareRootWeight(iMeasurement->dErrorSquared, mdSigmaSquared);
      // Re-weight error:
      // NOTE: Keep in mind that v2Epsilon is already WEIGHTED by the variance we got from the tracker 
      //      (recall, its the inverse pyramidal scale of the tracked feature)!
      iMeasurement->v2Epsilon = dWeight * iMeasurement->v2Epsilon;
      
      if(dWeight == 0) {
	
	  iMeasurement->bBad = true;  
	  dCurrentError += 1.0;
	  continue;
      }
      
      dCurrentError += MEstimator::ObjectiveScore(iMeasurement->dErrorSquared, mdSigmaSquared);
      
      // To re-weight the jacobians, I'll just re-weight the camera param matrix
      // This is only used for the jacs and will save a few fmuls
      cv::Matx<float, 2, 2> m2CamDerivs = dWeight * iMeasurement->m2CamDerivs;
      
      const double invDepth = 1.0 / iMeasurement->v3Cam[2];
      const cv::Vec<float, 4> v4Cam = CvUtils::backproject(iMeasurement->v3Cam); // to get the Lie generators...
      
      // *****************  Calculate A: (the projection derivativess WRT the camera pose) *********************
      if(cam.bFixed) iMeasurement->m26A = cv::Matx<float, 2, 6>::zeros();
      else  {
	
	for(int m = 0; m < 6; m++) {
	      // derivative of the point wrt the translation and the axis angle vector
	      const cv::Vec<float,4 > v4Motion = SE3<>::generator_field(m, v4Cam);
	      
 	      cv::Vec<float, 2 > v2CamFrameMotion( (v4Motion[0] - v4Cam[0] * v4Motion[2] * invDepth) * invDepth,
					          (v4Motion[1] - v4Cam[1] * v4Motion[2] * invDepth) * invDepth
					        );
 	      //meas.m26A.T()[m] = meas.dSqrtInvNoise * m2CamDerivs * v2CamFrameMotion;
	      iMeasurement->m26A(0, m) = iMeasurement->dSqrtInvNoise * ( m2CamDerivs(0, 0) * v2CamFrameMotion[0] +  
									 m2CamDerivs(0, 1) * v2CamFrameMotion[1] );
	      iMeasurement->m26A(1, m) = iMeasurement->dSqrtInvNoise * ( m2CamDerivs(1, 0) * v2CamFrameMotion[0] +  
									 m2CamDerivs(1, 1) * v2CamFrameMotion[1] );
	    
	}
      }
      
     
      // ********************************** Calculate B: The projection derivs WRT the point *****************************************
      for(int m = 0; m < 3; m++) {
	  
	 cv::Matx<float, 3, 1> v3Motion = cam.se3CfW.get_rotation().get_matrix().col(m);
	
	 cv::Vec<float, 2> v2CamFrameMotion( (v3Motion(0, 0) - v4Cam[0] * v3Motion(2, 0) * invDepth) * invDepth,
				             (v3Motion(1, 0) - v4Cam[1] * v3Motion(2, 0) * invDepth) * invDepth
				           );
	  //meas.m23B.T()[m] = meas.dSqrtInvNoise * m2CamDerivs * v2CamFrameMotion;
	  iMeasurement->m23B(0, m) = iMeasurement->dSqrtInvNoise * ( m2CamDerivs(0, 0) * v2CamFrameMotion[0] + 
								      m2CamDerivs(0, 1) * v2CamFrameMotion[1] );
	  iMeasurement->m23B(1, m) = iMeasurement->dSqrtInvNoise * ( m2CamDerivs(1, 0) * v2CamFrameMotion[0] + 
								     m2CamDerivs(1, 1) * v2CamFrameMotion[1] );
	
      }
      
        
      // Update the accumulators
      if(!cam.bFixed) {
	  //	  cam.m6U += meas.m26A.T() * meas.m26A; 	  // SLOW SLOW this matrix is symmetric
	  BundleTriangle_UpdateM6U_LL(cam.m6U, iMeasurement->m26A );
	  cam.v6EpsilonA += iMeasurement->m26A.t() * iMeasurement->v2Epsilon;
	  // NOISE COVAR OMITTED because it's the 2-Identity
      }
      
      //            point.m3V += meas.m23B.T() * meas.m23B;             // SLOW-ish this is symmetric too
      BundleTriangle_UpdateM3V_LL(point.m3V, iMeasurement->m23B );
      point.v3EpsilonB += iMeasurement->m23B.t() * iMeasurement->v2Epsilon;
      
      if(cam.bFixed) iMeasurement->m63W = cv::Matx<float, 6, 3>::zeros();
      else 
	iMeasurement->m63W = iMeasurement->m26A.t() * iMeasurement->m23B;
	
      
  }
  
  // OK, done (i) and most of (ii) except calcing Yij; this depends on Vi, which should 
  // be finished now. So we can find V*i (by adding lambda) and then invert.
  // The next bits depend on mdLambda! So loop this next bit until error goes down.
  double dNewError = dCurrentError + 9999;
  while(dNewError > dCurrentError && !mbConverged && !mbHitMaxIterations && !*pbAbortSignal) {
    
    // Rest of part (ii) : find V*i inverse
    vector<BAPoint>::iterator iBAPoint;
    for( iBAPoint = mvPoints.begin(); iBAPoint != mvPoints.end(); iBAPoint++) {
	  
      cv::Matx<float, 3, 3> m3VStar = iBAPoint->m3V; 
      // augmenting the diagonal now
      m3VStar(0, 0) *= (1.0 + mdLambda); m3VStar(1, 1) *= (1.0 + mdLambda); m3VStar(2, 2) *= (1.0 + mdLambda);
	
      
      // CASE #1 : if V* is NOT Positive Semi-Definite, we zero the inverse. 
      float dummy[9];
      if ( !CvUtils::M3PSD_LT_Cholesky<>(m3VStar.val, dummy) ) // checking if B is a PSD matrix (it should be if 
								// matrix multiplications are correct. In the worst case )
    
      //if (m3VStar(0, 0) * m3VStar(1, 1) == 0 )  // This is an alternative (weaker) condition for non PSD matrix but it should work as well here.
      {
	  iBAPoint->m3VStarInv = cv::Matx<float, 3, 3>::zeros();
	  cout <<"*********************** NOT a PSD matrix!!! ***************************"<<endl;
      }
      else 
      {
	          
	// CASE #2: Determinant is very small. Again zero the inverse
	if ( fabs(CvUtils::M3Symm_LT_Det( m3VStar.val ) ) < 10e-5) 
	{
		
	  // Zero the inversew in this case too. If V* is nearly non-invertible, this
	  // most probably means that the accumulated gradients are vanishing and therefore the points are well in place...
	  iBAPoint->m3VStarInv = cv::Matx<float, 3, 3>::zeros();
	  cout <<"Points have vanishing gradients (accumulated in V*) !!!"<<endl;
	}
	else // CASE #3: Use the standard (adjoint) inversion formula for 3x3 symmetric matrices
	{
	    CvUtils::M3Symm_LT_Inverse(m3VStar.val, iBAPoint->m3VStarInv.val); // standard inversion using LT only
	}    
	      
      }
    }

      // Done part (ii), except calculating Yij;
      // But we can do this inline when we calculate S in part (iii).
      
      // Part (iii): Construct the the big block-matrix S which will be inverted.
      cv::Mat_<float> mS = cv::Mat_<float>::zeros(mnCamsToUpdate * 6, mnCamsToUpdate * 6);
      
      cv::Mat_<float> vE = cv::Mat_<float>::zeros(mnCamsToUpdate * 6, 1); // a vector as a mnCamsToUpdate x 1 matrix
      
      cv::Matx<float, 6, 6> m6(6, 6); // Temp working space
      cv::Vec<float, 6> v6; // Temp working space
      
      // Calculate on-diagonal blocks of S (i.e. only one camera at a time:)
      for(unsigned int cam1Index = 0; cam1Index < mvCameras.size(); cam1Index++) {
	
	  BACamera &cam1 = mvCameras[cam1Index];
	  if(cam1.bFixed) continue;
	  int nCam1StartRow = cam1.nStartRow;
	  
	  // First, do the diagonal elements.
	  for(int r = 0; r < 6; r++) {
	    
	     m6(r, r) = cam1.m6U(r, r);
	     
	     for(int c = 0; c < r; c++) 
		m6(r, c) = m6(c, r) = cam1.m6U(r, c);
	      
	  }   
	  
	  // Augmenting again... Hardcoding the iteration (in case it's gonna save a bit of time...)
	  m6(0, 0) *= (1.0 + mdLambda); m6(1, 1) *= (1.0 + mdLambda); m6(2, 2) *= (1.0 + mdLambda);
	  m6(3, 3) *= (1.0 + mdLambda); m6(4, 4) *= (1.0 + mdLambda); m6(5, 5) *= (1.0 + mdLambda);
	  
	  v6 = cam1.v6EpsilonA; // former "j" cam
	  
	  vector<BAMeasurement*> &vMeasLUTCam1 = mvMeasLUTs[cam1Index];
	  // Sum over measurements (points):
	  for(unsigned int pointIndex = 0; pointIndex < mvPoints.size(); pointIndex++) {
	    
	      BAMeasurement* pMeasurement = vMeasLUTCam1[pointIndex];
	      
	      if ( pMeasurement == NULL ) continue;
	      if ( pMeasurement->bBad ) continue;
	      
	      m6 -= pMeasurement->m63W * ( mvPoints[pointIndex].m3VStarInv * pMeasurement->m63W.t() );  // SLOW SLOW should by 6x6sy
	      v6 -= pMeasurement->m63W * ( mvPoints[pointIndex].m3VStarInv * mvPoints[pointIndex].v3EpsilonB );
	    }
	   //mS.slice(nCamJStartRow, nCamJStartRow, 6, 6) = m6;
	   //vE.slice(nCamJStartRow,6) = v6;
	 
	  // doing a PSD 6x6 block assignment (and respective error vector)
	  int r_, c_;
	  for (r_ = 0; r_ < 6; r_++) {
	    
	    vE(nCam1StartRow + r_, 0) = v6[r_];    			// the error vector
	    mS(nCam1StartRow + r_, nCam1StartRow + r_) = m6(r_, r_);    // diagonal of mS
	    
	    for (c_ = 0; c_ < r_; c_++)
	      mS(nCam1StartRow + r_, nCam1StartRow + c_) = mS(nCam1StartRow + c_, nCam1StartRow + r_) = m6(r_, c_);
	  }
	  
	 
      }
      
      // Now find off-diag elements of S. These are camera-point-camera combinations, of which there are lots.
      // New code which doesn't waste as much time finding point-(cam1, cam2) pairs; these are pre-stored in a per-point list.
      for(unsigned int pointIndex = 0; pointIndex < mvPoints.size(); pointIndex++) {
	
	  BAPoint &point = mvPoints[pointIndex];
	  int nCurrent1 = -1;
	  int nCam1Row = -1;
	  BAMeasurement* pMeasurement_Cam1;
	  BAMeasurement* pMeasurement_Cam2;
	  
	  cv::Matx<float, 6, 3> m63_MIJW_x_m3VStarInv; // 6x3 to be sure...
	  
	  // Now get all the off-diagonal links for this particular BA point
	  vector<OffDiagScriptEntry>::iterator iOffDiagLink;
	  for(iOffDiagLink = point.vOffDiagonalScript.begin(); iOffDiagLink != point.vOffDiagonalScript.end(); iOffDiagLink++) {
	    
	      // Get the Cam2 measurement (could be CamIndex but just being consistent with the original) of the measurement list 
	      // (indexed by iOffDiagLink->Cam2Index, which is a camera entry)
	      pMeasurement_Cam2 = mvMeasLUTs[iOffDiagLink->cam2Index][pointIndex];
	      
	      if( pMeasurement_Cam2 == NULL ) continue;
	      if( pMeasurement_Cam2->bBad ) continue;
	      
	      if(iOffDiagLink->cam1Index != nCurrent1) {
		  // Now get the other camera measuremenmt for the point
		  pMeasurement_Cam1 = mvMeasLUTs[iOffDiagLink->cam1Index][pointIndex];
		  
		  if ( pMeasurement_Cam1 == NULL ) continue;
		  if ( pMeasurement_Cam1->bBad ) continue;
		  
		  nCurrent1 = iOffDiagLink->cam1Index;
		  nCam1Row = mvCameras[iOffDiagLink->cam1Index].nStartRow;
		  
		  m63_MIJW_x_m3VStarInv = pMeasurement_Cam1->m63W * point.m3VStarInv;
	      }
	      
	      int nCam2Row = mvCameras[pMeasurement_Cam2->camIndex].nStartRow;
	      
	      //mS.slice(nJRow, nKRow, 6, 6) -= m63_MIJW_times_m3VStarInv * pMeas_ik->m63W.T();
	      
	      int _r, _c;
	      for (_r = 0; _r < 6; _r++) 
		for (_c = 0; _c < 6; _c++)
		  mS(nCam1Row + _r, nCam2Row + _c) -= m63_MIJW_x_m3VStarInv(_r , 0) * pMeasurement_Cam2->m63W(_c, 0) +
						      m63_MIJW_x_m3VStarInv(_r , 1) * pMeasurement_Cam2->m63W(_c, 1) +
						      m63_MIJW_x_m3VStarInv(_r , 2) * pMeasurement_Cam2->m63W(_c, 2);
	      
	      
	      assert(nCam2Row < nCam1Row);
	    }
	}
      
      // Did this purely LL triangle - now update the TR bit as well!
      // (This is actually unneccessary since the lapack cholesky solver
      // uses only one triangle, but I'm leaving it in here anyway.)
      //
      // NOTE: OpenCV DOES NOT use only a triangle in cv::solve(), 
      //        even when Cholesky is specified. Thus, we need the entire matrix!
      for(int i=0; i<mS.rows; i++)
	for(int j=0; j<i; j++) 
	  mS(j, i) = mS(i, j);
      
      // Got fat matrix S and vector E from part(iii). Now Cholesky-decompose
      // the matrix, and find the camera update vector.
      cv::Mat_<float> mvCamerasUpdate(mS.rows, 1);
      
      cv::solve(mS, vE, mvCamerasUpdate, cv::DECOMP_CHOLESKY);
      
      // Part (iv): Compute the map updates
      cv::Mat_<float> mvMapUpdates(mvPoints.size() * 3, 1);
      for(unsigned int pointIndex = 0; pointIndex < mvPoints.size(); pointIndex++) {
	
	  cv::Vec<float, 3> v3Sum(0, 0, 0);
	  for(unsigned int camIndex = 0; camIndex < mvCameras.size(); camIndex++) {
	    
	      BACamera &cam = mvCameras[camIndex];
	      
	      if(cam.bFixed) continue;
	      
	      BAMeasurement *pMeasurement = mvMeasLUTs[camIndex][pointIndex];
	      
	      if ( pMeasurement == NULL ) continue;
	      if ( pMeasurement->bBad ) continue;
	      //v3Sum+=pMeas->m63W.t() * mvCamerasUpdate.slice(cam.nStartRow,6);
	      int _c;
	      for (_c = 0; _c < 3; _c++) 
		v3Sum[_c] += pMeasurement->m63W(0, _c) * mvCamerasUpdate(cam.nStartRow + 0, 0) +
			     pMeasurement->m63W(1, _c) * mvCamerasUpdate(cam.nStartRow + 1, 0) +
			     pMeasurement->m63W(2, _c) * mvCamerasUpdate(cam.nStartRow + 2, 0) + 
			     pMeasurement->m63W(3, _c) * mvCamerasUpdate(cam.nStartRow + 3, 0) + 
			     pMeasurement->m63W(4, _c) * mvCamerasUpdate(cam.nStartRow + 4, 0) + 
			     pMeasurement->m63W(5, _c) * mvCamerasUpdate(cam.nStartRow + 5, 0);
			   
	  }
	  cv::Vec<float, 3> v3 = mvPoints[pointIndex].v3EpsilonB - v3Sum;
	  
	  //vMapUpdates.slice(i * 3, 3) = mvPoints[i].m3VStarInv * v3;
	  mvMapUpdates(pointIndex * 3 + 0, 0) = mvPoints[pointIndex].m3VStarInv(0, 0) * v3[0] + mvPoints[pointIndex].m3VStarInv(0, 1) * v3[1] + mvPoints[pointIndex].m3VStarInv(0, 2) * v3[2];
	  mvMapUpdates(pointIndex * 3 + 1, 0) = mvPoints[pointIndex].m3VStarInv(1, 0) * v3[0] + mvPoints[pointIndex].m3VStarInv(1, 1) * v3[1] + mvPoints[pointIndex].m3VStarInv(1, 2) * v3[2];
	  mvMapUpdates(pointIndex * 3 + 2, 0) = mvPoints[pointIndex].m3VStarInv(2, 0) * v3[0] + mvPoints[pointIndex].m3VStarInv(2, 1) * v3[1] + mvPoints[pointIndex].m3VStarInv(2, 2) * v3[2];
	  
	  //if(isnan(vMapUpdates.slice(i * 3, 3) * vMapUpdates.slice(i * 3, 3)))
	  if(isnan( mvMapUpdates(pointIndex * 3 + 0, 0) * mvMapUpdates(pointIndex * 3 + 0, 0) + 
		    mvMapUpdates(pointIndex * 3 + 1, 0) * mvMapUpdates(pointIndex * 3 + 1, 0) + 
		    mvMapUpdates(pointIndex * 3 + 2, 0) * mvMapUpdates(pointIndex * 3 + 2, 0) ) 
	    )
	    {
	      cerr << "NANNERY! " << endl;
	      cerr << mvPoints[pointIndex].m3VStarInv << endl;
	    }
	}
      
      // OK, got the two update vectors.
      // First check for convergence..
      // (this is a very poor convergence test)
      double dSumSquaredUpdate = cv::norm(mvCamerasUpdate, cv::NORM_L2SQR) + cv::norm(mvMapUpdates, cv::NORM_L2SQR);
      
      if(dSumSquaredUpdate< *mgvdUpdateConvergenceLimit) mbConverged = true;
      
      // Now re-project everything and measure the error;
      // NB we don't keep these projections, SLOW, bit of a waste.
      
      // Temp versions of updated pose and pos:
      for(unsigned int camIndex = 0; camIndex < mvCameras.size(); camIndex++) {
	  
	if(mvCameras[camIndex].bFixed) mvCameras[camIndex].se3CfWNew = mvCameras[camIndex].se3CfW;
	  else
	    //mvCameras[j].se3CfWNew = SE3<>::exp(vCamerasUpdate.slice(mvCameras[j].nStartRow, 6)) * mvCameras[j].se3CfW;
	  mvCameras[camIndex].se3CfWNew = SE3<>::exp(cv::Vec<float, 6>( mvCamerasUpdate(mvCameras[camIndex].nStartRow + 0, 0), 
									mvCamerasUpdate(mvCameras[camIndex].nStartRow + 1, 0),
									mvCamerasUpdate(mvCameras[camIndex].nStartRow + 2, 0),
									mvCamerasUpdate(mvCameras[camIndex].nStartRow + 3, 0),
									mvCamerasUpdate(mvCameras[camIndex].nStartRow + 4, 0),
									mvCamerasUpdate(mvCameras[camIndex].nStartRow + 5, 0) ) 
								       ) * mvCameras[camIndex].se3CfW;
      }
      
      for(unsigned int pointIndex = 0; pointIndex<mvPoints.size(); pointIndex++)
	//mvPoints[i].v3PosNew = mvPoints[i].v3Pos + vMapUpdates.slice(i*3, 3);
	mvPoints[pointIndex].v3PosNew = cv::Vec<float, 3>( mvPoints[pointIndex].v3Pos[0] + mvMapUpdates(pointIndex*3 + 0, 0),
							   mvPoints[pointIndex].v3Pos[1] + mvMapUpdates(pointIndex*3 + 1, 0),
							   mvPoints[pointIndex].v3Pos[2] + mvMapUpdates(pointIndex*3 + 2, 0)
							 );
      // Calculate new error by re-projecting, doing tukey, etc etc:
      dNewError = FindNewError<MEstimator>();
      
      cout <<setprecision(1) << "L" << mdLambda << setprecision(3) <<  "\tOld " << dCurrentError << "  New " << dNewError << "  Diff " << dCurrentError - dNewError << "\t";
      
      // Was the step good? If not, modify lambda and try again!!
      // (if it was good, will break from this loop.)
      if(dNewError > dCurrentError) {
	  cout << " TRY AGAIN " << endl;
	  ModifyLambda_BadStep();
      }
      
      mnCounter++;
      if(mnCounter >= *mgvnMaxIterations) mbHitMaxIterations = true;
    }   // End of while error too big loop
  
  // Was the last step a good one?
  if(dNewError < dCurrentError) { 
    
      cout << " WINNER            ------------ " << endl;
      // Woo! got somewhere. Update lambda and make changes permanent.
      ModifyLambda_GoodStep();
      for(unsigned int camIndex = 0; camIndex < mvCameras.size(); camIndex++) 
	mvCameras[camIndex].se3CfW = mvCameras[camIndex].se3CfWNew;
      for(unsigned int pointIndex = 0; pointIndex < mvPoints.size(); pointIndex++) 
	mvPoints[pointIndex].v3Pos = mvPoints[pointIndex].v3PosNew; 
      mnAccepted++;
  }
  
  // Finally, ditch all the outliers.
  vector<list<BAMeasurement>::iterator> vMeasIterators;
  list<BAMeasurement>::iterator ilMeas;
  for( ilMeas = mMeasList.begin(); ilMeas != mMeasList.end(); ilMeas++)
    if ( ilMeas->bBad ) {
	vMeasIterators.push_back(ilMeas);
	mvOutlierMeasurementIdx.push_back(make_pair(ilMeas->pointIndex, ilMeas->camIndex));
	mvPoints[ilMeas->pointIndex].nOutliers++;
	mvMeasLUTs[ilMeas->camIndex][ilMeas->pointIndex] = NULL;
    }
  
  for(unsigned int i=0; i<vMeasIterators.size(); i++) mMeasList.erase(vMeasIterators[i]);
  
  cout << "Nuked " << vMeasIterators.size() << " measurements." << endl;
  return true;
}

// Find the new total error if cameras and points used their 
// new coordinates
template<class MEstimator>
double Bundle::FindNewError()
{
  ofstream ofs;
  double dNewError = 0;
  vector<double> vdErrorSquared;
  list<BAMeasurement>::iterator ilMeas;
  for( ilMeas = mMeasList.begin(); ilMeas != mMeasList.end(); ilMeas++) {
    
      // Project the point.
      cv::Vec<float, 3> v3Cam = mvCameras[ilMeas->camIndex].se3CfWNew * mvPoints[ilMeas->pointIndex].v3PosNew;
      
      if(v3Cam[2] <= 0) {
	  dNewError += 1.0;
	  cout << ".";
	  continue;
      }
      cv::Vec<float, 2> v2EucPlane = CvUtils::pproject(v3Cam);
      cv::Vec<float, 2> v2Image   = mCamera.Project(v2EucPlane);
      // NOTE: In PTAMM, for some reason, the normalized Euclidean projection of v3Cam is compared 
      //       to the vFound entry of the BAMeasurement structure. This cannot be valid, 
      //       because the map maker passes "vRootPos" of the KFMeasurement to the bundle adjuster;
      //       and we know for fact thay the tracker is assigning the "v2Found" member of trackerdata
      //       to vRootPos, which is an image position! Thus, the error below is correct, although the PTAMM comparison
      //       it does not produce horrible results and therefore may went unnoticed...
      cv::Vec<float, 2> v2Error =   ilMeas->dSqrtInvNoise * (ilMeas->v2Found - v2Image); 
      double dErrorSquared = v2Error[0] * v2Error[0] + v2Error[1] * v2Error[1];
      dNewError += MEstimator::ObjectiveScore(dErrorSquared, mdSigmaSquared);
  }
  return dNewError;
}

// Optimisation: make a bunch of tables, one per camera
// which point to measurements (if any) for each point
// This is faster than a std::map lookup.
void Bundle::GenerateMeasurementLUTs() {
  
  mvMeasLUTs.clear();
  // Create a measurement lookup table
  // and foreach CAMERA VIEW create a vector of NULL-initialized entries for ALL POINTS
  for(unsigned int nCam = 0; nCam < mvCameras.size(); nCam++) {
    
      mvMeasLUTs.push_back(vector<BAMeasurement*>());
      mvMeasLUTs.back().resize(mvPoints.size(), NULL);
    }
  // Now filling the point entries for each camera (if a measurement does
  // not concern a mappoint, then the respective entry in the vector will remain NULL
  list<BAMeasurement>::iterator ilMeas;
  for( ilMeas = mMeasList.begin(); ilMeas != mMeasList.end(); ilMeas++)
    mvMeasLUTs[ilMeas->camIndex][ilMeas->pointIndex] =  &(*ilMeas);
}

// Optimisation: make a per-point list of all
// observation camera-camera pairs; this is then
// scanned to make the off-diagonal elements of matrix S
// In other words, the list of off-diagonal pairs represents the pairs of camera views
// that have at least one measurement (observed mappoint)in common. 
void Bundle::GenerateOffDiagScripts() {
  
  for(unsigned int pointIndex = 0; pointIndex < mvPoints.size(); pointIndex++) {
    
      BAPoint &point = mvPoints[pointIndex];
      point.vOffDiagonalScript.clear();
      
      // Going through all the registered camera views FOR THIS PARTICULAR POINT! (i.e., p.sCameras)
      //                                               -------------------------
      for(set<int>::iterator iCam1Index = point.sCameras.begin(); iCam1Index != point.sCameras.end(); iCam1Index ++) {
	
	  // if the camera is fixed, do not bother...
	  // NOTE: It would seem that by skipping we ignore the constraint in terms of the (under optimization) mappoint coordinates 
	  //       but that relationship has already been registered.
	  //
	  //       So, in effect, the off-diagonal entries are helping us to setup the camera POSE entries affected by a single measurement
	  if(mvCameras[*iCam1Index].bFixed) continue;
	  
	  BAMeasurement *pCam1Measurement = mvMeasLUTs[*iCam1Index][pointIndex];
	  assert(pCam1Measurement != NULL);
	  // Now, again go through the camera list of the point, only up to the index of Cam1 (that's because of symmetry, not time precedence)
	  for(set<int>::iterator iCam2Index = point.sCameras.begin(); iCam2Index != iCam1Index; iCam2Index++) {
	    
	      // Same thing on the other view. If the camera is fixed, then skip the off-diagonal entry
	      // because it does not affect pose variables...
	      if(mvCameras[*iCam2Index].bFixed) continue;
	      
	      BAMeasurement *pCam2Measurement = mvMeasLUTs[*iCam2Index][pointIndex];
	      assert(pCam2Measurement != NULL);
	      
	      // Finally, update the off-diagonal entries of the MapPoint
	      OffDiagScriptEntry e;
	      e.cam1Index = *iCam1Index;
	      e.cam2Index = *iCam2Index;
	      
	      point.vOffDiagonalScript.push_back(e);
	    }
	}
    }
}

void Bundle::ModifyLambda_GoodStep()
{
  mdLambdaFactor = 2.0;
  mdLambda *= 0.3;
}

void Bundle::ModifyLambda_BadStep()
{
  mdLambda = mdLambda * mdLambdaFactor;
  mdLambdaFactor = mdLambdaFactor * 2;
}


cv::Vec<float, 3> Bundle::GetPointCoords(int n)
{
  return mvPoints.at(n).v3Pos;
}

SE3<> Bundle::GetCameraPose(int n)
{
  return mvCameras.at(n).se3CfW;
}

set<int> Bundle::GetOutliers()
{
  set<int> sOutliers;
  set<int>::iterator hint = sOutliers.begin();
  for(unsigned int i=0; i<mvPoints.size(); i++) {
    
      BAPoint &p = mvPoints[i];
      if(p.nMeasurements > 0 && p.nMeasurements == p.nOutliers)
	hint = sOutliers.insert(hint, i);
  }
  
  return sOutliers;
}


vector<pair<int, int> > Bundle::GetOutlierMeasurements()
{
  return mvOutlierMeasurementIdx;
}









