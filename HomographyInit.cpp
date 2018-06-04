//   George Terzakis 2016
//
// University of Portsmouth
//
//
// Code based on PTAM by Klein and Murray (Copyright 2008 Isis Innovation Limited)

#include "HomographyInit.h"

#include <utility>



//#include "GCVD/GraphSLAM.h"
#include "MEstimator.h"

using namespace std;
//using namespace Optimization;


// This function marks a homography match as inliner by simply checking if 
// the first point is projected to a second point with error below the MLESAC threshold...
inline bool HomographyInit::IsHomographyInlier(cv::Matx<float, 3, 3> m3Homography, InitializerMatch match) {
  
  
  cv::Vec2f v2Projected = CvUtils::pproject(  m3Homography * CvUtils::backproject(match.v2EucPlaneFirst) );
  //cv::Vec2f v2Error = match.v2EucPlaneSecond - v2Projected;
  //cv::Vec2f v2PixelError = match.m2PixelProjectionJac * v2Error;
  //cv::Vec2f v2PixelError = pCamera->Project(match.v2EucPlaneSecond) - pCamera->Project(v2Projected); 
  cv::Vec2f v2PixelError = cv::Vec<float, 2>(match.imPtSecond.x, match.imPtSecond.y) - pCamera->Project(v2Projected);
  double dSquaredError = v2PixelError[0] * v2PixelError[0] + v2PixelError[1] * v2PixelError[1]; // again, avoid overloads and inline...
  // dMaxPixelErrorSquared is the sequared robust cutoff error by Torr and Zisserman
  return (dSquaredError < mdMaxPixelErrorSquared); 
}

inline double HomographyInit::MLESACScore(cv::Matx<float, 3, 3> m3Homography, InitializerMatch match) {
  
  cv::Vec2f v2Projected = CvUtils::pproject( m3Homography * CvUtils::backproject(match.v2EucPlaneFirst) );
  //cv::Vec2f v2Error = match.v2EucPlaneSecond - v2Projected;
  //cv::Vec2f v2PixelError = match.m2PixelProjectionJac * v2Error;
  //cv::Vec2f v2PixelError = pCamera->Project(match.v2EucPlaneSecond) - pCamera->Project(v2Projected);
  cv::Vec2f v2PixelError = cv::Vec<float, 2>(match.imPtSecond.x, match.imPtSecond.y) - pCamera->Project(v2Projected);
  double dSquaredError = v2PixelError[0] * v2PixelError[0] + v2PixelError[1] * v2PixelError[1];
  
  // The "robust" Torr-Zisserman error (i.e. e^2 error^2 =  e^2 < T^2 ? e^2 : T^2;
  return dSquaredError > mdMaxPixelErrorSquared ? mdMaxPixelErrorSquared : dSquaredError;
}

bool HomographyInit::Compute(vector<InitializerMatch> vMatches, double dMaxPixelError, SE3<> &se3SecondFromFirst)
{
  mdMaxPixelErrorSquared = dMaxPixelError * dMaxPixelError;
  mvMatches = vMatches;
  
  // Find best homography from minimal sets of image matches
  BestHomographyFromMatches_MLESAC();
  
  // Generate the inlier set, and refine the best estimate using this
  mvHomographyInliers.clear();
  for(unsigned int i=0; i<mvMatches.size(); i++)
    if(IsHomographyInlier(mm3BestHomography, mvMatches[i])) mvHomographyInliers.push_back(mvMatches[i]);
  
    // NOTE: Now I was looking throughout the code to find the point where this RARE but
    //       LIKELY (set webcamera to Autofocus ON and you are likely to see it happen) 
    //       issue occurs and I accidentally cast my eye on ETH PTAM and found Weiss'
    //       check. So credit goes to him...
  if (mvHomographyInliers.size() < 4) {
    
    return false;
    // now, keep in mind that if it goes back, there must be some way of unlocking the mapmaker
    // from having few or no inliers (yet a lot of trail matches) over and over again...
  }
    
  // Now refine the homography based on inliners
  // using slow, G-N iteration but only 5 times...
  for(int iteration = 0; iteration < 5; iteration++) {
    //cout <<"Refinement iteration : " <<iteration <<endl;
    RefineHomographyWithInliers();
  }
  
  // Decompose the best homography into a set of possible decompositions
  DecomposeHomography();
  
  // At this stage should have FOUR or TWO decomposition options, if all went according to plan
  //if(mvDecompositions.size() != 8) return false;
  if(mvDecompositions.size() < 2) {
    cout << "bad decompositions! Size of list : "<<mvDecompositions.size() << endl;
    return false;
  }
  cout << "Good decomposition! number of decompositions : "<<mvDecompositions.size()<<endl;
  
  // And choose the best one based on visibility constraints
  ChooseBestDecomposition();
  
  se3SecondFromFirst = mvDecompositions[0].se3SecondFromFirst;
  return true;
}

// Just compute the homography out of a set of matches (N >= 4)
// using the classic homogeneous LS method
cv::Matx<float, 3, 3> HomographyInit::HomographyFromMatches(vector<InitializerMatch> vMatches)
{
  unsigned int nPoints = vMatches.size();
  assert(nPoints >= 4);
  
  // Avoid the large matrix (original PTAM code) and do the gram-matrix with sums instead. It should be alright because 
  // normalized Euclidean coordinates are small figures in general...
  // Furthermore, we wont have to deal with the situation of zero-padding the data matrix...
  cv::Matx<double, 9, 9> m9D = cv::Matx<double, 9, 9>::zeros();
  for(unsigned int n=0; n<nPoints; n++) {
      double x2 = vMatches[n].v2EucPlaneSecond[0];
      double y2 = vMatches[n].v2EucPlaneSecond[1];
      
      double x1 = vMatches[n].v2EucPlaneFirst[0];
      double y1 = vMatches[n].v2EucPlaneFirst[1];
      // filling ONLY the Upper triangle of m9D...
      //
      // 1. Filling the upper triangle of the upper 3x3 block (which is equal to  the second 3x3 diagonal block)
      m9D(0, 0) += x1*x1;  m9D(0, 1) += x1*y1; m9D(0, 2) += x1;
			   m9D(1, 1) += y1*y1;   m9D(1, 2) += y1;
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
  // Sow no filling-in the gaps:
  // 1. Filling the missing lower part of the upper 3x3 diagonal block and copying to the second diagonal 3x3 block
  m9D(1, 0) = m9D(4, 3) = m9D(3, 4) = m9D(0, 1); 
  m9D(2, 0) = m9D(5, 3) = m9D(3, 5) = m9D(0, 2);   m9D(2, 1) = m9D(5, 4) = m9D(4, 5) = m9D(1, 2);
  
  // 2. the diagonal elements from 3 - 5 are the same ones from 0-2:
  m9D(3, 3) = m9D(0, 0); m9D(4, 4) = m9D(1, 1); m9D(5, 5) = m9D(2, 2);
  
  // 3. Now copying the last 3 columns (down-to and exluding the diagonal) to the last 3 rows...
  m9D(6, 0) = m9D(0, 6); m9D(6, 1) = m9D(1, 6); m9D(6, 2) = m9D(2, 6); m9D(6, 3) = m9D(3, 6); m9D(6, 4) = m9D(4, 6); m9D(6, 5) = m9D(5, 6);
  m9D(7, 0) = m9D(0, 7); m9D(7, 1) = m9D(1, 7); m9D(7, 2) = m9D(2, 7); m9D(7, 3) = m9D(3, 7); m9D(7, 4) = m9D(4, 7); m9D(7, 5) = m9D(5, 7); m9D(7, 6) = m9D(6, 7);
  m9D(8, 0) = m9D(0, 8); m9D(8, 1) = m9D(1, 8); m9D(8, 2) = m9D(2, 8); m9D(8, 3) = m9D(3, 8); m9D(8, 4) = m9D(4, 8); m9D(8, 5) = m9D(5, 8); m9D(8, 6) = m9D(6, 8); m9D(8, 7) = m9D(7, 8); 
  
   
  // The right null-space or the last eigenvector of m3D9 gives the homography...
  cv::Matx<double, 9, 9> U, Vt;
  cv::Matx<double, 9, 1> w;
  cv::SVD::compute(m9D, w, U, Vt);
  cv::Matx<float, 3, 3> m3Homography(3, 3); // the 3x3 homography
  m3Homography(0, 0) = Vt(8, 0); m3Homography(0, 1) = Vt(8, 1); m3Homography(0, 2) = Vt(8, 2);
  m3Homography(1, 0) = Vt(8, 3); m3Homography(1, 1) = Vt(8, 4); m3Homography(1, 2) = Vt(8, 5);
  m3Homography(2, 0) = Vt(8, 6); m3Homography(2, 1) = Vt(8, 7); m3Homography(2, 2) = Vt(8, 8);
  
  
  return m3Homography;
}

// Now this function refines the homography for what it really is: 
// *** A NON-LINEAR PROJECTIVE RELATIONSHIP OF THE FORM: 
//
//        [m1' zeros(1, 6); zeros(1, 3), m1', zeros(1, 3)] * h 
//m2 =   -------------------------------------------------------
//               [m1; 1]'*[zeros(3, 6), eye(3)] * h 
//
// So it does a G-N refinement GraphSLAM-style using only the inliers.
// The whole thing takes place in 2 stages: a) Compute the projection Jacobians and, 
//						Compute Tukey (or any other robust) weights
//					     b) Do the G-N thing... 
// 


// NOTE: The following function uses the actual pixel reprojection error in the second image as opposed
// 	 to the approximation employed originally by PTAM. Thus, now HomographyInit requires a reference to the 
//       camera object used by the MapMaker in order to compute the correct camera derivatives upon during reprojection
//       for error computation.

void HomographyInit::RefineHomographyWithInliers()
{
  // identity prior 
  cv::Matx<float, 9, 9> m9Omega = cv::Matx<float, 9, 9>::eye(); // Identity for prior 
  cv::Vec<float, 9> v9ksi(0, 0, 0, 0, 0, 0, 0, 0, 0);           // zeros in the information matrix (n.b. we are solving for a perturbation Dh)
  
  // The list Jacobians wrt h per datum.
  // NOTE! Jacobians describe pixel-to-pixel correspondences (not Euclidean coordinates)
  vector<cv::Matx<float, 2, 9> > vmJacobians; 
  // reprojection errors in pixels
  vector<cv::Vec2f > vvErrors;          
  vector<double> vdErrorSquared;
  double avgSqError = 0;
  
  for(unsigned int i=0; i<mvHomographyInliers.size(); i++) {
    
      // First, find error.
      cv::Vec2f v2m1 = mvHomographyInliers[i].v2EucPlaneFirst; 		       // first measurement in normalized Euclidean plane
      cv::Vec3f v3m2tild_pred = mm3BestHomography * CvUtils::backproject(v2m1); // The unnormalized predicted Euclidean prediction
      cv::Vec2f v2m2_pred = CvUtils::pproject(v3m2tild_pred);                   // Normalized Euclidean prediction
      //cv::Vec2f v2m2 = mvHomographyInliers[i].v2EucPlaneSecond;                // The measured normalized Euclidean coordinate 
      
      
      // The original PTAM error doesn't feel entirely sane however I may look at it... Perhaps only as an approximation...
      // I agree we do need a projection derivative, but the cannopt be the cached ones by the MapMaker, because
      // they simply do not correspond to the projection derivatives of the prediction on the image.
      // We therefore need to do mCamera.Project (which implies referencing camera object from 
      // the homography  initializer) and THEN, ONLY THEN, compute the derivatives - which will only be necessary for the Jacobian 
      // and NOT for the error computation!
      // ********* Thus, to get the error we need to PROJECT THE PREDICTION ON THE IMAGE and take its difference
      // 	   from the actual measurement (added two fields in homographyMatch "imPtFirst" and "imPtSecond" for obvious reasons...).
      
      //DEPRECATED ERROR: cv::Vec2f v2Error = mvHomographyInliers[i].m2PixelProjectionJac * (v2m2 - v2Second);
      
      // *************** The NEW ERROR *******************
      // NOTE!!! we now need to project the prediction on the image first.
      
      cv::Vec2f v2p2_pred = pCamera->Project(v2m2_pred); // n.b. this projection will now cache the necessary results for GetProjectionDerivs()...
      // caching the correct projection derivatives now!
      mvHomographyInliers[i].m2PixelProjectionGrad = pCamera->GetProjectionDerivs(); // This derivative is useless for the mapMaker henceforth. 
								                    // I therefore cache it freely here...
      			    
      
      cv::Vec2f v2Error( mvHomographyInliers[i].imPtSecond.x - v2p2_pred[0],
			 mvHomographyInliers[i].imPtSecond.y - v2p2_pred[1] );
      
	avgSqError += v2Error[0] * v2Error[0] + v2Error[1] * v2Error[1];
      // store the error
      vdErrorSquared.push_back( v2Error[0] * v2Error[0] + v2Error[1] * v2Error[1] );
      
      // store the error vector
      vvErrors.push_back(v2Error);
      
      // NOTE!!! Just the  jacobian of H*m1 / ( m1'*H(3,:) ) - NOT The overall Jacobian!!!!
      cv::Matx<float, 2, 9> m29J;
      float m1_dot_h3 = v3m2tild_pred[2]; // caching m1'*H(3, :)
      //float m1_dot_h2 = v3m2tild_pred[1]; // caching m1'*H(2, :)
      //float m1_dot_h1 = v3m2tild_pred[0]; // caching m1'*H(1, :)
      
      // Row #1 : Elements 0-2 ( m1' / (m1'*Hr3) ).
      m29J(0, 0) = v2m1[0] / m1_dot_h3; 
      m29J(0, 1) = v2m1[1] / m1_dot_h3; 
      m29J(0, 2) = 1.0     / m1_dot_h3;
      
      // Row #1 : Elements 3-5 ( zeros(1, 3) ).
      m29J(0, 3) = m29J(0, 4) = m29J(0, 5) = 0;
      
      // Row #1 : Elements 6 - 8 ( (m1'*Hr1) * m1' / (m1'*Hr3)
      m29J(0, 6) = -v2m2_pred[0] * v2m1[0] / m1_dot_h3; 
      m29J(0, 7) = -v2m2_pred[0] * v2m1[1] / m1_dot_h3; 
      m29J(0, 8) = -v2m2_pred[0] * 1.0     / m1_dot_h3;
      
      // 
      // Row2 #2 : Elements 0-2 ( zeros(1, 3) ).
      m29J(1, 0) = m29J(1, 1) = m29J(1, 2) = 0;
      
      // Row #2 : Elements 3-5 ( m1' / (m1'*Hr3) ).
      m29J(1, 3) = v2m1[0] / m1_dot_h3; 
      m29J(1, 4) = v2m1[1] / m1_dot_h3; 
      m29J(1, 5) = 1.0     / m1_dot_h3;
      
      // Row2 #2 : Elements 6 - 8 ( (m1'*Hr2) * m1' / (m1'*Hr3)
      m29J(1, 6) = -v2m2_pred[1] * v2m1[0] / m1_dot_h3; 
      m29J(1, 7) = -v2m2_pred[1] * v2m1[1] / m1_dot_h3; 
      m29J(1, 8) = -v2m2_pred[1] * 1.0     / m1_dot_h3;
      
      
      // store the overall Jacobian (i.e., pixel coordinates wrt to H)
      vmJacobians.push_back(mvHomographyInliers[i].m2PixelProjectionGrad * m29J);
  } 
  
  // Calculate robust sigma:
  vector<double> vdd = vdErrorSquared;
  
  double dSigmaSquared = Tukey::FindSigmaSquared(vdd);
  
  avgSqError /= mvHomographyInliers.size();
  //cout <<"Number of inliers used to optimize homography : "<<mvHomographyInliers.size()<<endl;
  //cout <<"Average error : "<<avgSqError<<endl;
  
  
  // Now successively updating Omega and ksi with the measurement entries
  for(unsigned int i=0; i<mvHomographyInliers.size(); i++) {
    
      float dWeight = Tukey::Weight(vdErrorSquared[i], dSigmaSquared);
      
      m9Omega += dWeight * ( vmJacobians[i].t() * vmJacobians[i] );
      v9ksi += dWeight * ( vmJacobians[i].t() * vvErrors[i] );
  }
  
  // Dh = inv(Omega) * ksi ...
  cv::Matx<float, 9, 1> v9Update;
  cv::solve(m9Omega, v9ksi, v9Update, cv::DECOMP_CHOLESKY);
  
  cv::Matx<float, 3, 3> m3Update;          // create a 3x3 matrix by which to perturb the homography
  m3Update(0, 0) = v9Update(0, 0); m3Update(0, 1) = v9Update(1, 0); m3Update(0, 2) = v9Update(2, 0);
  m3Update(1, 0) = v9Update(3, 0); m3Update(1, 1) = v9Update(4, 0); m3Update(1, 2) = v9Update(5, 0);
  m3Update(2, 0) = v9Update(6, 0); m3Update(2, 1) = v9Update(7, 0); m3Update(2, 2) = v9Update(8, 0);
  
  // update homography and cut-out...
  mm3BestHomography += m3Update;
}

/*
void HomographyInit::RefineHomographyWithInliers()
{
  // creating a 9D GraphSLAM object (weighted least squares) for the refinement of the homography
  // Adding an identity prior (add_prior puts a 1 in all diagonal elements of Omega)
  cv::Matx<float, 9, 9> Omega = cv::Matx<float, 9, 9>::eye(); // Identity for prior 
  cv::Vec<float, 9> ksi(0, 0, 0, 0, 0, 0, 0, 0, 0);           // zeros in the information matrix (recall we are solving for a perturbation)
  
  // The list Jacobians wrt h per datum.
  // NOTE! Jacobians describe pixel-to-pixel correspondences (not Euclidean coordinates)
  vector<cv::Matx<float, 2, 9> > vmJacobians; 
  // reprojection errors in pixels
  vector<cv::Vec2f > vvErrors;          
  vector<double> vdErrorSquared;
  double avgSqError = 0;
  
  for(unsigned int i=0; i<mvHomographyInliers.size(); i++) {
    
      // First, find error.
      cv::Vec2f v2First = mvHomographyInliers[i].v2EucPlaneFirst;
      cv::Vec3f v3Second = mm3BestHomography * CvUtils::backproject(v2First);
      cv::Vec2f v2Second = CvUtils::pproject(v3Second);
      cv::Vec2f v2Second_real = mvHomographyInliers[i].v2EucPlaneSecond;
      cv::Vec2f v2Error = mvHomographyInliers[i].m2PixelProjectionGrad * (v2Second_real - v2Second);
      
      vdErrorSquared.push_back( v2Error[0] * v2Error[0] + v2Error[1] * v2Error[1] );
      avgSqError += v2Error[0] * v2Error[0] + v2Error[1] * v2Error[1];
      vvErrors.push_back(v2Error);
      
      
      
      cv::Matx<float, 2, 9> m29Jacobian(2, 9);
      // ok we really need the normalizer in order for the expressions not to grow out of proportions...
      float dDenominator = v3Second[2];
      
      // Jacobians wrt to the elements of the homography:
      // NOTE! NOTE! I am keeping -for now- old PTAM code for error checking (otherwise I'll have to redo everything on paper...)
      // For x:
      //m29Jacobian[0].slice<0,3>() = unproject(v2First) / dDenominator;
      cv::Vec3f v2First_euc = CvUtils::backproject(v2First); // store here the backprojection of v2First and inline everything else!
      m29Jacobian(0, 0) = v2First_euc[0] / dDenominator; 
      m29Jacobian(0, 1) = v2First_euc[1] / dDenominator; 
      m29Jacobian(0, 2) = v2First_euc[2] / dDenominator;
    
      //m29Jacobian[0].slice<3,3>() = Zeros;
      m29Jacobian(0, 3) = 
      m29Jacobian(0, 4) = 
      m29Jacobian(0, 5) = 0.0;
      
      
      float dNumerator = v3Second[0];
      
      //m29Jacobian[0].slice<6,3>() = -unproject(v2First) * dNumerator / (dDenominator * dDenominator);
      m29Jacobian(0, 6) = -v2First_euc[0] * dNumerator / (dDenominator * dDenominator); 
      m29Jacobian(0, 7) = -v2First_euc[1] * dNumerator / (dDenominator * dDenominator); 
      m29Jacobian(0, 8) = -v2First_euc[2] * dNumerator / (dDenominator * dDenominator);
      
      // For y:
      //m29Jacobian[1].slice<0,3>() = Zeros;
      m29Jacobian(1, 0) = 
      m29Jacobian(1, 1) = 
      m29Jacobian(1, 2) = 0.0;
      
      //m29Jacobian[1].slice<3,3>()  = unproject(v2First) / dDenominator;;
      m29Jacobian(1, 3) = v2First[0] / dDenominator;
      m29Jacobian(1, 4) = v2First[1] / dDenominator;
      m29Jacobian(1, 5) = v2First[2] / dDenominator;
      
      
      dNumerator = v3Second[1];
      //m29Jacobian[1].slice<6,3>() = -unproject(v2First) * dNumerator / (dDenominator * dDenominator);
      m29Jacobian(1, 6) = -v2First_euc[0] * dNumerator / (dDenominator * dDenominator);
      m29Jacobian(1, 7) = -v2First_euc[1] * dNumerator / (dDenominator * dDenominator);
      m29Jacobian(1, 8) = -v2First_euc[2] * dNumerator / (dDenominator * dDenominator);
      
      
      vmJacobians.push_back(mvHomographyInliers[i].m2PixelProjectionGrad * m29Jacobian);
    }
  avgSqError /= mvHomographyInliers.size();
  // Calculate robust sigma:
  vector<double> vdd = vdErrorSquared;
  double dSigmaSquared = Tukey::FindSigmaSquared(vdd);
  cout <<"Average squared error is"<<avgSqError<<endl;
      
  // Now inocrporating the LS system by
  // updating measurements in the information matrix and vector
  for(unsigned int i=0; i<mvHomographyInliers.size(); i++) {
    
      float dWeight = Tukey::Weight(vdErrorSquared[i], dSigmaSquared);
      //wls.add_mJ(vvErrors[i][0], vmJacobians[i][0], dWeight);
      //wls.add_mJ(vvErrors[i][1], vmJacobians[i][1], dWeight);
     
      // George: The original PTAM code does the GraphSLAM updates in a two step process, although this is not necessary. 
      // On the other hand, doing the one-off method may require a couple more matrix multiplication
      // creation and multiplications...
      
  
      //wls.add_mJ(y, cv::Mat(vmJacobians[i]), dWeight * I2);
      Omega += dWeight * vmJacobians[i].t() * vmJacobians[i];
      ksi += dWeight * (vmJacobians[i].t() * vvErrors[i]);
  }
  
  //wls.compute(); // solve!
  
  //cv::Mat_<float> v9Update = wls.get_mu(); // get the solution as a 9x1 matrix
  cv::Matx<float, 9, 1> v9Update;
  cv::solve(Omega, ksi, v9Update, cv::DECOMP_CHOLESKY);
  
  cv::Matx<float, 3, 3> m3Update;          // create a 3x3 matrix by which to perturb the homography
  m3Update(0, 0) = v9Update(0, 0); m3Update(0, 1) = v9Update(1, 0); m3Update(0, 2) = v9Update(2, 0);
  m3Update(1, 0) = v9Update(3, 0); m3Update(1, 1) = v9Update(4, 0); m3Update(1, 2) = v9Update(5, 0);
  m3Update(2, 0) = v9Update(6, 0); m3Update(2, 1) = v9Update(7, 0); m3Update(2, 2) = v9Update(8, 0);
  
  mm3BestHomography += m3Update;
} */


void HomographyInit::BestHomographyFromMatches_MLESAC()
{
  // Not many matches? Don't do ransac, throw them all in a pot and see what comes out.
  if(mvMatches.size() < 10) {
      mm3BestHomography = HomographyFromMatches(mvMatches);
      return;
  }
  
  // Enough matches? Run MLESAC.
  int anIndices[4];
  
  mm3BestHomography = cv::Matx<float, 3, 3>::eye();
  double dBestError = 999999999999999999.9;
  
  // Do 300 MLESAC trials.
  for(int nR = 0; nR < 300 ; nR++) { 
      // sampling four uniquedata indexes to fit a homography
      vector<InitializerMatch> vMinimalMatches;
      for(int i=0; i<4; i++) {
	
	  bool isUnique = false; // indicates unique sample index
	  int n;
	  while(!isUnique) {
	    
	      n = rand() % mvMatches.size();
	      isUnique =true;
	      for(int j=0; j<i && isUnique; j++)
		if(anIndices[j] == n) isUnique = false;
	  }
	  anIndices[i] = n;
	  vMinimalMatches.push_back(mvMatches[n]);
      }
      
      // Compute the homography now...
      cv::Matx<float, 3, 3> m3Homography = HomographyFromMatches(vMinimalMatches);
      
      //..and sum resulting MLESAC score
      double dError = 0.0;
      for(unsigned int i=0; i < mvMatches.size(); i++)
 	dError += MLESACScore(m3Homography, mvMatches[i]);
      
      // update the best homography and error
      if(dError < dBestError) {
	  mm3BestHomography = m3Homography;
	  dBestError = dError;
      }
    }
}

// The following function decomposes a homography to the following:
// 
//    a. The rotation matrix R the rows of which are the second camera frame.
//    b. The translation t = -R*pos where pos is the position of the center camera frame (wrt the first taken as world frame)
//    c. The normal n of the observed (presumably) planar surface. (not necessary here...)
//
// Everything here is an implementation of Faugeras':
//  ' Motion and Structure from Motion in a Piecewise-Planar Environment '
//
//                Paper can be accessed here: https://hal.inria.fr/inria-00075698/document
//
// Perhaps I will be adding a more analytical method for homography decomposition ...
// NOTE: Malis in 2007 has come up with an analytical solution to the problem, but I think it would be much easier
//       to simply eigendecompose H'*H analytically
void HomographyInit::DecomposeHomography() {
  mvDecompositions.clear();
  
  if (CvUtils::M3Det(mm3BestHomography) == 0) {
    cout << "Singular homography! Try again..."<<endl;
    return;
  }
  
  // TODO: Do this analytically!!!!! It is only a 3x3 eigenvalue problem
  cv::Vec3f w;
  cv::Matx<float, 3, 3> U;
  cv::Matx<float, 3, 3> Vt;
  
  
  cv::SVD::compute(mm3BestHomography, w, U, Vt);
  
  float d1 = w[0]; 
  float d2 = w[1]; 
  float d3 = w[2]; 
  
  // 			************************ Taking cases ****************************
  //
  // Case #1: All singular values equal. Then most likely the homography is a pure rotation 99.999999%.
  //	      Alternatively, it could be so that the second camera center is looking at a transparent plane 
  // 	      from behind along the same axis (reflexion of some kind...) and at the same distance 
  // 	      (see Faugeras' paper Figure 4) with t = -2*R*n. 
  if ( d1 - d2 < 10e-6 && d2 - d3 < 10e-6) {
   cout << "Homography is a pure rotation or you somehow transported the camera  across the room! Either way, reconstruction is not possible!"<<endl;
   return;
  }
  
  // Now we need the sign of the determinant of V: (HtH = V*w^2*V')
  float s = CvUtils::M3Det(Vt); // This can be 1 or -1...
  Vt = s * Vt;
  
  // Now remove arbitrary (decomposition by-product) scale from everythying
  d1 /= d2;
  float s1 = d1*d1;
  //float s2 = 1.0; // not necessary so commenting out
  d3 /= d2;
  float s3 = d3 * d3;
  // Remove arbitrary scale from the homography. We will be needing it...
  mm3BestHomography *=  1.0 / d2;
  // now setting d2 to 1.0. Arbitrary scale has been removed
  d2 = 1.0;
  
  
  
  
  // Now, to business:
  
  
  
  
  float alphasq, betasq, alpha, beta;
  
  //
  // Cases #2 and #3: All singular values not equal (the most usual situation - 4 solutions)
  //		      or exactly 2 singular valuies equal (2 solutions)
  //
  
  // Obtaining two oposite vectors on the plane (to which n is normal)
  // See Ma, Soato, Kosecka, et al for the formulas. The idea is to obtain
  // these vectors as a linear combinations of the eigenvectors of HtH that are not 
  // mutually perpendicular to the normal. 
  // alpha and beta
  alphasq = (1 - s3) / (s1 - s3);
  betasq = (s1 - 1)  / (s1 - s3);
  alpha = sqrt(alphasq);
  beta = sqrt(betasq);
    
  
  // So, if u = alpha * v1 + beta * v3 (recall v2 is mutually perpendicular to n and t)
  // then in order for Hu = R*u it must that norm(H*u) = 1; with a little pen - and-paper 
  // we get two solutions (u1 and u2) as follows:
  cv::Vec3f u1 = cv::Vec<float, 3>( alpha * Vt(0, 0) + beta * Vt(2, 0),
				    alpha * Vt(0, 1) + beta * Vt(2, 1),
				    alpha * Vt(0, 2) + beta * Vt(2, 2)
				  );
			   
  cv::Vec3f u2 = cv::Vec<float, 3>( alpha * Vt(0, 0) - beta * Vt(2, 0),
				    alpha * Vt(0, 1) - beta * Vt(2, 1),
				    alpha * Vt(0, 2) - beta * Vt(2, 2)
				  );
  
  // Creating the following triads:
			    //
			    // a.  U1 = [v2, u1, cross(v2, u1)] and W1 = [H*v2, H*u1, cross(H*v2, H*u1)]
			    //
			    // b.  U2 = [v2, u1, cross(v2, u1)] and W2 = [H*v2, H*u1, cross(H*v2, H*u1)]
			    //
			    // and the rotation comes as a triad solution: R1 = W1*U1' and R2 = W2*U2'
			    // of course, the logic here is that vectors v2 u1, v2, u2 get transformed 
			   // by the homography as with the rotation (i.e., H*x = R*x where x = v2, u1. u2 )
    
  cv::Matx<float, 3, 3> H = mm3BestHomography; // NOT a reference; just a shorter name
					       // for the sake of shorter expressions...
    
  // **** U1
  cv::Matx<float, 3, 3> U1;
  // **** W1
  cv::Matx<float, 3, 3> W1;
    
  // *** W2...
  cv::Matx<float, 3, 3> W2;
  
  // **** U2
  cv::Matx<float, 3, 3> U2;
  // if alpha or beta is zero, then the solutions we get with either U1, W1 or U2, W2 arte the same (thus, only 2),
  // so we can drop two (I arbitrarily chose to drop U1, W2 when alpha==0 and U2, W2 whene beta == 0 (could be the other way around of course...)
  if (alphasq > 10e-5) {
    
    // A. U1
    //
    // column 1: U1(:, 0) = Vt(1,:)'
    
    U1(0, 0) = Vt(1, 0); U1(1, 0) = Vt(1, 1); U1(2, 0) = Vt(1, 2);
    
    // column 2: U1(:, 1) = u1
    U1(0, 1) = u1[0]; U1(1, 1) = u1[1]; U1(2, 1) = u1[2];
    
    // column 3: U1(:, 0) x U1(:, 1) (cross product of the above two columns)
    U1(0, 2) = -U1(2, 0) * U1(1, 1) + U1(1, 0) * U1(2, 1);
    U1(1, 2) =  U1(2, 0) * U1(0, 1) - U1(0, 0) * U1(2, 1);
    U1(2, 2) = -U1(1, 0) * U1(0, 1) + U1(0, 0) * U1(1, 1);
    
    // B. W1
    //
    // column 1: W1(:, 0) = H * Vt(1, :)'
    //
    W1(0, 0) =  H(0, 0) * Vt(1, 0) + H(0, 1) * Vt(1, 1) + H(0, 2) * Vt(1, 2); 
    W1(1, 0) =  H(1, 0) * Vt(1, 0) + H(1, 1) * Vt(1, 1) + H(1, 2) * Vt(1, 2); 
    W1(2, 0) =  H(2, 0) * Vt(1, 0) + H(2, 1) * Vt(1, 1) + H(2, 2) * Vt(1, 2); 
    
    // column 2
    W1(0, 1) =  H(0, 0) * u1[0] + H(0, 1) * u1[1] + H(0, 2) * u1[2]; 
    W1(1, 1) =  H(1, 0) * u1[0] + H(1, 1) * u1[1] + H(1, 2) * u1[2]; 
    W1(2, 1) =  H(2, 0) * u1[0] + H(2, 1) * u1[1] + H(2, 2) * u1[2]; 
    
    // column 3 (cross product of the above two columns)
    W1(0, 2) = -W1(2, 0) * W1(1, 1) + W1(1, 0) * W1(2, 1);
    W1(1, 2) =  W1(2, 0) * W1(0, 1) - W1(0, 0) * W1(2, 1);
    W1(2, 2) = -W1(1, 0) * W1(0, 1) + W1(0, 0) * W1(1, 1);
    
     // 1. Homography decomposition #1
    HomographyDecomposition decomposition1;
   
    cv::Matx<float, 3, 3> m3R = W1 * U1.t();
    // n = v2 x u1
    decomposition1.v3n = cv::Vec<float, 3>( -Vt(1, 2) * u1[1] + Vt(1, 1) * u1[2],
					     Vt(1, 2) * u1[0] - Vt(1, 0) * u1[2],
					    -Vt(1, 1) * u1[0] + Vt(1, 0) * u1[1] );
    // (1/d) * t = (H - R) * n   (translation up to scale)
    cv::Vec<float, 3> v3t = (H - m3R) * decomposition1.v3n;
    
    // So Just copying results into the SE3 member of decomposition1...
    decomposition1.se3SecondFromFirst.get_rotation() =  SO3<>( m3R  ) ;
    decomposition1.se3SecondFromFirst.get_translation() = v3t;
    
    // 2. Decomposition #3 (negation of #1)
    HomographyDecomposition decomposition3;
    decomposition3.v3n = -decomposition1.v3n;
    // So Just copying results into the SE3 member
    decomposition3.se3SecondFromFirst.get_rotation() =  SO3<>( m3R  ) ;
    decomposition3.se3SecondFromFirst.get_translation() = -v3t;
    
    mvDecompositions.push_back(decomposition1);
    mvDecompositions.push_back(decomposition3);
    
  }
    
    
    
 if (betasq > 10e-5) {
   // A. U2
   //
   // column 1: U2(:, 0) = Vt(1, :)'
   U2(0, 0) = Vt(1, 0); U2(1, 0) = Vt(1, 1); U2(2, 0) = Vt(1, 2);
    
   // column 2: U2(:, 1) = u2
   U2(0, 1) = u2[0]; U2(1, 1) = u2[1]; U2(2, 1) = u2[2];
   
   // column 3: U(:, 2) = U2(:, 0) x U2(:, 1) (cross product of the above two columns)
   U2(0, 2) = -U2(2, 0) * U2(1, 1) + U2(1, 0) * U2(2, 1);
   U2(1, 2) =  U2(2, 0) * U2(0, 1) - U2(0, 0) * U2(2, 1);
   U2(2, 2) = -U2(1, 0) * U2(0, 1) + U2(0, 0) * U2(1, 1);
    
   // B. W2
   //
   // column 1: H*v2 
   W2(0, 0) =  H(0, 0) * Vt(1, 0) + H(0, 1) * Vt(1, 1) + H(0, 2) * Vt(1, 2); 
   W2(1, 0) =  H(1, 0) * Vt(1, 0) + H(1, 1) * Vt(1, 1) + H(1, 2) * Vt(1, 2); 
   W2(2, 0) =  H(2, 0) * Vt(1, 0) + H(2, 1) * Vt(1, 1) + H(2, 2) * Vt(1, 2); 
    
   // column 2: H*u2
   W2(0, 1) =  H(0, 0) * u2[0] + H(0, 1) * u2[1] + H(0, 2) * u2[2]; 
   W2(1, 1) =  H(1, 0) * u2[0] + H(1, 1) * u2[1] + H(1, 2) * u2[2]; 
   W2(2, 1) =  H(2, 0) * u2[0] + H(2, 1) * u2[1] + H(2, 2) * u2[2]; 
    
   // column 3: (H*v2) x (H*u2) (cross product of the above two columns)
   W2(0, 2) = -W2(2, 0) * W2(1, 1) + W2(1, 0) * W2(2, 1);
   W2(1, 2) =  W2(2, 0) * W2(0, 1) - W2(0, 0) * W2(2, 1);
   W2(2, 2) = -W2(1, 0) * W2(0, 1) + W2(0, 0) * W2(1, 1);

   // 2. Decomposition #2
   HomographyDecomposition decomposition2;
    
   cv::Matx<float, 3, 3> m3R = W2 * U2.t();
   decomposition2.v3n = cv::Vec<float, 3>( -Vt(1, 2) * u2[1] + Vt(1, 1) * u2[2],
				            Vt(1, 2) * u2[0] - Vt(1, 0) * u2[2],
				           -Vt(1, 1) * u2[0] + Vt(1, 0) * u2[1] );
   //decomposition2.v3Tp = (H - decomposition2.m3Rp) * decomposition2.v3n;
   cv::Vec<float, 3> v3t = (H - m3R) * decomposition2.v3n;
   // Just copying results into the SE3 member
   decomposition2.se3SecondFromFirst.get_rotation() =  SO3<>( m3R  ) ;
   decomposition2.se3SecondFromFirst.get_translation() = v3t;
    
   // 2. Decomposition #4 (negation of #2)
   HomographyDecomposition decomposition4;
   decomposition4.v3n = -decomposition2.v3n;
   decomposition4.se3SecondFromFirst.get_rotation() =  SO3<>( decomposition2.se3SecondFromFirst.get_rotation() ) ;
   decomposition4.se3SecondFromFirst.get_translation() = -decomposition2.se3SecondFromFirst.get_translation();
    
   mvDecompositions.push_back(decomposition2);
   mvDecompositions.push_back(decomposition4);
    
  }
    
  
    
}


/*void HomographyInit::DecomposeHomography() {
  
  mvDecompositions.clear();
  
  // TODO: Do this analytically!!!!! It is only a 3x3 eigenvalue problem
  cv::Vec3f w;
  cv::Matx<float, 3, 3> U;
  cv::Matx<float, 3, 3> Vt;
  
  cv::SVD::compute(mm3BestHomography, w, U, Vt);
 
  float d1 = w[0]; 
  float d2 = w[1]; 
  float d3 = w[2]; 
  
  
  // Now we get the sign of the decomposition.
  // Although rare, it maybe so be that one of the two matrices has -1 determinant (reflection of some kind...)
  // and therefore we need to compensate for that (just multiply by -1) later in order to get real rotations in the process...
  // So keep the sign stored... (following paper symbolology, 's' is used). 
  float s = CvUtils::M3Det(U) * CvUtils::M3Det(Vt); // This can be 1 or -1...
  
  // This is the scale of the homography (middle singular value)
  // NOTE!!!! NOT to be confused with the distance of the plane from the origin!!!!
  // Faugeras for some reason switches to the formula dp*Rp + tp*np' when dp = s*d is a GLOBAL scale factor and 
     // it has nothing to do with the distance d
  double dPrime_PM = d2; 
  
  cv::Matx<float, 3, 3> V = Vt.t();
  
  // Before taking cases, remove scale from the homography and take d' = 1.0 so that no harm is done in any case...
  mm3BestHomography *=  1.0/dPrime_PM ;
  // and the singular values are scaled as well...
  d1 /= dPrime_PM;
  d2 = 1.0;
  d3 /= dPrime_PM;
  // so now also d' = d2 = 1.0
  dPrime_PM = 1.0; 
  
  
  int nCase;
  // Now for the decomposition, we take cases as Faugeras does in his paper:
  //
  // Case #1: Sigular values not equal in pairs - 8 solutions.  
  if(d1 != d2 && d2 != d3) nCase = 1;
  else 
    // Case #2: All singular values equal - Undetermined ("partially"); this is a pure rotation
    if( d1 == d2 && d2 == d3) nCase = 3;
    // Case #3: Exactly 2 singular values equal - 4 solutions (implemented here - NOT in PTAM).
  else
    nCase = 2;
  
  // We just 
  if(nCase == 2) {
      cout << "  Homography Initialization failed: Homography impossible to decompose (3 equal singular values)! " << endl;
      return;
  }
  
  double x1_PM;
  double x2;
  double x3_PM;

  // All below deals with the case = 1 .
  // Case 1 implies (d1 != d3 != d2) 
  { // Eq. 12
    x1_PM = sqrt((d1*d1 - d2*d2) / (d1*d1 - d3*d3));
    x2    = 0;
    x3_PM = sqrt((d2*d2 - d3*d3) / (d1*d1 - d3*d3));
  };
  
  double e1[4] = {1.0,-1.0, 1.0,-1.0};
  double e3[4] = {1.0, 1.0, -1.0,-1.0};
    
  
  HomographyDecomposition decomposition;
  cv::Matx<float, 3, 3> m3Rp; // Faugeras' transformed R
  cv::Vec<float, 3> v3Tp;     // Faugeras' transformed t
  cv::Vec3f v3np;             // Faugeras' transformed normal
  cv::Matx<float, 3, 3> I3 = cv::Matx<float, 3, 3>::eye(); // just the 3x3 identity 
  // Case 1, d' > 0:
  for(int signs=0; signs<4; signs++) {
    
      // Eq 13
      m3Rp = I3;
      double dSinTheta = (d1 - d3) * x1_PM * x3_PM * e1[signs] * e3[signs] / d2;
      double dCosTheta = (d1 * x3_PM * x3_PM + d3 * x1_PM * x1_PM) / d2;
      m3Rp(0, 0) = dCosTheta;                
      m3Rp(0, 2) = -dSinTheta;
      m3Rp(2, 0) = dSinTheta;                
      m3Rp(2, 2) = dCosTheta;
      
      // Eq 14
      v3Tp[0] = (d1 - d3) * x1_PM * e1[signs];
      v3Tp[1] = 0.0;
      v3Tp[2] = (d1 - d3) * -x3_PM * e3[signs];
  
      v3np[0] = x1_PM * e1[signs];
      v3np[1] = x2;
      v3np[2] = x3_PM * e3[signs];
      decomposition.v3n = V * v3np;
      decomposition.se3SecondFromFirst.get_rotation() =  SO3<>( s * U *  m3Rp * Vt  ) ;
      decomposition.se3SecondFromFirst.get_translation() = U * v3Tp;
      
      
      mvDecompositions.push_back(decomposition);
    }
  // Case 1, d' < 0:
  for(int signs=0; signs<4; signs++) {
    
      // Eq 15
      m3Rp = -1 * I3;
      double dSinPhi = (d1 + d3) * x1_PM * x3_PM * e1[signs] * e3[signs] / d2;
      double dCosPhi = (d3 * x1_PM * x1_PM - d1 * x3_PM * x3_PM) / d2;
      m3Rp(0, 0) = dCosPhi;                
      m3Rp(0, 2) = dSinPhi;
      m3Rp(2, 0) = dSinPhi;                
      m3Rp(2, 2) = -dCosPhi;
      
      // Eq 16
      v3Tp[0] = (d1 + d3) * x1_PM * e1[signs];
      v3Tp[1] = 0.0;
      v3Tp[2] = (d1 + d3) * x3_PM * e3[signs];

      v3np[0] = x1_PM * e1[signs];
      v3np[1] = x2;
      v3np[2] = x3_PM * e3[signs];
      decomposition.v3n = V * v3np;
      
      
      decomposition.se3SecondFromFirst.get_rotation() =  SO3<>( s * U *  m3Rp * Vt  ) ;
      decomposition.se3SecondFromFirst.get_translation() = U * v3Tp;
      
      mvDecompositions.push_back(decomposition);
    }
  
  
}*/



bool operator <(const HomographyDecomposition lhs, const HomographyDecomposition rhs)
{
  return lhs.nScore < rhs.nScore;
}

static double SampsonusError(cv::Vec<float, 2> &v2Dash, const cv::Matx<float, 3, 3> &m3Essential, cv::Vec<float, 2> &v2) {
  
  cv::Vec3f v3Dash = CvUtils::backproject(v2Dash);
  cv::Vec3f v3 = CvUtils::backproject(v2);  
  
  double dError = v3Dash.dot( m3Essential * v3);
  
  // fv3 = E * v3
  cv::Vec3f fv3( m3Essential(0, 0) * v3[0] + m3Essential(0, 1) * v3[1] + m3Essential(0, 2) * v3[2],
		 m3Essential(1, 0) * v3[0] + m3Essential(1, 1) * v3[1] + m3Essential(1, 2) * v3[2],
		 m3Essential(2, 0) * v3[0] + m3Essential(2, 1) * v3[1] + m3Essential(2, 2) * v3[2]
	       );
		 
  //fTv3Dash = E' * v3Dash;
  cv::Vec3f fTv3Dash( m3Essential(0, 0) * v3Dash[0] + m3Essential(1, 0) * v3Dash[1] + m3Essential(2, 0) * v3Dash[2],
		      m3Essential(0, 1) * v3Dash[0] + m3Essential(1, 1) * v3Dash[1] + m3Essential(2, 1) * v3Dash[2],
		      m3Essential(0, 2) * v3Dash[0] + m3Essential(1, 2) * v3Dash[1] + m3Essential(2, 2) * v3Dash[2]
		    );
  //Vector<2> fv3Slice = fv3.slice<0,2>();
  cv::Vec2f fv3Slice( fv3[0],
		      fv3[1]
		    );
		      
  //Vector<2> fTv3DashSlice = fTv3Dash.slice<0,2>();
  cv::Vec2f fTv3DashSlice( fTv3Dash[0], 
			   fTv3Dash[1] 
			 );
  
  return dError * dError / ( fv3Slice[0] * fv3Slice[0] + fv3Slice[1] * fv3Slice[1] + 
			     fTv3DashSlice[0] * fTv3DashSlice[0] + fTv3DashSlice[1] * fTv3DashSlice[1] );
}


// This function simply tests what's the best homography decomposition 
void HomographyInit::ChooseBestDecomposition()
{
  // now decompositions NEED NOT be 8! But they need to be at least two...
  assert(mvDecompositions.size() >1);
  // TEST #1: Punish homographies that map points out of the screen
  for(unsigned int i=0; i<mvDecompositions.size(); i++) {
     
      HomographyDecomposition &decom = mvDecompositions[i];
      int nPositive = 0;
      for(unsigned int m=0; m<mvHomographyInliers.size(); m++) {
	
	  cv::Vec2f v2 = mvHomographyInliers[m].v2EucPlaneFirst;
	  // Testing for positive depth of the feature
	  double dVisibilityTest = mm3BestHomography(2, 0) * v2[0] + 
				   mm3BestHomography(2, 1) * v2[1] + 
				   mm3BestHomography(2, 2) ;// / decom.d;
	  // increase positive count
	  if(dVisibilityTest > 0.0) nPositive++;
      }
      // now store count negatively for ordering purposes (the most negative will come first)
      decom.nScore = -nPositive;
  }
  // Now, we take only the first 4 homographies
  sort(mvDecompositions.begin(), mvDecompositions.end());
  mvDecompositions.resize(4);
  
  // Testing  prositivity of depth
  for(unsigned int i=0; i<mvDecompositions.size(); i++) {
    
      HomographyDecomposition &decomp = mvDecompositions[i];
      int nPositive = 0;
      for(unsigned int m=0; m<mvHomographyInliers.size(); m++) {
	
	  cv::Vec<float, 3> v3 = CvUtils::backproject(mvHomographyInliers[m].v2EucPlaneFirst);
	  double dVisibilityTest = v3[0] * decomp.v3n[0] + v3[1] * decomp.v3n[1] + v3[2] * decomp.v3n[2];
	  
	  if(dVisibilityTest > 0.0) nPositive++;
      }
      decomp.nScore = -nPositive;
  }
  
  // Now get the best 2 remaining homographies
  sort(mvDecompositions.begin(), mvDecompositions.end());
  mvDecompositions.resize(2);
  
  // According to Faugeras and Lustman, ambiguity exists if the two scores are equal
  // but in practive, better to look at the ratio!
  double dRatio = (double) mvDecompositions[1].nScore / (double) mvDecompositions[0].nScore;

  if(dRatio < 0.9) // no ambiguity! 
    mvDecompositions.erase(mvDecompositions.begin() + 1);
  // two-way ambiguity. We essentially have two distinct decompositions 
  // with more-less the same performance. 
  else {             
    
      double dErrorSquaredLimit  = mdMaxPixelErrorSquared * 4; // recall, "mdMaxPixelErrorSquared" is the squared cutoff bound of MLESAC...
      double adSampsonusScores[2];
      for(int i=0; i<2; i++) {
	
	  cv::Matx<float, 3, 3> m3Essential;
	  cv::Vec<float, 3> v3t =  mvDecompositions[i].se3SecondFromFirst.get_translation();
	  cv::Matx<float, 3, 3> m3R =  mvDecompositions[i].se3SecondFromFirst.get_rotation().get_matrix();
	  // E = [t]x * R
	  for(int j=0; j<3; j++) {
	    
	    m3Essential(0, j) = -v3t[2] * m3R(1, j) + v3t[1] * m3R(2, j);
	    m3Essential(1, j) =  v3t[2] * m3R(0, j) - v3t[0] * m3R(2, j);
	    m3Essential(2, j) = -v3t[1] * m3R(0, j) + v3t[0] * m3R(1, j);
	    
	  }
	  
	  double dSumError = 0;
	  for(unsigned int m=0; m < mvMatches.size(); m++ ) {
	    
	      double d = SampsonusError(mvMatches[m].v2EucPlaneSecond, m3Essential, mvMatches[m].v2EucPlaneFirst);
	      
	      if(d > dErrorSquaredLimit) d = dErrorSquaredLimit;
	      dSumError += d;
	    }
	  
	  adSampsonusScores[i] = dSumError;
	}

      if(adSampsonusScores[0] <= adSampsonusScores[1])
	mvDecompositions.erase(mvDecompositions.begin() + 1);
      else
	mvDecompositions.erase(mvDecompositions.begin());
    }
  cout << "OLE! Just puicked a SE3 from first to second! "<<endl;
}


