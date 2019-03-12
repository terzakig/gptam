//   George Terzakis 2016
//
// University of Portsmouth
//
//

// Initialize SLAM with epipolar Geometry2views
// recover the essential matrix with RANSAC and refine it
// using the classic constraint m2'*E*m1 for iteration 'till convergence;
// E is parametrized with stereographic parameters

#include "EssentialInit.h"
#include "GCVD/Geometry2views.h"

#include <utility>

#include "GCVD/Addedutils.h"
#include "GCVD/SE3.h"
#include "GCVD/Quaternion.h"

#include "MEstimator.h"


using namespace std;


// This function marks a homography match as inliner by simply checking if 
// the first point is projected to a second point with error below the MLESAC threshold...
inline bool EssentialInit::IsEssentialInlier(const EssentialRelativePose &rpose, const InitializerMatch &match) {
  
  // 1. Obtain the normalized Euclidean projections of the correspondences
  cv::Vec<float, 3> v3EucPlaneFirst = CvUtils::backproject(match.v2EucPlaneFirst); 
  cv::Vec<float, 3> v3EucPlaneSecond = CvUtils::backproject(match.v2EucPlaneSecond); 
  
  // obtain projections of u1 and R'*u2 on the plane to which the baseline is normal. 
  cv::Vec<float, 3> p1 = rpose.ProjectFromView1 * v3EucPlaneFirst;
  cv::Vec<float, 3> p2 = rpose.ProjectFromView2 * v3EucPlaneSecond;
  
  // normalize the projections and get the cosine
  double AngleCosError = 1 - CvUtils::normalize(p1).dot( CvUtils::normalize(p2) );
  
  
  // dMaxPixelErrorSquared is the sequared robust cutoff error by Torr and Zisserman
  return (AngleCosError < mdMaxAngleCosError); 
}

inline double EssentialInit::MLESACScore(const EssentialRelativePose &rpose , const InitializerMatch &match) {
  
  // We use the cosine of the angle misalignment between epipolar planes <R*m2, b> and <m1, b> as the intersect the plane
  // which has the baseline as normal and constains the origin (i.e., first camera center)
  // 
  // 1. Obtain the normalized Euclidean projections of the correspondences
  cv::Vec<float, 3> v3EucPlaneFirst = CvUtils::backproject(match.v2EucPlaneFirst); 
  cv::Vec<float, 3> v3EucPlaneSecond = CvUtils::backproject(match.v2EucPlaneSecond); 
  
  // obtain projections of u1 and R'*u2 on the plane to which the baseline is normal. 
  cv::Vec<float, 3> p1 = rpose.ProjectFromView1 * v3EucPlaneFirst;
  cv::Vec<float, 3> p2 = rpose.ProjectFromView2 * v3EucPlaneSecond;
  
  // normalize the projections and get the cosine
  double AngleCosError = 1 - CvUtils::normalize(p1).dot( CvUtils::normalize(p2) );
  
  // Finally, obtain the robust Torr-Zisserman error 
  return AngleCosError > mdMaxAngleCosError ? mdMaxAngleCosError : AngleCosError;
}

bool EssentialInit::Compute(vector<InitializerMatch> vMatches, double dMaxAngleCosError, SE3<> &se3SecondFromFirst) {
  
  mdMaxAngleCosError = dMaxAngleCosError;
  mvMatches = vMatches;
  
  // Find best homography from minimal sets of image matches
  BestEssentialFromMatches_MLESAC();
  
  // get the inliers for the best essential model
  mvEssentialInliers.clear();
  
  cv::Matx<float, 3, 3> R = mrpBestPose.se3.get_rotation().get_matrix();
  cv::Vec<float, 3> b = mrpBestPose.se3.get_translation();
  
  for(unsigned int i = 0; i < mvMatches.size(); i++) {
    // checking if inlier
    if(!IsEssentialInlier(mrpBestPose, mvMatches[i])) continue;
    // checking if it projects correctly
    // projection to the second view
    
    double Z1 = TwoViewGeometry::recontructDepth2Views(vMatches[i].v2EucPlaneFirst[0], vMatches[i].v2EucPlaneFirst[1],
							vMatches[i].v2EucPlaneSecond[0], vMatches[i].v2EucPlaneSecond[1],
							R, 
							b);
    // obtaining the predicted coordinates in the second view frame
    cv::Vec<float, 3> M2 = R.t() * ( Z1 * CvUtils::backproject(vMatches[i].v2EucPlaneFirst) - b );
    cv::Vec<float, 2> m2 = CvUtils::pproject(M2);
    // test against largest radius on the normalized Euclidean plane
    if ( cv::norm(m2) > pCamera->LargestRadiusInImage()  ) {
      //cout<<"DEBUG: -------------------- Found point of out of radius! "<<endl;
      continue;
    }
    // now project to image and check if out-of-bounds
    cv::Vec<float, 2> p2 = pCamera->Project(m2);
    if (p2[1] > pCamera->GetImageSize().height - 1 || p2[1] < 0 || p2[0] > pCamera->GetImageSize().width - 1 || p2[0] < 0) {
      //cout<<"DEUG:------------------------- Found oint out of bounds"<<endl;
      continue;
    }
    
    mvEssentialInliers.push_back(mvMatches[i]);  
  
  }
  
  if (mvEssentialInliers.size() < 10) {
    
    cout <<"DEBUG: Essential matrix inliers too few!"<<endl;
    
    return false;
    // now, keep in mind that if it goes back, there must be some way of unlocking the mapmaker
    // from having few or no inliers (yet a lot of trail matches) over and over again...
  }
    
  // Now refine the R, t on inliners
  // using fast G-N iteration on E = R'*[b]x
  RefineEssentialWithInliers();
  
  
   // Changing the best pose from R'*(M - b) to the classic R*M+t fashion
  mrpBestPose.se3.get_rotation().get_matrix() = mrpBestPose.se3.get_rotation().get_matrix().t();
  mrpBestPose.se3.get_translation() = -mrpBestPose.se3.get_rotation().get_matrix() * mrpBestPose.se3.get_translation();
  
  // storing in the referenced SE3 object
  se3SecondFromFirst = mrpBestPose.se3;
  
  return true;
}

// Simply a Linear Least Squares computation of a 
cv::Matx<float, 3, 3> EssentialInit::EssentialFromMatchesLLS(vector<InitializerMatch> vMatches)
{
  unsigned int nPoints = vMatches.size();
  assert(nPoints >= 8);
  
  // We do the gram-matrix with sums instead. It should be alright because 
  // normalized Euclidean coordinates are small figures in general...
  cv::Matx<double, 9, 9> m9D = cv::Matx<double, 9, 9>::zeros();
  // And a cache for the 1x9 matrix per datum...
  double d1, d2, d3, d4, d5, d6, d7, d8; //  d9 = 1;
  for(unsigned int n=0; n<nPoints; n++) {
      
    
    double x2 = vMatches[n].v2EucPlaneSecond[0];
    double y2 = vMatches[n].v2EucPlaneSecond[1];
      
    double x1 = vMatches[n].v2EucPlaneFirst[0];
    double y1 = vMatches[n].v2EucPlaneFirst[1];
    // filling ONLY the Upper triangle of m9D...
    //
    // get the data matrix first...
    d1 = x2*x1; d2 = x2*y1; d3 = x2; d4 = y2*x1; d5 = y2*y1; d6 = y2; d7 = x1; d8 = y1;  
    
    // 1. Filling the upper triangle of the 9x9 gram-matrix accumulator
    
    m9D(0, 0) += d1*d1; m9D(0, 1) += d1*d2; m9D(0, 2) += d1*d3; m9D(0, 3) += d1*d4; m9D(0, 4) += d1*d5; m9D(0, 5) += d1*d6; m9D(0, 6) += d1*d7; m9D(0, 7) += d1*d8; m9D(0, 8) += d1;
			m9D(1, 1) += d2*d2; m9D(1, 2) += d2*d3; m9D(1, 3) += d2*d4; m9D(1, 4) += d2*d5; m9D(1, 5) += d2*d6; m9D(1, 6) += d2*d7; m9D(1, 7) += d2*d8; m9D(1, 8) += d2;
					    m9D(2, 2) += d3*d3; m9D(2, 3) += d3*d4; m9D(2, 4) += d3*d5; m9D(2, 5) += d3*d6; m9D(2, 6) += d3*d7; m9D(2, 7) += d3*d8; m9D(2, 8) += d3;
								m9D(3, 3) += d4*d4; m9D(3, 4) += d4*d5; m9D(3, 5) += d4*d6; m9D(3, 6) += d4*d7; m9D(3, 7) += d4*d8; m9D(3, 8) += d4;
										    m9D(4, 4) += d5*d5; m9D(4, 5) += d5*d6; m9D(4, 6) += d5*d7; m9D(4, 7) += d5*d8; m9D(4, 8) += d5;
													m9D(5, 5) += d6*d6; m9D(5, 6) += d6*d7; m9D(5, 7) += d6*d8; m9D(5, 8) += d6;
															    m9D(6, 6) += d7*d7; m9D(6, 7) += d7*d8; m9D(6, 8) += d7;
  																		m9D(7, 7) += d8*d8; m9D(7, 8) += d8;
																				    m9D(8, 8) += 1;
  
  }  
  // So now filling-in the gaps:
  m9D(1, 0) = m9D(0, 1);
  m9D(2, 0) = m9D(0, 2); m9D(2, 1) = m9D(1, 2);
  m9D(3, 0) = m9D(0, 3); m9D(3, 1) = m9D(1, 3); m9D(3, 2) = m9D(2, 3);
  m9D(4, 0) = m9D(0, 4); m9D(4, 1) = m9D(1, 4); m9D(4, 2) = m9D(2, 4); m9D(4, 3) = m9D(3, 4);
  m9D(5, 0) = m9D(0, 5); m9D(5, 1) = m9D(1, 5); m9D(5, 2) = m9D(2, 5); m9D(5, 3) = m9D(3, 5); m9D(5, 4) = m9D(4, 5);
  m9D(6, 0) = m9D(0, 6); m9D(6, 1) = m9D(1, 6); m9D(6, 2) = m9D(2, 6); m9D(6, 3) = m9D(3, 6); m9D(6, 4) = m9D(4, 6); m9D(6, 5) = m9D(5, 6);
  m9D(7, 0) = m9D(0, 7); m9D(7, 1) = m9D(1, 7); m9D(7, 2) = m9D(2, 7); m9D(7, 3) = m9D(3, 7); m9D(7, 4) = m9D(4, 7); m9D(7, 5) = m9D(5, 7); m9D(7, 6) = m9D(6, 7);
  m9D(8, 0) = m9D(0, 8); m9D(8, 1) = m9D(1, 8); m9D(8, 2) = m9D(2, 8); m9D(8, 3) = m9D(3, 8); m9D(8, 4) = m9D(4, 8); m9D(8, 5) = m9D(5, 8); m9D(8, 6) = m9D(6, 8); m9D(8, 7) = m9D(7, 8);
  
  
  
  // The right null-space or the last eigenvector of m3D9 gives the linear estimate of the essential matrix ...
  cv::Matx<double, 9, 9> U, Vt;
  cv::Matx<double, 9, 1> w;
  cv::SVD::compute(m9D, w, U, Vt);
  cv::Matx<float, 3, 3> E(3, 3); // the 3x3 homography
  E(0, 0) = Vt(8, 0); E(0, 1) = Vt(8, 1); E(0, 2) = Vt(8, 2);
  E(1, 0) = Vt(8, 3); E(1, 1) = Vt(8, 4); E(1, 2) = Vt(8, 5);
  E(2, 0) = Vt(8, 6); E(2, 1) = Vt(8, 7); E(2, 2) = Vt(8, 8);
  
  // removing arbitrary scale from the homography using the trace of E'*E
  double trEtE = E(0, 0) * E(0, 0) + E(1, 0) * E(1, 0) + E(2, 0) * E(2, 0) +
		 E(0, 1) * E(0, 1) + E(1, 1) * E(1, 1) + E(2, 1) * E(2, 1) +
		 E(0, 2) * E(0, 2) + E(1, 2) * E(1, 2) + E(2, 2) * E(2, 2);
  
  E = sqrt(2.0 / trEtE) * E ;
  
    
  return E;
}


/// This method refines the essential matrix to convergence
// 
//   The cost function employed is the standard:
//
//          f = e(psi, u)' * Q * e(psi, u)
//
// where Q is a precomputed PSD matrix on the data,
// e is the essential matrix as a 9-vector and
// psi, u are the stereographic coordinates of the rotation 
// and baseline respectively. 
//
// I initially thought that I maybe needing robust weights. In this case,
// we would have to lose time in calculating the error and Jacobian point-by-point.
// I think following RANSAC using the m-estimator is an overkill and results seem to be very good so far.
// Initialization is 99.9% successful with very good reconstruction (which reflects in the 
// quality of subsequent camera localization). So this works nicely.
void EssentialInit::RefineEssentialWithInliers(int maxIteration) {
  
  // filling the data matrix
  cv::Matx<double, 9, 9> m9D;
  double d1, d2, d3, d4, d5, d6, d7, d8; //  d9 = 1; 
  for (unsigned int i = 0; i<mvEssentialInliers.size(); i++) {
    
    double x1 = mvEssentialInliers[i].v2EucPlaneFirst[0];
    double y1 = mvEssentialInliers[i].v2EucPlaneFirst[1];
    
    double x2 = mvEssentialInliers[i].v2EucPlaneSecond[0];
    double y2 = mvEssentialInliers[i].v2EucPlaneSecond[1];
    
    // get the data matrix first...
    d1 = x2*x1; d2 = x2*y1; d3 = x2; d4 = y2*x1; d5 = y2*y1; d6 = y2; d7 = x1; d8 = y1;  
    
    // 1. Filling the upper triangle of the 9x9 gram-matrix accumulator
    
    m9D(0, 0) += d1*d1; m9D(0, 1) += d1*d2; m9D(0, 2) += d1*d3; m9D(0, 3) += d1*d4; m9D(0, 4) += d1*d5; m9D(0, 5) += d1*d6; m9D(0, 6) += d1*d7; m9D(0, 7) += d1*d8; m9D(0, 8) += d1;
			m9D(1, 1) += d2*d2; m9D(1, 2) += d2*d3; m9D(1, 3) += d2*d4; m9D(1, 4) += d2*d5; m9D(1, 5) += d2*d6; m9D(1, 6) += d2*d7; m9D(1, 7) += d2*d8; m9D(1, 8) += d2;
					    m9D(2, 2) += d3*d3; m9D(2, 3) += d3*d4; m9D(2, 4) += d3*d5; m9D(2, 5) += d3*d6; m9D(2, 6) += d3*d7; m9D(2, 7) += d3*d8; m9D(2, 8) += d3;
								m9D(3, 3) += d4*d4; m9D(3, 4) += d4*d5; m9D(3, 5) += d4*d6; m9D(3, 6) += d4*d7; m9D(3, 7) += d4*d8; m9D(3, 8) += d4;
										    m9D(4, 4) += d5*d5; m9D(4, 5) += d5*d6; m9D(4, 6) += d5*d7; m9D(4, 7) += d5*d8; m9D(4, 8) += d5;
													m9D(5, 5) += d6*d6; m9D(5, 6) += d6*d7; m9D(5, 7) += d6*d8; m9D(5, 8) += d6;
															    m9D(6, 6) += d7*d7; m9D(6, 7) += d7*d8; m9D(6, 8) += d7;
  																		m9D(7, 7) += d8*d8; m9D(7, 8) += d8;
																				    m9D(8, 8) += 1;
  
  } 
  
  // Filling-in the gaps:
  m9D(1, 0) = m9D(0, 1);
  m9D(2, 0) = m9D(0, 2); m9D(2, 1) = m9D(1, 2);
  m9D(3, 0) = m9D(0, 3); m9D(3, 1) = m9D(1, 3); m9D(3, 2) = m9D(2, 3);
  m9D(4, 0) = m9D(0, 4); m9D(4, 1) = m9D(1, 4); m9D(4, 2) = m9D(2, 4); m9D(4, 3) = m9D(3, 4);
  m9D(5, 0) = m9D(0, 5); m9D(5, 1) = m9D(1, 5); m9D(5, 2) = m9D(2, 5); m9D(5, 3) = m9D(3, 5); m9D(5, 4) = m9D(4, 5);
  m9D(6, 0) = m9D(0, 6); m9D(6, 1) = m9D(1, 6); m9D(6, 2) = m9D(2, 6); m9D(6, 3) = m9D(3, 6); m9D(6, 4) = m9D(4, 6); m9D(6, 5) = m9D(5, 6);
  m9D(7, 0) = m9D(0, 7); m9D(7, 1) = m9D(1, 7); m9D(7, 2) = m9D(2, 7); m9D(7, 3) = m9D(3, 7); m9D(7, 4) = m9D(4, 7); m9D(7, 5) = m9D(5, 7); m9D(7, 6) = m9D(6, 7);
  m9D(8, 0) = m9D(0, 8); m9D(8, 1) = m9D(1, 8); m9D(8, 2) = m9D(2, 8); m9D(8, 3) = m9D(3, 8); m9D(8, 4) = m9D(4, 8); m9D(8, 5) = m9D(5, 8); m9D(8, 6) = m9D(6, 8); m9D(8, 7) = m9D(7, 8);
  
  // ********* Prepping the LM iteration here **************
  
  cv::Vec<float, 3> b = mrpBestPose.se3.get_translation();
  
  
  cv::Matx<float, 3, 3> Bx;
  Bx(0, 0) =  0;    Bx(0, 1) = -b[2]; Bx(0, 2) =  b[1];
  Bx(1, 0) =  b[2]; Bx(1, 1) = 0;     Bx(1, 2) = -b[0];
  Bx(2, 0) = -b[1]; Bx(2, 1) = b[0];  Bx(2, 2) = 0;
  
   // create the attitude quaternion from the best pose rotation matrix
  Quaternion<> q(mrpBestPose.se3.get_rotation().get_matrix().val );
  cv::Vec<float, 3> sop;  q.MRPCoordinates(sop.val);
  cv::Matx<float, 3, 3> R; q.RotationMatrix(R.val);
  cv::Matx<float, 3, 3> E = R.t() * Bx;
  cv::Vec<float, 9> v9e( E(0, 0), E(0, 1), E(0, 2), E(1, 0), E(1, 1), E(1, 2), E(2, 0), E(2, 1), E(2, 2) );
  double lambda = 0.0001;
  double lambdaFactor = 2.0;
  double tol = 10e-8;
  double errorsq = (m9D * v9e).dot(v9e); 
  cout<<"DEBUG: Initial squared error : "<<errorsq<<endl;
  double prevErrorsq = errorsq;
  double minErrorsq = errorsq;
  
  
  cv::Matx<float, 5, 5> m5Omega = cv::Matx<float, 5, 5>::eye();
  cv::Vec<float, 5> v5ksi(0, 0, 0, 0, 0 ); 
  
  cv::Vec<float, 5> mu(sop[0], sop[1], sop[2], b[0] / ( 1 + b[2] ), b[1] / (1 + b[2] ) );
  cv::Vec<float, 5> v5Update;
  cv::Matx<float, 9, 5> m95J;
  cv::Matx<float, 5, 9> m59JtD;
  
  const cv::Matx<float, 5, 5> I5 = cv::Matx<float, 5, 5>::eye();
  
  bool stop = false;
  int step = 0;
  
  while (!stop) {
    // obtain the Jacobian
    ComputeEssentialJac(m9D, q, b, m95J);
    // solve!
    m59JtD = m95J.t() * m9D;
    m5Omega = m59JtD * m95J + lambda * I5;
    v5ksi = -m59JtD * v9e;
    // temporary solution
    cv::solve(m5Omega, v5ksi, v5Update, cv::DECOMP_CHOLESKY);
    cv::Vec<float, 5> mutemp = mu + v5Update;
    Quaternion<> qtmp = Quaternion<>::CreateFromMRPs(mutemp.val); // the method will only access the first 3 SOP parameters
    cv::Vec<float, 3> btmp( 2 * mutemp[3], 2 * mutemp[4] , 1 - mutemp[3]*mutemp[3] - mutemp[4]*mutemp[4]); btmp = (1.0 / (1.0 + mutemp[3]*mutemp[3] + mutemp[4]*mutemp[4] ) ) * btmp;  
    cv::Matx<float, 3, 3> Bxtmp = TwoViewGeometry::CrossPoductMatrix(btmp);
    cv::Matx<float, 3, 3> Rtmp; qtmp.RotationMatrix(Rtmp.val); 
    cv::Matx<float, 3, 3> Etmp = Rtmp.t() * Bxtmp;
    cv::Vec<float, 9> etmp(Etmp(0, 0), Etmp(0, 1), Etmp(0, 2), Etmp(1, 0), Etmp(1, 1), Etmp(1, 2), Etmp(2, 0), Etmp(2, 1), Etmp(2, 2) );
    errorsq = (m9D * etmp).dot(etmp);
    cout <<"DEBUG: new squared error : "<<errorsq<<endl;
    if (errorsq < minErrorsq ) {
	
      minErrorsq = errorsq;
      R = Rtmp;
      b = btmp;
      E = Etmp;
      Bx = Bxtmp;
      v9e = etmp;
      
      // divide lambda by the lambdaFactor
      lambdaFactor = 2.0;
      lambda *= 0.3;
    } else {

      lambda = lambda * lambdaFactor;
      lambdaFactor = lambdaFactor * 2;
    }

	if (errorsq < tol || step > maxIteration || fabs(prevErrorsq - errorsq) < 10e-12 ) stop = true;
	else {
	  
	  prevErrorsq = errorsq;
	  step++;
	}
	
  } // end L-M while-loop
  
 cout <<"DEBUG: FInal Squared error : "<<minErrorsq<<endl;
 mrpBestPose.se3.get_rotation().get_matrix() = R;
 mrpBestPose.se3.get_translation() = b;
  
}


// The following method is an accurate refinement iteration on the 
// misalignment between epipolar planes defined by a pair of correspondences
// and the baseline.
// The cost function is:
//
//		   /  u2'*R'*(I3 - b*b')*u1  \
//  cos(phi) = sum | ------------------------ |
//                 \  sqrt(1 - (b'*u1)^2)    /
//
// where R is relative orientation, b is the unit-norm baseline and I3 is the identity
//
// The cost function must be MAXIMIZED and is sensitive to greater-than-180 degree epipolar misalignement.
void EssentialInit::RefineEssentialWithInliers1(int maxIteration) {
  
  
  // ********* Prepping coordinate ascent **************
  
  float gamma = 0.0001; // step size
  
  cv::Vec<float, 3> b = mrpBestPose.se3.get_translation();
  double k = b[0];
  double l = b[1];
  if ( fabs( b[2] + 1) < 0.0001 ) {
    k /= 0.0001;
    l /= 0.0001;
  } else {
    k /= 1.0 + b[2];
    l /= 1.0 + b[2];
  }
  
  // create the attitude quaternion from the best pose rotation matrix
  cv::Matx<float, 3, 3> R = mrpBestPose.se3.get_rotation().get_matrix();
  Quaternion<> q( R.val );
  cv::Vec<float, 3> sop; q.MRPCoordinates(sop.val);
  
  
  // stop if average cost drops below this value
  double cost_tol = 1 - cos(0.0001);
  // stop if cost doesnt change above this value
  double dcost_tol = 10e-9;
  // current and previous cost
  double prev_cost = -99999999999,
	 cur_cost = -9999999;
  
  
  
  // the current solution estimate
  cv::Vec<float, 5> mu(sop[0], sop[1], sop[2], k, l);
  // cache for the 1x5 Jacobian of the cost function
  cv::Vec<float, 5> J;
  
  
  int step = 0;
  
  do {
    // obtain the Jacobian
    cur_cost = 1 - ComputeMisalignementCostAndJac(q, b, J);
    
    cout<<"DEBUG: Current cost (misalignement cosine) : "<<cur_cost<<endl;
    
    
    if (cur_cost < cost_tol || fabs(cur_cost - prev_cost) < dcost_tol) break;
    prev_cost = cur_cost; 
    
    mu += gamma * J;
    // updating quaternion, rotation and baseline
    q = Quaternion<>::CreateFromMRPs<>( mu.val ); // will access ONLY the first 3 elements anyway...
    q.RotationMatrix(R.val);
    float n2 = mu[3] * mu[3] + mu[4] * mu[4];
    float inv_one_plus_n2 = 1.0 / ( 1 + n2 );
    b[0] = 2 * mu[3] * inv_one_plus_n2 ; b[1] = 2 * mu[4] * inv_one_plus_n2; b[2] = ( 1 - n2 ) * inv_one_plus_n2;
     
    step++;
	
  } while  (step <  100); // end gradient ascent while-loop
  
  
 cout <<"DEBUG: FInal cost (misalignement) : "<<cur_cost<<endl;
 mrpBestPose.se3.get_rotation().get_matrix() = R;
 mrpBestPose.se3.get_translation() = b;
  exit(1);
}

// This function computes the overall misalignement angular cost and  the Jacobian 
// using the currently stored essential inliers
double EssentialInit::ComputeMisalignementCostAndJac(Quaternion<> &q,  // only referenced, because the rotation matrix method normalizes it 
						const cv::Vec<float, 3> &v3b,
						cv::Vec<float, 5> &m15J
 					      ) {
  // Creating a projector P = I3 - b*b' matrix (we'll have to do this every time we get a new baseline... )
  cv::Matx<float, 3, 3> P;
  P(0, 0) =  1 - v3b[0]*v3b[0];      P(0, 1) =  -v3b[0]*v3b[1];         P(0, 2) =  -v3b[0]*v3b[2];
  P(1, 0) =  -v3b[1]*v3b[0];         P(1, 1) =  1 - v3b[1]*v3b[1];      P(1, 2) =  -v3b[1]*v3b[2];
  P(2, 0) =  -v3b[2]*v3b[0];         P(2, 1) =  -v3b[2]*v3b[1];         P(2, 2) = 1 - v3b[2]*v3b[2];
  
  // obtain the rotation matrix. Redundant, but not a big deal...
  cv::Matx<float, 3, 3> R; q.RotationMatrix(R.val);
  
  // 1. Obtain the rotation derivatives
  cv::Matx<float, 3, 3> dRdpsi0; q.RotationJacWRTMRPs(0, dRdpsi0.val); // rotation derivative wrt psi0
  cv::Matx<float, 3, 3> dRdpsi1; q.RotationJacWRTMRPs(1, dRdpsi1.val); // rotation derivative wrt psi1
  cv::Matx<float, 3, 3> dRdpsi2; q.RotationJacWRTMRPs(2, dRdpsi2.val); // rotation derivative wrt psi2
  
  // 2. Obtain the derivative of the baseline
  // a. derivative wrt 1st seterographic coordinate (k)
  cv::Vec<float, 3> dbdk( -v3b[0] * v3b[0] + 1 + v3b[2], 
			  -v3b[0] * v3b[1],
			  -v3b[0] * (1 + v3b[2]) );
  
  // b. derivative of baseline wrt 2nd seterographic coordinate (l)
  cv::Vec<float, 3> dbdl( -v3b[1] * v3b[0], 
			  -v3b[1] * v3b[1] + 1 + v3b[2],
			  -v3b[1] * (1 + v3b[2]) );
  // 2. Derivative of the projector wrt to the baseline SOP parameters k and l
  cv::Matx<float, 3, 3> dPdk;
  dPdk(0, 0) = -(dbdk[0] * v3b[0] + v3b[0] * dbdk[0]); dPdk(0, 1) = -(dbdk[0] * v3b[1] + v3b[0] * dbdk[1]); dPdk(0, 2) = -(dbdk[0] * v3b[2] + v3b[0] * dbdk[2]);
  dPdk(1, 0) = -(dbdk[1] * v3b[0] + v3b[1] * dbdk[0]); dPdk(1, 1) = -(dbdk[1] * v3b[1] + v3b[1] * dbdk[1]); dPdk(1, 2) = -(dbdk[1] * v3b[2] + v3b[1] * dbdk[2]);
  dPdk(2, 0) = -(dbdk[2] * v3b[0] + v3b[2] * dbdk[0]); dPdk(2, 1) = -(dbdk[2] * v3b[1] + v3b[2] * dbdk[1]); dPdk(2, 2) = -(dbdk[2] * v3b[2] + v3b[2] * dbdk[2]);
  
  cv::Matx<float, 3, 3> dPdl;
  dPdl(0, 0) = -(dbdl[0] * v3b[0] + v3b[0] * dbdl[0]); dPdl(0, 1) = -(dbdl[0] * v3b[1] + v3b[0] * dbdl[1]); dPdl(0, 2) = -(dbdl[0] * v3b[2] + v3b[0] * dbdl[2]);
  dPdl(1, 0) = -(dbdl[1] * v3b[0] + v3b[1] * dbdl[0]); dPdl(1, 1) = -(dbdl[1] * v3b[1] + v3b[1] * dbdl[1]); dPdl(1, 2) = -(dbdl[1] * v3b[2] + v3b[1] * dbdl[2]);
  dPdl(2, 0) = -(dbdl[2] * v3b[0] + v3b[2] * dbdl[0]); dPdl(2, 1) = -(dbdl[2] * v3b[1] + v3b[2] * dbdl[1]); dPdl(2, 2) = -(dbdl[2] * v3b[2] + v3b[2] * dbdl[2]);
  
  
  // ok ready. Now we need to go through the entire dataset and sum up the Jacobians and costs
  m15J[0] = m15J[1] = m15J[2] = m15J[3] = m15J[4] = 0;
  double Cost = 0;
  for (unsigned int i = 0; i<mvEssentialInliers.size(); i++) {
    
    // cache the Euclidean coordinates of m1
    double x1 = mvEssentialInliers[i].v2EucPlaneFirst[0];
    double y1 = mvEssentialInliers[i].v2EucPlaneFirst[1];
    double z1 = 1.0;
    // normalizing the coordinates 
    double inv_norm1 = 1.0 / sqrt( x1 * x1 + y1 * y1 + z1 * z1 );
    x1 *= inv_norm1; y1 *= inv_norm1; z1 *= inv_norm1;
    
    // coordinates of m2
    double x2 = mvEssentialInliers[i].v2EucPlaneSecond[0];
    double y2 = mvEssentialInliers[i].v2EucPlaneSecond[1];
    double z2 = 1.0;
    
    // obtaining second-point coordinates in world frame (applying rotation R)
    double x2r = R(0, 0) * x2 + R(0, 1) * y2 + R(0, 2) * z2;
    double y2r = R(1, 0) * x2 + R(1, 1) * y2 + R(1, 2) * z2;
    double z2r = R(2, 0) * x2 + R(2, 1) * y2 + R(2, 2) * z2;
    
    // normalizing the rotated m2 now
    double inv_norm2r = 1.0 / sqrt( x2r * x2r + y2r * y2r + z2r * z2r );
    x2r *= inv_norm2r; y2r *= inv_norm2r; z2r *= inv_norm2r;
    // normalizing m2
    double inv_norm2 = 1.0 / sqrt( x2*x2 + y2*y2 + z2 * z2 );
    x2 *= inv_norm2; y2 *= inv_norm2; z2 *= inv_norm2;
    
    
    // compute the numerator (R*u2) * P * u1 
    double numerator = ( x2r * P(0, 0) + y2r * P(1, 0) + z2r * P(2, 0) ) * x1 +
		       ( x2r * P(0, 1) + y2r * P(1, 1) + z2r * P(2, 1) ) * y1 +
		       ( x2r * P(0, 2) + y2r * P(1, 2) + z2r * P(2, 2) ) * z1;
    double dot_u1_b = x1 * v3b[0] + y1 * v3b[1] + z1 * v3b[2];
    double inv_denominator = 1.0 / sqrt( 1 - dot_u1_b * dot_u1_b  );
    
    // 1. Derivatives wrt stereographic parameters of rotation
    //
    //       We first need to compute shortcuts for the multiplied coordinates of m2 with the derivatives of R wrt psi0, psi1, and psi2:
    double x2_psi0 = dRdpsi0(0, 0) * x2 + dRdpsi0(0, 1) * y2 + dRdpsi0(0, 2) * z2, 
	   y2_psi0 = dRdpsi0(1, 0) * x2 + dRdpsi0(1, 1) * y2 + dRdpsi0(1, 2) * z2,
	   z2_psi0 = dRdpsi0(2, 0) * x2 + dRdpsi0(2, 1) * y2 + dRdpsi0(2, 2) * z2;
    double dfdpsi0 = ( ( x2_psi0 * P(0, 0) + y2_psi0 * P(1, 0) + z2_psi0 * P(2, 0) ) * x1 +
		       ( x2_psi0 * P(0, 1) + y2_psi0 * P(1, 1) + z2_psi0 * P(2, 1) ) * y1 +
		       ( x2_psi0 * P(0, 2) + y2_psi0 * P(1, 2) + z2_psi0 * P(2, 2) ) * z1 ) * inv_denominator;
    double x2_psi1 = dRdpsi1(0, 0) * x2 + dRdpsi1(0, 1) * y2 + dRdpsi1(0, 2) * z2, 
	   y2_psi1 = dRdpsi1(1, 0) * x2 + dRdpsi1(1, 1) * y2 + dRdpsi1(1, 2) * z2,
	   z2_psi1 = dRdpsi1(2, 0) * x2 + dRdpsi1(2, 1) * y2 + dRdpsi1(2, 2) * z2;
    
    double dfdpsi1 = ( ( x2_psi1 * P(0, 0) + y2_psi1 * P(1, 0) + z2_psi1 * P(2, 0) ) * x1 +
		       ( x2_psi1 * P(0, 1) + y2_psi1 * P(1, 1) + z2_psi1 * P(2, 1) ) * y1 +
		       ( x2_psi1 * P(0, 2) + y2_psi1 * P(1, 2) + z2_psi1 * P(2, 2) ) * z1 ) * inv_denominator;
    
    double x2_psi2 = dRdpsi2(0, 0) * x2 + dRdpsi2(0, 1) * y2 + dRdpsi2(0, 2) * z2, 
	   y2_psi2 = dRdpsi2(1, 0) * x2 + dRdpsi2(1, 1) * y2 + dRdpsi2(1, 2) * z2,
	   z2_psi2 = dRdpsi2(2, 0) * x2 + dRdpsi2(2, 1) * y2 + dRdpsi2(2, 2) * z2;
    
    double dfdpsi2 = ( ( x2_psi2 * P(0, 0) + y2_psi2 * P(1, 0) + z2_psi2 * P(2, 0) ) * x1 +
		       ( x2_psi2 * P(0, 1) + y2_psi2 * P(1, 1) + z2_psi2 * P(2, 1) ) * y1 +
		       ( x2_psi2 * P(0, 2) + y2_psi2 * P(1, 2) + z2_psi2 * P(2, 2) ) * z1 ) * inv_denominator;
    
    // 2. Derivatives wrt stereographic parameters of the baseline
    // The dot product of 
    double dfdk = ( ( x2r * dPdk(0, 0) + y2r * dPdk(1, 0) + z2r * dPdk(2, 0) ) * x1 + 
		    ( x2r * dPdk(0, 1) + y2r * dPdk(1, 1) + z2r * dPdk(2, 1) ) * y1 +
		    ( x2r * dPdk(0, 2) + y2r * dPdk(1, 2) + z2r * dPdk(2, 2) ) * x1 ) * inv_denominator + 
		  numerator * dot_u1_b *( dbdk[0] * x1 + dbdk[1] * y1 + dbdk[2] * z1 ) * inv_denominator * inv_denominator * inv_denominator;
    
    double dfdl = ( ( x2r * dPdl(0, 0) + y2r * dPdl(1, 0) + z2r * dPdl(2, 0) ) * x1 + 
		    ( x2r * dPdl(0, 1) + y2r * dPdl(1, 1) + z2r * dPdl(2, 1) ) * y1 +
		    ( x2r * dPdl(0, 2) + y2r * dPdl(1, 2) + z2r * dPdl(2, 2) ) * x1 ) * inv_denominator + 
		  numerator * dot_u1_b * ( dbdl[0] * x1 + dbdl[1] * y1 + dbdl[2] * z1 ) * inv_denominator * inv_denominator * inv_denominator;
    
    // 3. accumulating the derivatives
    m15J[0] += dfdpsi0; m15J[1] += dfdpsi1; m15J[2] += dfdpsi2; m15J[3] += dfdk; m15J[4] += dfdl;
    Cost += numerator * inv_denominator;
  }

  return Cost / mvEssentialInliers.size();
}


/// Returns the Jacobian of the classic cost function e'*D*e 
/// where D is the 9x9 accumulator data matrix
void EssentialInit::ComputeEssentialJac(const cv::Matx<float, 9, 9> &m9D, // the 9x9 data matrix
				        Quaternion<> &q, // relative orientation (must be referenced because the quaternion 
							   // normalizes itself when the rotation matrix method is invoked)
					const cv::Vec<float, 3> &v3b,       // baseline
					cv::Matx<float, 9, 5> &m95J      // reference to the Jacobian matrix
					) 
{
  
  // 1. Obtain the rotation derivatives
  cv::Matx<float, 3, 3> dRdpsi0; q.RotationJacWRTMRPs(0, dRdpsi0.val); // rotation derivative wrt psi0
  cv::Matx<float, 3, 3> dRdpsi1; q.RotationJacWRTMRPs(1, dRdpsi1.val); // rotation derivative wrt psi1
  cv::Matx<float, 3, 3> dRdpsi2; q.RotationJacWRTMRPs(2, dRdpsi2.val); // rotation derivative wrt psi2
  
  // 2. The derivatives of the baseline skew-symmetrix matrix 
  // 	We compute as:
  //	G0 * dbdk(0) + G1 * dbdk(1) + G2 * dbdk(2)
  // where G0, G1, G2 are the Lie generators and dbdk is the SOP derivative wrt k (l is the other parameter)
  cv::Matx<float, 3, 3> dBxdk;
  dBxdk(0, 0) = 0;                       dBxdk(0, 1) =  v3b[0] * (1 + v3b[2]); dBxdk(0, 2) = -v3b[0] * v3b[1];
  dBxdk(1, 0) = -dBxdk(0, 1);            dBxdk(1, 1) =  0;                     dBxdk(1, 2) =  v3b[0] * v3b[0] - 1 - v3b[2];
  dBxdk(2, 0) = -dBxdk(0, 2);            dBxdk(2, 1) = -dBxdk(1, 2);           dBxdk(2, 2) = 0;
  
  cv::Matx<float, 3, 3> dBxdl;
  dBxdl(0, 0) = 0;                       dBxdl(0, 1) =  v3b[1] * (1 + v3b[2]); dBxdl(0, 2) = -v3b[1] * v3b[1] + v3b[2] + 1;
  dBxdl(1, 0) = -dBxdl(0, 1);            dBxdl(1, 1) =  0;                     dBxdl(1, 2) =  v3b[0] * v3b[1] ;
  dBxdl(2, 0) = -dBxdl(0, 2);            dBxdl(2, 1) = -dBxdl(1, 2);           dBxdl(2, 2) = 0;
  
  // 3. Get the essential matrix derivatives as matrix products
  cv::Matx<float, 3,3 > Bx;
  Bx(0, 0) = Bx(1, 1) = Bx(2, 2) = 0;
  Bx(0, 1) = -v3b[2]; Bx(0, 2) = v3b[1]; 
  Bx(1, 0) = -Bx(0, 1); Bx(1, 2) = -v3b[0];
  Bx(2, 0) = -Bx(0, 2); Bx(2, 1) = -Bx(1, 2);
  
  cv::Matx<float, 3, 3> dEdpsi0 = dRdpsi0.t() * Bx;
  cv::Matx<float, 3, 3> dEdpsi1 = dRdpsi1.t() * Bx;
  cv::Matx<float, 3, 3> dEdpsi2 = dRdpsi2.t() * Bx;
  
  cv::Matx<float, 3, 3> R; q.RotationMatrix(R.val);
  
  cv::Matx<float, 3, 3> dEdk   = R.t() * dBxdk;
  cv::Matx<float, 3, 3> dEdl   = R.t() * dBxdl;
  
  // And FINALLY, the Jacobian matrix...
  m95J(0, 0) = dEdpsi0(0, 0); m95J(0, 1) = dEdpsi0(0, 1); m95J(0, 2) = dEdpsi0(0, 2);
  m95J(0, 3) = dEdpsi0(1, 0); m95J(0, 4) = dEdpsi0(1, 1); m95J(0, 5) = dEdpsi0(1, 2);
  m95J(0, 6) = dEdpsi0(2, 0); m95J(0, 7) = dEdpsi0(2, 1); m95J(0, 8) = dEdpsi0(2, 2);
  
  m95J(1, 0) = dEdpsi1(0, 0); m95J(1, 1) = dEdpsi1(0, 1); m95J(1, 2) = dEdpsi1(0, 2);
  m95J(1, 3) = dEdpsi1(1, 0); m95J(1, 4) = dEdpsi1(1, 1); m95J(1, 5) = dEdpsi1(1, 2);
  m95J(1, 6) = dEdpsi1(2, 0); m95J(1, 7) = dEdpsi1(2, 1); m95J(1, 8) = dEdpsi1(2, 2);
  
  m95J(2, 0) = dEdpsi2(0, 0); m95J(2, 1) = dEdpsi2(0, 1); m95J(2, 2) = dEdpsi2(0, 2);
  m95J(2, 3) = dEdpsi2(1, 0); m95J(2, 4) = dEdpsi2(1, 1); m95J(2, 5) = dEdpsi2(1, 2);
  m95J(2, 6) = dEdpsi2(2, 0); m95J(2, 7) = dEdpsi2(2, 1); m95J(2, 8) = dEdpsi2(2, 2);
  
  m95J(3, 0) = dEdk(0, 0); m95J(3, 1) = dEdk(0, 1); m95J(3, 2) = dEdk(0, 2);
  m95J(3, 3) = dEdk(1, 0); m95J(3, 4) = dEdk(1, 1); m95J(3, 5) = dEdk(1, 2);
  m95J(3, 6) = dEdk(2, 0); m95J(3, 7) = dEdk(2, 1); m95J(3, 8) = dEdk(2, 2);
  
  m95J(4, 0) = dEdl(0, 0); m95J(4, 1) = dEdl(0, 1); m95J(4, 2) = dEdl(0, 2);
  m95J(4, 3) = dEdl(1, 0); m95J(4, 4) = dEdl(1, 1); m95J(4, 5) = dEdl(1, 2);
  m95J(4, 6) = dEdl(2, 0); m95J(4, 7) = dEdl(2, 1); m95J(4, 8) = dEdl(2, 2);
  
  
  
}



void EssentialInit::BestEssentialFromMatches_MLESAC() {
  
  // If not many matches, just try to estimate a single essential matrix.
  // And we take it from there ...
  if(mvMatches.size() < 20) {
      
    cv::Matx<float, 3,3 > m3E = EssentialFromMatchesLLS(mvMatches);
    
    mrpBestPose = PickCorrectRelativePose(m3E, mvMatches);
    
    return;
  }
  
  // Otherwise do the RANSAC...
  int anIndices[8];
  
  double dBestError = 999999999999999999.9;
  
  // Do 300 MLESAC trials.
  for(int nR = 0; nR < 300 ; nR++) { 
      // sampling 8 indexes to fit a model
      vector<InitializerMatch> vMinimalMatches;
      for(int i = 0; i < 8; i++) {
	
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
      
      // Compute an essential matrix with ordinary least squares
      // from the minimal set
      cv::Matx<float, 3, 3> E = EssentialFromMatchesLLS(vMinimalMatches);
      
      EssentialRelativePose pose = PickCorrectRelativePose(E, mvMatches);
      
      //Now obtaining the RANSAC score
      double dError = 0.0;
      for(unsigned int i=0; i < mvMatches.size(); i++)
 	dError += MLESACScore(pose, mvMatches[i]);
      //cout <<"DEBUG: Average RANSAC error for the "<<nR<<" RANSAC iteration: <<"<<dError / mvMatches.size()<<endl;
      // update the best homography and error
      if(dError < dBestError) {
	  mrpBestPose = pose;
	  dBestError = dError;
      }
    }
    //cout << "DEBUG: Best pose from MLESAC : "<<mrpBestPose.se3<<endl;
}

// This function simply picks the best relative pose out of the 4 possible
// with respect to the points
// NOTE!!! Returns in standard fashion rotation - baseline fashion 
//         (i.e., R contains the second camera frame column-wise,
//         and b is the baseline vector - also position of the second camera frame wrt first.
EssentialRelativePose EssentialInit::PickCorrectRelativePose(cv::Matx<float, 3, 3> &m3E, vector<InitializerMatch> &vMatches) {
  
  cv::Matx<float, 3, 3> m3R1, m3R2;
  cv::Vec<float, 3> v3b;
  // Get the 4 possible transformations from the essential matrix
  // NOTE: Scale is removed from m3E and singular value constraits are imposed
  //       if necessary.
  TwoViewGeometry::extractSE3FromEssentialMatrix(m3E, m3R1, m3R2, v3b);
  
  SE3<> poses[4];
  poses[0] = SE3<>(m3R1,  v3b);   
  poses[1] = SE3<>(m3R1, -v3b);  
  poses[2] = SE3<>(m3R2,  v3b);   
  poses[3] = SE3<>(m3R2, -v3b);  
  
  //  cache for a PSD matrix (upper triangle only)
  double C[6];
  // scores for each transformation (need to minimize)
  int Scores[4];
  Scores[0] = Scores[1] = Scores[2] = Scores[3] = 0;
  
  int bestScore = 30000; // big enough
  int bestScoreIndex = -1;
  
  for (unsigned int i = 0; i < 4; i++) {
   
    cv::Matx<float, 3,3 > R = poses[i].get_rotation().get_matrix();
    cv::Vec<float, 3> b = poses[i].get_translation();
    
    Scores[i] = 0;
    // now go through the points...
    for (unsigned int j = 0; j < vMatches.size(); j++) {
      // ontaining the normalized Euclidean correspondences
      float x1 = vMatches[i].v2EucPlaneFirst[0];
      float y1 = vMatches[i].v2EucPlaneFirst[1];
      
      float x2 = vMatches[i].v2EucPlaneSecond[0];
      float y2 = vMatches[i].v2EucPlaneSecond[1];
    
      // now need to obtain the depth :
      // We use the formula:
      // 
      //	   m1' * R * Q * R' * b	
      //    Z =   ----------------------
     //            m1' * R * Q * R' * m1
      // where
      //          Q = (m2*1z' - eye(3))'*(m2*1z' - eye(3))
    
      // *********** computing C = R * Q * R' (only upper triangle) *****************
      C[0] = ( R(0, 0) - R(0, 2) * x2 ) * ( R(0, 0) - R(0, 2) * x2 ) + ( R(0, 1) - R(0, 2) * y2 ) * ( R(0, 1) - R(0, 2) * y2 );
      
      C[1] = ( R(0, 0) - R(0, 2) * x2 ) * ( R(1, 0) - R(1, 2) * x2 ) + ( R(0, 1) - R(0, 2) * y2 ) * ( R(1, 1) - R(1, 2) * y2 );  
      
      C[2] = ( R(0, 0) - R(0, 2) * x2 ) * ( R(2, 0) - R(2, 2) * x2 ) + ( R(0, 1) - R(0, 2) * y2 ) * ( R(2, 1) - R(2, 2) * y2 );
     
      C[3] = ( R(1, 0) - R(1, 2) * x2 ) * ( R(1, 0) - R(1, 2) * x2 ) + ( R(1, 1) - R(1, 2) * y2 ) * ( R(1, 1) - R(1, 2) * y2 );
      
      C[4] = ( R(1, 0) - R(1, 2) * x2 ) * ( R(2, 0) - R(2, 2) * x2 ) + ( R(1, 1) - R(1, 2) * y2 ) * ( R(2, 1) - R(2, 2) * y2 );
     
      C[5] = ( R(2, 0) - R(2, 2) * x2 ) * ( R(2, 0) - R(2, 2) * x2 ) + ( R(2, 1) - R(2, 2) * y2 ) * ( R(2, 1) - R(2, 2) * y2 );
     
      // now obtain the numerator and denominator that yield the depth fraction 
      double denominator = x1 * ( x1 * C[0] + y1 * C[1] + C[2] ) + 
	 		   y1 * ( x1 * C[1] + y1 * C[3] + C[4] ) +
			          x1 * C[2] + y1 * C[4] + C[5];
				 
      double numerator = b[0] * ( x1 * C[0] + y1 * C[1] + C[2] ) + 
			 b[1] * ( x1 * C[1] + y1 * C[3] + C[4] ) +
			 b[2] * ( x1 * C[2] + y1 * C[4] + C[5] );
      
      if ( fabs(denominator) < 10e-6 ) {
       
        Scores[i]++;
        continue;
      }
      
      // depth in the first (source) view
      double Z1 = numerator / denominator;
      // depth in the second view: Z2 = R(:,3)' * (Z1*m1 - b) 
      double Z2 =  R(0, 2) * ( Z1 * x1  - b[0] ) + 
		   R(1, 2) * ( Z1 * y1  - b[1] ) + 
		   R(2, 2) * ( Z1 * 1.0 - b[2] );
     
      if (Z1 < 0 || Z2 < 0) {
       
        Scores[i]++;
        continue;
      }
     
    }
    // update best
    if ( bestScore > Scores[i] ) {
	
      bestScore = Scores[i];
      bestScoreIndex = i;
    }
     
  }
  
  // Ok, we have the best score index, so we construct the SE3 object
  EssentialRelativePose bestRelPose;
  // cache the projector from view1
  cv::Vec<float, 3> b = poses[bestScoreIndex].get_translation();
  bestRelPose.ProjectFromView1(0, 0) = 1 - b[0] * b[0]; bestRelPose.ProjectFromView1(0, 1) = -b[0] * b[1]; bestRelPose.ProjectFromView1(0, 2) = -b[0] * b[2];
  bestRelPose.ProjectFromView1(1, 0) = -b[1] * b[0];    bestRelPose.ProjectFromView1(1, 1) = 1 - b[1] * b[1];  bestRelPose.ProjectFromView1(1, 2) = -b[1] * b[2];
  bestRelPose.ProjectFromView1(2, 0) = -b[2] * b[0];    bestRelPose.ProjectFromView1(2, 1) = -b[2] * b[1];  bestRelPose.ProjectFromView1(2, 2) = 1 -b[2] * b[2];
  
  bestRelPose.ProjectFromView2 = bestRelPose.ProjectFromView1 * poses[bestScoreIndex].get_rotation().get_matrix();
  
  bestRelPose.se3 = poses[bestScoreIndex];
  bestRelPose.nScore = Scores[bestScoreIndex]; // this will NOT have to be negated in the process. 
  
  return bestRelPose;
  
}


// Need this for sorting candidate transformations...
bool operator <(const EssentialRelativePose lhs, const EssentialRelativePose rhs) {
  
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


