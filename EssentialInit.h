// George Terzakis 2016
//
// University of Portsmouth
//


//  
// Another way of bootsrapping SLAM ; with Epipolar geometry.

// We do not estimate the fundamental matrix, because the distortion model is non-linear
// so we rosprt to a RANSAC based (MLESAC) method as if we were dealing with an essential matrix
// and then we parametrize R and b and do a bit of linear refinements.


#ifndef __ESSENTIAL_INIT_H
#define __ESSENTIAL_INIT_H

#include "ATANCamera.h"
#include "GCVD/SE3.h"
#include "Initializer.h"
#include "OpenCV.h"

#include "GCVD/Quaternion.h"

#include <vector>





using namespace RigidTransforms;

// A struct to store epipolar matches
/*struct EssentialMatch : public Initializer
{
  cv::Vec<float, 2> v2EucPlaneFirst; // first view normalized Euclidean coordinates
  cv::Vec<float, 2> v2EucPlaneSecond; // second view normalized Euclidean coordinates
  					      
};*/

// Store the rigid transformation corresponding to relative pose
// recovered from an essential matrix
struct EssentialRelativePose
{  
  // A matrix that projects vectors from view #1 onto 
  // the plane that contains the origin of view #1 and is orthogonal to the baseline 
  cv::Matx<float, 3, 3> ProjectFromView1;
  // A matrix that projects vectors in view #2 onto
  // the plane that contains the origin of view #1  and is orthogonal to the baseline  
  cv::Matx<float, 3, 3> ProjectFromView2;
  
  SE3<> se3; // The rigid transformation in fashion Rotation - baseline
	     // points in the first view to points in the second as m2 = R' * ( Z1 * p1 - b) / ( R(:, 3)' * (Z1*m1 - b) )
    
  int nScore;        // A multi-faceted goodness score...

  
};

class EssentialInit : Initializer {
  
public:
  // The RANSAC threshold for the epipolar misalignement set to 3 degrees . This is the suggested Torr-Zisserman cutoff bound for the error
  static constexpr double RANSAC_DEFAULT_THRESHOLD_BOUND = 1 - cos(2.0 * M_PI / 180);
  
  bool Compute(std::vector<InitializerMatch> vMatches, double dMaxAngleCosError, SE3<> &se3SecondCameraPose);
  EssentialInit(ATANCamera* pcam) : Initializer(pcam) {};

protected:
  
  cv::Matx<float, 3, 3> EssentialFromMatchesLLS(std::vector<InitializerMatch> vMatches);
  void BestEssentialFromMatches_MLESAC();
  
  EssentialRelativePose PickCorrectRelativePose( cv::Matx<float, 3, 3> &m3E, vector<InitializerMatch> &vMatches );
  
  void ComputeEssentialJac(const cv::Matx<float, 9, 9> &m9D, // the 9x9 data matrix
			   Quaternion<> &q,            // relative orientation
			   const cv::Vec<float, 3> &v3b,       // baseline
			   cv::Matx<float, 9, 5> &v95J        // The Jacobian
			  );
  double ComputeMisalignementCostAndJac(Quaternion<> &q,	// Quaternion is only references
								// because the rotation matrix method will have to normalize it
					const cv::Vec<float, 3> &v3b,
					cv::Vec<float, 5> &m15J
 					);
  
  void ChooseBestDecomposition();
  void RefineEssentialWithInliers(int maxIteration = 30);
  void RefineEssentialWithInliers1(int maxIteration = 30);
  
  bool IsEssentialInlier(const EssentialRelativePose &rpose, const InitializerMatch &match);
  double MLESACScore(const EssentialRelativePose &rpose, const InitializerMatch &match);
  
  double mdMaxAngleCosError;          // This is angular error between epipolar planes!
  EssentialRelativePose mrpBestPose;  // 
  std::vector<InitializerMatch> mvMatches;
  std::vector<InitializerMatch> mvEssentialInliers;
  std::vector<EssentialRelativePose> mvRelPoses;
};



#endif
