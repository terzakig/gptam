#ifndef CVD_NONMAX_SUPPRESSION_H
#define CVD_NONMAX_SUPPRESSION_H

#include <vector>


#include "../OpenCV.h"



namespace FAST
{
	/**Perform nonmaximal suppression on a set of features, in a 3 by 3 window.
	   The test is strict: a point must be greater than its neighbours.
	@param corners The corner locations
	@param scores  The corners' scores
	@param max_corners The locally maximal corners.
	*/
	void nonmax_suppression_strict(const std::vector<cv::Point2i> &corners, const std::vector<int>& scores, std::vector<cv::Point2i> &nmax_corners);
	/**Perform nonmaximal suppression on a set of features, in a 3 by 3 window.
	   The test is non-strict: a point must be at least as large as its neighbours.
	@param corners The corner locations
	@param scores  The corners' scores
	@param max_corners The locally maximal corners.
	*/
	void nonmax_suppression(const std::vector<cv::Point2i> &corners, const std::vector<int>& scores, std::vector<cv::Point2i>& nmax_corners);


	/**Perform nonmaximal suppression on a set of features, in a 3 by 3 window.
	   Non strict.
	@param corners The corner locations
	@param scores  The corners' scores
	@param max_corners The locally maximal corners, and their scores.
	*/
	void nonmax_suppression_with_scores(const std::vector<cv::Point2i>& corners, const std::vector<int>& scores, std::vector<std::pair<cv::Point2i,int> > &max_corners);

}

#endif