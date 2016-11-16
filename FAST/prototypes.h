#ifndef FAST_PROTOTYPES_H
#define FAST_PROTOTYPES_H

#include <vector>

#include "../OpenCV.h"

namespace FAST
{
	using namespace std;
	void fast_corner_detect_plain_7(const cv::Mat_<uchar> &i, std::vector<cv::Point2i> &corners, int b);
	int fast_corner_score_7(const uchar* cache_0, const int offset[], int b);
	void fast_corner_detect_plain_8(const cv::Mat_<uchar> &i, std::vector<cv::Point2i> &corners, int b);
	int fast_corner_score_8(const uchar* cache_0, const int offset[], int b);
	void fast_corner_detect_plain_9(const cv::Mat_<uchar> &i, vector<cv::Point2i> &corners, int b);
	int fast_corner_score_9(const uchar* cache_0, const int offset[], int b);
	
    void fast_corner_detect_plain_10(const cv::Mat_<uchar> &i, vector<cv::Point2i> &corners, int b);
    int fast_corner_score_10(const uchar* cache_0, const int offset[], int b);
    
   void fast_corner_detect_plain_11(const cv::Mat_<uchar> &i, vector<cv::Point2i> &corners, int b);
   int fast_corner_score_11(const uchar* cache_0, const int offset[], int b);
   
    void fast_corner_detect_plain_12(const cv::Mat_<uchar> &i, vector<cv::Point2i> &corners, int b);
    int fast_corner_score_12(const uchar* cache_0, const int offset[], int b);
}


#endif