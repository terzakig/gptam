#include "fast_corner.h"
#include "nonmax_suppression.h"
#include "prototypes.h"
//#include "cvd/config.h"

//#include <cvd/byte.h>
//#include <cvd/image.h>

//using namespace CVD;
using namespace std;


namespace FAST
{

const cv::Point2i fast_pixel_ring[16] =
{
	cv::Point2i(0, 3),
	cv::Point2i(1, 3),
	cv::Point2i(2, 2),
	cv::Point2i(3, 1),
	cv::Point2i(3, 0),
	cv::Point2i(3, -1),
	cv::Point2i(2, -2),
	cv::Point2i(1, -3),
	cv::Point2i(0, -3),
	cv::Point2i(-1, -3),
	cv::Point2i(-2, -2),
	cv::Point2i(-3, -1),
	cv::Point2i(-3, 0),
	cv::Point2i(-3, 1),
	cv::Point2i(-2, 2),
	cv::Point2i(-1, 3)
};

struct CornerPositive { inline static int sub(int a, int b) { return a-b; } };
struct CornerNegative { inline static int sub(int a, int b) { return b-a; } };


int old_style_corner_score(const cv::Mat_<uchar> &im, cv::Point2i c, const int *pointer_dir, int barrier)
{
	//The score for a positive feature is sum of the difference between the pixels
	//and the barrier if the difference is positive. Negative is similar.
	//The score is the max of those two.
	//
	// B = {x | x = points on the Bresenham circle around c}
	// Sp = { I(x) - t | x E B , I(x) - t > 0 }
	// Sn = { t - I(x) | x E B, t - I(x) > 0}
	//
	// Score = max sum(Sp), sum(Sn)

	//const byte* imp = &im[c];
	const uchar* imp = im.ptr<uchar>(c.y, c.x);
	
	int cb = *imp + barrier;
	int c_b = *imp - barrier;
	int sp=0, sn = 0;

	for(int i=0; i<16; i++)
	{
		int p = imp[pointer_dir[i]];

		if(p > cb)
			sp += p-cb;
		else if(p < c_b)
			sn += c_b-p;
	}
	
	if(sp > sn)
		return sp;
	else 
		return sn;
}

void compute_fast_score_old(const cv::Mat_<uchar> &im, const vector<cv::Point2i> &corners, int barrier, vector<int> &scores)
{
	int	pointer_dir[16];
	int stride = (int)im.step;
	for(int i=0; i < 16; i++)
		pointer_dir[i] = fast_pixel_ring[i].x + fast_pixel_ring[i].y * stride;

	scores.resize(corners.size());

	for(unsigned int i=0; i < corners.size(); i++)
		scores[i] = old_style_corner_score(im, corners[i], pointer_dir, barrier);
}



void fast_nonmax(const cv::Mat_<uchar> &im, const vector<cv::Point2i> &corners, int barrier, vector<cv::Point2i> &max_corners)
{
	vector<int> scores;
	compute_fast_score_old(im, corners, barrier, scores);
	nonmax_suppression(corners, scores, max_corners);
}

void fast_nonmax_with_scores(const cv::Mat_<uchar> &im, const vector<cv::Point2i> &corners, int barrier, vector<pair<cv::Point2i, int> > &max_corners)
{
	vector<int> scores;
	compute_fast_score_old(im, corners, barrier, scores);
	nonmax_suppression_with_scores(corners, scores, max_corners);
}

}