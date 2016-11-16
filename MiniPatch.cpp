// George Terzakis 2016
//
// University of Portsmouth
//
// Code based on PTAM by Klein and Murray (Copyright 2008 Isis Innovation Limited)

#include "MiniPatch.h"


using namespace std;
using namespace cv;

// Scoring function. This is just a sum of squared differences!
inline int MiniPatch::SSDAtPoint(cv::Mat_<uchar> &im, const cv::Point2i &ir)
{
  if(!CvUtils::in_image_with_border(ir.y, ir.x, im, mnHalfPatchSize, mnHalfPatchSize ) ) return mnMaxSSD + 1;
  
  cv::Point2i irImgBase(ir.x - mnHalfPatchSize, ir.y - mnHalfPatchSize);
  
  int nRows = mimOrigPatch.rows;
  int nCols = mimOrigPatch.cols;
  uchar *imagepointer;
  uchar *templatepointer;
  int nDiff;
  int nSumSqDiff = 0;
  for(int nRow = 0; nRow < nRows; nRow++) {
      
    imagepointer = im.ptr<uchar>(irImgBase.y + nRow, irImgBase.x);  
    templatepointer = mimOrigPatch.ptr<uchar>(nRow);
      
    for(int nCol = 0; nCol < nCols; nCol++) {
	
      nDiff = imagepointer[nCol] - templatepointer[nCol];
	nSumSqDiff += nDiff * nDiff;
    }
  }
  
  return nSumSqDiff;
}

// Find a patch by searching at FAST corners in an input image
// If available, a row-corner LUT is used to speed up search through the
// FAST corners
bool MiniPatch::FindPatch(cv::Point2i &irPos, 
			  cv::Mat_<uchar> &im, 
			  int nRange, 
			  vector<cv::Point2i> &vCorners,
			  std::vector<int> *pvRowLUT)
{
  cv::Point2i irCenter = irPos;
  cv::Point2i irBest;
  int nBestSSD = mnMaxSSD + 1; // se the best square difference to something not acceptable (abovw the maximum allowable difference)
  cv::Point2i irBBoxTL = cv::Point2i(irPos.x - nRange, irPos.y - nRange);
  cv::Point2i irBBoxBR = cv::Point2i(irPos.x + nRange, irPos.y + nRange);
  vector<cv::Point2i>::iterator i;
  // first we need to find where on earth are the FAST corners in the
  // region from [posx - nrange, posx + nrange] x [posy - nrange, posy + nrange]
  // So, if no lookuo table exists, we simply need to loop through the corners until
  // we find a corner with bopth coorindates inside the search region above.
  if(!pvRowLUT) {
    
      for(i = vCorners.begin(); i!=vCorners.end(); i++)
	if(i->y >= irBBoxTL.y) break;
  }
  // if we have a LookUp Table, then we simply start from the 
  // point indexed by the row of the left top corner pf the search region
  else {
      int nTopRow = irBBoxTL.y;
      if(nTopRow < 0) nTopRow = 0; // if below 0 then start from the first (0) row.
      
      // same thing in case the top left corner  is beyond the last row
      if(nTopRow >= (int) pvRowLUT->size()) nTopRow = (int) pvRowLUT->size() - 1;
      // set the iterator at the lookup index
      i = vCorners.begin() + (*pvRowLUT)[nTopRow];
  }
  // And now brute-force go through the remaining FAST corners in the list 
  // and do SSD with the ones that are in the region
  for(; i!=vCorners.end(); i++) {
    
      if(i->x < irBBoxTL.x  || i->x > irBBoxBR.x) continue;
      if(i->y > irBBoxBR.y) break;
      int nSSD = SSDAtPoint(im, *i);
      
      if(nSSD < nBestSSD) {
	  
	irBest = *i;
	nBestSSD = nSSD;
      }
  } // end for-loop that finds the best match
  // if we actually found something better than the maximum difference,
  // then return the match 
  if(nBestSSD < mnMaxSSD) {
    
      irPos = irBest;
      return true;
  }
  else
    return false;
}

// Just copy the patch from an input image
void MiniPatch::SampleFromImage(cv::Point2i irPos, cv::Mat_<uchar> &im)
{
  assert(CvUtils::in_image_with_border(irPos.y, irPos.x, im, mnHalfPatchSize, mnHalfPatchSize ) 
		  && "irPos out of bounds when minipatch sampling!!! That should have been taken care of earlier!" );
  
  // full patch size (odd number to account for the middle)
  cv::Size2i irPatchSize( 2 * mnHalfPatchSize + 1 , 2 * mnHalfPatchSize + 1);
  
  // Top-left corner of the patch to be copied into mimOrigpatch
  cv::Point2i TLCorner(irPos.x - mnHalfPatchSize, irPos.y - mnHalfPatchSize );
  // And the lower - right corner of ther region to be lifted in the image
  cv::Point2i LRCorner(TLCorner.x + irPatchSize.width, TLCorner.y + irPatchSize.height);
  
  cv::Mat_<uchar> tempPatch = im( cv::Range(TLCorner.y, LRCorner.y), 
				  cv::Range(TLCorner.x, LRCorner.x) );
  // copyTo() should do the necessary allocation and sizing
  tempPatch.copyTo(mimOrigPatch);
  
  
}

// Static members
int MiniPatch::mnHalfPatchSize = 4;
int MiniPatch::mnRange = 10;
int MiniPatch::mnMaxSSD = 9999;
