// **********************************************
//		George Terzakis 2016 
//
// 	     University of Portsmouth 
//
// The code in this file is based on the original Parallel tracking and mapping (PTAM) 
// by klein and Murray (Copyright 2008 Isis Innovation Limited)

#include "ShiTomasi.h"
#include <math.h>

#include "OpenCV.h"

double FindShiTomasiScoreAtPoint(cv::Mat_<uchar> &image, int nHalfBoxSize, cv::Point2i irCenter) {
  
  double dXX = 0;
  double dYY = 0;
  double dXY = 0;

  cv::Point2i irStart = irCenter - cv::Point2i(nHalfBoxSize, nHalfBoxSize);
  cv::Point2i irEnd = irCenter + cv::Point2i(nHalfBoxSize, nHalfBoxSize);

  int r, c;
  for(r = irStart.y; r <= irEnd.y; r++)
    for(c = irStart.x; c <= irEnd.x; c++) {

      double dx = image(r, c + 1) - image(r , c - 1);
      double dy = image(r + 1, c) - image(r - 1, c);
      dXX += dx*dx;
      dYY += dy*dy;
      dXY += dx*dy;
    }

  int nPixels = (irEnd.x - irStart.x + 1) * (irEnd.y - irStart.y + 1);
  dXX = dXX / (2.0 * nPixels);
  dYY = dYY / (2.0 * nPixels);
  dXY = dXY / (2.0 * nPixels);

  // Find and return smaller eigenvalue:
  return 0.5 * (dXX + dYY - sqrt( (dXX + dYY) * (dXX + dYY) - 4 * (dXX * dYY - dXY * dXY) ));
}



