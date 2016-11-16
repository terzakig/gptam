// -*- c++ *--
// Copyright 2008 Isis Innovation Limited
//
// VideoSource.h
// Declares the VideoSource class
// 
// This is a very simple class to provide video input; this can be
// replaced with whatever form of video input that is needed.  It
// should open the video input on construction, and provide two
// function calls after construction: Size() must return the video
// format as an ImageRef, and GetAndFillFrameBWandRGB should wait for
// a new frame and then overwrite the passed-as-reference images with
// GreyScale and Colour versions of the new frame.

#include "OpenCV.h"

using namespace cv;

struct VideoSourceData;

class VideoSource
{
 public:
  VideoSource();
  
  void GetAndFillFrameBWandRGB(cv::Mat_<uchar> &imBW, cv::Mat &imRGB);
  
  cv::Size2i getSize();
  
 private:
  cv::VideoCapture *pcap;
  
  cv::Size2i mirSize;
};
