
#include "VideoSource.h"

#include "Persistence/instances.h"


#include <iostream>
#include <sstream>

using namespace std;




// For the time being, I am implementing webcam live capture.... All being well, more will follow...



VideoSource::VideoSource(int camera_index) 
{

  std::cout << "  Initiating capture device (whatever it is)..." << std::endl;

  camera_index_ = camera_index;
  pcap = new VideoCapture(camera_index_); // by device number
  
  
  if(!pcap->isOpened()) {
    cerr << "Cannot open default capture device. Exiting... " << endl;
    exit(-1);
  }

  std::cout << "  Now capturing...." << std::endl;
  // obtaining the capture size
  int width = (int)pcap->get(cv::CAP_PROP_FRAME_WIDTH);
  int height = (int)pcap->get(cv::CAP_PROP_FRAME_HEIGHT);
  mirSize = cv::Size2i(width, height);
  cout << " Screen size (width , height) : " << width << " , " <<height <<endl;
};

cv::Size2i VideoSource::getSize()
{ 
  return mirSize;
};



void VideoSource::GetAndFillFrameBWandRGB(cv::Mat_<uchar> &imBW, cv::Mat &imRGB)
{
  if ( !pcap->grab() ) {
    cout << " Could not even grab the first frame! exiting..." << endl;
    exit(-1);
  }
    
  cv::Mat capFrame;
  pcap->retrieve(capFrame);
  /*cv::namedWindow("framed");
  cv::imshow("framed", capFrame);
  cv::waitKey(-1);*/
  capFrame.copyTo(imRGB);
  //imRGB = frame.clone(); // deep copy (in BGR!!!! We may need to do the conversion in the following line instead)
  //cv::cvtColor(frame, imRGB, CV_BGR2RGB); // We 'll see about this...
  imBW.create(imRGB.rows, imRGB.cols);

  cv::cvtColor(imRGB, imBW, cv::COLOR_BGR2GRAY); // conversion from BGR (OpenCV default) to grayscale
  
}


