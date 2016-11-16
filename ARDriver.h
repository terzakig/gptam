// ************ George Terzakis 2016
// ******** University of Portsmouth

// ARDriver.h
// This file declares the ARDriver class
// This code is based on the original PTAM code by Klein and murrary (Isis Innovation Limited)


// ARDriver provides basic graphics services for drawing augmented
// graphics. It manages the OpenGL setup and the camera's radial
// distortion so that real and distorted virtual graphics can be
// properly blended.
//
#ifndef __AR_Driver_H
#define __AR_Driver_H

#include "GCVD/SE3.h"
#include "ATANCamera.h"
#include "GLWindow2.h"
#include "EyeGame.h"

#include "OpenGL.h"
#include "OpenCV.h"


using namespace std;


class ARDriver
{
 public:
  ARDriver(const ATANCamera &cam, GLWindow2 &glw);
  void Render(cv::Mat &imFrame, SE3<> se3CamFromWorld);
  void Reset();
  void Init();
 protected:
  ATANCamera mCamera;
  GLWindow2 &mGLWindow;
  void DrawFadingGrid();
  void MakeFrameBuffer();
  void DrawFBBackGround();
  void DrawDistortedFB();
  void SetFrustum();
  
  // Texture stuff:
  GLuint mnFrameBuffer;
  GLuint mnFrameBufferTex;
  GLuint mnFrameTex;
  
  int mnCounter;
  cv::Size2i mirFBSize;
  cv::Size2i mirFrameSize;
  SE3<> mse3;
  bool mbInitialised;

  // Eyeballs:
  EyeGame mGame;
};
#endif
