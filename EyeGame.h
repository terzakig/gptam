// George Terzakis 2016
//
// University of Portsmouth
//
// Code in this file is practically unaltered from the original 
// PTAM by Klein and Murray (Copyright 2008 Isis Innovation Limited)
//
// EyeGame.h
// Declares the EyeGame class
// EyeGame is a trivial AR app which draws some 3D graphics
// Draws a bunch of 3d eyeballs remniscient of the 
// AVL logo
//
#ifndef __EYEGAME_H
#define __EYEGAME_H


#include "OpenCV.h"
#include "GCVD/SE3.h"
#include "OpenGL.h"

using namespace RigidTransforms;

class EyeGame
{
 public:
  EyeGame();
  void DrawStuff(cv::Vec3f v3CameraPos);
  void Reset();
  void Init();

  
 protected:
  bool mbInitialised;
  void DrawEye();
  void LookAt(int nEye, cv::Vec3f v3, double dRotLimit);
  void MakeShadowTex();
 
  GLuint mnEyeDisplayList;
  GLuint mnShadowTex;
  double mdEyeRadius;
  double mdShadowHalfSize;
  SE3<> ase3WorldFromEye[4];
  int mnFrameCounter;

};


#endif
