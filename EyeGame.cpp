// George Terzakis 2016
//
// University of Portsmouth
//
// This Code is functionally unaltered from 
// the original PTAM by Klein and Murray (Copyright 2008 Isis Innovation Limited)

#include "EyeGame.h"

#include "GCVD/GLHelpers.h"


EyeGame::EyeGame()
{
  mdEyeRadius = 0.1;
  mdShadowHalfSize = 2.5 * mdEyeRadius;
  mbInitialised = false;
}

void EyeGame::DrawStuff(cv::Vec3f v3CameraPos)
{
  if(!mbInitialised) Init();

  mnFrameCounter ++;

  glDisable(GL_BLEND);
  glEnable(GL_CULL_FACE);
  glFrontFace(GL_CW);
  glEnable(GL_DEPTH_TEST);
  glDepthFunc(GL_LEQUAL);
  glEnable(GL_LIGHTING); 
  glEnable(GL_LIGHT0);
  glEnable(GL_NORMALIZE);
  glEnable(GL_COLOR_MATERIAL);
  
  GLfloat af[4]; 
  af[0]=0.5; af[1]=0.5; af[2]=0.5; af[3]=1.0;
  glLightfv(GL_LIGHT0, GL_AMBIENT, af);
  glLightfv(GL_LIGHT0, GL_DIFFUSE, af);
  af[0]=1.0; af[1]=0.0; af[2]=1.0; af[3]=0.0;
  glLightfv(GL_LIGHT0, GL_POSITION, af);
  af[0]=1.0; af[1]=1.0; af[2]=1.0; af[3]=1.0;
  glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, af);
  glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, 50.0);
  
  glMatrixMode(GL_MODELVIEW);
  
  for(int i=0; i<4; i++)
    {
      if(mnFrameCounter < 100) {
	cv::Vec<float, 3> v3Look( ( i < 2 ? -1 : 1 )*(mnFrameCounter < 50 ? -1 : 1) * -0.4 , 
				       -0.1, 
					1 );
	
	LookAt( i, 500.0 * v3Look ,  0.05 );
      
      }
      else
	LookAt(i, v3CameraPos, 0.02 );
      
      glLoadIdentity();
      GLXInterface::glMultMatrix(ase3WorldFromEye[i]);
      glScaled(mdEyeRadius, mdEyeRadius, mdEyeRadius);
      glCallList(mnEyeDisplayList);
    }
  
  glDisable(GL_LIGHTING);
  
  glLoadIdentity();
  glEnable(GL_TEXTURE_2D);
  glBindTexture(GL_TEXTURE_2D, mnShadowTex);
  glEnable(GL_BLEND);
  glColor4f(0,0,0,0.5);
  glBegin(GL_QUADS);
  glTexCoord2f(0,0);
  glVertex2d(-mdShadowHalfSize, -mdShadowHalfSize);
  glTexCoord2f(0,1);
  glVertex2d(-mdShadowHalfSize,  mdShadowHalfSize);
  glTexCoord2f(1,1);
  glVertex2d( mdShadowHalfSize,  mdShadowHalfSize);
  glTexCoord2f(1,0);
  glVertex2d( mdShadowHalfSize, -mdShadowHalfSize);
  glEnd();
  glDisable(GL_TEXTURE_2D);
  glDisable(GL_DEPTH_TEST);
  glDisable(GL_CULL_FACE);
}


void EyeGame::Reset()
{
  for(int i=0; i<4; i++)
    ase3WorldFromEye[i] = SE3<>();

  
  ase3WorldFromEye[0].get_translation()[0] = -mdEyeRadius;
  ase3WorldFromEye[1].get_translation()[0] = mdEyeRadius;
  ase3WorldFromEye[2].get_translation()[0] = -mdEyeRadius;
  ase3WorldFromEye[3].get_translation()[0] = mdEyeRadius;

  ase3WorldFromEye[0].get_translation()[1] = -mdEyeRadius;
  ase3WorldFromEye[1].get_translation()[1] = -mdEyeRadius;
  ase3WorldFromEye[2].get_translation()[1] = mdEyeRadius;
  ase3WorldFromEye[3].get_translation()[1] = mdEyeRadius;

  ase3WorldFromEye[0].get_translation()[2] = mdEyeRadius;
  ase3WorldFromEye[1].get_translation()[2] = mdEyeRadius;
  ase3WorldFromEye[2].get_translation()[2] = mdEyeRadius;
  ase3WorldFromEye[3].get_translation()[2] = mdEyeRadius;
  mnFrameCounter = 0;
}


void EyeGame::DrawEye()
{
  
  int nSegments = 45;
  int nSlices = 45;
  
  double dSliceAngle = M_PI / (double)(nSlices);
  double dSegAngle = 2.0 * M_PI / (double)(nSegments);

  glColor3f(0.0,0.0,0.0);
  
  {  // North pole:
    double Z = sin(M_PI/2.0 - dSliceAngle);
    double R = cos(M_PI/2.0 - dSliceAngle);
    glBegin(GL_TRIANGLE_FAN);
    glNormal3f(0,0,1);
    glVertex3f(0,0,1);
    
    for(int i=0; i<nSegments;i++) {
      
	glNormal3f(R * sin((double)i * dSegAngle), R * cos((double)i * dSegAngle),  Z);
	glVertex3f(R * sin((double)i * dSegAngle), R * cos((double)i * dSegAngle),  Z);
    }
    
    glNormal3f(0,R,Z);
    glVertex3f(0,R,Z);
    glEnd();
  }
  
  int nBlueSlice = 3;
  int nWhiteSlice = 6;
  for(int j = 1; j<nSlices;j++) {
    
      if(j == nBlueSlice) glColor3f(0,0,1);
      if(j == nWhiteSlice) glColor4d(0.92, 0.9, 0.85,1);
      
      glBegin(GL_QUAD_STRIP);
      double zTop = sin(M_PI/2.0 - dSliceAngle * (double)j);
      double zBot = sin(M_PI/2.0 - dSliceAngle * (double)(j+1));
      double rTop = cos(M_PI/2.0 - dSliceAngle * (double)j);
      double rBot = cos(M_PI/2.0 - dSliceAngle * (double)(j+1));
      
      for(int i=0; i<nSegments;i++) {
	
	  glNormal3f(rTop*sin((double)i*dSegAngle), rTop*cos((double)i*dSegAngle), zTop);
	  glVertex3f(rTop*sin((double)i*dSegAngle), rTop*cos((double)i*dSegAngle), zTop);
	  glNormal3f(rBot*sin((double)i*dSegAngle), rBot*cos((double)i*dSegAngle), zBot);
	  glVertex3f(rBot*sin((double)i*dSegAngle), rBot*cos((double)i*dSegAngle), zBot);
      }
      
      glNormal3f(0,rTop, zTop);
      glVertex3f(0,rTop, zTop);
      glNormal3f(0,rBot, zBot);
      glVertex3f(0,rBot, zBot);
      glEnd();
  }

  {
    // South pole:
    double Z = sin(M_PI/2.0 - dSliceAngle);
    double R = cos(M_PI/2.0 - dSliceAngle);
    glBegin(GL_TRIANGLE_FAN);
    glNormal3f(0,0,-1);
    glVertex3f(0,0,-1);
    
    for(int i=0; i<nSegments;i++) {
      
	glNormal3f(R * sin((double)i * -dSegAngle), R * cos((double)i * -dSegAngle),  -Z);
	glVertex3f(R * sin((double)i * -dSegAngle), R * cos((double)i * -dSegAngle),  -Z);
    }
    
    glNormal3f(0,R,-Z);
    glVertex3f(0,R,-Z);
    glEnd();
  }
  
}

void EyeGame::Init()
{
  if(mbInitialised) return;
  mbInitialised = true;
  // Set up the display list for the eyeball.
  mnEyeDisplayList = glGenLists(1);
  glNewList(mnEyeDisplayList,GL_COMPILE);
  DrawEye();
  glEndList();
  MakeShadowTex();
}


void EyeGame::LookAt(int nEye, cv::Vec3f v3, double dRotLimit)
{
  cv::Vec3f v3E = ase3WorldFromEye[nEye].inverse() * v3;
  
  float v3Enorm = cv::norm(v3E);
  if( v3Enorm  == 0.0) return;
  
  v3E = v3E / v3Enorm;
  
  // Gram-Schmidt stuff...
  cv::Matx<float, 3, 3> m3Rot = cv::Matx<float, 3, 3>::eye();
  //m3Rot[2] = v3E;
  m3Rot(2, 0) = v3E[0]; m3Rot(2, 1) = v3E[1]; m3Rot(2, 2) = v3E[2];
  
  // row1 = row1 - (row1' * row3) * row3; row1 = row1 / norm(row)
  float dot13 = v3E[0]; // recall, m4Rot is initialized as the identity
  cv::Matx<float, 1, 3> r0t = CvUtils::normalize( m3Rot.row(0) - ( dot13 * m3Rot.row(2) ) );
  // assign the result
  m3Rot(0, 0) = r0t(0, 0); m3Rot(0, 1) = r0t(0, 1); m3Rot(0, 2) = r0t(0, 2);
  
  //m3Rot[1] = m3Rot[2] ^ m3Rot[0];
  m3Rot(1, 0) = -m3Rot(2, 2) * m3Rot(0, 1) + m3Rot(2, 1) * m3Rot(0, 2);
  m3Rot(1, 1) =  m3Rot(2, 2) * m3Rot(0, 0) - m3Rot(2, 0) * m3Rot(0, 2);
  m3Rot(1, 2) = -m3Rot(2, 1) * m3Rot(0, 0) + m3Rot(2, 0) * m3Rot(0, 1);
  
  
  SO3<> so3Rotator = m3Rot;
  cv::Vec3f v3Log = so3Rotator.ln();
  v3Log[2] = 0.0;
  double dMagn = cv::norm(v3Log);
  if(dMagn > dRotLimit) {
    
      v3Log = v3Log * ( dRotLimit / dMagn);
  }
  ase3WorldFromEye[nEye].get_rotation() = ase3WorldFromEye[nEye].get_rotation() * SO3<>::exp(-v3Log);
}

void EyeGame::MakeShadowTex()
{
  const int nTexSize = 256;
  cv::Mat_<uchar> imShadow(nTexSize, nTexSize);
  double dFrac = 1.0 - mdEyeRadius / mdShadowHalfSize;
  double dCenterPos = dFrac * nTexSize / 2 - 0.5;
  cv::Point2i irCenter((int) dCenterPos, (int) dCenterPos);
  int nRadius = int ((nTexSize / 2 - irCenter.x) * 1.05);
  unsigned int nRadiusSquared = nRadius*nRadius;
  int r, c;
  for(r = 0; 2 * r < nTexSize; r++)
    for(c = 0; 2 * c < nTexSize; c++) {
	uchar val = 0;
	if( (r - irCenter.y) * (r - irCenter.y)+
	    (c - irCenter.x) * (c - irCenter.x) < nRadiusSquared ) val = 255;
	imShadow(r, c) = val;
	imShadow(r, nTexSize - 1 - c) = val;
	imShadow(nTexSize - 1 - r, nTexSize - 1 - c ) = val;
	imShadow(nTexSize - 1 - r, c ) = val;
      
    }
  
  // blurring the shadow image...
  double blurSigma = 4.0;
  int gkerSize = (int)ceil(blurSigma*3.0); // where 3.0 is the default "sigmas" parameter in libCVD
  gkerSize += (gkerSize % 2 == 0) ? 1 : 0;
   
  cv::Mat_<uchar> imTemp; // no allocation necessary
  cv::GaussianBlur(imShadow, imTemp, cv::Size(gkerSize, gkerSize), blurSigma );
  imTemp.copyTo(imShadow);
  
  //ok, put the stuff in a texture...
  glGenTextures(1, &mnShadowTex);
  glBindTexture(GL_TEXTURE_2D,mnShadowTex);
  
  
	
  GLXInterface::glTexSubImage2DGRAY(imShadow); // n.b. the GLXInterface function uses GL_LUMINANCE instead ogf GL_ALPHA; we'll see...
					       // But it really takes care of memory management
		
  /*glTexImage2D(GL_TEXTURE_2D, 0, 
	       GL_ALPHA, nTexSize, nTexSize, 0,  
	       GL_ALPHA, GL_UNSIGNED_BYTE, imShadow.data); 
  */
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
}





