//************** George Terzakis 2016 **************
//
//  		University of Portsmouth      

// This file is a slight modification of the original PTAM code by Klein and Murrary (Isis Innovation Limited)

#define GL_GLEXT_PROTOTYPES 1

#include "ARDriver.h"
#include "GCVD/GLHelpers.h"

#include "Persistence/instances.h"


using namespace Persistence;
using namespace std;

static bool CheckFramebufferStatus();

ARDriver::ARDriver(const ATANCamera &cam, GLWindow2 &glw) :mCamera(cam), mGLWindow(glw)
{
  mirFrameSize = mCamera.GetImageSize();
 
  mbInitialised = false;
}

void ARDriver::Init()
{
  mbInitialised = true;
  // broken down the size of the framebuffer into two persistent integer variables :
  mirFBSize.width = PV3::get<int>("ARDriver.FrameBufferWidth", 1200, SILENT);
  mirFBSize.height = PV3::get<int>("ARDriver.FrameBufferHeight", 900, SILENT);
  
  glGenTextures(1, &mnFrameTex);
  glBindTexture(GL_TEXTURE_RECTANGLE_ARB,mnFrameTex);
  glTexImage2D(GL_TEXTURE_RECTANGLE_ARB, 0, 
	       GL_RGBA, mirFrameSize.width, mirFrameSize.height, 0, 
	       GL_RGBA, GL_UNSIGNED_BYTE, NULL); 
  MakeFrameBuffer();
  mGame.Init();
  
}

void ARDriver::Reset()
{
  mGame.Reset();
  mnCounter = 0;
}

void ARDriver::Render(cv::Mat &imFrame, SE3<> se3CfromW)
{
  if(!mbInitialised) {
    
      Init();
      Reset();
    }
  
  mnCounter++;
  
  // Upload the image to our frame texture
  glBindTexture(GL_TEXTURE_RECTANGLE_ARB, mnFrameTex);
  /*glTexSubImage2D(GL_TEXTURE_RECTANGLE_ARB,
		  0, 0, 0,
		  mirFrameSize.width, mirFrameSize.height,
		  GL_RGB,
		  GL_UNSIGNED_BYTE,
		  imFrame.data);*/ 
  GLXInterface::glTexSubImage2DBGR(imFrame, 0, 0, GL_TEXTURE_RECTANGLE_ARB);
  // Set up rendering to go the FBO, draw undistorted video frame into BG
  glBindFramebufferEXT(GL_FRAMEBUFFER_EXT,mnFrameBuffer);
  CheckFramebufferStatus();
  glViewport(0,0,mirFBSize.width,mirFBSize.height);
  DrawFBBackGround();
  glClearDepth(1);
  glClear(GL_DEPTH_BUFFER_BIT);
  
  // Set up 3D projection
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  
  GLXInterface::glMultMatrix4x4(mCamera.MakeUFBLinearFrustumMatrix(0.005, 100));
  GLXInterface::glMultMatrix(se3CfromW);
  
  DrawFadingGrid();
  
  mGame.DrawStuff(se3CfromW.inverse().get_translation());
  
  glDisable(GL_DEPTH_TEST);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glDisable(GL_BLEND);
  
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
  
  // Set up for drawing 2D stuff:
  glBindFramebufferEXT(GL_FRAMEBUFFER_EXT,0);
  
  DrawDistortedFB();
  
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
  mGLWindow.SetupViewport();
  mGLWindow.SetupVideoOrtho();
  mGLWindow.SetupVideoRasterPosAndZoom();
}



void ARDriver::MakeFrameBuffer()
{
  // Needs nvidia drivers >= 97.46
  cout << "  ARDriver: Creating FBO... ";
  
  glGenTextures(1, &mnFrameBufferTex);
  glBindTexture(GL_TEXTURE_RECTANGLE_ARB,mnFrameBufferTex);
  glTexImage2D(GL_TEXTURE_RECTANGLE_ARB, 0, 
	       GL_RGBA, mirFBSize.width, mirFBSize.height, 0, 
	       GL_RGBA, GL_UNSIGNED_BYTE, NULL); 
  glTexParameteri(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
  glTexParameteri(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_MAG_FILTER, GL_NEAREST);

  GLuint DepthBuffer;
  glGenRenderbuffersEXT(1, &DepthBuffer);
  glBindRenderbufferEXT(GL_RENDERBUFFER_EXT, DepthBuffer);
  glRenderbufferStorageEXT(GL_RENDERBUFFER_EXT, GL_DEPTH_COMPONENT24, mirFBSize.width, mirFBSize.height);

  glGenFramebuffersEXT(1, &mnFrameBuffer);
  glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, mnFrameBuffer); 
  glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT0_EXT, 
			    GL_TEXTURE_RECTANGLE_ARB, mnFrameBufferTex, 0);
  glFramebufferRenderbufferEXT(GL_FRAMEBUFFER_EXT, GL_DEPTH_ATTACHMENT_EXT, 
  			       GL_RENDERBUFFER_EXT, DepthBuffer);
  
  CheckFramebufferStatus();
  cout << " .. created FBO." << endl;
  glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, 0);
}

static bool CheckFramebufferStatus()         
{                                            
  
  GLenum n;                                            
  n = glCheckFramebufferStatusEXT(GL_FRAMEBUFFER_EXT);
  if(n == GL_FRAMEBUFFER_COMPLETE_EXT) return true; // All good
  
  cout << "glCheckFrameBufferStatusExt returned an error." << endl;
  return false;
}

void ARDriver::DrawFBBackGround()
{
  static bool bFirstRun = true;
  static GLuint nList;
  mGLWindow.SetupUnitOrtho();
  
  glEnable(GL_TEXTURE_RECTANGLE_ARB);
  glBindTexture(GL_TEXTURE_RECTANGLE_ARB, mnFrameTex);  
  glTexParameteri(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  glDisable(GL_POLYGON_SMOOTH);
  glDisable(GL_BLEND);
  // Cache the cpu-intesive projections in a display list..
  if(bFirstRun)
    {
      bFirstRun = false;
      nList = glGenLists(1);
      glNewList(nList, GL_COMPILE_AND_EXECUTE);
      glColor3f(1,1,1);
      // How many grid divisions in the x and y directions to use?
      int nStepsX = 24; // Pretty arbitrary..
      int nStepsY = (int) (nStepsX * ((double) mirFrameSize.width / mirFrameSize.height)); // Scaled by aspect ratio
      
      if(nStepsY < 2) nStepsY = 2;
      
      for(int ystep = 0; ystep< nStepsY; ystep++) {
	
	  glBegin(GL_QUAD_STRIP);
	  for(int xstep = 0; xstep <= nStepsX; xstep++)
	    for(int yystep = ystep; yystep<=ystep+1; yystep++) // Two y-coords in one go - magic.
	      {
		cv::Vec2d v2Iter( (double) xstep / nStepsX,
				  (double) yystep / nStepsY );
		// If this is a border quad, draw a little beyond the
		// outside of the frame, this avoids strange jaggies
		// at the edge of the reconstructed frame later:
		if(xstep == 0 || yystep == 0 || xstep == nStepsX || yystep == nStepsY)
		  for(int i=0; i<2; i++)
		    v2Iter[i] = v2Iter[i] * 1.02 - 0.01; 
		cv::Vec2d v2UFBDistorted = v2Iter; 
		cv::Vec2f v2UFBUnDistorted = mCamera.UFBLinearProject(mCamera.UFBUnProject(v2UFBDistorted));
		glTexCoord2d(v2UFBDistorted[0] * mirFrameSize.width, v2UFBDistorted[1] * mirFrameSize.height);
		glVertex(v2UFBUnDistorted);
	      }
	  glEnd();
	}
      glEndList();
    }
  else
    glCallList(nList);
  glDisable(GL_TEXTURE_RECTANGLE_ARB);
}


void ARDriver::DrawDistortedFB()
{
  static bool bFirstRun = true;
  static GLuint nList;
  mGLWindow.SetupViewport();
  mGLWindow.SetupUnitOrtho();
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
  glEnable(GL_TEXTURE_RECTANGLE_ARB);
  glBindTexture(GL_TEXTURE_RECTANGLE_ARB, mnFrameBufferTex);  
  glTexParameteri(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  glDisable(GL_POLYGON_SMOOTH);
  glDisable(GL_BLEND);
  if(bFirstRun)
    {
      bFirstRun = false;
      nList = glGenLists(1);
      glNewList(nList, GL_COMPILE_AND_EXECUTE);
      // How many grid divisions in the x and y directions to use?
      int nStepsX = 24; // Pretty arbitrary..
      int nStepsY = (int) (nStepsX * ((double) mirFrameSize.width / mirFrameSize.height)); // Scaled by aspect ratio
      if(nStepsY < 2) nStepsY = 2;
      glColor3f(1,1,1);
      for(int ystep = 0; ystep<nStepsY; ystep++)
	{  
	  glBegin(GL_QUAD_STRIP);
	  for(int xstep = 0; xstep<=nStepsX; xstep++)
	    for(int yystep = ystep; yystep<=ystep + 1; yystep++) // Two y-coords in one go - magic.
	      {
		cv::Vec2d v2Iter( (double) xstep / nStepsX, 
				  (double) yystep / nStepsY );
		cv::Vec2d v2UFBDistorted = v2Iter; 
		cv::Vec2f v2UFBUnDistorted = mCamera.UFBLinearProject(mCamera.UFBUnProject(v2UFBDistorted));
		glTexCoord2d(v2UFBUnDistorted[0] * mirFBSize.width, (1.0 - v2UFBUnDistorted[1]) * mirFBSize.height);
		glVertex(v2UFBDistorted);
	      }	 
	  glEnd();
	}
      glEndList();
    }
  else
    glCallList(nList);
  glDisable(GL_TEXTURE_RECTANGLE_ARB);
}

void ARDriver::DrawFadingGrid()
{
  double dStrength;
  if(mnCounter >= 60)
    return;
  if(mnCounter < 30)
    dStrength = 1.0;
  dStrength = (60 - mnCounter) / 30.0;
  
  glColor4f(1,1,1,dStrength);
  int nHalfCells = 8;
  if(mnCounter < 8) nHalfCells = mnCounter + 1;
  int nTot = nHalfCells * 2 + 1;
  cv::Vec3f  aaVertex[17][17];
  for(int i=0; i<nTot; i++)
    for(int j=0; j<nTot; j++)
      {
	cv::Vec3f v3 ( (i - nHalfCells) * 0.1, 
		       (j - nHalfCells) * 0.1, 
		       0.0 );
	aaVertex[i][j] = v3;
      }

  glEnable(GL_LINE_SMOOTH);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glLineWidth(2);
  for(int i=0; i<nTot; i++)
    {
      glBegin(GL_LINE_STRIP);
      for(int j=0; j<nTot; j++)
	glVertex(aaVertex[i][j]);
      glEnd();
      
      glBegin(GL_LINE_STRIP);
      for(int j=0; j<nTot; j++)
	glVertex(aaVertex[j][i]);
      glEnd();
    }
}







