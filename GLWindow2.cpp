// George Terzakis 2016
//
// University of Portsmouth
//
// Code based on PTAM by Klein and Murray (Copyright 2008 Isis Innovation Limited)

#include "OpenGL.h"
#include "GLWindow2.h"
#include "GLWindowMenu.h"
#include "GCVD/GLHelpers.h"

#include <stdlib.h>
#include "Persistence/GStringUtil.h"
#include "Persistence/instances.h"


using namespace std;
using namespace GLXInterface;
using namespace Persistence;

#include <GL/gl.h>
#include <GL/glut.h>

GLWindow2::GLWindow2(cv::Size2i irSize, string sTitle) : GLWindow(irSize, sTitle)
{

#ifdef WIN32
  // On windows, have to initialise GLEW at the start to enable access
  // to GL extensions
  static bool bGLEWIsInit = false;
  if(!bGLEWIsInit)
  {
	GLenum err = glewInit();
	if (GLEW_OK != err)
	{
		std::cout << "GLEW Error:  " << glewGetErrorString(err) << std::endl;;
		exit(0);
	}
	bGLEWIsInit = true;
  }
#endif

  mirVideoSize = irSize;
  GUI.RegisterCommand("GLWindow.AddMenu", GUICommandCallBack, this);
  glSetFont("sans");
  mvMCPoseUpdate = cv::Vec<float, 6>(0, 0, 0, 0, 0 , 0);
  mvLeftPoseUpdate = cv::Vec<float, 6>(0, 0, 0, 0, 0 , 0);
};


void GLWindow2::AddMenu(string sName, string sTitle)
{
  GLWindowMenu* pMenu = new GLWindowMenu(sName, sTitle); 
  mvpGLWindowMenus.push_back(pMenu);
}

void GLWindow2::GUICommandCallBack(void* ptr, string sCommand, string sParams)
{
  ((GLWindow2*) ptr)->GUICommandHandler(sCommand, sParams);
}

void GLWindow2::GUICommandHandler(string sCommand, string sParams)  // Called by the callback func..
{
  vector<string> vs = ChopAndUnquoteString(sParams);
  if(sCommand=="GLWindow.AddMenu")
    {
      switch(vs.size())
	{
	case 1:
	  AddMenu(vs[0], "Root");
	  return;
	case 2:
	  AddMenu(vs[0], vs[1]);
	  return;
	default:
	  cout << "? AddMenu: need one or two params (internal menu name, [caption])." << endl;
	  return;
	};
    };
  
  // Should have returned to caller by now - if got here, a command which 
  // was not handled was registered....
  cout << "! GLWindow::GUICommandHandler: unhandled command "<< sCommand << endl;
  exit(1);
}; 

void GLWindow2::DrawMenus()
{
  glDisable(GL_STENCIL_TEST);
  glDisable(GL_DEPTH_TEST);
  glDisable(GL_TEXTURE_2D);
  glDisable(GL_TEXTURE_RECTANGLE_ARB);
  glDisable(GL_LINE_SMOOTH);
  glDisable(GL_POLYGON_SMOOTH);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glColorMask(1,1,1,1);
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
  SetupWindowOrtho();
  glLineWidth(1);
  
  int nTop = 30;
  int nHeight = 30;
  for(vector<GLWindowMenu*>::iterator i = mvpGLWindowMenus.begin(); i!= mvpGLWindowMenus.end(); i++) {
    
      (*i)->Render(nTop, nHeight, size().width, *this);
      nTop+=nHeight+1;
    }
  
}

// STOP THE PRESS!!!! 
// etting up unit ortho 
void GLWindow2::SetupUnitOrtho()
{
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  glOrtho(0,1,1,0,0,1);
}

void GLWindow2::SetupWindowOrtho()
{
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  GLXInterface::glOrtho(size());
  
}

// STOP THE PRESS!!!! This glOrtho configuration is used during the calibration
// and it also described the ATANCamera projection/unprojection functions...
// The 0.5 margin given as a made-up additional minor offset, which -most probably- is Klein's way 
// of preventing border image points from falling off the visible OpenGL canvas.
/// We setup a window coordinate system as (-0.5 to -0.5 + IMG_WIDTH) x (-0.5 + IMG_HEIGHT, -0.5). In other words, we draw driectly in image coordinates
void GLWindow2::SetupVideoOrtho()
{
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  glOrtho(-0.5,(double)mirVideoSize.width - 0.5, (double)mirVideoSize.height - 0.5, -0.5, -1.0, 1.0);
}

void GLWindow2::SetupVideoRasterPosAndZoom()
{ 
  glRasterPos2d(-0.5,-0.5);
  float adZoom[2];
  adZoom[0] = (float) size().width / (float) mirVideoSize.width;
  adZoom[1] = (float) size().height / (float) mirVideoSize.height;
  glPixelZoom(adZoom[0], -adZoom[1]);
}

void GLWindow2::SetupViewport()
{
  glViewport(0, 0, size().width, size().height);
}

void GLWindow2::PrintString(cv::Point2i irPos, std::string s)
{
  glMatrixMode(GL_PROJECTION);
  glPushMatrix();
  glTranslatef(irPos.x, irPos.y, 0.0);
  glScalef(8,-8,1);
  GLXInterface::glDrawText(s, GLXInterface::NICE, 1.6, 0.1);
  glPopMatrix();
}

void GLWindow2::DrawCaption(string s)
{
  if(s.length() == 0) return;
  
  SetupWindowOrtho();
  // Find out how many lines are in the caption:
  // Count the endls
  int nLines = 0;
  {
    string sendl("\n");
    string::size_type st=0;
    while(1) {
	nLines++;
	st = s.find(sendl, st);
	if(st==string::npos)  break;
	else
	  st++;
    }
  }
  
  int nTopOfBox = size().height - nLines * 17;
  
  // Draw a grey background box for the text
  glColor4f(0,0,0,0.4);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glBegin(GL_QUADS);
  glVertex2d(-0.5, nTopOfBox);
  glVertex2d(size().width, nTopOfBox);
  glVertex2d(size().width, size().height);
  glVertex2d(-0.5, size().height);
  glEnd();
  
  // Draw the caption text in yellow
  glColor3f(1,1,0);      
  PrintString(cv::Point2i(10, nTopOfBox + 13), s);
}


void GLWindow2::HandlePendingEvents()
{
  handle_events(*this);
}

void GLWindow2::on_mouse_move(GLWindow& win, cv::Point2i where, int state)
{
  cv::Point2i irMotion = where - mirLastMousePos;
  mirLastMousePos = where;
  
  double dSensitivity = 0.01;
  if(state & BUTTON_LEFT && ! (state & BUTTON_RIGHT)) {
    
      mvMCPoseUpdate[3] -= irMotion.y * dSensitivity;
      mvMCPoseUpdate[4] += irMotion.x * dSensitivity;
   }
  else if(!(state & BUTTON_LEFT) && state & BUTTON_RIGHT) {
    
      mvLeftPoseUpdate[4] -= irMotion.x * dSensitivity;
      mvLeftPoseUpdate[3] += irMotion.y * dSensitivity;
  }
  else if(state & BUTTON_MIDDLE  || (state & BUTTON_LEFT && state & BUTTON_RIGHT)) {
    
      mvLeftPoseUpdate[5] -= irMotion.x * dSensitivity;
      mvLeftPoseUpdate[2] += irMotion.y * dSensitivity;
  }
  
}

void GLWindow2::on_mouse_down(GLWindow& win, cv::Point2i where, int state, int button)
{
  bool bHandled = false;
  for(unsigned int i=0; !bHandled && i<mvpGLWindowMenus.size(); i++)
    bHandled = mvpGLWindowMenus[i]->HandleClick(button, state, where.x, where.y);

}

void GLWindow2::on_event(GLWindow& win, int event)
{
  if(event==EVENT_CLOSE)
    GUI.ParseLine("quit");
}

pair<cv::Vec<float, 6>, cv::Vec<float, 6> > GLWindow2::GetMousePoseUpdate()
{
  pair<cv::Vec<float, 6>, cv::Vec<float, 6> > result = make_pair(mvLeftPoseUpdate, mvMCPoseUpdate);
  mvLeftPoseUpdate = cv::Vec<float, 6>(0, 0, 0, 0, 0, 0); 
  mvMCPoseUpdate = cv::Vec<float, 6>(0, 0, 0, 0, 0, 0);
  
  return result;
}



#ifndef WIN32
#include <X11/keysym.h>
void GLWindow2::on_key_down(GLWindow&, int k)
{
  string s;
  switch(k)
    {
    case XK_a:   case XK_A:  s="a"; break;
    case XK_b:   case XK_B:  s="b"; break;
    case XK_c:   case XK_C:  s="c"; break;
    case XK_d:   case XK_D:  s="d"; break;
    case XK_e:   case XK_E:  s="e"; break;
    case XK_f:   case XK_F:  s="f"; break;
    case XK_g:   case XK_G:  s="g"; break;
    case XK_h:   case XK_H:  s="h"; break;
    case XK_i:   case XK_I:  s="i"; break;
    case XK_j:   case XK_J:  s="j"; break;
    case XK_k:   case XK_K:  s="k"; break;
    case XK_l:   case XK_L:  s="l"; break;
    case XK_m:   case XK_M:  s="m"; break;
    case XK_n:   case XK_N:  s="n"; break;
    case XK_o:   case XK_O:  s="o"; break;
    case XK_p:   case XK_P:  s="p"; break;
    case XK_q:   case XK_Q:  s="q"; break;
    case XK_r:   case XK_R:  s="r"; break;
    case XK_s:   case XK_S:  s="s"; break;
    case XK_t:   case XK_T:  s="t"; break;
    case XK_u:   case XK_U:  s="u"; break;
    case XK_v:   case XK_V:  s="v"; break;
    case XK_w:   case XK_W:  s="w"; break;
    case XK_x:   case XK_X:  s="x"; break;
    case XK_y:   case XK_Y:  s="y"; break;
    case XK_z:   case XK_Z:  s="z"; break;
    case XK_1:   s="1"; break;
    case XK_2:   s="2"; break;
    case XK_3:   s="3"; break;
    case XK_4:   s="4"; break;
    case XK_5:   s="5"; break;
    case XK_6:   s="6"; break;
    case XK_7:   s="7"; break;
    case XK_8:   s="8"; break;
    case XK_9:   s="9"; break;
    case XK_0:   s="0"; break;
    case XK_KP_Prior: case XK_Page_Up:     s="PageUp"; break;
    case XK_KP_Next:  case XK_Page_Down:   s="PageDown"; break;
    case XK_Return: s="Enter"; break;
    case XK_space:  s="Space"; break;
    case XK_BackSpace:  s="BackSpace"; break;
    case XK_Escape:  s="Escape"; break;
    default: ;
    }

  if(s!="") GUI.ParseLine("try KeyPress "+s);
}
#else
void GLWindow2::on_key_down(GLWindow&, int k)
{
  string s;
  // ASCI chars can be mapped directly:
  if( (k >= 48 && k <=57) || ( k >=97 && k <= 122) || (k >= 65 && k <= 90))
  {
	char c = k;
	if(c >= 65 && c <= 90) c = c + 32;
	s = c;
  }
  else 
    switch (k) // Some special chars are translated:
  {
    case 33: s="PageUp"; break;
    case 34: s="PageDown"; break;
    case 13: s="Enter"; break;
    case 32:  s="Space"; break;
    case 8:  s="BackSpace"; break;
    case 27:  s="Escape"; break;
    default: break;
  }
  
  if(s!="")
    GUI.ParseLine("try KeyPress "+s);
}
#endif

