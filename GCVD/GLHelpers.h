#ifndef GLHELPERS_H
#define GLHELPERS_H

#include <iostream>
#include <map>
#include <utility>


#ifdef WIN32
#include <windows.h>
#endif

#include <GL/gl.h>
#include <GL/glu.h>

#include "../OpenCV.h"

#include "SO3.h"
#include "SE3.h"
#include "SO2.h"

using namespace RigidTransforms;


namespace GLXInterface
{
	// ******************** SOME FORWARD DECLARATIONS FOUND IN GLText.cpp *******************************
  
	/// returns the name of the currently active font
	const std::string & glGetFont();

	/// different style for font rendering
	enum TEXT_STYLE {
	  FILL = 0,       ///< renders glyphs as filled polygons
	  OUTLINE = 1,    ///< renders glyphs as outlines with GL_LINES
	  NICE = 2        ///< renders glyphs filled with antialiased outlines
	};

	/// renders a string in GL using the current settings.
	/// Font coordinates are +X along the line and +Y along the up direction of glyphs.
	/// The origin is at the top baseline at the left of the first character. Characters have a maximum size of 1.
	/// linefeed is interpreted as a new line and the start is offset in -Y direction by @ref spacing . Individual characters
	/// are separated by @ref kerning + plus their individual with.
	/// @param text string to be rendered, unknown characters are replaced with '?'
	/// @param style rendering style
	/// @param spacing distance between individual text lines
	/// @param kerning distance between characters
	std::pair<double, double> glDrawText(const std::string & text, enum TEXT_STYLE style = NICE, double spacing = 1.5, double kerning = 0.1);

	/// returns the size of the bounding box of a text to be rendered, similar to @ref glDrawText but without any visual output
	std::pair<double, double> glGetExtends(const std::string & text, double spacing = 1.5, double kerning = 0.1);

  
	/// @defgroup gGLText OpenGL text rendering
	/// sets the font to use for future font rendering commands. currently sans, serif and mono are available.
	/// @param fontname string containing font name
	void glSetFont( const std::string & fontname );
  
	
	
	// ************************************* END FORWAR DECLARATIONS ****************************************
	
	/// Specify the (x,y) co-ordinates of a vertex
	/// @param i The vertex location
	inline void glVertex(const cv::Point2i &i)
	{
		glVertex2i(i.x, i.y);
	}

	/// Specify the (s,t) texture co-ordinates
	/// @param i The texture coordinates
	///@ingroup gGL
	inline void glTexCoord(const cv::Point2i &i)
	{
		glTexCoord2i(i.x, i.y);
	}


	/// Specify the (x,y) co-ordinates of the current raster position
	/// @param i The raster position
	///@ingroup gGL
	inline void glRasterPos(const cv::Point2i &i)
	{
		glRasterPos2i(i.x, i.y);
	}

	/// Draws a rectangle by specifing two opposing vertices
	/// @param p the first vertex
	/// @param q the second vertex
	/// @ingroup gGL
	inline void glRect( const cv::Point2i &p, const cv::Point2i &q)
	{
	    glRecti(p.x, p.y, q.x, q.y);
	}

	//#ifdef CVD_HAVE_TOON
	/// Specify the (x,y) co-ordinates of a vertex
	/// @param v The vertex location
	///@ingroup gGL
	inline void glVertex(const cv::Vec2f v)
	{
		glVertex2d(v[0], v[1]);
	}

	/// Specify the (x,y,z) co-ordinates of a vertex
	/// @param v The vertex location
	///@ingroup gGL
	inline void glVertex(const cv::Vec3f &v)
	{
		glVertex3d(v[0], v[1], v[2]);
	}

	/// Specify the (x,y,z,w) co-ordinates of a vertex
	/// @param v The vertex location
	///@ingroup gGL
	inline void glVertex(const cv::Vec4f &v)
	{
		glVertex4d(v[0], v[1], v[2], v[3]);
	}

	/// Specify the (s,t) texture coordinates
	/// @param v The texture coordinates
	///@ingroup gGL
	inline void glTexCoord(const cv::Vec2f &v)
	{
		glTexCoord2d(v[0], v[1]);
	}

	/// Specify the (s,t,r) texture coordinates
	/// @param v The texture coordinates
	///@ingroup gGL
	inline void glTexCoord(const cv::Vec3f &v)
	{
		glTexCoord3d(v[0], v[1], v[2]);
	}

	/// Specify the (s,t,r,q) texture coordinates
	/// @param v The texture coordinates
	///@ingroup gGL
	inline void glTexCoord(const cv::Vec4f &v)
	{
		glTexCoord4d(v[0], v[1], v[2], v[3]);
	}
	
	/// Draws a rectangle by specifing two opposing vertices
	/// @param p the first vertex
	/// @param q the second vertex
	/// @ingroup gGL
	inline void glRect( const cv::Vec2f &p, const cv::Vec2f &q)
	{
	    glRectd(p[0], p[1], q[0], q[1]);
	}

      //#ifdef GL_GLEXT_PROTOTYPES
	/// Specify the (s,t) texture coordinates for a specific texture unit
	/// @param v The texture coordinates
	/// @param unit The texture unit
	///@ingroup gGL
	inline void glMultiTexCoord(GLenum unit, const cv::Vec2f &v)
	{
	        glMultiTexCoord2d(unit, v[0], v[1]);
	}

	/// Specify the (s,t,r) texture coordinates for a specific texture unit
	/// @param v The texture coordinates
	/// @param unit The texture unit
	///@ingroup gGL
	inline void glMultiTexCoord(GLenum unit, const cv::Vec3f &v)
	{
	        glMultiTexCoord3d(unit, v[0], v[1], v[2]);
	}

	/// Specify the (s,t,r,q) texture coordinates for a specific texture unit
	/// @param v The texture coordinates
	/// @param unit The texture unit
	inline void glMultiTexCoord(GLenum unit, cv::Vec3f &v)
	{
	        glMultiTexCoord4d(unit, v[0], v[1], v[2], v[3]);
	}


	/// Specify the (x,y) co-ordinates of the current raster position
	/// @param v The raster position
	inline void glRasterPos(const cv::Vec2f &v)
	{
		glRasterPos2d(v[0], v[1]);
	}

	/// Specify the (x,y,z) co-ordinates of the current raster position
	/// @param v The raster position
	inline void glRasterPos(const cv::Vec3f &v)
	{
		glRasterPos3d(v[0], v[1], v[2]);
	}

	/// Specify the (x,y,z,w) co-ordinates of the current raster position
	/// @param v The raster position
	inline void glRasterPos(const cv::Vec4f &v)
	{
		glRasterPos4d(v[0], v[1], v[2], v[3]);
	}

	/// Specify the current vertex normal
	/// @param n The normal vector
	inline void glNormal(const cv::Vec3f &n)
	{
	        glNormal3d(n[0], n[1], n[2]);
	}

        /// add a translation specified by an cv::Point2i
        /// @param v the translation 
	inline void glTranslate( const cv::Point2i  &v ) 
	{
        
	  glTranslatef( static_cast<GLfloat>(v.x), static_cast<GLfloat>(v.y), 0);
	}

	

	/// add a translation specified from the first two coordinates of a 2-vector
	/// z is set to zero here
	/// @param v the translation vector
	inline void glTranslate( const cv::Vec2f &v)
	{
		glTranslated(v[0], v[1], 0);
	}

	
	/// multiply a matrix onto the current matrix stack. Works for matrizes
	/// of size n >= 4 and uses the upper left 4x4 submatrix. The matrix is also
	/// transposed to account for GL's column major format.
	/// @param m the transformation matrix
	template <class P> inline void glMultMatrix4x4( const cv::Matx<P, 4, 4> &m )
	{
		GLdouble glm[16];
		glm[0] = m(0, 0);  glm[1] = m(1, 0);  glm[2] = m(2, 0);  glm[3] = m(3, 0);
		glm[4] = m(0, 1);  glm[5] = m(1, 1);  glm[6] = m(2, 1);  glm[7] = m(3, 1);
		glm[8] = m(0, 2);  glm[9] = m(1, 2);  glm[10] = m(2, 2); glm[11] = m(3, 2);
		glm[12] = m(0, 3); glm[13] = m(1, 3); glm[14] = m(2, 3); glm[15] = m(3, 3);
		glMultMatrixd(glm);
	}

	/// multiply a TooN 3x3 matrix onto the current matrix stack. The GL matrix
	/// last column and row are set to 0 with the lower right element to 1.
	/// The matrix is also transposed to account for GL's column major format.
	/// @param m the transformation matrix
	template <class P> inline void glMultMatrix3x3( const cv::Matx<P, 3, 3> &m )
	{
		GLdouble glm[16];
		glm[0] = m(0, 0); glm[1] = m(1, 0); glm[2] = m(2, 0); glm[3] = 0;
		glm[4] = m(0, 1); glm[5] = m(1, 1); glm[6] = m(2, 1); glm[7] = 0;
		glm[8] = m(0, 2); glm[9] = m(1, 2); glm[10] = m(2, 2); glm[11] = 0;
		glm[12] = 0; glm[13] = 0; glm[14] = 0; glm[15] = 1;
		
		glMultMatrixd(glm);
	}

	/// multiply a TooN 2x2 matrix onto the current matrix stack. The TooN matrix
	/// will only occupy the upper left hand block, the remainder will be from the
	/// identity matrix. The matrix is also transposed to account for GL's column major format.
	/// @param m the transformation matrix
	template <class P> inline void glMultMatrix2x2( const cv::Matx<P, 3, 3> &m )
	{
		GLdouble glm[16];
		glm[0] = m(0, 0); glm[1] = m(1, 0); glm[2] = 0; glm[3] = 0;
		glm[4] = m(0, 1); glm[5] = m(1, 1); glm[6] = 0; glm[7] = 0;
		glm[8] = 0; glm[9] = 0; glm[10] = 1; glm[11] = 0;
		glm[12] = 0; glm[13] = 0; glm[14] = 0; glm[15] = 1;
		glMultMatrixd(glm);
	}

	/// multiplies a SO3 onto the current matrix stack
	/// @param so3 the SO3
	template <typename P>
	inline void glMultMatrix( const RigidTransforms::SO3<P> &so3 )
	{
		glMultMatrix3x3( so3.get_matrix());
	}

	// translate by a 3 -float vector
	inline void glTranslate( const cv::Vec3f &t)
	{
	  glTranslatef(t[0], t[1], t[2]);
	 
	}
	// translate by a 3 - double vector
	inline void glTranslate( const cv::Vec3d &t)
	{
	  glTranslated(t[0], t[1], t[2]);
	 
	}
	
	/// multiplies a SE3 onto the current matrix stack. This multiplies
	/// the SO3 and the translation in order.
	/// @param se3 the SE3
	template <typename P>
	inline void glMultMatrix( const RigidTransforms::SE3<P> &se3 )
	{
		glTranslate( se3.get_translation());
		glMultMatrix3x3( se3.get_rotation().get_matrix());
	}

	/// multiplies a SO2 onto the current matrix stack
	/// @param so2 the SO2
	template <typename P>
	inline void glMultMatrix( const RigidTransforms::SO2<P> &so2 )
	{
		glMultMatrix2x2( so2.get_matrix());
	}

	/// multiplies a SE2 onto the current matrix stack. This multiplies
	/// the SO2 and the translation in order.
	/// @param se3 the SE2
	template <typename P>
	inline void glMultMatrix( const RigidTransforms::SE2<P> & se2 )
	{
		glTranslate( se2.get_translation());
		glMultMatrix2x2( se2.get_rotation().get_matrix() );
	}

	
	
	
	/// Sets up an ortho projection suitable for drawing onto individual pixels of a
	/// gl window (or video image.) glVertex2f(0.0,0.0) will be the top left pixel and
	/// glVertex2f(xsize-1.0, ysize-1.0) will be the bottom right pixel. Depth is set
	/// from -1 to 1.
        /// n.b. You first need to set up the matrix environment yourself,
	/// e.g. glMatrixMode(GL_PROJECTION); glLoadIdentity();
	/// @param size ImageRef containing the size of the GL window.
	inline void glOrtho( const cv::Size2i& size, const double nearPlane = -1.0, const double farPlane = 1.0)
	{
	    ::glOrtho( -0.375, size.width - 0.375, size.height - 0.375, -0.375, nearPlane, farPlane );
	}

	/// Sets up an ortho projection from a simple Vector<6>
        /// n.b. You first need to set up the matrix environment yourself,
	/// e.g. glMatrixMode(GL_PROJECTION); glLoadIdentity();
	/// @param param 6-vector containing the parameters of the projection
	inline void glOrtho( const cv::Vec<float, 6> &param)
	{
		::glOrtho( param[0], param[1], param[2], param[3], param[4], param[5]);
	}

	/// sets a gl frustum from the linear camera parameters, image size and near and far plane.
	/// The camera will be in OpenGL style with camera center in the origin and the viewing direction
	/// down the negative z axis, with y pointing upwards and x pointing to the left and the image plane
	/// at z=-1.
	/// Images coordinates need to be rotated around the x axis to make sense here, because typically
	/// the camera is described as y going down (pixel lines) and image plane at z=1.
	/// @param params vector containing fu, fv, pu, pv as in the linear part of camera parameters
	/// @param width width of the image plane in pixels, here the viewport for example
	/// @param height height of the image plane in pixels, here the viewport for example
	/// @param near near clipping plane
	/// @param far far clipping plane
	template <typename P>
	inline void glFrustum( const cv::Vec<P, 4> &params, double width, double height, double nearPlane = 0.1, double farPlane = 100)
	{
		GLdouble left, right, bottom, top;
		left = -nearPlane * params[2] / params[0];
		top = nearPlane * params[3] / params[1];
		right = nearPlane * ( width - params[2] ) / params[0];
		bottom = - nearPlane * ( height - params[3] ) / params[1];
		::glFrustum( left, right, bottom, top, nearPlane, farPlane );
	}

	

	/// Sets up an ortho projection from a simple Vector<6>
        /// n.b. You first need to set up the matrix environment yourself,
	/// e.g. glMatrixMode(GL_PROJECTION); glLoadIdentity();
	/// @param param 6-vector containing the parameters of the projection
	inline void glFrustum( const cv::Vec<float, 6> &param)
	{
		::glFrustum( param[0], param[1], param[2], param[3], param[4], param[5]);
	}

	/// Set the new colour to the red, green and blue components given in the Vector
	/// (where 0.0 represents zero intensity and 1.0 full intensity)
	/// @param v The new colour
	inline void glColor(const cv::Vec3f &v)
	{
		glColor3d(v[0], v[1], v[2]);
	}

	/// Set the new colour to the red, green, blue and alpha components given in the Vector
	/// (where 0.0 represents zero intensity and 1.0 full intensity)
	/// @param v The new colour
	inline void glColor(const cv::Vec4f &v)
	{
		glColor4d(v[0], v[1], v[2], v[3]);
	}

	/// Set the new clear colour to the red, green, blue and alpha components given in the Vector
	/// (where 0.0 represents zero intensity and 1.0 full intensity)
	/// @param v The new colour
	inline void glClearColor(const cv::Vec4f &v)
	{
		::glClearColor((GLclampf)v[0], (GLclampf)v[1], (GLclampf)v[2], (GLclampf)v[3]);
	}

	/// Set the new clear colour to the red, green, blue components given in the Vector
	/// alpha is set to 0
	/// @param v The new colour
	inline void glClearColor(const cv::Vec3f &v)
	{
		::glClearColor((GLclampf)v[0], (GLclampf)v[1], (GLclampf)v[2], 0);
	}


	/// draws a line from x1 to x2
	/// any type that is accepted by glVertex is possible
	template <class P1, class P2> inline void glLine(const P1 &x1, const P2 &x2) 
	{
		glBegin(GL_LINES);
		glVertex(x1);
		glVertex(x2);
		glEnd();
	}

	/// sets a whole list of vertices stored in a std::vector. It uses the various
	/// glVertex helpers defined in this header file.
	/// @param list the list of vertices
	/*template<class C> inline void glVertex( const C &list )
	{
		for(typename C::const_iterator v = list.begin(); v != list.end(); ++v)
			glVertex(*v);
	}*/

	template<typename P> inline void glVertex( const std::vector<P> &list )
	{
		for(typename P::const_iterator v = list.begin(); v != list.end(); ++v)
			glVertex(*v);
	}


	/*inline void glVertex( const std::vector<cv::Vec2f> &list )
	{
		
		for( std::vector<cv::Vec2f>::const_iterator v = list.begin(); v != list.end(); ++v) glVertex(*v);
	}
	
	inline void glVertex( const std::vector<cv::Vec3f> &list )
	{
		for(std::vector<cv::Vec3f>::const_iterator v = list.begin(); v != list.end(); ++v) glVertex(*v);
	}
	
	inline void glVertex( const std::vector<cv::Vec4f> &list )
	{
		for(std::vector<cv::Vec4f>::const_iterator v = list.begin(); v != list.end(); ++v) glVertex(*v);
	}
	
	inline void glVertex( const std::vector<cv::Vec2d> &list )
	{
		for(std::vector<cv::Vec2d>::const_iterator v = list.begin(); v != list.end(); ++v) glVertex(*v);
	}
	
	inline void glVertex( const std::vector<cv::Vec3d> &list )
	{
		for(std::vector<cv::Vec3d>::const_iterator v = list.begin(); v != list.end(); ++v) glVertex(*v);
	}
	
	inline void glVertex( const std::vector<cv::Vec4d> &list )
	{
		for(std::vector<cv::Vec4d>::const_iterator v = list.begin(); v != list.end(); ++v) glVertex(*v);
	}*/
	
	/// Set the new colour to the red, green, blue components given
	/// (where 0 represents zero intensity and 255 full intensity)
	/// @param c The new colour
	inline void glColor(const cv::Vec3b &c)
	{
		glColor3ub(c[0], c[1], c[2]);
	}

 	
 	
 	
 	/// Set the new colour to the red, green, blue and alpha components given
	/// (where 0 represents zero intensity and 255 full intensity)
	/// @param c The new colour
	inline void glColor(const cv::Vec4b &c)
	{
		glColor4ub(c[0], c[1], c[2], c[3]);
	}

 	static inline int cvdalignof(const void* ptr) {
	  
	  size_t p = (size_t)ptr;
	      if(p&3)
		if(p&1)
		  return 1;
		else 
		  return 2;
	      else
		if(p&4)
		  return 4;
		else
		  return 8;
	}
	

 	/// Draw an image to the frame buffer at the current raster position.
	/// Use glRasterPos to set the current raster position
	/// @param im The image to draw
	inline void glDrawPixelsBGR(const cv::Mat &im)
	{
	
		int alpha = std::min( cvdalignof( im.ptr<void>(0,0) ), 
				      cvdalignof( im.ptr<void>(1,0) ) );
	  
	  
		glPixelStorei(GL_UNPACK_ALIGNMENT, alpha );
		//set length of one complete row in data (doesn't need to equal image.cols)
		glPixelStorei(GL_UNPACK_ROW_LENGTH, im.step / im.elemSize());
		glDrawPixels(im.cols,im.rows, GL_BGR, GL_UNSIGNED_BYTE, im.data);
		glPixelStorei(GL_UNPACK_ROW_LENGTH, 0);
		
		
	}
	
		
	
	
	/// Draw an image to the frame buffer at the current raster position.
	/// Use glRasterPos to set the current raster position
	/// @param im The image to draw
	inline void glDrawPixelsGRAY(const cv::Mat &im)
	{
	
		//int alpha =  (im.step & 3) ? 1 : 4; 
		int alpha = std::min( cvdalignof( im.ptr<void>(0,0) ), 
				      cvdalignof( im.ptr<void>(1,0) ) );
	  
		glPixelStorei(GL_UNPACK_ALIGNMENT, alpha );
		//set length of one complete row in data (doesn't need to equal image.cols)
		//glPixelStorei(GL_UNPACK_ROW_LENGTH, im.step / im.elemSize());
		glPixelStorei(GL_UNPACK_ROW_LENGTH, im.step );
		glDrawPixels(im.cols,im.rows, GL_LUMINANCE, GL_UNSIGNED_BYTE, im.data);
		glPixelStorei(GL_UNPACK_ROW_LENGTH, 0);
		
	}

 	/// Read the current image from the colour buffer specified by glReadBuffer
	/// @param im The image to write the image data into. This must already be initialised to be an BasicImage (or Image) of the right size.
	/// @param origin The window co-ordinate of the first pixel to be read from the frame buffer
	inline void glReadPixelsBGR(cv::Mat &im, cv::Point2i origin = cv::Point2i(0,0) )
	{
		::glReadPixels(origin.x, origin.y, im.cols, im.rows, GL_BGR, GL_UNSIGNED_BYTE, im.data);
	}

 	/// Read the current image from the colour buffer specified by glReadBuffer
	/// @param size   The size of the area to read.
	/// @param origin The window co-ordinate of the first pixel to be read from the frame buffer
	inline cv::Mat glReadPixelsBGR(cv::Size2i size, cv::Point2i origin = cv::Point2i(0,0) )
	{
		cv::Mat img(size.height, size.width, CV_8UC3);
		
		//glReadPixels(origin.x, origin.y, img.cols, img.rows, GL_BGR, GL_UNSIGNED_BYTE);
		glReadPixelsBGR(img, origin);
		
		return img;
	}

	
	inline void glReadPixelsGRAY(cv::Mat &im, cv::Point2i origin = cv::Point2i(0,0) )
	{
		glReadPixels(origin.x, origin.y, im.cols, im.rows, GL_LUMINANCE, GL_UNSIGNED_BYTE, im.data);
	}

 	/// Read the current image from the colour buffer specified by glReadBuffer
	/// @param size   The size of the area to read.
	/// @param origin The window co-ordinate of the first pixel to be read from the frame buffer
	inline cv::Mat glReadPixelsGRAY(cv::Size2i size, cv::Point2i origin = cv::Point2i(0,0) )
	{
		cv::Mat img(size.height, size.width, CV_8UC1);
		
		glReadPixelsGRAY(img, origin);
		
		return img;
	}
	
	
	
	/// Sets an image as a texture sub region.
	/// note the reordering of the various parameters to make better use of default parameters
	/// @param im the image to set as texture
	inline void glTexSubImage2DBGR( const cv::Mat &img, GLint xoffset = 0, GLint yoffset = 0, GLenum target = GL_TEXTURE_2D, GLint level = 0)
	{
		int alpha = std::min( cvdalignof( img.ptr<void>(0,0) ), 
				      cvdalignof( img.ptr<void>(1,0) ) );
		::glPixelStorei(GL_UNPACK_ALIGNMENT, alpha);
		::glPixelStorei(GL_UNPACK_ROW_LENGTH, img.step / img.elemSize());
		::glTexSubImage2D(target, level, xoffset, yoffset, img.cols, img.rows, GL_BGR, GL_UNSIGNED_BYTE, img.data);
		::glPixelStorei(GL_UNPACK_ROW_LENGTH, 0);
		
		
	}

	inline void glTexSubImage2DGRAY( const cv::Mat_<uchar> &img, GLint xoffset = 0, GLint yoffset = 0, GLenum target = GL_TEXTURE_2D, GLint level = 0)
	{
		int alpha = std::min( cvdalignof( img.ptr<void>(0,0) ), 
				      cvdalignof( img.ptr<void>(1,0) ) );
		::glPixelStorei(GL_UNPACK_ALIGNMENT, alpha);
		::glPixelStorei(GL_UNPACK_ROW_LENGTH, img.step );
		::glTexSubImage2D(target, level, xoffset, yoffset, img.cols, img.rows, GL_LUMINANCE, GL_UNSIGNED_BYTE, img.data);
		::glPixelStorei(GL_UNPACK_ROW_LENGTH, 0);
	}
	
	/// Sets an image as a texture.
	/// note the reordering of the various parameters to make better use of default parameters
	/// @param i the image to set as texture
	inline void glTexImage2DBGR( const cv::Mat &img, 
				     GLint border = 0, 
				     GLenum target = GL_TEXTURE_2D, 
				     GLint level = 0)
	{
		int alpha = std::min( cvdalignof( img.ptr<void>(0,0) ), 
				      cvdalignof( img.ptr<void>(1,0) ) );
		::glPixelStorei(GL_UNPACK_ALIGNMENT, alpha);
		::glPixelStorei(GL_UNPACK_ROW_LENGTH, img.step / img.elemSize());
		::glTexImage2D(target, level, GL_BGR, img.cols, img.rows, border, GL_BGR, GL_UNSIGNED_BYTE, img.data);
		::glPixelStorei(GL_UNPACK_ROW_LENGTH, 0);
	
	  
		
		
	}
	
	

	/// Prints the current errors on the gl error stack
	inline void glPrintErrors(void) 
	{
	  GLenum code;
	  while(code = glGetError() != GL_NO_ERROR)
	    
	    std::cout << "GL:" << code << ":" << gluGetString(code) << std::endl;
	 
	}

	

	
} // END namespace
	

#endif