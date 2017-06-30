// George Terzakis 2016
//
// University of Portsmouth
//
// some simple additional utils that implement CVD funcationality 
// and have to be added here...

#ifndef ADDED_UTILS_H
#define ADDED_UTILS_H

#include "../OpenCV.h"
#include "Operators.h"

//#include "scalar_convert.h"

namespace CvUtils {
  
  
  
// Euler angles for R = Rz*Ry*Rx
// Based on a (most probably correct) report by G. Slabaugh
template<typename P>
  inline bool EulerAnglesZYX(const cv::Matx<P, 3, 3> &R, 
				    P &thetax,
				    P &thetay,
				    P &thetaz) 
  {
    if ( fabs(cv::determinant(R) - 1) < 0.00001 ) return false;
	
    P R11 = R(0, 0), R12 = R(0, 1), R13 = R(0, 2),
      R21 = R(1, 0), R22 = R(1, 1), R23 = R(1, 2),
      R31 = R(2, 0), R32 = R(2, 1), R33 = R(2, 2);
    
      // 1. Obtaining the angle about y
      if ( fabs( fabs(R31) - 1 ) > 0.0000001) { // angles near +- π / 2
      
	// now solve
	thetay = -asin(R31);
	
	// 2. Solving for thetax. Since we rejected the case where 
	P cos_thetay = cos(thetay);
	thetax = atan2( R32 / cos_thetay , R33 / cos_thetay  );
	
	// 3. Solving for thetaz
	thetaz = atan2( R21 / cos_thetay  , R11 / cos_thetay  );
	
	return true; // all well...
      }
      else {
	// thetaz is ambiguous (gimball lock!), set to zero
	thetaz = 0;
	if (R31 < 0) { // R31 == -1
	  
	  thetay = M_PI / 2;
	  thetax = thetaz + atan2(R12, R13);
	}
	 else {
	 
	   thetay = - M_PI / 2;
	   thetax = -thetaz + atan2(-R12 , -R13);
	}
	return false; // indicate gimball lock
      }
  }

  
/// Decimate grayscale image a la Rosten...
template <typename T>
void halfSample(const cv::Mat_<T> &src, cv::Mat_<T> &dest) {

  int rows = src.rows / 2;
  int cols = src.cols / 2;
		 
  if (dest.rows != rows && dest.cols != cols) dest.create(rows, cols);
  int r, c;
  T* dRowPtr;
  T* sRowPtr00;
  T* sRowPtr10;
  for(r=0; r < rows; r++) {
    dRowPtr = (T*)(dest.data + dest.step * r);
    sRowPtr00 = (T*)(src.data + src.step * (2 * r) );
    sRowPtr10 = (T*)(src.data + src.step * (2 * r + 1) );
			
    for(c=0; c < cols; c++)
      dRowPtr[c] = ( sRowPtr00[c*2] + sRowPtr10[c*2] + sRowPtr00[c*2+1] + sRowPtr10[c*2+1])/4;
  }
	
}


// get the mean of an image/matrix
// T better NOT be uchar 9see below for this case)
template <typename T>
inline double mavg(const cv::Mat_<T> &m) {

  		 
  int r, c;
  T* mRowPtr;
  
  double avg = 0.0;
  
  for(r = 0; r < m.rows; r++) {
    
    mRowPtr = (T*)(m.data + m.step * r);
    			
    for(c = 0; c < m.cols; c++) 
      avg += mRowPtr[c];
  }
  
  return avg / (m.rows * m.cols);
	
}



inline void pause(cv::Mat img)
{
  cv::namedWindow("pause");
  cv::imshow("pause", img);
  cv::waitKey(-1);
}

// After I gave it some thinking, i decided to add project/unproject functions for 2D and 3D vectors

// here's toon's "unproject" for 2D vectors (BUT WITHOUT the dynamic stuff - that's why i fixed the size)
// it should be as fast as it can be...
template<typename P>
inline cv::Vec<P, 3> backproject(const cv::Vec<P, 2> &v) {
  
  return cv::Vec<P, 3>(v[0], v[1], 1);
}
/// Returns the normalized homogeneous vector of the 2D coordinates
template<typename P>
inline cv::Vec<P, 4> backproject(const cv::Vec<P, 3> &v) {
  
  return cv::Vec<P, 4>(v[0], v[1], v[2], 1);
}

/// perspective projection of the lamest form.... (NOTE: NO division by zero povisions taken...)
template<typename P>
inline cv::Vec<P, 2> pproject(const cv::Vec<P, 3> &v) {
  
  if (v[2] == 0) return cv::Vec<P, 2>(v[0], v[1]);
  
  P invDepth = 1.0 / v[2];
  
  return cv::Vec<P, 2>(v[0] * invDepth, v[1] * invDepth);
}

/// perspective projection of the lamest form.... (NOTE: NO division by zero povisions are made...)
template<typename P>
inline cv::Vec<P, 3> pproject(const cv::Vec<P, 4> &v) {
 
   if (v[3] == 0) return cv::Vec<P, 3>(v[0], v[1], v[2] );
  
   P invDepth = 1.0 / v[3];
 
  return cv::Vec<P, 3>(v[0] * invDepth, v[1] * invDepth, v[2] * invDepth);
}



/// Round image location coordinates (nearest neighbor in a sense...)
template<typename P>
inline cv::Point2i roundIL(const cv::Vec<P, 2> &v) {

  return cv::Point2i((int)std::round(v[0]), std::round(v[1]) );
}

/// TRUNCATE image location coordinates
template<typename P>
inline cv::Point2i IL(const cv::Vec<P, 2> &v) {

  return cv::Point2i((int)v[0], (int)v[1] );
}


template <typename Tin, typename Tout, typename P> 
inline void sample(const cv::Mat_<Tin> &im, 
		    P x_, // x - coordinate to sample in the "in" matrix
		    P y_, // y - coordinate to sample in the "in" matrix
		    Tout &result    // location to bestow the (potentially) multichanneled samples
		  );

inline bool in_image_with_border(cv::Point2i pos, cv::Mat im, cv::Size2i border)
{
  return ( pos.x - border.width >= 0 ) && 
	 ( pos.y - border.height >= 0 ) &&
	 ( pos.x + border.width  < im.cols ) &&
         ( pos.y + border.height < im.rows );
};

inline bool in_image_with_border(int row, int col, cv::Mat im, int bwidth, int bheight)
{
  return ( col - bwidth >= 0 ) && 
	 ( row - bheight >= 0 ) &&
	 ( col + bwidth  < im.cols ) &&
         ( row + bheight < im.rows );
};

// 2x2 inverse!!!!!
template<typename T>
inline cv::Mat_<T> M2Inverse(const cv::Mat_<T> &m)
{
  cv::Mat_<T> m2Res(2,2);
  T tDet = m(0, 0) * m(1, 1) - m(1, 0) * m(0, 1);
  assert(tDet!=0.0);
  
  m2Res(0, 0) = m(1, 1) / tDet;
  m2Res(1, 1) = m(0, 0) / tDet;
  m2Res(1, 0) = -m(1, 0) / tDet;
  m2Res(0, 1) = -m(0, 1) / tDet;
  
  
  return m2Res;
}

// 2x2 inverse for Matx objects!!!!!!!!
template<typename T>
inline cv::Matx<T,2, 2> M2Inverse(const cv::Matx<T, 2, 2> &m)
{
  cv::Matx<T, 2,2 > m2Res;
  T tDet = m(0, 0) * m(1, 1) - m(1, 0) * m(0, 1);
  assert(tDet!=0.0);
  
  m2Res(0, 0) = m(1, 1) / tDet;
  m2Res(1, 1) = m(0, 0) / tDet;
  m2Res(1, 0) = -m(1, 0) / tDet;
  m2Res(0, 1) = -m(0, 1) / tDet;
  
  return m2Res;
}

/// 3x3 inverse!!!!!
template<typename T>
inline cv::Mat_<T> M3Inverse(const cv::Mat_<T> &m)
{
  cv::Mat_<T> m3Res(3,3);
  T det = m(0, 0) * ( m(1, 1) * m(2, 2)  - m(1, 2) * m(2, 1) ) - 
	       m(0, 1) * ( m(1, 0) * m(2, 2)  - m(1, 2) * m(2, 0) ) + 
	       m(0, 2) * ( m(1, 0) * m(2, 1)  - m(1, 1) * m(2, 0) );
  assert(det!=0.0);
  T invDet = 1.0 / det;
  m3Res(0, 0) =  ( m(1, 1) * m(2, 2) - m(2, 1) * m(1, 2) ) * invDet;
  m3Res(1, 0) = -( m(1, 0) * m(2, 2) - m(2, 0) * m(1, 2) ) * invDet;
  m3Res(2, 0) =  ( m(1, 0) * m(2, 1) - m(2, 0) * m(1, 1) ) * invDet;
 
  m3Res(0, 1) = -( m(0, 1) * m(2, 2) - m(2, 1) * m(0, 2) ) * invDet;
  m3Res(1, 1) =  ( m(0, 0) * m(2, 2) - m(2, 0) * m(0, 2) ) * invDet;
  m3Res(2, 1) = -( m(0, 0) * m(2, 1) - m(2, 0) * m(0, 1) ) * invDet;
  
  m3Res(0, 2) =  ( m(0, 1) * m(1, 2) - m(1, 1) * m(0, 2) ) * invDet;
  m3Res(1, 2) = -( m(0, 0) * m(1, 2) - m(1, 0) * m(0, 2) ) * invDet;
  m3Res(2, 2) =  ( m(0, 0) * m(1, 1) - m(1, 0) * m(0, 1) ) * invDet;
  
 
  
  return m3Res;
}

/// 3x3 inverse for Matx objects!!!!!
template<typename T>
inline cv::Matx<T, 3, 3> M3Inverse(const cv::Matx<T, 3, 3> &m)
{
  cv::Matx<T, 3, 3> m3Res;
  T det = m(0, 0) * ( m(1, 1) * m(2, 2)  - m(1, 2) * m(2, 1) ) - 
	       m(0, 1) * ( m(1, 0) * m(2, 2)  - m(1, 2) * m(2, 0) ) + 
	       m(0, 2) * ( m(1, 0) * m(2, 1)  - m(1, 1) * m(2, 0) );
  assert(det!=0.0);
  T invDet = 1.0 / det;
  m3Res(0, 0) =  ( m(1, 1) * m(2, 2) - m(2, 1) * m(1, 2) ) * invDet;
  m3Res(1, 0) = -( m(1, 0) * m(2, 2) - m(2, 0) * m(1, 2) ) * invDet;
  m3Res(2, 0) =  ( m(1, 0) * m(2, 1) - m(2, 0) * m(1, 1) ) * invDet;
 
  m3Res(0, 1) = -( m(0, 1) * m(2, 2) - m(2, 1) * m(0, 2) ) * invDet;
  m3Res(1, 1) =  ( m(0, 0) * m(2, 2) - m(2, 0) * m(0, 2) ) * invDet;
  m3Res(2, 1) = -( m(0, 0) * m(2, 1) - m(2, 0) * m(0, 1) ) * invDet;
  
  m3Res(0, 2) =  ( m(0, 1) * m(1, 2) - m(1, 1) * m(0, 2) ) * invDet;
  m3Res(1, 2) = -( m(0, 0) * m(1, 2) - m(1, 0) * m(0, 2) ) * invDet;
  m3Res(2, 2) =  ( m(0, 0) * m(1, 1) - m(1, 0) * m(0, 1) ) * invDet;
  
 
  
  return m3Res;
}


/// 3x3 inverse for Symmetric Matx using the Upper Triangle !!!!!
/// NOTE: The inverse is filled completely
template<typename T>
inline void M3Symm_UT_Inverse(const T (&V)[9], T (&Vinv)[9])
{
 // 1. Get the elements of the matrix
  T a = V[0], b = V[1], c = V[2],
    d = V[1], e = V[4], f = V[5],
    g = V[2], h = V[5], i = V[8];
 
 // 2. Determinant
 T det = a*e*i + b*f*g + c*d*h - g*e*c - h*f*a - i*d*b;
 assert(det != 0 && "Zero determinant!");
 T invDet = 1.0 / det;
 
 // 3. Adjoint and inverse
 Vinv[0] = ( e*i - f*h ) * invDet; Vinv[1] = ( h*c - i*b ) * invDet;   Vinv[2] = ( b*f - c*e ) * invDet;
 Vinv[3] =  Vinv[1];               Vinv[4] = ( a*i - g*c ) * invDet;   Vinv[5] = ( d*c - a*f ) * invDet;
 Vinv[6] =  Vinv[2];               Vinv[7] =   Vinv[5];                Vinv[8] = ( a*e - d*b ) * invDet;
 
}



/// 3x3 inverse for Symmetric Matx using the Lower Triangle !!!!!
/// NOTE: The inverse is filled completely
template<typename T>
inline void M3Symm_LT_Inverse(const T (&V)[9], T (&Vinv)[9])
{
 // 1. Get the elements of the matrix
  T a = V[0], b = V[3], c = V[6],
    d = V[3], e = V[4], f = V[7],
    g = V[6], h = V[7], i = V[8];
 
 // 2. Determinant
 T det = a*e*i + b*f*g + c*d*h - g*e*c - h*f*a - i*d*b;
 assert(det != 0 && "Zero determinant!");
 T invDet = 1.0 / det;
 
 // 3. Adjoint and inverse
 Vinv[0] = ( e*i - f*h ) * invDet; Vinv[1] = ( h*c - i*b ) * invDet;   Vinv[2] = ( b*f - c*e ) * invDet;
 Vinv[3] =  Vinv[1];               Vinv[4] = ( a*i - g*c ) * invDet;   Vinv[5] = ( d*c - a*f ) * invDet;
 Vinv[6] =  Vinv[2];               Vinv[7] =   Vinv[5];                Vinv[8] = ( a*e - d*b ) * invDet;
 
}

// Cholesky decomposition for 3x3 matrices PSD with data stored in the lower triangle
template<typename T>
inline bool M3PSD_LT_Cholesky(const T (&M)[9], T (&L)[9])
{
  
   if ( M[0] <= 0) return false;
   
   // L11
   T L11 = L[0] = sqrt(M[0]), L21, L22, L31, L32, L33;
   
   
   // L21
   T invL11 = 1.0 / L11;
   
   L21 = L[3] = invL11 * M[3];
   
   // L22
   if ( M[4] - L21 * L21 <= 0 ) return false;
   L22 = L[4] = sqrt( M[4] - L21 * L21);
   
   // L31
   L31 = L[6] = invL11 * M[6];
   
   // L32
   L32 = L[7] = ( 1.0 / L22 ) * ( M[7] - L31 * L21 );
   
   if ( M[8] - L32 * L32 - L31 * L31 < 0 ) return false; // leave the 0-case in to tell us if the matrix is singular
   
   L33 = L[8] = sqrt( M[8] - L32 * L32 - L31 * L31 );
   
   L[1] = L[2] = L[5] = 0;
   
   return true;
}



// This is the "templated" version of Rosten's code in OpenCV terms.
// I am using his transform function instead of OpenCV's WarpAffine
// because the devil hides in the details!
template <typename Tin, typename Tout, typename P>
int transform(const cv::Mat_<Tin> &in, 
	      cv::Mat_<Tout> &out, 
	      const cv::Mat_<P> &M,   // a 2x2 matrix (affine scaling) 
	      const cv::Vec<P, 2> &inOrig, // a 2x1 vector (translation)
	      const cv::Vec<P, 2> &outOrig, // and another 2x1 vector!
	      const Tout defaultValue = Tout())   // default value for boundary pixels
{
    const int w = out.cols, h = out.rows, iw = in.cols, ih = in.rows; 
    const cv::Vec<P, 2> across( M(0, 0), M(1, 0) ); // column #1 of M
    const cv::Vec<P, 2> down( M(0, 1), M(1, 1) );  // column #2 of M
    const cv::Vec<P, 2> p0 = inOrig - cv::Vec<P, 2>( M(0,0) * outOrig[0] + M(0,1) * outOrig[1], 
						     M(1,0) * outOrig[0] + M(1,1) * outOrig[1] );
    // trying to avoid the -UNTESTED- operator overloads for scalr * vector, just in case (they do work though...).
    const cv::Vec<P, 2> p1( p0[0]+ w*across[0], 
			    p0[1]+ w*across[1]
			  );
    const cv::Vec<P, 2> p2( p0[0] + h*down[0],
			    p0[1] + h*down[1] 
			  );
    const cv::Vec<P, 2> p3( p0[0] + w*across[0] + h*down[0], 
			    p0[1] + w*across[1] + h*down[1]
			  );
        
    // ul --> p0
    // ur --> w*across + p0
    // ll --> h*down + p0
    // lr --> w*across + h*down + p0
    P min_x = p0[0], min_y = p0[1];
    P max_x = min_x, max_y = min_y;
   
    // Minimal comparisons needed to determine bounds
    if (across[0] < 0)
	min_x += w*across[0];
    else
	max_x += w*across[0];
    
    if (down[0] < 0)
	min_x += h*down[0];
    else
	max_x += h*down[0];
    
    if (across[1] < 0)
	min_y += w*across[1];
    else
	max_y += w*across[1];
    
    if (down[1] < 0)
	min_y += h*down[1];
    else
	max_y += h*down[1];
   
    // This gets from the end of one row to the beginning of the next
    const cv::Vec<P, 2> carriage_return( down[0] - w*across[0],
					 down[1] - w*across[1] 
				       );

    //If the patch being extracted is completely in the image then no 
    //check is needed with each point.
    if (min_x >= 0 && min_y >= 0 && max_x < iw-1 && max_y < ih-1) 
    {
	cv::Vec<P, 2> p = p0;
	for (int r=0; r<h; ++r, p+=carriage_return)
	    for (int c=0; c<w; ++c, p+=across) 
		sample(in, p[0],p[1], out(r, c) );
	    
	return 0;
    } 
    else // Check each source location
    {
	// Store as doubles to avoid conversion cost for comparison
	const P x_bound = iw-1;
	const P y_bound = ih-1;
	int count = 0;
	cv::Vec<P, 2> p = p0;
	for (int r=0; r<h; ++r, p+=carriage_return) {
	    for (int c=0; c<w; ++c, p+=across) {
		//Make sure that we are extracting pixels in the image
		if (0 <= p[0] && 0 <= p[1] &&  p[0] < x_bound && p[1] < y_bound)
		    sample(in, p[0],p[1], out(r, c) );
		else {
		    out[r][c] = defaultValue;
		    ++count;
		}
	    }
	}
	return count;
    }
}


// transform overload for cv::Matx transformation matrix (preferred)
template <typename Tin, typename Tout, typename P>
int transform(const cv::Mat_<Tin> &in, 
	      cv::Mat_<Tout> &out, 
	      const cv::Matx<P, 2, 2> &M,   // a 2x2 matrix (affine scaling) 
	      const cv::Vec<P, 2> &inOrig, // a 2x1 vector (translation)
	      const cv::Vec<P, 2> &outOrig, // and another 2x1 vector!
	      const Tout defaultValue = Tout())   // default value for boundary pixels
{
    const int w = out.cols, h = out.rows, iw = in.cols, ih = in.rows; 
    const cv::Vec<P, 2> across( M(0, 0), M(1, 0) ); // column #1 of M
    const cv::Vec<P, 2> down( M(0, 1), M(1, 1) );  // column #2 of M
    const cv::Vec<P, 2> p0 = inOrig - cv::Vec<P, 2>( M(0,0) * outOrig[0] + M(0,1) * outOrig[1], 
						     M(1,0) * outOrig[0] + M(1,1) * outOrig[1] );
    // trying to avoid the -UNTESTED- operator overloads for scalar * vector, just in case (they do work though...).
    const cv::Vec<P, 2> p1( p0[0]+ w*across[0], 
			    p0[1]+ w*across[1]
			  );
    const cv::Vec<P, 2> p2( p0[0] + h*down[0],
			    p0[1] + h*down[1] 
			  );
    const cv::Vec<P, 2> p3( p0[0] + w*across[0] + h*down[0], 
			    p0[1] + w*across[1] + h*down[1]
			  );
        
    // ul --> p0
    // ur --> w*across + p0
    // ll --> h*down + p0
    // lr --> w*across + h*down + p0
    P min_x = p0[0], min_y = p0[1];
    P max_x = min_x, max_y = min_y;
   
    // Minimal comparisons needed to determine bounds
    if (across[0] < 0)
	min_x += w*across[0];
    else
	max_x += w*across[0];
    if (down[0] < 0)
	min_x += h*down[0];
    else
	max_x += h*down[0];
    if (across[1] < 0)
	min_y += w*across[1];
    else
	max_y += w*across[1];
    if (down[1] < 0)
	min_y += h*down[1];
    else
	max_y += h*down[1];
   
    // This gets from the end of one row to the beginning of the next
    const cv::Vec<P, 2> carriage_return( down[0] - w*across[0],
					 down[1] - w*across[1] 
				       );

    //If the patch being extracted is completely in the image then no 
    //check is needed with each point.
    if (min_x >= 0 && min_y >= 0 && max_x < iw-1 && max_y < ih-1) 
    {
	cv::Vec<P, 2> p = p0;
	for (int r=0; r<h; ++r, p+=carriage_return)
	    for (int c=0; c<w; ++c, p+=across) 
		sample(in,p[0],p[1], out(r, c) );
	return 0;
    } 
    else // Check each source location
    {
	// Store as doubles to avoid conversion cost for comparison
	const P x_bound = iw-1;
	const P y_bound = ih-1;
	int count = 0;
	cv::Vec<P, 2> p = p0;
	for (int r=0; r<h; ++r, p+=carriage_return) {
	    for (int c=0; c<w; ++c, p+=across) {
		//Make sure that we are extracting pixels in the image
		if (0 <= p[0] && 0 <= p[1] &&  p[0] < x_bound && p[1] < y_bound)
		    sample(in,p[0],p[1], out(r, c) );
		else {
		    out(r, c) = defaultValue;
		    ++count;
		}
	    }
	}
	return count;
    }
}


/// Interpolated sampling a la Rosten from single channel images - 
// works only for single channel images thank you!
template <typename Tin, typename Tout, typename P> 
inline void sample(const cv::Mat_<Tin> &im, 
		    P x_, // x - coordinate to sample in the "in" matrix
		    P y_, // y - coordinate to sample in the "in" matrix
		    Tout &result    // location to bestow the (potentially) multichanneled samples
		  )
{
  assert(im.channels() == 1 && "No more channels in sample() for the time being... Thank you!");
  
  double x = x_, y = y_;
  
  const int lx = (int)x;
  const int ly = (int)y;
  x -= lx;
  y -= ly;
  
  // Summing per channel. I think that this template concoction should work in any case...
  Tin* rowPtr0 = (Tin*)im.ptr(ly, lx);
  Tin* rowPtr1 = (Tin*)im.ptr(ly+1, lx);
	
  const double v00 = 1.0 * rowPtr0[0];
  const double v10 = 1.0 * rowPtr1[0];
  const double v01 = 1.0 * rowPtr0[1];
  const double v11 = 1.0 * rowPtr1[1];
  
  result = (Tout)(  (1-y)*( (1-x)*v00 + x*v01 ) + y * ( (1-x)*v10 + x*v11 ) ); 
 
}


// Determinant of 2x2	
template<typename T>
inline T M2Det(const cv::Mat_<T> &m)
{
  return m(0, 0) * m(1, 1)  - m(0, 1) * m(1, 0);
}

// Determinant of 2x2 Matx object!
template<typename T>
inline T M2Det(const cv::Matx<T, 2, 2> &m)
{
  return m(0, 0) * m(1, 1)  - m(0, 1) * m(1, 0);
}


// Determinant of 3x3
template<typename T>
inline T M3Det(const cv::Mat_<T> &m )
{
  return  
    m(0, 0) * ( m(1, 1) * m(2, 2)  - m(1, 2) * m(2, 1) ) - 
    m(0, 1) * ( m(1, 0) * m(2, 2)  - m(1, 2) * m(2, 0) ) + 
    m(0, 2) * ( m(1, 0) * m(2, 1)  - m(1, 1) * m(2, 0) );
}

// Determinant of 3x3 Matx object
template<typename T>
inline T M3Det(const cv::Matx<T, 3, 3> &m )
{
  return  
    m(0, 0) * ( m(1, 1) * m(2, 2)  - m(1, 2) * m(2, 1) ) - 
    m(0, 1) * ( m(1, 0) * m(2, 2)  - m(1, 2) * m(2, 0) ) + 
    m(0, 2) * ( m(1, 0) * m(2, 1)  - m(1, 1) * m(2, 0) );
}

// Determinant of 3x3 Symmetric Matx object from Lower triangle
template<typename T>
inline T M3Symm_LT_Det(const T (&V)[9] )
{
  // 1. Get the elements of the matrix
  T a = V[0], b = V[3], c = V[6],
    d = V[3], e = V[4], f = V[7],
    g = V[6], h = V[7], i = V[8];
 
 // 2. Determinant
 return a*e*i + b*f*g + c*d*h - g*e*c - h*f*a - i*d*b;

}


// Determinant of 3x3 Symmetric Matx object from Upper triangle
template<typename T>
inline T M3Symm_UT_Det(const T (&V)[9] )
{
  // 1. Get the elements of the matrix
  T a = V[0], b = V[1], c = V[2],
    d = V[1], e = V[4], f = V[5],
    g = V[2], h = V[5], i = V[8];
 
 // 2. Determinant
 return a*e*i + b*f*g + c*d*h - g*e*c - h*f*a - i*d*b;
}


template <typename P, int Sz>
inline cv::Matx<P, Sz, 1> normalize(const cv::Matx<P, Sz, 1> &v) {
 
  return ( 1.0 / cv::norm(v) ) * v; 
}

template <typename P, int Sz>
inline cv::Vec<P, Sz> normalize(const cv::Vec<P, Sz> &v) {
 
  return ( 1.0 / cv::norm(v) ) * v; 
}

template <typename P, int Sz>
inline cv::Matx<P, 1, Sz> normalize(const cv::Matx<P, 1, Sz> &v) {
 
  return ( 1.0 / cv::norm(v) ) * v;  
}



template <typename P>
inline cv::Mat_<P> normalize(const cv::Mat_<P> &m) {
  P n = cv::norm(m);
  
  return n > 0 ? m / n : m; 
}


// wouldn't use this unless I really needed it...
template <typename P, int Sz1, int Sz2>
const cv::Vec<P, Sz2>& slice(const cv::Vec<P,Sz1> &v, int offset) {
  cv::Vec<P, Sz2> ret;
  for (int i = 0; i < ret.rows; i++)
    ret[i] = v[i+offset];
  
  return ret;
}


// conversion from cv::mat_ to cv::vector
template<typename P, int Sz> 
inline cv::Vec<P, Sz>  mat2Vec(const cv::Mat_<P> &m) {
  cv::Vec<P, Sz> ret;
  
  int sz = m.rows >= m.cols ? m.rows : m.cols; 
  bool columnVec = m.rows == sz;
  for (int i = 0; i<sz; i++)
    ret[i] = columnVec ? m(i, 0) : m(0, i);
  
  return ret;
}


// conversion from cv::Vec to cv::Mat
template<typename P, int Sz> 
inline const cv::Mat_<P> mat2Vec(const cv::Vec<P, Sz> &v) {
  cv::Mat_<P> ret(v.rows, 1);
  
  for (int i = 0; i<v.rows; i++)
    ret[i] = v[i];
  
  return ret;
}


}




#endif