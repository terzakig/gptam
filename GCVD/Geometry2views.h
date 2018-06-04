// George Terzakis 2016
//
// University of Portsmouth

// This file contains functions for relative pose recovery and scene reconstruction from 2 camera views
// This stuff is useful for SLAM initialization and relative pose estimation with/without a map

#ifndef TWO_VIEW_GEOMETRY_H
#define TWO_VIEW_GEOMETRY_H

#include "../OpenCV.h"

#include <math.h>

namespace TwoViewGeometry {
  
  /// Horn's relative pose extraction from an essential matrix
  /// The function returns two posible rotation matrices and one unit baseline vector
  // 
  // The essential matrix convention is:
  //
  //              E = R'*[b]x
  //
  // where R is the rotation matrix containing the second view frame (wrt the first) in columnwise arangement
  // and b is the baseline vector in first vierw coordinates (i.e., the transformation can be represewnted as 
  // [R' | -R'*b]   
  template<typename P>
  bool extractSE3FromEssentialMatrix( cv::Matx<P, 3, 3> &m3E,  // The (pure) essential matrix
				      cv::Matx<P, 3, 3> &m3R1, // The recovered rotation matrix #1
				      cv::Matx<P, 3, 3> &m3R2, // The recovered rotation matrix #2
				      cv::Vec<P, 3> &v3b         // The recovered unit baseline
				    ) {
    
    bool ret = true;
    // first make sure that m3E is actually an essnetial matrix
    cv::Vec<P, 3> w;
    cv::Matx<P, 3, 3> U, Vt;
    cv::SVD::compute(m3E, w, U, Vt);
   
    
    if ( !(fabs(w[0] - w[1]) <10e-5 && w[1] > 0 && fabs(w[2]) < 10e-5 ) ) {
    
      ret =  false; // not a genuine essential matrix
    } 
    
    // imposing s1 = s2 = 1 and s3 = 0 anyway...
    cv::Matx<P, 3, 3> S = cv::Matx<P, 3, 3>::eye(); 
      
    S(2, 2) = 0;
    
    m3E = U * S * Vt;
    
    
    
    // OK, Move on...
    
    // 1. First need to obtain the adjoint of the essential matrix
    
    P e11, e12, e13, e21, e22, e23, e31, e32, e33;
    
    e11 = m3E(0, 0); e12 = m3E(0, 1); e13 = m3E(0, 2);
    e21 = m3E(1, 0); e22 = m3E(1, 1); e23 = m3E(1, 2);
    e31 = m3E(2, 0); e32 = m3E(2, 1); e33 = m3E(2, 2);
    
    // computing the adjoint of E
    cv::Matx<P, 3, 3> C;
    C(0,0) =   e22 * e33 - e32 * e23;  C(0,1) = -(e21 * e33 - e31 * e23); C(0,2) =   e21 * e32 - e31 * e22;
    C(1,0) = -(e12 * e33 - e32 * e13); C(1,1) =   e11 * e33 - e31 * e13;  C(1,2) = -(e11 * e32 - e31 * e12);
    C(2,0) =   e12 * e23 - e22 * e13;  C(2,1) = -(e11 * e23 - e21 * e13); C(2,2) =   e11 * e22 - e21 * e12;
    
   // 2. Computing a baseline vector.
    
    // get the diagonal elements of E'*E
    P ee1 = e11 * e11 + e21 * e21 + e31 * e31; 
    P ee2 = e12 * e12 + e22 * e22 + e32 * e32; 
    P ee3 = e13 * e13 + e23 * e23 + e33 * e33; 
    
    // Get the (1, 2) and (2, 3) off-diagonal elements of E'*E
    P ee12 = e11 * e12 + e21 * e22 + e31 * e32;
    P ee23 = e12 * e13 + e22 * e23 + e32 * e33;
    P ee13 = e11 * e13 + e21 * e23 + e31 * e33;
    
    // Now compute the absolute values of the baseline components
    P bx = sqrt( fabs(1 - ee1) ), 
      by = sqrt( fabs(1 - ee2) ), 
      bz = sqrt( fabs(1 - ee3) );
   
   // 2. Now working-out the signs, provided that the maximum of bx, by, bz is positive 
   // (Just an arbitrary choice of direction, since the other baseline vector will be -b)
   if ( bx > by && bx > bz ) {
	
     if ( ee12 > 0 ) by = -by;
     if ( ee13  > 0) bz = -bz;
   }
   else if ( by > bz ) {
        
       if ( ee12 > 0) bx = -bx;
       if ( ee23 > 0) bz = -bz;
   }
    else {
        
      if ( ee13 > 0 ) bx = -bx;  
      if ( ee23 > 0 ) by = -by;
    }
   
   
   // 3. Just computing the rotation matrices!
   // a. R1 = C' + [b]x*E'
   m3R1(0, 0) = C(0, 0) - bz *  e12 +  by  *  e13;
   m3R1(0, 1) = C(1, 0) - bz *  e22 +  by  *  e23;
   m3R1(0, 2) = C(2, 0) - bz *  e32 +  by  *  e33;
   
   m3R1(1, 0) = C(0, 1) + bz *  e11 -  bx  *  e13;
   m3R1(1, 1) = C(1, 1) + bz *  e21 -  bx  *  e23;
   m3R1(1, 2) = C(2, 1) + bz *  e31 -  bx  *  e33;
   
   m3R1(2, 0) = C(0, 2) - by *  e11 +  bx  *  e12;
   m3R1(2, 1) = C(1, 2) - by *  e21 +  bx  *  e22;
   m3R1(2, 2) = C(2, 2) - by *  e31 +  bx  *  e32;
   
   // b. R2 = C' - [b]x*E'
   m3R2(0, 0) = C(0, 0) - (-bz *  e12 +  by  *  e13);
   m3R2(0, 1) = C(1, 0) - (-bz *  e22 +  by  *  e23);
   m3R2(0, 2) = C(2, 0) - (-bz *  e32 +  by  *  e33);
   
   m3R2(1, 0) = C(0, 1) - ( bz *  e11 -  bx  *  e13);
   m3R2(1, 1) = C(1, 1) - ( bz *  e21 -  bx  *  e23);
   m3R2(1, 2) = C(2, 1) - ( bz *  e31 -  bx  *  e33);
   
   m3R2(2, 0) = C(0, 2) - (-by *  e11 +  bx  *  e12);
   m3R2(2, 1) = C(1, 2) - (-by *  e21 +  bx  *  e22);
   m3R2(2, 2) = C(2, 2) - (-by *  e31 +  bx  *  e32);
   
   // 4. assigining baseline
   v3b[0] = bx; v3b[1] = by; v3b[2] = bz;
   
  
   return ret;
   
  } // End extractSE3FromEssentialMatrix

  
  
  /// Returns the cross-product skew symmetric matrix of a 3-vector
  template<typename P>
  inline cv::Matx<P, 3, 3> CrossPoductMatrix(const cv::Vec<P, 3> &v3u) {
   
    cv::Matx<P, 3, 3> m;
    m(0, 0) = 0;        m(0, 1) = -v3u[2];   m(0, 2) =  v3u[1];
    m(1, 0) =  v3u[2];  m(1, 1) = 0;         m(1, 2) = -v3u[0];
    m(2, 0) = -v3u[1];  m(2, 1) = v3u[0];    m(2, 2) = 0;
    
    return m;
  }
  
  
  // Obtain the depth of a point in two view two view geometry 
  // from the relative pose (in fashion m2 = R'*(Z1 * m1 - b) )
  template<typename P1, typename P2, typename P3>
  inline double recontructDepth2Views(P1 x1, P1 y1, 
				       P2 x2, P2 y2, 
				       const cv::Matx<P2, 3, 3> &m3R, 
				       const cv::Vec<P3, 3> &v3b)  {
    
      
      double C[6];
      // now need to obtain the depth :
      // We use the formula:
      // 
      //	   m1' * R * Q * R' * b	
      //    Z =   ----------------------
     //            m1' * R * Q * R' * m1
      // where
      //          Q = (m2*1z' - eye(3))'*(m2*1z' - eye(3))
    
      // *********** computing C = R * Q * R' (only upper triangle) *****************
      C[0] = ( m3R(0, 0) - m3R(0, 2) * x2 ) * ( m3R(0, 0) - m3R(0, 2) * x2 ) + ( m3R(0, 1) - m3R(0, 2) * y2 ) * ( m3R(0, 1) - m3R(0, 2) * y2 );
      
      C[1] = ( m3R(0, 0) - m3R(0, 2) * x2 ) * ( m3R(1, 0) - m3R(1, 2) * x2 ) + ( m3R(0, 1) - m3R(0, 2) * y2 ) * ( m3R(1, 1) - m3R(1, 2) * y2 );  
      
      C[2] = ( m3R(0, 0) - m3R(0, 2) * x2 ) * ( m3R(2, 0) - m3R(2, 2) * x2 ) + ( m3R(0, 1) - m3R(0, 2) * y2 ) * ( m3R(2, 1) - m3R(2, 2) * y2 );
     
      C[3] = ( m3R(1, 0) - m3R(1, 2) * x2 ) * ( m3R(1, 0) - m3R(1, 2) * x2 ) + ( m3R(1, 1) - m3R(1, 2) * y2 ) * ( m3R(1, 1) - m3R(1, 2) * y2 );
      
      C[4] = ( m3R(1, 0) - m3R(1, 2) * x2 ) * ( m3R(2, 0) - m3R(2, 2) * x2 ) + ( m3R(1, 1) - m3R(1, 2) * y2 ) * ( m3R(2, 1) - m3R(2, 2) * y2 );
     
      C[5] = ( m3R(2, 0) - m3R(2, 2) * x2 ) * ( m3R(2, 0) - m3R(2, 2) * x2 ) + ( m3R(2, 1) - m3R(2, 2) * y2 ) * ( m3R(2, 1) - m3R(2, 2) * y2 );
     
      // now obtain the numerator and denominator that yield the depth fraction 
      double denominator = x1 * ( x1 * C[0] + y1 * C[1] + C[2] ) + 
	 		   y1 * ( x1 * C[1] + y1 * C[3] + C[4] ) +
			          x1 * C[2] + y1 * C[4] + C[5];
				 
      double numerator = v3b[0] * ( x1 * C[0] + y1 * C[1] + C[2] ) + 
			 v3b[1] * ( x1 * C[1] + y1 * C[3] + C[4] ) +
			 v3b[2] * ( x1 * C[2] + y1 * C[4] + C[5] );
      
      if ( fabs(denominator) < 10e-6 ) return -1;
      
      return numerator / denominator;
       
  }
  
  
} // End the namespace clause


#endif
