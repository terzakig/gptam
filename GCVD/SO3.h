#ifndef SO3_H
#define SO3_H


#include <cassert>

#include "Operators.h"
#include "Addedutils.h"

#include <math.h>
#include <iostream>

#include "../OpenCV.h"

using namespace CvUtils;
using namespace MyOperatorOverloads;

namespace RigidTransforms {

template <typename Precision> class SO3;
template <typename Precision> class SE3;
//template <typename Precision> class SIM3;

template<class Precision> inline std::istream & operator >>(std::istream &, SO3<Precision> & );
template<class Precision> inline std::istream & operator >>(std::istream &, SE3<Precision> & );
//template<class Precision> inline std::istream & operator >>(std::istream &, SIM3<Precision> & );





/// Class to represent a three-dimensional rotation matrix. Three-dimensional rotation
/// matrices are members of the Special Orthogonal Lie group SO3. This group can be parameterised
/// three numbers (a vector in the space of the Lie Algebra). In this class, the three parameters are the
/// finite rotation vector, i.e. a three-dimensional vector whose direction is the axis of rotation
/// and whose length is the angle of rotation in radians. Exponentiating this vector gives the matrix,
/// and the logarithm of the matrix gives this vector.
template <typename Precision = float>
class SO3 {
  
private:

	struct Invert {}; 
	// This weird constructor simple creates an SO3 from another SO3 by using its transposed matrix.
	// What Rosten does, is he is using a dummy struct (not the first time) in order to point-out the constructor 
	// that does this job as opposed to the default SO3(SO3) constructor...
	inline SO3(const SO3& so3, const Invert&) : mat_(so3.mat_.t()) {}
	
	cv::Matx<Precision, 3, 3> mat_;
  
  
public:
	friend std::istream& operator>> <Precision> (std::istream& is, SO3<Precision> & rhs);
	friend std::istream& operator>> <Precision> (std::istream& is, SE3<Precision> & rhs);
	//friend std::istream& operator>> <Precision> (std::istream& is, SIM3<Precision> & rhs);
	friend class SE3<Precision>;
	//friend class SIM3<Precision>;

	/// Default constructor. Initialises the matrix to the identity (no rotation)
	SO3() : mat_( cv::Matx<Precision, 3, 3>::eye() ) {} 
	
	
	/// Construct from the axis of rotation (and angle given by the magnitude).
	template <typename P>
	SO3(const cv::Vec<P, 3> &v) { *this = exp(v); }
	
	/// Construct from a rotation matrix.
	template <typename P>
	SO3(const cv::Matx<P, 3, 3> &r) : mat_(r) { coerce(); 
	  //orthoProcrustes();
	}
	
	/// creates an SO3 as a rotation that takes Vector a into the direction of Vector b
	/// with the rotation axis along a ^ b. If |a ^ b| == 0, it creates the identity rotation.
	/// An assertion will fail if Vector a and Vector b are in exactly opposite directions.
	template <typename P1, typename P2>
	SO3(const cv::Vec<P1, 3> &a, const cv::Vec<P2, 3> &b ) {
		
		// Get the third axis
		cv::Vec<Precision, 3> n = a ^ b;
		// if zero norm, then the two vectors are collinear. bad matterials to build a rotation matrix. Identity or exit!
		if(cv::norm(n) == 0) {
			//check that the vectors are in the same direction if cross product is 0. If not,
			//this means that the rotation is 180 degrees, which leads to an ambiguity in the rotation axis.
			assert(a * b >= 0 && "Attempted to construct an SO3 from two collinear oposite vectors!");
			mat_ = cv::Mat_<Precision>::eye(3,3);
			return;
		}
		
		// make n a unit vector
		n = cv::normalize(n);
		cv::Matx<Precision, 3, 3> R1;
		cv::Vec<P1, 3>  a_normalized = cv::normalize(a);
		// 1. Put normalized a in the first column of the rotation matrux
		R1(0, 0) = a_normalized[0]; R1(1, 0) = a_normalized[1]; R1(2, 0) = a_normalized[2];
		
		// 2. Now put normalized n in the second column of the rotatin matrix (why not in the 3d???? - see below for rationale)
		R1(0,1) = n[0]; R1(1,1) = n[1]; R1(2,1) = n[2]; //R1.t()[1] = n;
		
		//2. Finally, put their cross product in the 3d column : 
		R1(0, 2) = -a_normalized[2] * n[1] + a_normalized[1] * n[2];
		R1(1, 2) =  a_normalized[2] * n[0] - a_normalized[0] * n[2];
		R1(2, 2) = -a_normalized[1] * n[0] + a_normalized[0] * n[1];
		
		
		// *************** ALSO: One more rotation matrix starting from b this time! **********************:
		cv::Vec<P2, 3> b_normalized = cv::normalize(b);
		//1. Store normalized b in first column of mat_ 
		mat_(0,0) = b_normalized[0]; mat_(1,0) = b_normalized[1]; mat_(2,0) = b_normalized[2];
		
		//2. Again, store n in the middle ... 
		mat_(0, 1) = n[0]; mat_(1, 1) = n[1]; mat_(2, 1) = n[2];
		
		//3. Finally, take the cross product bxn and store it in the third column ... mat_.T()[2] = mat_.T()[0] ^ n;
		cv::Vec<Precision, 3> mc1_cross_n = b_normalized ^ n;
		mat_(0, 2) = -b_normalized[2] * n[1] + b_normalized[1] * n[2];
		mat_(1, 2) =  b_normalized[2] * n[0] - b_normalized[0] * n[2];
		mat_(2, 2) = -b_normalized[1] * n[0] + b_normalized[0] * n[1];
		
		
		// FINALLY,multiply mat_ by R1' and store in mat_
		mat_ = mat_ * R1.t();
	}
	
	/// Assignment operator from a general matrix. This also calls coerce()
	/// to make sure that the matrix is a valid rotation matrix.
	template <typename P>
	SO3& operator =(const cv::Matx<P, 3, 3>  &r) {
		mat_ = r;
		coerce(); // avoid this &^*$*$ for now
		return *this;
	}
	
	// Some helper functions...
	inline void assignRowAt(int r, const cv::Matx<Precision, 3, 1> &v) {
	  mat_(r, 0) = v(0, 0); 
	  mat_(r, 1) = v(1, 0); 
	  mat_(r, 2) = v(2, 0);
	}
	inline void assignRowAt(int r, const cv::Matx<Precision, 1, 3> &v) {
	  mat_(r, 0) = v(0, 0); 
	  mat_(r, 1) = v(0, 1); 
	  mat_(r, 2) = v(0, 2);
	}
	inline void assignColAt(int c, const cv::Matx<Precision, 3, 1> &v) {
	  mat_(0, c) = v(0, 0); 
	  mat_(1, c) = v(1, 0); 
	  mat_(2, c) = v(2, 0);
	}
	inline void assignColAt(int c, const cv::Matx<Precision, 1, 3> &v) {
	  mat_(0, c) = v(0, 0); 
	  mat_(1, c) = v(0, 1); 
	  mat_(2, c) = v(0, 2);
	}
	/// Modifies the matrix to make sure it is a valid rotation matrix.
	// Perhaps the procrustean approach would be better as it gives the closest (in the least squares sense)
	void coerce() {
	  
	  // 1. normalize the 1st row
	  assignRowAt(0, CvUtils::normalize( mat_.row(0) ) ); // use CvUtils::normalize for Max vectors
	
	  // 2. Project the second on the first and normalize the difference of the second minus the projection
	  assignRowAt(1, CvUtils::normalize( mat_.row(1) - ( mat_.row(0).dot( mat_.row(1) ) ) * mat_.row(0) ) );
		
	  // 3. Project the 3d row to the 1st and the 2nd and then take the normalized difference from the sum of the projections
	  assignRowAt(2, CvUtils::normalize( mat_.row(2) - ( mat_.row(1).dot( mat_.row(2) ) ) * mat_.row(1) - ( mat_.row(0).dot( mat_.row(2) ) ) * mat_.row(0) ) );
		
	  // check for positive determinant <=> right handed coordinate system of row vectors
	  //assert( (mat_[0] ^ mat_[1]) * mat_[2] > 0 ); 
	  // in essence, GK asserts that ( row1 x row2 ) . row3 = 1 (and perhaps, NOT -1)
	  //cout << " SO3 determinant : " << (rowAt(0) ^ rowAt(1)) * rowAt(2) << endl;
	  //cout << "The matrix : " << get_matrix()<<endl;
	  
	  assert( ( mat_.row(0) ^ mat_.row(1) ).dot( mat_.row(2) ) > 0  && "Coercing did not produce a rotation matrix, or you gave me low-rank materials!" ); 
	  
	}	
	
	/// The orthogonal Procrustes method (see papers by Grace Wabha and Berthold Horm on recovering orientation using rotation matrices)
	// The procrustean methdo returns the CLOSEST orthinormal matrix matrix in the LS sense
	void orthoProcrustes() {
	  // SVD decomposing mat_
	  cv::Matx<Precision, 3, 3> U;
	  cv::Matx<Precision, 3, 3> Vt;
	  cv::Vec<Precision, 3> w;
	  cv::SVD::compute(mat_, w, U, Vt);
	  
	  mat_ = U * Vt;
	  
	  assert(CvUtils::M3Det(mat_) > 0 && "This matrix has negative determinant!");
	}
	
	/// Exponentiate a vector in the Lie algebra to generate a new SO3.
	/// See the Detailed Description for details of this vector.
	template<typename VP> inline static SO3 exp(const cv::Vec<VP, 3>&);
	
	/// Take the logarithm of the matrix, generating the corresponding vector in the Lie Algebra.
	/// See the Detailed Description for details of this vector.
	inline cv::Vec<Precision, 3> ln() const;
	
	/// Returns the inverse of this matrix (=the transpose, so this is a fast operation)
	SO3 inverse() const { return SO3(*this, Invert()); }

	/// Right-multiply by another rotation matrix
	template <typename P>
	SO3& operator *=(const SO3<P >&right) {
		
	   *this = *this * right;
	   
	   return *this;
	}

	/// Right-multiply by another SO3
	template<typename P>
	SO3<typename MyOperatorOverloads::MultiplyType<Precision, P>::type> operator *(const SO3<P> &right) const { 
	  
	    return SO3<>( this->get_matrix() * right.get_matrix() ); 
	}

	/// Returns the SO3 as a Matrix<3>
	const cv::Matx<Precision, 3, 3>& get_matrix() const {return mat_;}

	cv::Matx<Precision, 3, 3>& get_matrix() {return mat_;}
	/// Returns the i-th generator.  The generators of a Lie group are the basis
	/// for the space of the Lie algebra.  For %SO3, the generators are three
	/// \f$3\times3\f$ matrices representing the three possible (linearised)
	/// rotations.
	inline static cv::Matx<Precision, 3, 3> generator(int i) {
	  
		cv::Matx<Precision, 3, 3> result = cv::Matx<Precision, 3, 3>::zeros();
		result( (i+1)%3, (i+2)%3) = -1;
		result( (i+2)%3, (i+1)%3) = 1;
		return result;
	}

	/// Returns the i-th generator times pos. George:This essentially is a shortcut for the derivative in terms of the axis-angle vector
      inline static cv::Vec<Precision, 3> generator_field(int i, const cv::Vec<Precision, 3> &pos) {
    
	cv::Vec<Precision, 3> result;
	
	result[i]=0;
	result[(i+1)%3] = -pos[(i+2)%3];
	result[(i+2)%3] = pos[(i+1)%3];
	return result;
      }	
      /// Transfer a vector in the Lie Algebra from one
      /// co-ordinate frame to another such that for a matrix 
	/// \f$ M \f$, the adjoint \f$Adj()\f$ obeys
      /// \f$ e^{\text{Adj}(v)} = Me^{v}M^{-1} \f$
      inline cv::Vec<Precision, 3> adjoint(const cv::Vec<Precision, 3> &v) const  { 
		
	  return *this * v; 
      }

      template <typename PA, typename PB>
      inline SO3(const SO3<PA> &a, const SO3<PB> &b) : mat_(a.get_matrix()*b.get_matrix()) {}
	
}; // ********** Class SO3 ends here! - Class SO3 ends here! - Class SO3 ends here! - Class SO3 ends here! **********


} // ************* Closing RogidTransforms in order to define operator overloads and statics/independent functions *******************


/// Write an SO3 to a stream 
template <typename Precision>
inline std::ostream& operator << (std::ostream& os, const RigidTransforms::SO3<Precision> &so3) {

  return os << so3.get_matrix();
 }

 /// Read from SO3 to a stream 
template <typename Precision>
inline std::istream& operator >>(std::istream& is, RigidTransforms::SO3<Precision>& so3) {
	
  is >> so3.mat_;
  so3.coerce(); 
  return is;
}




///Compute a rotation exponential using the Rodrigues Formula.
///The rotation axis is given by \f$\vec{w}\f$, and the rotation angle must
///be computed using \f$ \theta = |\vec{w}|\f$. This is provided as a separate
///function primarily to allow fast and rough matrix exponentials using fast 
///and rough approximations to \e A and \e B.
///
///@param w Vector about which to rotate.
///@param A \f$\frac{\sin \theta}{\theta}\f$
///@param B \f$\frac{1 - \cos \theta}{\theta^2}\f$
///@param R Matrix to hold the return value.
///@relates SO3
template <typename PV, typename Ps, typename P>
inline static void rodrigues_so3_exp(const cv::Vec<PV, 3> &w,  const Ps A, const Ps B, cv::Matx<P, 3, 3> &R) {
    	
	// This is basically Rodrigues' formula given the sin and cos ratios in the argument list
        // (which are provided by a separate fnction i order to work out singular quantitities in the formula (AKA "A" and "B" near the origin)
	
  
	// NOTE! There is an easy workaournd to the singularites of the exponential map and its derivatives to rotations:
	// 	 Of course it is quaternions and, yes(!) sterographic projection.
	// The above do not change Rosten's cool solutions to numerical instabilities near the origin... 
	{	
	const P wx2 = (P)w[0]*w[0];
	const P wy2 = (P)w[1]*w[1];
	const P wz2 = (P)w[2]*w[2];
	
	R(0, 0) = 1.0 - B*(wy2 + wz2);
	R(1, 1) = 1.0 - B*(wx2 + wz2);
	R(2, 2) = 1.0 - B*(wx2 + wy2);
	}
	
	{
	const P a = A*w[2];
	const P b = B*(w[0]*w[1]);
	R(0, 1) = b - a;
	R(1, 0) = b + a;
	}
	
	{
	const P a = A*w[1];
	const P b = B*(w[0]*w[2]);
	R(0, 2) = b + a;
	R(2, 0) = b - a;
	}
	
	{
	const P a = A*w[0];
	const P b = B*(w[1]*w[2]);
	R(1, 2) = b - a;
	R(2, 1) = b + a;
	}
	
}


  
///Perform the exponential of the matrix \f$ \sum_i w_iG_i\f$
///@param w Weightings of the generator matrices.
template <typename Precision>
template<typename VP>
inline RigidTransforms::SO3<Precision> RigidTransforms::SO3<Precision>::exp(const cv::Vec<VP, 3> &w) {
	using std::sqrt;
	using std::sin;
	using std::cos;
	
	static const Precision one_6th = 1.0/6.0;
	static const Precision one_20th = 1.0/20.0;
	
	RigidTransforms::SO3<Precision> result;
	
	// Get the angle
	const Precision theta_sq = w[0]*w[0] + w[1]*w[1] + w[2]*w[2];
	const Precision theta = sqrt(theta_sq);
	// These quntities (A, B) are terms of the Rodrigues formula. Rosten is cleverly
	// precalulating A and B in order to use Taylor approximations when the angle (theta) is very small...
	// That's where these 1/6 and 1/20 terms come from I presume (need to verify on paper though...)
	Precision A, B;
	//Use a Taylor series expansion near zero. This is required for
	//accuracy, since sin t / t and (1-cos t)/t^2 are both 0/0.
	
	if (theta_sq < 1e-8) {
		A = 1.0 - one_6th * theta_sq;
		B = 0.5;
	} else {
		if (theta_sq < 1e-6) {
			B = 0.5 - 0.25 * one_6th * theta_sq;
			A = 1.0 - theta_sq * one_6th*(1.0 - one_20th * theta_sq);
		} else {
			//const Precision inv_theta = 1.0/theta;
			A = sin(theta) / theta;
			B = (1 - cos(theta)) / theta_sq;
		}
	}
	// now invoke the rest of the Rodrigues formula computation...
	rodrigues_so3_exp( w, A, B, result.get_matrix() );
	
	return result;
}

/// get the axis-angle (Lie) vector of a rotation
// This is the most BRILLIANT Lie logarithm implementation I have seen! I hope it generally is as robust as it seems.
// The latter doesnt change my opinion of Lie exponentials..... This class will soon use quaternions as well and their exponentials are robust
template <typename Precision>
inline cv::Vec<Precision, 3> RigidTransforms::SO3<Precision>::ln() const {
    using std::sqrt;
    cv::Vec<Precision, 3> result;
	
    const Precision cos_angle = (mat_(0, 0) + mat_(1, 1) + mat_(2, 2) - 1.0) * 0.5;
    result[0] = ( mat_(2, 1) - mat_(1, 2) ) / 2;
    result[1] = ( mat_(0, 2) - mat_(2, 0) ) / 2;
    result[2] = ( mat_(1, 0) - mat_(0, 1) ) / 2;
	
    Precision sin_angle_abs = cv::norm(result);
    if (cos_angle > M_SQRT1_2) {            // [0 - Pi/4] use asin
      
      if(sin_angle_abs > 0)
	    result *= asin(sin_angle_abs) / sin_angle_abs;
    
    } 
     else if( cos_angle > -M_SQRT1_2) {    // [Pi/4 - 3Pi/4] use acos, but antisymmetric part
	
       const Precision angle = acos(cos_angle);
	result *= angle / sin_angle_abs;        
    
    } else {  // rest use symmetric part
	// antisymmetric part vanishes, but still large rotation, need information from symmetric part
	const Precision angle = M_PI - asin(sin_angle_abs);
	const Precision d0 = mat_(0, 0) - cos_angle, 
			d1 = mat_(1, 1) - cos_angle,
			d2 = mat_(2, 2) - cos_angle;
	cv::Vec<Precision, 3> r2;
	if(d0*d0 > d1*d1 && d0*d0 > d2*d2){ // first is largest, fill with first column
	  
			r2[0] = d0;
			r2[1] = ( mat_(1, 0) + mat_(0, 1) ) / 2;
			r2[2] = ( mat_(0, 2) + mat_(2, 0) ) / 2;
		} 
		else if(d1*d1 > d2*d2) { // second is largest, fill with second column
		  
			r2[0] = ( mat_(1, 0) + mat_(0, 1) ) / 2;
			r2[1] = d1;
			r2[2] = ( mat_(2, 1) + mat_(1, 2) ) / 2;
		} 
		else {	// third is largest, fill with third column
		  
			r2[0] = ( mat_(0, 2) + mat_(2, 0) ) / 2;
			r2[1] = ( mat_(2, 1) + mat_(1, 2) ) / 2;
			r2[2] = d2;
		}
		// flip, if we point in the wrong direction!
		if(r2.dot( result ) < 0) r2 *= -1;
		r2 = (1 / cv::norm(r2)) * r2;
		
		result = angle * r2; // scalar * vector
	} 
	return result;
}



// *************************************** Operator Overloads for SO3 objects ****************************************

/// Right-multiply by a Vector - relays VEC
template<typename P, typename PV> 
inline cv::Vec<typename MyOperatorOverloads::MultiplyType<P, PV>::type, 3> operator *(const RigidTransforms::SO3<P> &so3, const cv::Vec<PV, 3> &v) {
	
  
  typedef typename MyOperatorOverloads::MultiplyType<P, PV>::type P0;
  
  // the below is not necessary (operator is overloaded), but just in case....
  cv::Matx<P, 3, 3> m3R = so3.get_matrix();
  cv::Vec<P0, 3> res( m3R(0, 0) * v[0] + m3R(0, 1) * v[1] + m3R(0, 2) * v[2],
		      m3R(1, 0) * v[0] + m3R(1, 1) * v[1] + m3R(1, 2) * v[2], 
		      m3R(2, 0) * v[0] + m3R(2, 1) * v[1] + m3R(2, 2) * v[2] 
		      
		    );
  
  return res;
}

/// Left-multiply by a Vector - returns VEC
template<typename P, typename PV> 
inline cv::Vec<typename MyOperatorOverloads::MultiplyType<PV, P>::type, 3> operator *(const cv::Vec<PV, 3> &v, const RigidTransforms::SO3<P> &so3){

  typedef typename MyOperatorOverloads::MultiplyType<P, PV>::type P0;
  cv::Matx<P, 3, 3> m3R = so3.get_matrix();
  cv::Vec<P0, 3> res( m3R(0, 0) * v[0] + m3R(1, 0) * v[1] + m3R(2, 0) * v[2],
		      m3R(0, 1) * v[0] + m3R(1, 1) * v[1] + m3R(2, 1) * v[2],
		      m3R(0, 2) * v[0] + m3R(1, 2) * v[1] + m3R(2, 2) * v[2]
		    );
    
  return res;
}



/// Right-multiply by a matrix
/// returns mat
template<typename P, typename PM> 
inline cv::Mat_<typename MyOperatorOverloads::MultiplyType<P, PM>::type> operator *(const RigidTransforms::SO3<P> &so3, const cv::Mat_<PM> &M) {
	
  return cv::Mat(so3.get_matrix()) * M;
}

/// Left-multiply by a matrix
/// returns Mat
template<typename PM, typename P> 
inline cv::Mat_<typename MyOperatorOverloads::MultiplyType<PM, P>::type> operator *(const cv::Mat_<PM> &M, const RigidTransforms::SO3<P> &so3) {
	
  return M * cv::Mat(so3.get_matrix());
}




/// Right-multiply by a matrix Matx
/// returns Matx
template<typename P, typename PM, int Cols> 
inline cv::Matx<typename MyOperatorOverloads::MultiplyType<P, PM>::type, 3, Cols> operator *(const RigidTransforms::SO3<P> &so3, 
										    const cv::Matx<PM, 3, Cols> &M) {
	
  return so3.get_matrix() * M;
}

/// Left-multiply by a matrix
/// returns Mat
template<typename PM, int Rows, typename P> 
inline cv::Matx<typename MyOperatorOverloads::MultiplyType<PM, P>::type, Rows, 3> operator *(const cv::Matx<PM, Rows, 3> &M, 
											     const RigidTransforms::SO3<P> &so3) {
	
  return M * so3.get_matrix();
}




#endif