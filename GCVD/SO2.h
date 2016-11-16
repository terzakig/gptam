#ifndef SO2_H
#define SO2_H

#include "Operators.h"
#include "Addedutils.h"

#include "../OpenCV.h"

#include <math.h>


namespace RigidTransforms {

template <typename Precision> class SO2;
template <typename Precision> class SE2;
//template <typename Precision> class SIM2;

template<typename Precision> inline std::istream & operator >> (std::istream&, SO2<Precision>& );
template<typename Precision> inline std::istream & operator >> (std::istream&, SE2<Precision>& );
//template<typename Precision> inline std::istream & operator >> (std::istream&, SIM2<Precision>& );

/// Class to represent a two-dimensional rotation matrix. Two-dimensional rotation
/// matrices are members of the Special Orthogonal Lie group SO2. This group can be parameterised with
/// one number (the rotation angle).
template<typename Precision = float>
class SO2 {
	friend std::istream& operator>> <Precision>(std::istream&, SO2& );
	friend std::istream& operator>> <Precision>(std::istream&, SE2<Precision>& );
	//friend std::istream& operator>> <Precision>(std::istream&, SIM2<Precision>& );

private: 
	struct Invert {};
	inline SO2(const SO2 &so2, const Invert&) : mat_(so2.mat_.t()) {}
	template <typename PA, typename PB>
	inline SO2(const SO2<PA>& a, const SO2<PB>& b) : mat_(a.get_matrix()*b.get_matrix()) {}

	cv::Matx<Precision, 2, 2> mat_; // the 2x2 matrix containing the transformation.
	
	
public:
	
	
	/// Default constructor. Initialises the matrix to the identity (no rotation)
	SO2() :mat_( cv::Matx<Precision, 2, 2>::eye() )
	{}
	
	
	/// Construct from a rotation matrix.
	SO2(const cv::Matx<Precision, 2, 2> &rhs) : mat_(rhs) {  
		
		coerce(); // Gramm-Schmidt orthogonalize the matrix
	}

	
	// Construct from an angle (Lie logarithm).
	explicit SO2(const Precision angle) { *this = exp(angle); }
  
	/// Assigment operator from a general matrix. This also calls coerce()
	/// to make sure that the matrix is a valid rotation matrix.
	// This should not be sued a lot
	/*template <typename P> 
	SO2& operator =(const cv::Mat_<P> &R){
		mat_(0,0) = (Precision)R(0,0);
		mat_(0,1) = (Precision)R(0,1);
		mat_(1,0) = (Precision)R(1,0);
		mat_(1,1) = (Precision)R(1,1);
		
		coerce(); // skip for now... 
		return *this;
	}*/
	
	/// Assignment of Matx object
	template <typename P> 
	SO2& operator =(const cv::Matx<P, 2, 2> &R){
		mat_(0,0) = R(0,0);
		mat_(0,1) = R(0,1);
		mat_(1,0) = R(1,0);
		mat_(1,1) = R(1,1);
		
		coerce(); // Gramm-Schmidt the matrix
		
		return *this;
	}
	
	// some helper functions...
	
	
	inline void assignRowAt(int index, const cv::Matx<Precision, 2, 1> &v) {
	  mat_(index, 0) = v(0, 0);
	  mat_(index, 1) = v(1, 0);
	  
	}
	
	inline void assignColAt(int index, const cv::Matx<Precision, 2, 1> &v) {
	  mat_(0, index) = v(0, 0);
	  mat_(1, index) = v(1, 0);
	  
	}
	
	inline void assignRowAt(int index, const cv::Matx<Precision, 1, 2> &v) {
	  mat_(index, 0) = v(0, 0);
	  mat_(index, 1) = v(0, 1);
	  
	}
	
	inline void assignColAt(int index, const cv::Matx<Precision, 1, 2> &v) {
	  mat_(0, index) = v(0, 0);
	  mat_(1, index) = v(0, 1);
	  
	}

	
	/// Modifies the matrix to make sure it is a valid rotation matrix. Gram-Schmidt orthogonalization
	void coerce(){
		// 1. normalize first row
		assignRowAt(0, CvUtils::normalize( mat_.row(0) ) );
		
		// 2. Project 2nd row on the 1st and take the normalized difference of the 2nd from the prjection
		//mat_(1, 0) -= mat_[0] * (mat_[0]*mat_[1]);
		assignRowAt(1, CvUtils::normalize( mat_.row(1) - ( mat_.row(1).dot( mat_.row(0) ) )*mat_.row(0) ) );
		
	}

	/// Exponentiate an angle in the Lie algebra to generate a new SO2.
	inline static SO2 exp(const Precision &d) {
		SO2<Precision> result;
		
		result.mat_(0, 0) = result.mat_(1, 1) = cos(d);
		result.mat_(1, 0) = sin(d);
		result.mat_(0, 1) = -result.mat_(1, 0);
		
		return result;
	}

	/// extracts the rotation angle from the SO2
	Precision ln() const { return atan2(mat_(1, 0), mat_(0, 0)); }

	/// Returns the inverse of this matrix (=the transpose, so this is a fast operation)
	SO2 inverse() const { return SO2(*this, Invert()); }

	

	
	/// Self right-multiply by another rotation SO2
	template <typename P>
	SO2& operator *=(const SO2<P> &right){
		mat_ = mat_ * right.get_matrix();
		
		return *this;
	}

	

	/// Returns the SO2 as a Matrix<2>
	const cv::Matx<Precision, 2, 2>& get_matrix() const {return mat_;}
	
	cv::Matx<Precision, 2, 2>& get_matrix() {return mat_;} // IMPORTANT OVERLOAD!!!!!!!!

	/// returns Lie generator matrix (skew symmetric matrix)
	static cv::Mat_<Precision> generator() {
		
		cv::Mat_<Precision> result(2,2);
		result(0, 0) = 0; result(0, 1) = -1;
		result(1, 0) = 1; result(1, 1) = 0;
		
		return result;
	}


}; // *************** Here ends the class SO2 - Here ends the class SO2, in order to define binary operator overloads ***************

  
} // Ends RigidTransfomrs - Ends RigidTransfomrs - Ends RigidTransfomrs - Ends RigidTransfomrs - Ends RigidTransfomrs **********

/// Write an SO2 to a stream 
template <typename Precision>
inline std::ostream& operator << (std::ostream &os, const RigidTransforms::SO2<Precision> &right) {
  
	return os << right.get_matrix();
}

/// Read from SO2 to a stream 
template <typename Precision>
inline std::istream& operator>>(std::istream &is, RigidTransforms::SO2<Precision> &right) {
	is >> right.mat_;
	right.coerce(); // orthogonalize the data to be sure...
	
	return is;
}


/// Right-multiply by another SO2
template <typename Pl, typename Pr>
inline RigidTransforms::SO2<typename MyOperatorOverloads::MultiplyType<Pl, Pr>::type> operator *(const RigidTransforms::SO2<Pl> &left, 
												const RigidTransforms::SO2<Pr> &right)  { 
		 
  typedef typename MyOperatorOverloads::MultiplyType<Pl, Pr>::type P0;
	    
  	    
  return RigidTransforms::SO2<P0>( left.get_matrix() * right.get_matrix() ); 
}


/// Right-multiply by a 2-Vector
template<typename P, typename PV>
inline cv::Vec<typename MyOperatorOverloads::MultiplyType<P, PV>::type, 2> operator *(const RigidTransforms::SO2<P> &so2, const cv::Vec<PV, 2> &v) {
  
  
  return so2.get_matrix() * v;
}

/// Left-multiply by a Vector // this basically results in a vector u = R^T * v
template<typename PV, typename P>
inline cv::Vec<typename MyOperatorOverloads::MultiplyType<PV,P>::type, 2> operator *(const cv::Vec<PV,2> &v, const RigidTransforms::SO2<P> &so2) {

    
  return so2.get_matrix().t() * v;
}

/// Right-multiply by a cv::Mat matrix
template <typename P, typename PM> 
inline cv::Mat_<typename MyOperatorOverloads::MultiplyType<P,PM>::type> operator *(const RigidTransforms::SO2<P> &so2, const cv::Mat_<PM> &M){
	
  return so2.get_matrix() * M;
}

/// Right-multiply by a cv::Matx matrix
template <typename P, typename PM, int C> 
inline cv::Matx<typename MyOperatorOverloads::MultiplyType<P,PM>::type, 2, C> operator *(const RigidTransforms::SO2<P> &so2, const cv::Matx<PM, 2, C> &M){
	
  return so2.get_matrix() * M;
}


/// Left-multiply by a cv::Mat matrix
template <typename PM, typename P>
inline cv::Mat_<typename MyOperatorOverloads::MultiplyType<PM,P>::type> operator *(const cv::Mat_<PM> M, const RigidTransforms::SO2<P> &so2) {
	
  return M * so2.get_matrix();
}

/// Left-multiply by a cv::Matx matrix
template <typename PM, int R, typename P>
inline cv::Matx<typename MyOperatorOverloads::MultiplyType<PM,P>::type, R, 2> operator *(const cv::Matx<PM, R, 2> M, const RigidTransforms::SO2<P> &so2) {
	
  return M * so2.get_matrix();
}



#endif