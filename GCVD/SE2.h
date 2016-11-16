// **************** George Terzakis 2016 *********************
//
// This file is based on original TooN code:
// Copyright (C) 2005,2009 Tom Drummond (twd20@cam.ac.uk),
// Ed Rosten (er258@cam.ac.uk), Gerhard Reitmayr (gr281@cam.ac.uk)



#ifndef SE2_H
#define SE2_H

#include "Operators.h"
#include "SO2.h"

#include <iostream>

#include "../OpenCV.h"


namespace RigidTransforms {

/// Represent a two-dimensional Euclidean transformation (a rotation and a translation). 
/// This can be represented by a \f$2\times 3\f$ matrix operating on a homogeneous co-ordinate, 
/// so that a vector \f$\underline{x}\f$ is transformed to a new location \f$\underline{x}'\f$
/// by
/// \f[\begin{aligned}\underline{x}' &= E\times\underline{x}\\ \begin{bmatrix}x'\\y'\end{bmatrix} &= \begin{pmatrix}r_{11} & r_{12} & t_1\\r_{21} & r_{22} & t_2\end{pmatrix}\begin{bmatrix}x\\y\\1\end{bmatrix}\end{aligned}\f]
/// 
/// This transformation is a member of the Special Euclidean Lie group SE2. These can be parameterised with
/// three numbers (in the space of the Lie Algebra). In this class, the first two parameters are a
/// translation vector while the third is the amount of rotation in the plane as for SO2.

template <typename Precision = float>
class SE2 {
private:
	SO2<Precision> R_;
	cv::Vec<Precision, 2> t_;


public:
	/// Default constructor. Initialises the the rotation to zero (the identity) and the translation to zero
	SE2() : R_(SO2<Precision>()), t_(cv::Vec<Precision, 2>(0,0)) {}
	
	SE2(const SO2<Precision> &R, const cv::Vec<Precision,2> &T) : R_(R), t_(T) {}
	
	// matrix-matrix constructor...
	SE2(const SO2<Precision> &R, const cv::Mat_<Precision> &T) : R_(R) {
	  
	  t_ = cv::Vec<Precision, 2>(T(0,0), T(1, 0));
	  
	}
	
	template <typename P> SE2(const cv::Vec<P, 3> &v) { *this = exp(v); }

	
	
	
	/// Returns the rotation part of the transformation as a SO2
	SO2<Precision>& get_rotation(){return R_;}
	/// @overload
	const SO2<Precision>& get_rotation() const {return R_;}
	/// Returns the translation part of the transformation as a Vector
	cv::Vec<Precision, 2>& get_translation() {return t_;}
	/// @overload
	const cv::Vec<Precision, 2>& get_translation() const {return t_;}

	// this is the homogeneous tranformation 3x3 matrix correspondiong to the SE2
	cv::Matx<Precision, 3, 3> get_matrix() {
	 
	  cv::Matx<Precision, 3, 3> mat;
	  cv::Matx<Precision, 2, 2> Rm = R_.get_matrix();
	  mat(0, 0) = Rm(0,0); mat(0, 1) = Rm(0,1);
	  mat(0, 1) = Rm(0,1); mat(1, 1) = Rm(1,1);
	  mat(2, 0) = 0 = mat(2, 1) = 0; mat(2, 2) = 1;
	  mat(0, 2) = get_translation()[0]; mat(0, 2) = get_translation()[1];
	  
	  return mat;
	}
	const cv::Matx<Precision, 3, 3> get_matrix() const {
	 
	  cv::Matx<Precision, 3, 3> mat;
	  cv::Matx<Precision, 2, 2> Rm = R_.get_matrix();
	  mat(0, 0) = Rm(0,0); mat(0, 1) = Rm(0,1);
	  mat(0, 1) = Rm(0,1); mat(1, 1) = Rm(1,1);
	  mat(2, 0) = 0 = mat(2, 1) = 0; mat(2, 2) = 1;
	  mat(0, 2) = get_translation()[0]; mat(0, 2) = get_translation()[1];
	  
	  return mat;
	}
	
	/// Exponentiate a Vector in the Lie Algebra to generate a new SE2.
	/// See the Detailed Description for details of this vector.
	/// @param vect The Vector to exponentiate
	template <typename P>
	static inline SE2 exp(const cv::Vec<P, 3> &v);
	
	/// Take the logarithm of the matrix, generating the corresponding vector in the Lie Algebra.
	/// See the Detailed Description for details of this vector.
	static inline cv::Vec<Precision, 3> ln(const SE2 &se2);
	/// @overload
	cv::Vec<Precision, 3> ln() const { return SE2::ln(*this); }

	/// compute the inverse of the transformation
	SE2 inverse() const {
		
	  const SO2<Precision> &rinv = R_.inverse();
	  
	  return SE2(rinv, -(rinv.get_matrix()*t_));
	};

	/// Right-multiply by another SE2 (concatenate the two transformations)
	template <typename P>
	SE2<typename MyOperatorOverloads::MultiplyType<Precision,P>::type> operator *(const SE2<P> &right) const { 
		
	  typedef typename MyOperatorOverloads::MultiplyType<Precision,P>::type P0;
	  
	  cv::Matx<Precision, 2, 2> Rm = R_.get_matrix();
	  cv::Vec<P, 2> tr = right.get_translation();
	  cv::Vec<P0, 2> br( Rm(0, 0) * tr[0] + Rm(0, 1) * tr[1],
			     Rm(1, 0) * tr[0] + Rm(1, 1) * tr[1] );
	  
	  return SE2<P0>(R_*right.get_rotation(), t_ + br ); 
	}

	/// Self right-multiply by another SE2 (concatenate the two transformations)
	/// @param rhs The multipier
	template <typename P>
	inline SE2& operator *=(const SE2<P> &right) { 
		
	  *this = *this * right; 
	  return *this; 
	}

	/// returns the generators for the Lie group. These are a set of matrices that
	/// form a basis for the vector space of the Lie algebra.
	/// - 0 is translation in x
	/// - 1 is translation in y
	/// - 2 is rotation in the plane
	static inline cv::Matx<Precision, 3, 3> generator(int i) {
		
	  cv::Matx<Precision, 3, 3> result = cv::Matx<Precision, 3, 3>::zeros();
	  if(i < 2){
	    result(i, 2) = 1;
	    return result;
	  }
	  result(0, 1) = -1;
	  result(1, 0) = 1;
	  
	  return result;
	}

	/// transfers a vector in the Lie algebra, from one coord frame to another
	/// so that exp(adjoint(vect)) = (*this) * exp(vect) * (this->inverse())
	cv::Vec<Precision, 3> adjoint(const cv::Matx<Precision, 3, 1> &v) const {
		
	  cv::Vec<Precision, 3> result;
	  result[2] = v(2, 0);
	  //result.template slice<0,2>() = my_rotation * vect.template slice<0,2>();
	  result[0] = R_.get_matrix()(0,0) * v(0, 0) + R_.get_matrix()(0, 1) * v(1, 0);	
	  result[1] = R_.get_matrix()(1,0) * v(0, 0) + R_.get_matrix()(1, 1) * v(1, 0);	
	  
	  result[0] += v(2, 0) * t_[1];
	  result[1] -= v(2, 0) * t_[0];
	
	  return result;
	}
	
	cv::Matx<Precision, 3, 3> adjoint(const cv::Matx<Precision, 3, 3> &M) const {
		cv::Matx<Precision, 3, 3> result;
		cv::Vec<Precision, 3> adj;
		for(int i=0; i<3; ++i) {
		  //result.T()[i] = adjoint(M.T()[i]); 
		  adj = adjoint( cv::Vec<Precision, 3> ( M(0, i), 
							 M(1, i), 
							 M(2, i) ) 
			       );
		  result(0, i) = adj[0];
		  result(1, i) = adj[1];
		  result(2, i) = adj[2];
		  
		}
		
		for(int i=0; i<3; ++i) {	
		  //result[i] = adjoint(result[i]);
		  adj = adjoint( cv::Vec<Precision, 3>( result(i, 0), 
							result(i, 1), 
							result(i, 2) )  
			       );
		  result(i, 0) = adj[0];
		  result(i, 1) = adj[1];
		  result(i, 2) = adj[2];
		}
		
		return result;
	}

}; // *********************8 Ends SE2 - Ends SE2 - Ends SE2- Ends SE2- Ends SE2 *******************

} // ************************* Ends namespace RigidTransforms ***********************

/// Write an SE2 to a stream 
/// @relates SE2
template <class Precision>
inline std::ostream& operator <<(std::ostream &os, const RigidTransforms::SE2<Precision> &se2){
	
	// Let's just relay the problem to OpenCV... Not bad a solution by the looks of it...
	os << se2.get_rotation().get_matrix() << se2.get_translation() << '\n';
	
	return os;
}

/// Read an SE2 from a stream 
/// @relates SE2
template <class Precision>
inline std::istream& operator >>(std::istream& is, RigidTransforms::SE2<Precision> &se2){
	//for(int i=0; i<2; i++)
	//	is >> se2.get_rotation().mat_ >> rhs.get_translation()[i];
	// Ok, i am simply relaying the input operator to the Opencv overloads... It should probably work...
	is >> se2.get_rotation().get_matrix() >> se2.get_translation();
	se2.get_rotation().coerce();
	
	return is;
}


//////////////////
// operator *   //
// SE2 * Vector //
//////////////////

namespace MyOperatorOverloads {
// a SE2 * Vector multiplication template "constant"
template<typename P, typename PV> struct SE2VMult;
  



template<typename P, typename PV>
struct Operator<MyOperatorOverloads::SE2VMult<P,PV> > {
	const RigidTransforms::SE2<P> &se2;
	const cv::Vec<PV, 3> &v;
	
	Operator(const RigidTransforms::SE2<P> &se2_in, const cv::Vec<PV, 3> &v_in ) : se2(se2_in), v(v_in) {}
	
	
	typedef typename MyOperatorOverloads::MultiplyType<P, PV>::type P0;
	
	cv::Vec<P0, 3> compute() const { 
	     
	     cv::Vec<P0, 3> res; 
	     
	     res[0] = se2.get_rotation().get_matrix()(0,0) * v[0] + se2.get_rotation().get_matrix()(0,1) * v[1]; 
	     res[1] = se2.get_rotation().get_matrix()(1,0) * v[0] + se2.get_rotation().get_matrix()(1,1) * v[1]; 
	      
	     res[0] += se2.get_translation()[0] * v[2];
	     res[1] += se2.get_translation()[1] * v[2];
	     
	     res[2] = v[2];
	     
	     return res;
	}
	
};

// a Vector * SE2 multiplication template "constant"
template<typename PV, typename P> struct VSE2Mult;
  



template<typename PV, typename P>
struct Operator<MyOperatorOverloads::VSE2Mult<PV,P> > {
	const cv::Vec<PV, 3> &v;
	const RigidTransforms::SE2<P> &se2;
	
	
	Operator( const cv::Vec<PV, 3> &v_in, const RigidTransforms::SE2<P> &se2_in ) : se2(se2_in), v(v_in) {}
	
	
	typedef typename MyOperatorOverloads::MultiplyType<PV, P>::type P0;
	
	cv::Vec<P0, 3> compute() const { 
	     
	     cv::Vec<P0, 3> res; 
	     
	     cv::Mat_<P> R = se2.get_rotation.get_matrix();
	     cv::Vec<P, 2> t = se2.get_translation();
	     
	     res[0] = R(0,0) * v[0] + R(1,0) * v[1]; 
	     res[1] = R(0,1) * v[0] + R(1,1) * v[1]; 
	      
	     
	     res[2] = t*v + v[2];
	     
	     
	     return res;
	}
	
};


} // end namespace clause for MyOperatorOverloads


/// Right-multiply with a 3-vector
template<typename P, typename PV>
inline cv::Vec<typename MyOperatorOverloads::MultiplyType<P,PV>::type, 3> operator *(const RigidTransforms::SE2<P> &se2, const cv::Vec<PV, 3> &v) {
	
  return MyOperatorOverloads::Operator<MyOperatorOverloads::SE2VMult<P,PV> >(se2, v).compute();
}



/// Right-multiply with a 2-vector (result: R*v + t)
template <typename P, typename PV>
inline cv::Vec<typename MyOperatorOverloads::MultiplyType<P,PV>::type, 2> operator *(const RigidTransforms::SE2<P> &se2, const cv::Vec<PV, 2> &v) {
	
  
  cv::Vec3f v3(v[0], v[1],1); // turning it to a homogeneous vector
 
  typedef typename MyOperatorOverloads::MultiplyType<PV, P>::type P0;
  
  cv::Vec<P0, 3> result = se2 * v3;
  
  return cv::Vec<P0, 2>( result[0], 
			 result[1] );
}




//////////////////
// operator *   //
// Vector * SE2 //
//////////////////




/// Left-multiply with a 3-vector (i.e. homogeneous representation)
template<typename PV, typename P>
inline cv::Vec<typename MyOperatorOverloads::MultiplyType<PV, P>::type, 3> operator *(const cv::Vec<PV, 3> &v, const RigidTransforms::SE2<P> &se2) {
	
  return MyOperatorOverloads::Operator<MyOperatorOverloads::VSE2Mult<PV,P> >(v,se2).compute();
}


//////////////////
// operator *   //
// SE2 * Matrix //
//////////////////

namespace MyOperatorOverloads {
// template constant for right multiplication with matrix (SE2 * Matrix)
  template <typename P, typename PM> struct SE2MMult;


template<typename P, typename PM>
struct Operator<SE2MMult<P, PM> > {
	const RigidTransforms::SE2<P> &se2;
	const cv::Mat_<PM> &M;
	
	Operator(const RigidTransforms::SE2<P> &se2_in, const cv::Mat_<PM> &M_in ) : se2(se2_in), M(M_in) {}
	
	
	typedef typename MyOperatorOverloads::MultiplyType<P, PM>::type P0;
	
	cv::Mat_<P0> compute() const {
		
	  return se2.get_matrix() * M;
	}
	
};

} // End namespace MyOperatorOverloads interlopping clause

/// Right-multiply with a 3x3 matrix
/// @relates SE2
template <typename P, typename PM> 
inline cv::Mat_<typename MyOperatorOverloads::MultiplyType<P,PM>::type> operator *(const RigidTransforms::SE2<P> &se2, const cv::Mat_<PM> &M) {
	
  return MyOperatorOverloads::Operator<MyOperatorOverloads::SE2MMult<P, PM> >(se2,M).compute();
}

//////////////////
// operator *   //
// Matrix * SE2 //
//////////////////

namespace MyOperatorOverloads {
  // Template constant for matrix*SE2 multiplication
  template <typename PM, typename P> struct MSE2Mult;


template<typename PM, typename P>
struct Operator<MSE2Mult<PM, P> > {
	const cv::Mat_<PM> &M;
	const RigidTransforms::SE2<P> &se2;
	
	Operator( const cv::Mat_<PM> &M_in, const RigidTransforms::SE2<P> &se2_in) : M(M_in), se2(se2_in) {}
	
	typedef typename MyOperatorOverloads::MultiplyType<PM, P>::type P0;
	
	cv::Mat_<P0> compute() const {
		
	  return M * se2.get_matrix();
	}
	
};
}

/// Left-multiply with a Matrix<3>
/// @relates SE2
template <typename PM, typename P> 
inline cv::Mat_<typename MyOperatorOverloads::MultiplyType<PM,P>::type> operator *(const cv::Mat_<PM> &M, const RigidTransforms::SE2<P> &se2 ) {
	
  return MyOperatorOverloads::Operator<MyOperatorOverloads::MSE2Mult<PM, P> >(M, se2);
}


// This is a very robust exponential computation!
// It really looks into numerical degeneracies close to zero 
// (which INCIDENTALLY dobnt exist with sterographic coordinates!)

template <typename Precision>
template <typename PV>
inline RigidTransforms::SE2<Precision> RigidTransforms::SE2<Precision>::exp(const cv::Vec<PV, 3> &mu)
{
	// George: SOME DAY, ONE DAY, I need to do something about bad vector/matrix sizes............
	
	static const Precision one_6th = 1.0/6.0;
	static const Precision one_20th = 1.0/20.0;
  
	SE2<Precision> result;
  
	const Precision theta = mu[2]; // angle is stored in the last corindate of the 2D pose vector
	const Precision theta_sq = theta * theta;
  
	// the cross prodict of the axis-angle vector with the translation (is now very easy to compute due to the two zeros in the AA vector)
	const cv::Vec<Precision, 2> cross( -theta * mu[1], theta * mu[0] );
	result.get_rotation() = SO2<Precision>::exp(theta);

	if (theta_sq < 1e-8){
		//result.get_translation() = mu.template slice<0,2>() + 0.5 * cross;
		result.get_translation() = cv::Vec<Precision, 2>( mu[0] + 0.5 * cross[0], 
								  mu[1] + 0.5 * cross[1] );
	  
	} else {
		Precision A, B;
		if (theta_sq < 1e-6) {
			A = 1.0 - theta_sq * one_6th*(1.0 - one_20th * theta_sq);
			B = 0.5 - 0.25 * one_6th * theta_sq;
		} else {
			const Precision inv_theta = (1.0/theta);
			const Precision sine = result.get_rotation().get_matrix()(1, 0);
			const Precision cosine = result.get_rotation().get_matrix()(0, 0);
			A = sine * inv_theta;
			B = (1 - cosine) * (inv_theta * inv_theta);
		}
		//result.get_translation() = TooN::operator*(A,mu.template slice<0,2>()) + TooN::operator*(B,cross);
		result.get_translation() = cv::Vec<Precision, 2>( A * mu[0] + B * cross[0], 
								  A * mu[1] + B * cross[1]);
	  
	}
	return result;
}
 
// Most importantly, This is a brilliant logarithmic function, especially with all the problems incurred by the periodicity 
// of the Lie/Euler vectors and the singularity of the exponential map at zero!
template <typename Precision>
inline cv::Vec<Precision, 3> RigidTransforms::SE2<Precision>::ln(const RigidTransforms::SE2<Precision> &se2) {
	const Precision theta = se2.get_rotation().ln();

	Precision shtot = 0.5;  
	if(fabs(theta) > 0.00001)
		shtot = sin(theta/2)/theta;

	const SO2<Precision> halfrotator(theta * -0.5);
	cv::Vec<Precision, 3> result;
	//result.template slice<0,2>() = (halfrotator * se2.get_translation())/(2 * shtot);
	cv::Mat_<Precision> R = halfrotator.get_matrix();
	cv::Vec<Precision, 2> t = se2.get_translation();
	result[0] = ( R(0,0) * t[0] + 
		      R(0,1) * t[1]  ) / (2 * shtot);
	result[1] = ( R(1,0) * t[0] + 
		      R(1,1) * t[1]  ) / (2 * shtot);
	
	result[2] = theta;
	
	return result;
}

/// Multiply a SO2 with anSE2
template <typename Precision>
inline RigidTransforms::SE2<Precision> operator *(const RigidTransforms::SO2<Precision> &so2, const RigidTransforms::SE2<Precision> &se2){
	
  return RigidTransforms::SE2<Precision>( so2*se2.get_rotation(), so2.get_matrix()*se2.get_translation() );
}


#endif