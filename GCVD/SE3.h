//**************** George Terzakis 2016
//************* University of Portsmouth
//
// Class SE3 corresponds to a 3D rigid transformation. 
// Code adapted from Ed. Rosten's original briliant implementation

#ifndef SE3_H
#define SE3_H

#include "SO3.h"
#include "Operators.h"


#include "../OpenCV.h"



namespace RigidTransforms {


/// Represent a three-dimensional Euclidean transformation (a rotation and a translation). 
/// This can be represented by a \f$3\times\f$4 matrix operating on a homogeneous co-ordinate, 
/// so that a vector \f$\underline{x}\f$ is transformed to a new location \f$\underline{x}'\f$
/// by
/// \f[\begin{aligned}\underline{x}' &= E\times\underline{x}\\ \begin{bmatrix}x'\\y'\\z'\end{bmatrix} &= \begin{pmatrix}r_{11} & r_{12} & r_{13} & t_1\\r_{21} & r_{22} & r_{23} & t_2\\r_{31} & r_{32} & r_{33} & t_3\end{pmatrix}\begin{bmatrix}x\\y\\z\\1\end{bmatrix}\end{aligned}\f]
/// 
/// This transformation is a member of the Special Euclidean Lie group SE3. These can be parameterised
/// six numbers (in the space of the Lie Algebra). In this class, the first three parameters are a
/// translation vector while the second three are a rotation vector, whose direction is the axis of rotation
/// and length the amount of rotation (in radians), as for SO3
template <typename Precision = float>
class SE3 {
  
private:
	SO3<Precision> R_;
	cv::Vec<Precision, 3> t_;
  
  
public:
	/// Default constructor. Initialises the the rotation to zero (the identity) and the translation to zero
	inline SE3() : R_(SO3<Precision>()), t_( cv::Vec<Precision, 3>(0, 0, 0) ) {}

	template <typename P> 
	SE3(const SO3<Precision> &R, const cv::Vec<P, 3> &T) : R_(R), t_(T) {}
	
	template <typename P>
	SE3(const cv::Vec<P, 6> &v) { *this = SE3::exp(v); }

	template <typename P> 
	SE3(const cv::Vec<P, 3> &T) : t_(T), R_(SO3<Precision>()) {}

	/// Returns the rotation part of the transformation as a SO3
	inline SO3<Precision>& get_rotation(){return R_;}
	/// @overload
	inline const SO3<Precision>& get_rotation() const {return R_;}

	/// Returns the translation part of the transformation as a Vector
	inline cv::Vec<Precision, 3>& get_translation() {return t_;}
	/// @overload
	inline const cv::Vec<Precision, 3>& get_translation() const {return t_;}

	/// Exponentiate a Vector in the Lie Algebra to generate a new SE3.
	/// See the Detailed Description for details of this vector.
	/// @param vect The Vector to exponentiate
	template <typename P>
	static inline SE3 exp(const cv::Vec<P, 6> &v);


	/// Take the logarithm of the matrix, generating the corresponding vector in the Lie Algebra.
	/// See the Detailed Description for details of this vector.
	static inline cv::Vec<Precision, 6> ln(const SE3 &se3);
	/// @overload
	inline cv::Vec<Precision, 6> ln() const { return SE3::ln(*this); }

	inline SE3 inverse() const {
		const SO3<Precision> rinv = get_rotation().inverse();
		return SE3( rinv, -(rinv*t_) );
	}

	/// Right-multiply by another SE3 (concatenate the two transformations)
	/// @param rhs The multipier
	template<typename P>
	inline SE3& operator *=(const SE3<P> &se3) {
		
	  
	  get_translation() += R_ * se3.get_translation();
	  
	  get_rotation() *= se3.get_rotation();
	  
	  return *this;
	}

	/// Right-multiply by another SE3 (concatenate the two transformations)
	/// @param rhs The multipier
	template<typename P>
	inline SE3<> operator *(const SE3<P> &right) const {
	  
	    return SE3<>(R_*right.get_rotation(), t_ + R_*right.get_translation()); 
	}

	// multiply by a SE3 from the left and store in current object
	inline SE3& left_multiply_by(const SE3& left) {
		t_ = left.get_translation() + left.get_rotation() * t_;
		R_ = left.get_rotation() * R_;
		return *this;
	}

	static inline cv::Mat_<Precision> generator(int i) {
		cv::Mat_<Precision> result = cv::Mat_<Precision>::zeros(4, 4);
		if(i < 3){
		  
		  result(i, 3)=1;
		  return result;
		}
		
		result( (i+1)%3, (i+2)%3 ) = -1;
		result( (i+2)%3, (i+1)%3 ) = 1;
		
		return result;
	}

	/// Returns the i-th generator times pos
	inline static cv::Vec<Precision, 4> generator_field(int i, const cv::Vec<Precision, 4> &pos) {
    
	  cv::Vec<Precision, 4> result(0, 0, 0, 0);
    
	  // positional 
	  if(i < 3){
	    
	    result[i]=pos[3];      
	    return result;
	  }
	  // angular
	  result[(i+1)%3] = -pos[(i+2)%3];
	  result[(i+2)%3] = pos[(i+1)%3];
	  
	  return result;
	}
  
	/// Transfer a matrix in the Lie Algebra from one
	/// co-ordinate frame to another. This is the operation such that for a matrix 
	/// \f$ B \f$, 
	/// \f$ e^{\text{Adj}(v)} = Be^{v}B^{-1} \f$
	/// @param M The Matrix to transfer
	template<typename P2>
	inline cv::Vec<Precision, 6> adjoint(const cv::Vec<P2, 6> &v) const;

	/// Transfer covectors between frames (using the transpose of the inverse of the adjoint)
	/// so that trinvadjoint(vect1) * adjoint(vect2) = vect1 * vect2
	template<typename P2>
	inline cv::Vec<Precision, 6> trinvadjoint(const cv::Vec<P2, 6>&) const;
	
	///@overload
	template <typename P2>
	inline cv::Mat_<Precision> adjoint(const cv::Mat_<P2> &M) const; // 6x6 matrix

	///@overload
	template <typename P2>
	inline cv::Mat_<Precision> trinvadjoint(const cv::Mat_<P2> &M) const; // 6x6 matrix

}; // *************** Class SE3 Ends Here - Class SE3 Ends Here - Class SE3 Ends Here ******************

}; // ************ Close RigidTransforms temporarily in order to define additional Operator specializations!!!! *****

// transfers a vector in the Lie algebra
// from one coord frame to another
// so that exp(adjoint(vect)) = (*this) * exp(vect) * (this->inverse())
template<typename Precision>
template<typename P2>
inline cv::Vec<Precision, 6> RigidTransforms::SE3<Precision>::adjoint(const cv::Vec<P2, 6> &v) const {
	
	cv::Vec<Precision, 6> result;
	//result.template slice<3,3>() = get_rotation() * vect.template slice<3,3>();
	result[3] = R_.get_matrix()(0, 0) * v[3] + R_.get_matrix()(0, 1) * v[4] + R_.get_matrix()(0, 2) * v[5];
	result[4] = R_.get_matrix()(1, 0) * v[3] + R_.get_matrix()(1, 1) * v[4] + R_.get_matrix()(1, 2) * v[5];
	result[5] = R_.get_matrix()(2, 0) * v[3] + R_.get_matrix()(2, 1) * v[4] + R_.get_matrix()(2, 2) * v[5];
	
	//result.template slice<0,3>() = get_rotation() * vect.template slice<0,3>();
	result[0] = R_.get_matrix()(0, 0) * v[0] + R_.get_matrix()(0, 1) * v[1] + R_.get_matrix()(0, 2) * v[2];
	result[1] = R_.get_matrix()(1, 0) * v[0] + R_.get_matrix()(1, 1) * v[1] + R_.get_matrix()(1, 2) * v[2];
	result[2] = R_.get_matrix()(2, 0) * v[0] + R_.get_matrix()(2, 1) * v[1] + R_.get_matrix()(2, 2) * v[2];
	
	//result.template slice<0,3>() += get_translation() ^ result.template slice<3,3>();
	result[0] += -t_[2] * result[4] + t_[1]*result[5];
	result[1] +=  t_[2] * result[3] - t_[0]*result[5];
	result[2] += -t_[1] * result[3] + t_[0]*result[4];
	
	
	return result;
}

// tansfers covectors between frames
// (using the transpose of the inverse of the adjoint)
// so that trinvadjoint(vect1) * adjoint(vect2) = vect1 * vect2
template<typename Precision>
template<typename P2>
inline cv::Vec<Precision, 6> RigidTransforms::SE3<Precision>::trinvadjoint(const cv::Vec<P2, 6> &v) const { 
	
	cv::Vec<Precision, 6> result;
	//result.template slice<3,3>() = get_rotation() * vect.template slice<3,3>();
	result[3] = R_.get_matrix()(0, 0) * v[3] + R_.get_matrix()(0, 1) * v[4] + R_.get_matrix()(0, 2) * v[5];
	result[4] = R_.get_matrix()(1, 0) * v[3] + R_.get_matrix()(1, 1) * v[4] + R_.get_matrix()(1, 2) * v[5];
	result[5] = R_.get_matrix()(2, 0) * v[3] + R_.get_matrix()(2, 1) * v[4] + R_.get_matrix()(2, 2) * v[5];
	
	//result.template slice<0,3>() = get_rotation() * vect.template slice<0,3>();
	result[0] = R_.get_matrix()(0, 0) * v[0] + R_.get_matrix()(0, 1) * v[1] + R_.get_matrix()(0, 2) * v[2];
	result[1] = R_.get_matrix()(1, 0) * v[0] + R_.get_matrix()(1, 1) * v[1] + R_.get_matrix()(1, 2) * v[2];
	result[2] = R_.get_matrix()(2, 0) * v[0] + R_.get_matrix()(2, 1) * v[1] + R_.get_matrix()(2, 2) * v[2];
	
	//result.template slice<3,3>() += get_translation() ^ result.template slice<0,3>();
	result[3] += -t_[2] * result[1] + t_[1] * result[2];
	result[4] +=  t_[2] * result[0] - t_[0] * result[2];
	result[5] += -t_[1] * result[0] + t_[0] * result[1];
	
	
	return result;
}


template<typename Precision>
template<typename P2> 
inline cv::Mat_<Precision> RigidTransforms::SE3<Precision>::adjoint(const cv::Mat_<P2> &M) const { // NOTE: 6x6 matrix argument here...
	
	cv::Mat_<Precision> result(6, 6);
	
	for(int i=0; i<6; i++){
	  //result.T()[i] = adjoint(M.T()[i]);
	  cv::Vec<P2, 6> adj = adjoint( cv::Vec<P2, 6>( M(0, i), 
							M(1, i), 
							M(2, i), 
							M(3, i), 
							M(4, i), 
							M(5, i)) );
	  result(0, i) = adj[0]; result(1, i) = adj[1]; result(2, i) = adj[2];
	  result(3, i) = adj[3]; result(4, i) = adj[4]; result(5, i) = adj[5];
	}
	
	for(int i=0; i<6; i++) {
	  //result[i] = adjoint(result[i]);
	  cv::Vec<P2, 6> adj = adjoint( cv::Vec<P2, 6>(result(i, 0), 
						       result(i, 1), 
						       result(i, 2), 
						       result(i, 3), 
						       result(i, 4), 
						       result(i, 5)) 
				        );
	  result(i, 0) = adj[0]; result(i, 1) = adj[1]; result(i, 2) = adj[2];
	  result(i, 3) = adj[3]; result(i, 4) = adj[4]; result(i, 5) = adj[5];
	}
	
	return result;
}

// transposed inverse adjoint
template<typename Precision>
template<typename P2>
inline cv::Mat_<Precision> RigidTransforms::SE3<Precision>::trinvadjoint(const cv::Mat_<P2> &M) const{
	
	cv::Mat_<Precision> result(6, 6);
	
	for(int i=0; i<6; i++){
	  //result.T()[i] = trinvadjoint(M.T()[i]);
	  cv::Vec<P2, 6> tadj = trinvadjoint(cv::Vec<P2, 6>(M(0, i),
							    M(1, i),
							    M(2, i),
							    M(3, i),
							    M(4, i),
							    M(5, i) ) ); 
	  
	  result(0, i) = tadj[0]; result(1, i) = tadj[1]; result(2, i) = tadj[2];
	  result(3, i) = tadj[3]; result(4, i) = tadj[4]; result(5, i) = tadj[5];
	}
	for(int i=0; i<6; i++){
	  //result[i] = trinvadjoint(result[i]);
	  cv::Vec<P2, 6> tadj = trinvadjoint(cv::Vec<P2, 6>(result(i, 0),
							    result(i, 1),
							    result(i, 2),
							    result(i, 3),
							    result(i, 4),
							    result(i, 5) ) 
					     ); 
	result(i, 0) = tadj[0]; result(i, 1) = tadj[1]; result(i, 2) = tadj[2];
	result(i, 3) = tadj[3]; result(i, 4) = tadj[4]; result(i, 5) = tadj[5];
	}
	
	return result;
}

/// Write an SE3 to a stream 
template <typename Precision>
inline std::ostream& operator <<(std::ostream& os, const RigidTransforms::SE3<Precision>& se3){
	// just relaying the problem to OpenCV
	os << se3.get_rotation().get_matrix() << se3.get_translation() <<'\n';
	
	return os;
}


/// Reads an SE3 from a stream 
template <typename Precision>
inline std::istream& operator >>(std::istream& is, RigidTransforms::SE3<Precision>& se3) {
	is >> se3.get_rotation().get_matrix() >> se3.get_translation();
	
	se3.get_rotation().coerce();
	
	return is;
}



//////////////////
// operator *   //
// SE3 * Vector //
//////////////////

namespace MyOperatorOverloads {
// SE3 * vector template constant
template<typename P, typename PV> struct SE3VMult;




template<typename P, typename PV>
struct Operator<MyOperatorOverloads::SE3VMult<P, PV> > {
	const RigidTransforms::SE3<P> &se3;
	const cv::Vec<PV, 4> &v; // homogeneous vector
	
	Operator(const RigidTransforms::SE3<P> &se3_in, const cv::Vec<PV, 4> &v_in ) : se3(se3_in), v(v_in) {}
	
	typedef typename MyOperatorOverloads::MultiplyType<P, PV>::type P0;
	
	cv::Vec<P0, 4> compute() const {
	
	  cv::Vec<P0, 4> res;
	  cv::Mat_<P> R = se3.get_rotation().get_matrix();
	  cv::Vec<P, 3> t = se3.get_translation();
	  //res.template slice<0,3>()=lhs.get_rotation() * rhs.template slice<0,3>();
	  res[0] = R(0,0) * v[0] + R(0,1) * v[1] + R(0,2) * v[2];
	  res[1] = R(1,0) * v[0] + R(1,1) * v[1] + R(1,2) * v[2];
	  res[2] = R(2,0) * v[0] + R(2,1) * v[1] + R(2,2) * v[2];
	  
	  //res.template slice<0,3>()+=TooN::operator*(lhs.get_translation(),rhs[3]);
	  res[0] += v[3] * t[0];
	  res[1] += v[3] * t[1];
	  res[2] += v[3] * t[2];
	 
	  res[3] = v[3];
	  
	  return res;
	}
	
};
}// MyOperatorOverloads
/// Right-multiply by a homogeneous Vector (his majesty the operator itself!)
/// @relates SE3
template<typename P, typename PV> 
inline cv::Vec<typename MyOperatorOverloads::MultiplyType<P,PV>::type, 4> operator *(const RigidTransforms::SE3<P> &se3, const cv::Vec<PV,4> &v) { 
	
  return Operator<MyOperatorOverloads::SE3VMult<P, PV> >(se3,v).compute();
}

/// Right-multiply by a standard 3-Vector
/// @relates SE3
template <typename PV, typename P> 
inline cv::Vec<typename MyOperatorOverloads::MultiplyType<P,PV>::type, 3> operator *(const RigidTransforms::SE3<P> &se3, const cv::Vec<PV, 3> &v) {
	
  return se3.get_translation() + se3.get_rotation() * v;
}

//////////////////
// operator *   //
// Vector * SE3 //
//////////////////

namespace MyOperatorOverloads {
// template constant for multiplication from left with a vector (vector * SE3)
  template<typename PV, typename P> struct VSE3Mult;


// Operator for the homgeneous 4- vector
template<typename PV, typename P>
struct Operator<MyOperatorOverloads::VSE3Mult<PV, P> > {
	const cv::Vec<PV,4> &v;
	const RigidTransforms::SE3<P> &se3;
	
	Operator( const cv::Vec<PV, 4> &v_in, const RigidTransforms::SE3<P> &se3_in ) : v(v_in), se3(se3_in) {}
	
	
	typedef typename MyOperatorOverloads::MultiplyType<PV, P>::type P0;
	
	cv::Vec<P0, 4> compute() const {
	
	cv::Vec<P0, 4> res;
	//res.template slice<0,3>()=lhs.template slice<0,3>() * rhs.get_rotation();
	cv::Mat_<P> R = se3.get_rotation().get_matrix();
	cv::Vec<P, 3> t = se3.get_translation();
	res[0] = v[0] * R(0,0) + v[1] * R(1,0) + v[2] * R(2,0); 
	res[1] = v[0] * R(0,1) + v[1] * R(1,1) + v[2] * R(2,1); 
	res[2] = v[0] * R(0,2) + v[1] * R(1,2) + v[2] * R(2,2); 
	
	  
	res[3] = v[3];
	
	//res[3] += lhs.template slice<0,3>() * rhs.get_translation();
	res[3] += v[0] * t[0] + 
		  v[1] * t[1] +
		  v[2] * t[2];
	
	  
	return res;
	}
	
};

}
/// Left-multiply by a Vector
/// @relates SE3
template<typename PV, typename P> 
inline cv::Vec<typename MyOperatorOverloads::MultiplyType<PV, P>::type, 4> operator *( const cv::Vec<PV, 4> &v, const RigidTransforms::SE3<P> &se3){
	
  return Operator<MyOperatorOverloads::VSE3Mult<PV, P> >(v,se3).compute();
}

//////////////////
// operator *   //
// SE3 * Matrix //
//////////////////

namespace MyOperatorOverloads {
  // template constant for SE3 * Matrix
template <typename P, typename PM> struct SE3MMult;


// An the actual Operator object that does the job...
template<typename P, typename PM>
struct Operator<MyOperatorOverloads::SE3MMult<P, PM> > {
	const RigidTransforms::SE3<P> &se3;
	const cv::Mat_<PM> &M;
	
	Operator(const RigidTransforms::SE3<P> &se3_in, const cv::Mat_<PM> &M_in ) : se3(se3_in), M(M_in) {}
	
	typedef typename MyOperatorOverloads::MultiplyType<P, PM>::type P0;
	
	cv::Mat_<P0> compute() const {
		
	  cv::Mat_<P0> res(4, M.cols);
	  for(int i=0; i<M.cols; ++i) {
	    // turning this into a SE3 * vector business...
	    cv::Vec<P0, 4> column = se3 * cv::Vec<PM, 4>( M(0, i),
							  M(1, i),
							  M(2, i),
							  M(3, i) );
	    res(0, i) = column[0]; res(1, i) = column[1]; res(2, i) = column[2]; res(3, i) = column[3];
	   }
	
	  return res;
	}
	
};
}// close MyOperatorOverloads clause

/// Right-multiply by a Matrix operator
/// @relates SE3
template <typename P, typename PM> 
inline cv::Mat_<typename MyOperatorOverloads::MultiplyType<P,PM>::type> operator *(const RigidTransforms::SE3<P> &se3, const cv::Mat_<PM> &M) {
	
  return Operator<MyOperatorOverloads::SE3MMult<P, PM> >(se3, M).compute();
}

//////////////////
// operator *   //
// Matrix * SE3 //
//////////////////

namespace MyOperatorOverloads {
  // template constant for Matrix * Vector
template <typename PM, typename P> struct MSE3Mult;


template<typename PM, typename P>
struct Operator<MyOperatorOverloads::MSE3Mult<PM, P> > {
	const cv::Mat_<PM> &M;
	const RigidTransforms::SE3<P> &se3;
	
	Operator( const cv::Mat_<PM> &M_in, const RigidTransforms::SE3<P> &se3_in) : M(M_in), se3(se3_in) {}
	
	typedef typename MyOperatorOverloads::MultiplyType<PM, P>::type P0;
	
	cv::Mat_<P0> compute() const {
		
	  cv::Mat_<P0> res(M.rows, 4);
	  
	  for(int i=0; i< M.rows; ++i) {
	    // again, turning this to Vector4 * SE3 business...
	    //res[i] = lhs[i] * rhs;
	    cv::Vec<P0, 4> row = cv::Vec<PM, 4>(M(i, 0), 
						M(i, 1), 
						M(i, 2), 
						M(i, 3)  ) * se3;
	    
	    res(i, 0) = row[0]; res(i, 1) = row[1]; res(i, 2) = row[2]; res(i, 3) = row[3];
	   }
	   return res;
	}
	
};

}; // end MyOperatorOverloads

/// Left-multiply by a Matrix operator
/// @relates SE3
template <typename PM, typename P> 
inline cv::Mat_<typename MyOperatorOverloads::MultiplyType<PM, P>::type> operator *(const cv::Mat_<PM> &M, const RigidTransforms::SE3<P> &se3 ) {
	
  return Operator<MyOperatorOverloads::MSE3Mult<PM, P> >(M,se3).compute();
}




/// Get the SE3 object corresponding to a 6D pose vector
template <typename Precision>
template <typename P>
inline RigidTransforms::SE3<Precision> RigidTransforms::SE3<Precision>::exp(const cv::Vec<P, 6> &mu){
	
	static const Precision one_6th = 1.0/6.0;
	static const Precision one_20th = 1.0/20.0;
	using std::sqrt;
	
	SE3<Precision> result;
	
	//const Vector<3,Precision> w = mu.template slice<3,3>();
	// obtaining the axis-angle (Lie) vector w from the pose vector 
	const cv::Vec<Precision, 3> w(mu[3], mu[4], mu[5]);
	// also storing the position/translation vector
	const cv::Vec<Precision, 3> trans(mu[0], mu[1], mu[2]);
	
	// the squared angle of rotation
	const Precision theta_sq = w[0] * w[0] + w[1] * w[1] + w[2] * w[2];
	// the angle
	const Precision theta = sqrt(theta_sq);
	// Now, in terms of the Rodrigues formula, Rosten likes to precaluclate quantities A and B 
	// in order to cope with numerical instability due to angles close to zero...
	Precision A, B;
	
	//const Vector<3,Precision> cross = w ^ mu.template slice<0,3>();
	// This is the cross product of the axis-angle vector with the position vector
	const cv::Vec<Precision, 3> cross( -w[2] * trans[1] + w[1] * trans[2],
					    w[2] * trans[0] - w[0] * trans[2],
					   -w[1] * trans[0] + w[0] * trans[1] );
	
	//const cv::Vec<Precision, 3> cross = w ^ trans;
	// Now, if the angle is very-very-very small, the Rodrigues formula is ill-conditioned...
	if (theta_sq < 1e-8) {
		A = 1.0 - one_6th * theta_sq;
		B = 0.5;
		//result.get_translation() = mu.template slice<0,3>() + 0.5 * cross;
		result.get_translation() = trans + 0.5 * cross;
		
	} 
	  else { // else 
		Precision C;
		// if angle is just very-very (2 times only :) ) small, then the formula is ill-conditioned again, 
		// but slightly different formulas are used
		if (theta_sq < 1e-6) {
			C = one_6th*(1.0 - one_20th * theta_sq);
			A = 1.0 - theta_sq * C;
			B = 0.5 - 0.25 * one_6th * theta_sq;
		} else {
			const Precision inv_theta = 1.0/theta;
			A = sin(theta) * inv_theta;
			B = (1 - cos(theta)) * (inv_theta * inv_theta);
			C = (1 - A) * (inv_theta * inv_theta);
		}
		//result.get_translation() = mu.template slice<0,3>() + TooN::operator*(B, cross) + TooN::operator*(C, (w ^ cross));
		// storing the corss product of w and w x pos (which essentially is [w]^2 * pos
		cv::Vec<Precision, 3> temp(-w[2] * cross[1] + w[1] * cross[2],
					    w[2] * cross[0] - w[0] * cross[2],
					   -w[1] * cross[0] + w[0] * cross[1] );
		
		
		result.get_translation() = trans + B * cross + C * temp;
		
	}
	
	// For the rotation part, just invoke Rodrigues' formula using the precomputed A, B terms  for numerical stability
	rodrigues_so3_exp(w, A, B , result.get_rotation().get_matrix());
	
	
	
	return result;
}



template <typename Precision>
inline cv::Vec<Precision, 6> RigidTransforms::SE3<Precision>::ln(const SE3<Precision> &se3) {
	using std::sqrt;
	// obtaining the axis-angle (Lie) vector
	cv::Vec<Precision, 3> rot = se3.get_rotation().ln();
	// get the squared angle 
	const Precision theta2 = rot[0] * rot[0] + rot[1] * rot[1] + rot[2] * rot[2];
	// and the angle
	const Precision theta = sqrt(theta2);
	// some manipulation for small (actually, large) angles
	Precision shtot = 0.5;	
	if(theta > 0.00001) {
	  shtot = sin(theta/2)/theta;
	}
	
	// now do the rotation
	const SO3<Precision> halfrotator = SO3<Precision>::exp(rot * -0.5);
	cv::Vec<Precision, 3> rottrans = halfrotator * se3.get_translation();
	
	// theta is reasonably large. Using standard Rodrigues formula...
	if(theta > 0.001){
	  //rottrans -= TooN::operator*(rot, (se3.get_translation() * rot) * (1-2*shtot) / (rot*rot) );
	  Precision proj = (1 - 2 * shtot) * ( se3.get_translation()[0] * rot[0] + 
					       se3.get_translation()[1] * rot[1] + 
					       se3.get_translation()[2] * rot[2]   ) / theta2; 
			  
	  rottrans -=  proj * rot;
	  	
	} // angle is very small. Applying slightly different formula (on the limit)
	else {
		//rottrans -= TooN::operator*(rot, ((se3.get_translation() * rot)/24));
		Precision proj = (se3.get_translation()[0] * rot[0] + 
				  se3.get_translation()[1] * rot[1] + 
				  se3.get_translation()[2] * rot[2]) / 24.0;
		rottrans -= proj * rot;
	}
	
	rottrans /= (2 * shtot);
	
	// Assigning rotation (3:5) and "rotated translation" (rottrans 0:2)
	cv::Vec<Precision, 6> result;
	// "rotated translation" (0-2)
	result[0] = rottrans[0]; result[1] = rottrans[1]; result[2] = rottrans[2];
	// rotation (3-5)
	result[3] = rot[0]; result[4] = rot[1]; result[5] = rot[2];
	
	return result;
}

template <typename Precision>
inline RigidTransforms::SE3<Precision> operator *(const RigidTransforms::SO3<Precision> &left, const RigidTransforms::SE3<Precision>&right) {
	
  return RigidTransforms::SE3<Precision>(left*right.get_rotation(),left*right.get_translation());
}



//} // close rigidTransforms

#endif