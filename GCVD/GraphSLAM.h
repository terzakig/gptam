// ************************ This is the GRAPHSLAM approach to Weighted least squares (WLS) **************************
// *
// *				        Naming and method argument conventions adapted ON Ed. Rosten'S TooN::WLS class 
// *
// *					               George Terzakis 2016
// *
//						     University of Portsmouth
// *						
// *				WLS constructs and solves a Weighted LS non-liear system using the Fisher parametrization 
// *                                                       (see GraphSLAM by Thrun)

#ifndef GRAPHSLAM_H
#define GRAPHSLAM_H


#include "Operators.h"
#include "../OpenCV.h"

#include <cmath>

using namespace cv;

namespace Optimization {

/// Performs Gauss-Newton weighted least squares computation.
template <typename Precision = float>
class WLS {

private:
	cv::Mat_<Precision> Omega_; // Information matrix
	cv::Mat_<Precision> ksi_;   // Information vector. 
	cv::Mat_<Precision> mu_;    // The last known estimate of the mean
	
	unsigned int DecompositionType; // will be just passed to the inversion method
	int Dim;

public:

	/// Default constructor or construct with the number of dimensions for the Dynamic case
	WLS(int dim = 1, unsigned int decomposition = cv::DECOMP_CHOLESKY ) : Omega_( cv::Mat_<Precision>(dim, dim) ),
										 ksi_( cv::Mat_<Precision>(dim, 1) ),
										 mu_( cv::Mat_<Precision>(dim, 1) ),
										 DecompositionType(decomposition),
										 Dim(dim)
	{
		clear();
	}

	/// Clear all the measurements and apply a constant regularisation term. 
	void clear() {
	  
		Omega_ = cv::Mat_<Precision>::zeros(Dim, Dim);
		ksi_ = cv::Mat_<Precision>::zeros(Dim, 1);
	}

	/// Diagonal regularization with teh SAME value (i.e., Omega += val * eye(n, n) 
	void add_prior_diagonal(Precision val) {
		
	  for(int r = 0; r<Omega_.rows; r++)  Omega_(r, r)+=val;
		
	}
  
	/// Applies a regularisation term with a different strength for each parameter value. 
	/// Equates to a prior that says all the parameters are zero with \f$\sigma_i^2 = \frac{1}{\text{v}_i}\f$.
	void add_prior_diagonal(const cv::Mat_<Precision> &v) { // here v is a Dim x 1 matrix
		
	  for(int r = 0; r<Omega_.rows; r++)  Omega_(r, r) += v(r, 0);
		
	}

	/// Add a matrix to the information matrix (i.e., incorporating information from another gaussian). 
	void add_prior(const cv::Mat_<Precision> &m) {
	  
		Omega_ += m;
	}

	/// Add a single measurement 
	/// @param y The value of the measurement (just one measurement)
	/// @param J The Jacobian for the measurement as a 1xDim matrix 
	/// @param weight The inverse variance of the measurement (default = 1)
	inline void add_mJ(Precision y, const cv::Mat_<Precision> &J, Precision weight = 1) {
		
		//Upper right triangle only, for speed
		for(int r=0; r < Omega_.rows; r++)
		{
			Precision Jw = weight * J(0, r);
			ksi_(r, 0) += y * Jw;
			
			for(int c = r; c < Omega_.rows; c++) Omega_(r, c) += Jw * J(0, c);
		}
	}

	/// Add multiple measurements at once (much more efficiently) (N = number of measurements and Dim the number of unknowns)
	/// @param y The measurements to add (N x 1 matrix)
	/// @param J The Jacobian matrix ( N x Dim)
	/// @param Qinv The N x N inverse covariance of the measurement likelihood
	inline void add_mJ(const cv::Mat_<Precision> &y,  // N x 1 matrix
			   const cv::Mat_<Precision> &J,  // Jacobian (N x Dim) 
			   const cv::Mat_<Precision> &Qinv // The inverse covariance matrix of the measurement likelihood
			   ) {
		
		const cv::Mat_<Precision> temp =  J.t() * Qinv;  // this is a (1xDim)*(Dim x Dim) = 1 x Dim matrix
		Omega_ += temp * J;
		ksi_ += temp * y;
	}

	

	/// Add a single measurement at once with a sparse Jacobian (much, much more efficiently)
	/// Suppose that y is given in the form 1xM, where M <= Dim is the number of variables
	/// that have non-zero derivative. We therefore muster these M derivatives in a 1 x M Jacobian and incorporate it to
	/// the upper triangle of the information matrix (and vector)
	/// @param y The 1 x M measurement vector 
	/// @param J1 The 1 x M Jacobian matrix wrt to the M dependent variables 
	/// @param index starting index (in the state vector) of the block
	/// @param weight The inverse variance of the measurement values
	inline void add_sparse_mJ(const Precision y,             // N x 1 measurement vector
				  const cv::Mat_<Precision> &J1, // 1xM Jacobian vector as a cv::Mat_
				  const int index,		 // index of the first variable in the state vector
				  const Precision weight = 1) {
		//Upper right triangle only, for speed
		for(int r=0; r < J1.cols; r++)
		{
			Precision Jw = weight * J1(0, r);
			ksi_[r+index] += y * Jw;
			for(int c = r; c < J1.cols; c++)
				Omega_(r+index, c+index) += Jw * J1(0, c);
		}
	}

	/// Add N - multiple measurements at once with a sparse Jacobian (much, much more efficiently)
	/// @param y The N x 1 measurement vector as a cv:;Mat_
	/// @param J The NxM Jacobian matrix (we assume that the M variables are SUCCESIVELY laid in the state vector)
	/// @param index The index of the first Jacobian variable in the state vector
	/// @param Qinv N x N inverse covariance of the measurement likelihood
	inline void add_sparse_mJ_rows(const cv::Mat_<Precision> &y,  // N x 1 matrix
				       const cv::Mat_<Precision> &J,  // N x M matrix
				       const int index,	      // Starting index of the first dependent variable in the state vector
				       const cv::Mat_<Precision> &Qinv) {
		const cv::Mat_<Precision> temp1 = J.t() * Qinv; // this is a (N x M) * (N x N ) = M x N matrix 
		const int M = J.cols;
		// so now updating block-wise....
		cv::Mat_<Precision> OmegaBlockMxM = Omega_(cv::Range(index, index + M), cv::Range(index, index + M) ); 
		OmegaBlockMxM = OmegaBlockMxM + temp1 * J;
		//cv::Mat_<Precision> updatedBlock = OmegaBlockHead + temp1 * J; // so temp1 * J is (M x N)*(N x M) = M x M matrix
		//updatedBlock.copyTo( OmegaBlockHead );
		
		//my_vector.slice(index1, size1) += temp1 * m;
		cv::Mat_<Precision> ksiBlockMx1 = ksi_(cv::Range(index , index + M), cv::Range::all() );
		ksiBlockMx1 = ksiBlockMx1 + temp1 * y;
		//cv::Mat_<Precision> updatedKsiBlock = ksiBlockHead + temp1 * y;
		//updatedKsiBlock.copyTo( ksiBlockHead );
	  }

	/// Add multiple measurements divided  into sets (say, of size M1 and M2 respectively) of consecutive variables in the state vector
	/// @param y the N x 1 measurement vector
	/// @param J1 The N x M1 first block of the Jacobian matrix 
	/// @param index1 starting index for the first variable (first block)
	/// @param J2 The N x M2 second block of the Jacobian matrix 
	/// @param index2 starting index of the M1-th variable (second block) in the state vector
	/// @param Qinv The N x N inverse covariance of the measurement values
	inline void add_sparse_mJ_rows(const cv::Mat_<Precision> &y,
				       const cv::Mat_<Precision> &J1, 
				       const int index1,
				       const cv::Mat_<Precision> &J2, 
				       const int index2,
				       const cv::Mat_<Precision> &Qinv) {
		
		const cv::Mat_<Precision> temp1 = J1.t() * Qinv; // An (M1 x N) * (N x N) = M1 x N matrix
		const cv::Mat_<Precision> temp2 = J2.t() * Qinv; // An (M2 x N) * (N x N) = M2 x N matrix
		const cv::Mat_<Precision> mixed = temp1 * J2;    // A (M1 x N) * (N x M2) = M1 x M2 block
		const int M1 = J1.cols;
		const int M2 = J2.cols;
		//my_C_inv.slice(index1, index1, size1, size1) += temp1 * J1;
		cv::Mat_<Precision> OmegaBlockM1xM1 = Omega_( cv::Range(index1, index1 + M1), cv::Range(index1, index1 + M1) );
		OmegaBlockM1xM1 = OmegaBlockM1xM1 + temp1 * J1;
		//cv::Mat_<Precision> updatedBlock1 = OmegaBlock1Head + temp1 * J1;
		//updatedBlock1.copyTo( OmegaBlock1Head);
		
		//my_C_inv.slice(index2, index2, size2, size2) += temp2 * J2;
		cv::Mat_<Precision> OmegaBlockM2xM2 = Omega_( cv::Range(index2, index2 + M2), cv::Range(index2, index2 + M2) );
		OmegaBlockM2xM2 = OmegaBlockM2xM2 + temp2 * J2;
		//cv::Mat_<Precision> updatedBlock2 = OmegaBlock2Head + temp2 * J2;
		//updatedBlock2.copyTo( OmegaBlock2Head);
		
		//my_C_inv.slice(index1, index2, size1, size2) += mixed;
		cv::Mat_<Precision> OmegaBlockM1xM2 = Omega_( cv::Range(index1, index1 + M1), cv::Range(index2, index2 + M2) );
		OmegaBlockM1xM2 = OmegaBlockM1xM2 + mixed;
		//cv::Mat_<Precision> updatedBlock12 = OmegaBlock12Head + mixed;
		//updatedBlock12.copyTo( OmegaBlock12Head);
		
		//my_C_inv.slice(index2, index1, size2, size1) += mixed.T();
		cv::Mat_<Precision> OmegaBlockM2xM1 = Omega_( cv::Range(index2, index2 + M2), cv::Range(index1, index1 + M2));
		OmegaBlockM2xM1 = OmegaBlockM2xM1 + mixed.t();
		//cv::Mat_<Precision> updatedBlock21 = OmegaBlock21Head + mixed.t();
		//updatedBlock21.copyTo( OmegaBlock21Head);
		
		//my_vector.slice(index1, size1) += temp1 * m;
		cv::Mat_<Precision> ksiBlockM1 = ksi_( cv::Range(index1 , index1 + M1), cv::Range::all() );
		ksiBlockM1 = ksiBlockM1 + temp1 * y;
		//cv::Mat_<Precision> updatedKsiBlock1 = ksiBlock1Head + temp1 * y;
		//updatedKsiBlock1.copyTo( ksiBlock1Head );
		
		//my_vector.slice(index2, size2) += temp2 * m;
		cv::Mat_<Precision> ksiBlockM2 = ksi_( cv::Range(index2 , index2 + M2), cv::Range(0, 1) );
		ksiBlockM2 = ksiBlockM2 + temp2 * y;
		//cv::Mat_<Precision> updatedKsiBlock2 = ksiBlock2Head + temp2 * y;
		//updatedKsiBlock2.copyTo( ksiBlock2Head );
		
	}

	/// Process all the measurements and compute the weighted least squares set of parameter values
	/// stores the result internally which can then be accessed by calling get_mu()
	void compute() {
	
		//Copy the upper right triangle to the empty lower-left.
		for(int r=1; r < Omega_.rows; r++)
			for(int c=0; c < r; c++)
				Omega_(r, c) = Omega_(c, r);

		//mu_ = Omega_.inv(DecompositionType) * ksi_;
		cv::solve(Omega_, ksi_, mu_, DecompositionType);
	}

	/// Combine Gaussian distributions in Fisher form
	/// @param F The other gaussian/LS formulation
	void operator += (const WLS& F) {
		
	    ksi_ += F.ksi_;
	    Omega_ += F.Omega_;
	}

	/// Returns the inverse covariance matrix
	cv::Mat_<Precision>& get_Omega() {return Omega_;} /// Returns the information matrix
	const cv::Mat_<Precision>& get_Omega() const {return Omega_;} /// Returns information matrix (const overload)
	cv::Mat_<Precision>& get_mu(){return mu_;}  ///Returns the update. With no prior, this is the last computation inv(Omega_) * ksi_.
	const cv::Mat_<Precision>& get_mu() const {return mu_;} ///<Returns the update. With no prior, this is the result of \f$J^\dagger e\f$.
	cv::Mat_<Precision>& get_ksi(){return ksi_;} ///Returns the  information vector
	const cv::Mat_<Precision>& get_ksi() const {return ksi_;} ///Returns the  information vector (constant overload)
	unsigned int get_decomposition(){return DecompositionType;} /// Return the decomposition type (just an integer)
	

};

}

#endif