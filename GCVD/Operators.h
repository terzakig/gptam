// George Terzakis 2016
//
// University of Portsmouth


#ifndef OPERATORS_H
#define OPERATORS_H


#include "../OpenCV.h"

#include <limits.h>

using namespace std;
using namespace cv;



namespace MyOperatorOverloads {

  
// Rosten's clever way of deducting the type of a class
template<class C> C gettype();
	

// This is a BRILIANT way of working out and defining the type triplets of pairwise operations
template<class L, class R> struct AddType      { typedef decltype (gettype<L>()+gettype<R>()) type;};
template<class L, class R> struct SubtractType { typedef decltype (gettype<L>()-gettype<R>()) type;};
template<class L, class R> struct MultiplyType { typedef decltype (gettype<L>()*gettype<R>()) type;};
template<class L, class R> struct DivideType   { typedef decltype (gettype<L>()/gettype<R>()) type;};

	

//These are the operations in terms of the a) ACTUAL operation (static function op)
// and, b) The return type (struct Return)
struct Add{
  template<class A, class B, class C>      static A op(const B& b, const C& c){ return b+c; }
  template<class P1, class P2> struct Return { typedef typename AddType<P1,P2>::type Type; };
};

struct Subtract{
  template<class A, class B, class C> static A op(const B& b, const C& c){return b-c;}
  template<class P1, class P2> struct Return { typedef typename SubtractType<P1,P2>::type Type;};
};

struct Multiply{
  template<class A, class B, class C> static A op(const B& b, const C& c){return b*c;}
  template<class P1, class P2> struct Return { typedef typename MultiplyType<P1,P2>::type Type;};
};

// I have added Dot product in order to specialize the operation and use OpenCV's dot product 
// for same-type arguments 
struct DotProduct {
  //template<class A, class B, class C> static A op(const B& b, const C& c){return b*c;}
  template<class P1, class P2> struct Return { typedef typename MultiplyType<P1,P2>::type Type;};
};
	
struct Divide{
  template<class A, class B, class C>   static A op(const B& b, const C& c){return b/c;}
  template<class P1, class P2> struct Return { typedef typename DivideType<P1,P2>::type Type;};
};
	
	  
	






// ********************************** The general template for Operator ****************************
// This is the class (struct) that does the actual computation
// In other words, one layer before operator overloading
template<class Op> struct Operator{};


//////////////////////////////////////////////////////////////////////////////////
//                         Vector <op> Vector
//////////////////////////////////////////////////////////////////////////////////

// ******************* "template constant" for PAIRWISE VECTOR Operator sepcializations ************
// Now this struct IS ESSENTIALLY A TAG for further templating. In effect, it is used as a template "constant".
// It "says" that vector (hence the V in "V"Pairwise) element-wise
// operation involves a) The operation Op, b) The size of the vectors, c) the types P1 and P2
template<typename Op, typename P1, int S, typename P2> struct VElementwise;

// ****************** "template constant" for VECTOR NEGATIONS **********************
// This is easy. It's the vector negation template "constant".
// OpenCV has a negation overload, so it is unenecssary in the context
//template <int S, typename P> struct VNegate; 
	


// Now we need to build Operator classes (actually, structs)
// using the above template "constants" (i.e., vector pairwise operations and vector negation)

// *************** General pairwise Operator specialization ***********************
// This is a vector pairwise general operator (includes addition and multiplication as dot product) 
template<typename Op,typename P1, int S, typename P2> // Here Op is the operation (i.e., Add, Multiply, etc..)
struct Operator<VElementwise<Op, P1, S, P2> > {
	const cv::Vec<P1, S> &left;
	const cv::Vec<P2, S> &right;

	Operator(const cv::Vec<P1, S> &left_in, const cv::Vec<P2, S> &right_in) : left(left_in), right(right_in) {}

	
	typedef typename Op::template Return<P1, P2>::Type P0;
	
	cv::Vec<P0, S> compute() const
	{
		cv::Vec<P0, S> res = cv::Vec<P0, S>();
		for(int i=0; i < left.rows; ++i)
			res[i] = Op::template op<P0, P1, P2>(left[i], right[i]);
		
		return res;
	}
	
};

// The dot product as the only product between cv::Vec 
template<typename P1, int S, typename P2> // Here Op is specialized to DotProduct
struct Operator<VElementwise<DotProduct, P1, S, P2> > {
	const cv::Vec<P1, S> &left;
	const cv::Vec<P2, S> &right;

	Operator(const cv::Vec<P1, S> &left_in, const cv::Vec<P2, S> &right_in) : left(left_in), right(right_in) {}

	typedef typename MultiplyType<P1, P2>::type P0;
	
	P0 compute() const
	{
	  P0 res = 0;
	  for(int i=0; i<left.rows; i++) res += left[i]*right[i];
	
	  return res;
	}
	
};

// The dot product as the only product between cv::Vec for same-type vectors (use openCV's dot() function)
template<typename P, int S> // Here Op is specialized to DotProduct
struct Operator<VElementwise<DotProduct, P, S, P> > {
	const cv::Vec<P, S> &left;
	const cv::Vec<P, S> &right;

	Operator(const cv::Vec<P, S> &left_in, const cv::Vec<P, S> &right_in) : left(left_in), right(right_in) {}

	
	P compute() const
	{
	  return left.dot(right);
	}
	
};

} // ****************** close MyOperatorOverloads for operator overloads to follow... ******************************

// 1. Addition Vector + Vector 
template<typename P1, int S, typename P2> 
cv::Vec<typename MyOperatorOverloads::AddType<P1, P2>::type, S> operator +(const cv::Vec<P1, S> &v1, const cv::Vec<P2, S> &v2) {
	
  return MyOperatorOverloads::Operator<MyOperatorOverloads::VElementwise<MyOperatorOverloads::Add,P1,S, P2> >(v1,v2).compute();
  
}

// 2. Subtraction Vector - Vector operator
template<int S, typename P1, typename P2> 
cv::Vec<typename MyOperatorOverloads::SubtractType<P1, P2>::type, S> operator -(const cv::Vec<P1, S> &v1, const cv::Vec<P2, S> &v2)
{

  return MyOperatorOverloads::Operator<MyOperatorOverloads::VElementwise<MyOperatorOverloads::Subtract, P1, S, P2> >(v1,v2).compute();
  
}

// 3. diagmult Vector, Vector 
// (George: Rosten refers to elementwise multiplication as matrix multiplication with a diagonal matrix)
template < typename P1, int S, typename P2>
cv::Vec<typename MyOperatorOverloads::MultiplyType<P1,P2>::type, S> diagmult(const cv::Vec<P1,S> &v1, const cv::Vec<P2,S> &v2)
{
	
  return MyOperatorOverloads::Operator<MyOperatorOverloads::VElementwise<MyOperatorOverloads::Multiply, P1, S, P2> >(v1,v2).compute();

}



// Finally, the actual operator overloading for vector-vector multiplication
/*template<typename P1, int Sz, typename P2>
typename MyOperatorOverloads::MultiplyType<P1, P2>::type operator *(const cv::Vec<P1, Sz> &v1, const cv::Vec<P2, Sz> &v2) {
  
  return MyOperatorOverloads::Operator<MyOperatorOverloads::VElementwise<MyOperatorOverloads::DotProduct, P1, Sz, P2> >(v1, v2).compute();
}*/

// 6.a. ^ is the cross product between vectors (and 3x1 Matx objects)
template <typename P1, typename P2>
cv::Vec<typename MyOperatorOverloads::MultiplyType<P1,P2>::type, 3> operator ^(const cv::Matx<P1, 3, 1> &v1, const cv::Matx<P2, 3, 1 > &v2) {

	// assume the result of adding two restypes is also a restype
	typedef typename MyOperatorOverloads::MultiplyType<P1,P2>::type restype;

	cv::Vec<restype, 3> result;

	
	// [0 -v1(2) v1(1); v1(2) 0 -v1(0); -v1(1) v1(0) 0] * [v2(0); v2(1); v2(2) ]
	
	result[0] =  -v1(2, 0)*v2(1, 0) + v1(1, 0)*v2(2, 0);
	result[1] =   v1(2, 0)*v2(0, 0) - v1(0, 0)*v2(2, 0);
	result[2] =  -v1(1, 0)*v2(0, 0) + v1(0, 0)*v2(1, 0) ;

	return result;
}

// 6.b. cross product between 1x3 Matx objects
template <typename P1, typename P2>
cv::Matx<typename MyOperatorOverloads::MultiplyType<P1,P2>::type, 1, 3> operator ^(const cv::Matx<P1, 1, 3> &v1, const cv::Matx<P2, 1, 3 > &v2) {

	// assume the result of adding two restypes is also a restype
	typedef typename MyOperatorOverloads::MultiplyType<P1,P2>::type restype;

	cv::Matx<restype, 1, 3> result;

	
	// [0 -v1(2) v1(1); v1(2) 0 -v1(0); -v1(1) v1(0) 0] * [v2(0); v2(1); v2(2) ]
	
	result(0, 0) =  -v1(0, 2)*v2(0, 1) + v1(0, 1)*v2(0, 2);
	result(0, 1) =   v1(0, 2)*v2(0, 0) - v1(0, 0)*v2(0, 2);
	result(0, 2) =  -v1(0, 1)*v2(0, 0) + v1(0, 0)*v2(0, 1) ;

	return result;
}


// 7. Cross product between vectors stored in cv::Mat as 3x1 matrices (ugly, but could be the case...)
template <typename P1, typename P2>
cv::Mat_<typename MyOperatorOverloads::MultiplyType<P1,P2>::type> operator ^(const cv::Mat_<P1> &mv1, const cv::Mat_<P2> &mv2) {

	// assume the result of adding two restypes is also a restype
	typedef typename MyOperatorOverloads::MultiplyType<P1,P2>::type P0;

	cv::Mat_<P0> result(3, 1);

	
	// [0 -v1(2) v1(1); v1(2) 0 -v1(0); -v1(1) v1(0) 0] * [v2(0); v2(1); v2(2) ]
	
	result(0, 0) =  -mv1(2, 0)*mv2(1, 0) + mv1(1, 0)*mv2(2, 0);
	result(1, 0) =   mv1(2, 0)*mv2(0, 0) - mv1(0, 0)*mv2(2, 0);
	result(2, 0) =  -mv1(1, 0)*mv2(0, 0) + mv1(0, 0)*mv2(1, 0);

	return result;
}



// again opening my operators in order to specialize more Operator templates for matrices
namespace MyOperatorOverloads {

//********************************************************************************
//                            cv::Mat <op> cv::Mat
//
//			      cv::Matx <op> cv::Matx
//********************************************************************************

// **** Defining the (dummy) template "constants" corresponding to the groups of operations between matrices **********
	
// Template "constant" for cv::Mat elementwise operations
template<typename Op,typename P1, typename P2> struct MElementwise;

// Similar Template "constant" for cv::Matx elementwise operations
template<typename Op,typename P1, int R, int C, typename P2> struct MxElementwise;


// Template "constant" for cv::Mat matrix multiplication
template<typename P1,typename P2> struct MatrixMultiply;

// Similar template "constant" for cv::Matx matrix multiplication
template<typename P1, int R1, int C1, typename P2, int R2, int C2> struct MatxMultiply;


// Negation - not necessary with OpenCV matrices
//template<typename P> struct MNegate;


// Matrix generic Operator (on cv::Mat operands) specialized template for any two types of precision (P1, P2)
// for elementwise operations.
template<typename Op,typename P1, typename P2> 
struct Operator<MElementwise<Op, P1, P2> > {

	const cv::Mat_<P1> &left;
	const cv::Mat_<P2> &right;

  
	Operator(const cv::Mat_<P1> &left_in, const cv::Mat_<P2> &right_in) : left(left_in), right(right_in) { }

	typedef typename Op::template Return<P1, P2>::Type P0;
	
	cv::Mat_<P0> compute() const
	{
	    int rrows = min(left.rows, right.rows);
	    int rcols = min(left.cols, right.cols);
	    
	    cv::Mat_<P0> res(rrows, rcols);
		for(int r=0; r < rrows; ++r){
			for(int c=0; c < rcols; ++c){
			  	res(r,c) = Op::template op<P0, P1, P2>(left(r, c), right(r, c));
			}
		}
		return res;
	}
	
};

// Matrix generic Operator (on cv::Matx operands) specialized template for any two types of precision (P1, P2)
// for elementwise operations.
template<typename Op,typename P1, int R, int C, typename P2> 
struct Operator<MxElementwise<Op, P1, R, C, P2> > {

	const cv::Matx<P1, R, C> &left;
	const cv::Matx<P2, R, C> &right;

  
	Operator(const cv::Matx<P1, R, C> &left_in, const cv::Matx<P2, R, C> &right_in) : left(left_in), right(right_in) { 
	  assert(right_in.rows == left_in.rows && right_in.cols == left_in.cols && "array size mismatch");
	}

	typedef typename Op::template Return<P1, P2>::Type P0;
	
	cv::Matx<P0, R, C> compute() const
	{
	    int rrows = left.rows;
	    int rcols = left.cols;
	    
	    cv::Matx<P0, R, C> res(rrows, rcols);
		for(int r=0; r < rrows; ++r)
			for(int c=0; c < rcols; ++c)
			  res(r,c) = Op::template op<P0, P1, P2>(left(r, c), right(r, c));
			
		
	    return res;
	}
	
};


// Addition Operator struct spcialization for same-type arguments of cv::Mat matrices
// The purpose of this is to invoke the respective OpenCV operator for faster (?) implementation
template<typename P> struct Operator<MElementwise<Add, P, P> > {

	const cv::Mat_<P> &left;
	const cv::Mat_<P> &right;

  
	Operator(const cv::Mat_<P> &left_in, const cv::Mat_<P> &right_in) : left(left_in), right(right_in) { }

	cv::Mat_<P> compute() const
	{
	    return cv::operator+(left, right);
	}
	
};

// Addition Operator struct spcialization for same-type arguments of cv::Matx matrices
// The purpose of this is to invoke the respective OpenCV operator for faster (?) implementation
template<typename P, int R, int C> 
struct Operator<MxElementwise<Add, P, R, C, P> > {

	const cv::Matx<P, R, C> &left;
	const cv::Matx<P, R, C> &right;

  
	Operator(const cv::Matx<P, R, C> &left_in, const cv::Matx<P, R, C> &right_in) : left(left_in), right(right_in) { }

	cv::Matx<P, R, C> compute() const
	{
	    return cv::operator+(left, right); // size mismatches should be taken care by OpenCV...
	}
	
};


// Subtraction Operator struct specialization for cv::Mat of same-type 
template<typename P> struct Operator<MElementwise<Subtract, P, P> > {

	const cv::Mat_<P> &left;
	const cv::Mat_<P> &right;

	Operator(const cv::Mat_<P> &left_in, const cv::Mat_<P> &right_in) : left(left_in), right(right_in) { }

	cv::Mat_<P> compute() const
	{
	    return cv::operator-(left, right);
	}
	
};

// Subtraction Operator struct specialization for cv::Matx of same-type 
template<typename P, int R, int C> 
struct Operator<MxElementwise<Subtract, P, R, C, P> > {

	const cv::Matx<P, R, C> &left;
	const cv::Matx<P, R, C> &right;

	Operator(const cv::Matx<P, R, C> &left_in, const cv::Matx<P, R, C> &right_in) : left(left_in), right(right_in) { }

	cv::Matx<P, R, C> compute() const
	{
	    return cv::operator-(left, right); // assertions tshould be taken care by OpenCV...
	}
	
};



} // ****************** close MyOperatorOverloads for operator overloads to follow... ******************************

// Matrix Addition  operator '+' overload for cv::Mat objects
template<typename P1, typename P2> 
cv::Mat_<typename MyOperatorOverloads::AddType<P1, P2>::type> operator +(const cv::Mat_<P1> &m1, const cv::Mat_<P2> &m2) {

  return MyOperatorOverloads::Operator< MyOperatorOverloads::MElementwise<MyOperatorOverloads::Add,P1,P2> >(m1,m2).compute();
}


// Matrix Addition  operator '+' overload for cv::Matx objects
template<typename P1, int  R, int C, typename P2> 
cv::Matx<typename MyOperatorOverloads::AddType<P1, P2>::type, R, C> operator +(const cv::Matx<P1, R, C> &m1, const cv::Matx<P2, R, C> &m2) {

  return MyOperatorOverloads::Operator< MyOperatorOverloads::MxElementwise<MyOperatorOverloads::Add, P1, R, C, P2> >(m1,m2).compute();
}



// Matrix subtraction operator '-' overload for cv::Mat objects
template<typename P1, typename P2> 
cv::Mat_<typename MyOperatorOverloads::SubtractType<P1, P2>::type> operator -(const cv::Mat_<P1> &m1, const cv::Mat_<P2> &m2) {
	
  return MyOperatorOverloads::Operator< MyOperatorOverloads::MElementwise<MyOperatorOverloads::Subtract,P1,P2> >(m1,m2).compute();
  
}

// Matrix subtraction operator '-' overload for cv::Matx objects
template<typename P1, int R, int C, typename P2> 
cv::Matx<typename MyOperatorOverloads::SubtractType<P1, P2>::type, R, C> operator -(const cv::Matx<P1, R, C> &m1, const cv::Matx<P2, R, C> &m2) {
	
  return MyOperatorOverloads::Operator< MyOperatorOverloads::MxElementwise<MyOperatorOverloads::Subtract, P1, R, C, P2> >(m1,m2).compute();
  
}



// cv::Mat .* cv::Mat - Elementwise multiplication of cv::Mat matrices
template <typename P1, typename P2>
cv::Mat_<typename MyOperatorOverloads::MultiplyType<P1,P2>::type> mmult(const cv::Mat_<P1> &m1, const cv::Mat_<P2> &m2)
{
	
  return MyOperatorOverloads::Operator<MyOperatorOverloads::MElementwise<MyOperatorOverloads::Multiply,P1,P2 > >(m1,m2).compute();

}

// cv::Matx .* cv::Matx - Elementwise multiplication of cv::Matx matrices
template <typename P1, int R, int C, typename P2>
cv::Matx<typename MyOperatorOverloads::MultiplyType<P1,P2>::type, R, C> mmult(const cv::Matx<P1, R, C> &m1, const cv::Matx<P2, R, C> &m2)
{
	
  return MyOperatorOverloads::Operator<MyOperatorOverloads::MxElementwise<MyOperatorOverloads::Multiply,P1, R, C, P2 > >(m1,m2).compute();

}



// back to MyOperators again
namespace MyOperatorOverloads {

// Operator structure for Standard Matrix Multiplication Operator specialization for cv::Mat objects of different types 
template<typename P1, typename P2> struct Operator<MatrixMultiply<P1, P2> > {
	const cv::Mat_<P1> &left;
	const cv::Mat_<P2> &right;

	Operator(const cv::Mat_<P1> &left_in, const cv::Mat_<P2> &right_in) : left(left_in), right(right_in) {}

	
	// *********** Non-OpenCV (SLOW, O(n^3) ) multiplication unfortunately.... *******************
	typedef typename MyOperatorOverloads::MultiplyType<P1, P2>::type P0;
	
	cv::Mat_<P0> compute() const {
	  
		cv::Mat_<P0> res(left.rows, right.cols);
		for (int r = 0; r < left.rows; r++)
		  for(int c = 0; c < right.cols; ++c) {
		    P0 sum = 0;
		    for(int j=0; j < right.rows; j++) 
		      sum += left(r, j) * right(j, c);
		    res(r,c) = sum;
		  }
	    return res;
	}
};

// Matrix Multiplication Operator struct specialization for same-type arguments of cv::Mat objects
// Here we invoke the fast, OpenCV multiplication operator
template<typename P> struct Operator<MatrixMultiply<P, P> > {
	const cv::Mat_<P> &left;
	const cv::Mat_<P> &right;

	Operator(const cv::Mat_<P> &left_in, const cv::Mat_<P> &right_in) : left(left_in), right(right_in) {}

	
	// ***********Now, can use OpenCV's fast multiplication *******************
	cv::Mat_<P> compute() const
	{
	  return cv::operator*(left, right); 

	}
	
};


// Operator structure for Standard Matrix Multiplication Operator specialization for cv::Matx objects of different types 
template<typename P1, int R1, int C1, typename P2, int R2, int C2> struct Operator<MatxMultiply<P1, R1, C1, P2, R2, C2> > {
	const cv::Matx<P1, R1, C1> &left;
	const cv::Matx<P2, R2, C2> &right;
	
	
	Operator(const cv::Matx<P1, R1, C1> &left_in, const cv::Matx<P2, R2, C2> &right_in) : left(left_in), right(right_in) {
	  
	  assert(left_in.cols == right_in.rows && "Size mismatch");
	}

	
	// *********** Non-OpenCV (SLOW, VERY SLOW O(n^3) ) multiplication unfortunately.... *******************
	typedef typename MyOperatorOverloads::MultiplyType<P1, P2>::type P0;
	
	cv::Matx<P0, R1, C2> compute() const {
	  
		cv::Matx<P0, R1, C2> res;
		for (int r = 0; r < left.rows; r++)
		  for(int c = 0; c < right.cols; ++c) {
		    P0 sum = 0;
		    for(int j=0; j < right.rows; j++) 
		      sum += left(r, j) * right(j, c);
		    res(r,c) = sum;
		  }
	    return res;
	}
};

// Matrix Multiplication Operator struct specialization for same-type arguments of cv::Matx objects
// Here we invoke the fast, OpenCV multiplication operator (which should be the case most of the time...)
template<typename P, int R1, int C1, int R2, int C2> struct Operator<MatxMultiply<P, R1, C1, P, R2, C2> > {
	const cv::Matx<P, R1, C1> &left;
	const cv::Matx<P, R2, C2> &right;

	
	Operator(const cv::Matx<P, R1, C1> &left_in, const cv::Matx<P, R2, C2> &right_in) : left(left_in), right(right_in) {}

	
	// ***********Now can use OpenCV's fast multiplication *******************
	cv::Matx<P, R1, C2> compute() const
	{
	  return cv::operator*(left, right); // size mismatches left to OpenCV

	}
	
};


} // ****************** close MyOperatorOverloads for operator overloads to follow... ******************************

// Standard Matrix multiplication Matrix * Matrix for cv::Mat objects: The actual operator
template<typename P1, typename P2> 
cv::Mat_<typename MyOperatorOverloads::MultiplyType<P1, P2>::type> operator *(const cv::Mat_<P1> &M1, const cv::Mat_<P2> &M2) {
	
  return MyOperatorOverloads::Operator<MyOperatorOverloads::MatrixMultiply<P1, P2> >(M1,M2).compute();
}

// Standard Matrix multiplication Matrix * Matrix for cv::Matx objects: The actual operator
template<typename P1, int R1, int C1, typename P2, int R2, int C2> 
cv::Matx<typename MyOperatorOverloads::MultiplyType<P1, P2>::type, R1, C2> operator *(const cv::Matx<P1, R1, C1> &M1, const cv::Matx<P2, R2, C2> &M2) {
	
  return MyOperatorOverloads::Operator<MyOperatorOverloads::MatxMultiply<P1, R1, C1, P2, R2, C2> >(M1,M2).compute();
}


// again back to myOperaotrs...
namespace MyOperatorOverloads {
//**********************************************************************************
//                 cv::Mat <op> cv::Vec 
//
//		   cv::Matx <op> cv::Vec
//**********************************************************************************



// Template "constant" struct for cv::Mat * cv::Vec
// This Operator is an abomination really..... But it could save us some trouble...
template<typename PM, typename PV, int Sz> struct MatrixVectorMultiply;

// Template "constant" struct for cv::Matx * cv::Vec
template<typename PM, int R, int C, typename PV, int Sz> struct MatxVectorMultiply;


//Template "constant" struct for cv::Vec * cv::Mat.
// NOTE-NOTE!!!! When a cv::Vec is left-multiplied to a matrix and it is a Nx1 vector, 
// then the Operator struct will compute the multiplication as if the vector was horizontal (i.e. 1 xN)
// If on the other hand we have something like a cv::Vec::t() (transposed vector), then the type of the transposed is 
// automatically a Matx and therefore the operation is handled as a matrix multiplication.
// Again, for the obvious reasons, this Operator is an abomination really..... but it could be helpful
template<typename PV, int Sz, typename PM> struct VectorMatrixMultiply;

//Template "constant" struct for cv::Vec * cv::Matx.
// I am leaving this one out... Not a good idea to have it in...
//template<typename PV, int Sz, typename PM, int R, int C> struct VectorMatxMultiply;





// Standard multiplication cv::Mat * cv::Vec (NB: Since there is no OpenCV operator for cv::Mat * cv::Vec, 
// this is the only way to multiply these objects. BUT TRY TO AVOID IT!!! )
// The return type is cv::Mat_ for obvious reasons (cannot a-priori fix the size of cv::Mat objects). 
template<typename PM, typename PV, int Sz> 
struct Operator<MatrixVectorMultiply<PM, PV, Sz> > {
	const cv::Mat_<PM> &M;
	const cv::Vec<PV, Sz> &v;

	Operator(const cv::Mat_<PM> &M_in, const cv::Vec<PV, Sz> &v_in) : M(M_in), v(v_in) {
	   assert(M_in.cols == v.rows && "Size mismatch");
	}

	typedef typename MultiplyType<PM, PV>::type P0; 
	
	cv::Mat_<P0> compute() const {
	  
	  cv::Mat_<P0> res(M.rows, 1); // we construct a rowsx1 matrix (it should suffice
	
	  for(int r=0; r < M.rows; r++) {
	    res(r, 0) = 0;
	    for (int c = 0; c < v.rows; c++) 
		res(r, 0) += Multiply::template op<P0, PM, PV>(M(r, c) , v[c]); 
	  }
	  
	  return res;
	}
};

/// Standard multiplication cv::Matx * cv::Vec of DIFFERENT-TYPES 
/// Yields cv::Vec
template<typename PM, int R, int C, typename PV, int Sz> 
struct Operator<MatxVectorMultiply<PM, R, C, PV, Sz> > {
	const cv::Matx<PM, R, C> &M;
	const cv::Vec<PV, Sz> &v;

	
	Operator(const cv::Matx<PM, R, C> &M_in, const cv::Vec<PV, Sz> &v_in) : M(M_in), v(v_in) {
	  assert(M_in.cols == v.rows && "Size mismatch"); // need to do this here otherwise noone will shout...
	}

	typedef typename MultiplyType<PM, PV>::type P0;
	
	cv::Vec<P0, R> compute() const {
	  
	  cv::Vec<P0, R> res; 
	
	  for(int r=0; r < M.rows; ++r) {
	    res[r] = 0;
	    for (int c = 0; c < v.rows; c++) 
		res[r] += Multiply::template op<P0, PM, PV>(M(r, c) , v[c]); 
	  }
	  
	  return res;
	}
};

// Standard multiplication cv::Matx * cv::Vec of SAME-TYPE (OpenCV operator)
template<typename P, int R, int C, int Sz> 
struct Operator<MatxVectorMultiply<P, R, C, P, Sz> > {
	const cv::Matx<P, R, C> &M;
	const cv::Vec<P, Sz> &v;

	
	Operator(const cv::Matx<P, R, C> &M_in, const cv::Vec<P, Sz> &v_in) : M(M_in), v(v_in) {}

	
	cv::Vec<P, R> compute() const {
	  
	  return cv::operator*(M, v); // size mismatches should be taken care of by OpenCV
	}
};



} // ****************** close MyOperatorOverloads for operator overloads to follow... ******************************


// cv::Mat * cv::Vec multiplication operator actual overload.
template<typename PM, typename PV, int Sz>
cv::Mat_<typename MyOperatorOverloads::MultiplyType<PM, PV>::type> operator *(const cv::Mat_<PM> &m, const cv::Vec<PV, Sz> &v)
{
	return MyOperatorOverloads::Operator<MyOperatorOverloads::MatrixVectorMultiply<PM, PV, Sz> >(m,v).compute();
}
																	
// cv::Matx * cv::Vec multiplication operator actual overload.
template<typename PM, int R, int C, typename PV, int Sz>
cv::Vec<typename MyOperatorOverloads::MultiplyType<PM, PV>::type, R> operator *(const cv::Matx<PM, R, C> &m, const cv::Vec<PV, Sz> &v)
{
	return MyOperatorOverloads::Operator<MyOperatorOverloads::MatxVectorMultiply<PM, R, C, PV, Sz> >(m,v).compute();
}


// Now, it is easy to get the cv::Vec * cv::Mat operator...
template<typename PV, int Sz, typename PM> 
cv::Mat_<typename MyOperatorOverloads::MultiplyType<PV, PM>::type > operator *(const cv::Vec<PV, Sz> &v, const cv::Mat_<PM> &M) {
	
	return M.t() * v;
}

// And the operator overload for cv::Vec * cv::Matx accordingly...
// I decided to leave this one out. It can be the source of serious trouble. And I dont have a really good reason to have it in...
/*template<typename PV, int Sz, typename PM, int R, int C> 
cv::Vec<typename MyOperatorOverloads::MultiplyType<PV, PM>::type, R> operator *(const cv::Vec<PV, Sz> &v, const cv::Matx<PM, R, C> &M) {
	
	return M.t() * v;
}*/




#endif