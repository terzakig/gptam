



#ifndef PER_DEFAULT_H
#define PER_DEFAULT_H

#include <memory> 
#include "../OpenCV.h"

namespace Persistence
{
  
  
template<class C> struct DefaultValue
{
	 
  typedef C Type; // return the type 'cause we need to pass it to Serialization functions
			   // Almost every object has the same type EXCEPT the CvMatrixWrapper which has type cv::Mat_<P>
			   // I hope all this works to my benefit and not to my disaster...
  
  static inline const C val() { return C(); }
	
	
};

// signify bad declarations that the compiler allows...
template<class C> struct IsAwkward { static const int is = 0; };



// by default, cv::Vec<P, Sz> it is not awkward
template<typename P, int Sz> struct IsAwkward<cv::Vec<P, Sz> > { static const int is = 0; };
// but size -1, 0, 1 is...
template<typename P> struct IsAwkward<cv::Vec<P, -1> > { static const int is = 1; };
template<typename P> struct IsAwkward<cv::Vec<P, 1> > { static const int is = 1; };
template<typename P> struct IsAwkward<cv::Vec<P, 0> > { static const int is = 1; };


// for matrices we need a dummy wrapper template struct in oprder to be able to define rows and colums in the template....
template<typename P, int R, int C> struct CvMatrixWrapper;
  
// use this as template instantiation for Serialize functions};


// by default it is not awkward,
template<typename P, int R, int C> struct IsAwkward<CvMatrixWrapper<P, R, C> > { static const int is = 0; };
// but other confgurations are...
template<typename P, int R> struct IsAwkward<CvMatrixWrapper<P, R, 0> > { static const int is = 1; };
template<typename P, int R> struct IsAwkward<CvMatrixWrapper<P, R, -1> > { static const int is = 1; };
template<typename P, int C> struct IsAwkward<CvMatrixWrapper<P, 0, C> > { static const int is = 1; };
template<typename P, int C> struct IsAwkward<CvMatrixWrapper<P, -1, C> > { static const int is = 1; };


// The following specialization(s) concerns the case in whic a vector "is awkward"
template<typename P> struct DefaultValue<cv::Vec<P, 1> >
{
	// the default size
	static const int dim = 2;
	// can only return >= 2 long vectors. So a 2-vector it is..
	static inline const cv::Vec<P, dim> val() { return cv::Vec<P, dim>(0, 0); }
};
template<typename P> struct DefaultValue<cv::Vec<P, 0> >
{
	// the default size
	static const int dim = 2;
	// can only return >= 2 long vectors. So a 2-vector it is..
	static inline const cv::Vec<P, dim> val() { return cv::Vec<P, dim>(0, 0); }
	
};
template<typename P> struct DefaultValue<cv::Vec<P, -1> >
{
	// the default size
	static const int dim = 2;
	// can only return >= 2 long vectors. So a 2-vector it is..
	static inline const cv::Vec<P, dim> val() { return cv::Vec<P, dim>(0, 0); }
	
};






template<typename P, int Sz> struct DefaultValue<cv::Vec<P, Sz> > 
{
	// And the type!
	typedef typename cv::Vec<P, Sz> Type;
	
	// can only return >= 2 long vectors. So a 2-vector it is..
	static inline const cv::Vec<P, Sz> val() { return cv::Vec<P, Sz>::all(0); }
	
};



// matrix default: Ok, I am creating a wrapper in which dimensions have to be explicitly defined in the template.
// This is the only way i can think of creating proper sized defaults
template<typename P, int R, int C> struct DefaultValue<CvMatrixWrapper<P, R, C> >
{
	 // And the type!
	typedef typename cv::Mat_<P> Type;
	
	// Hmmmmm... default matrix
	static inline const cv::Mat_<P> val() { return cv::Mat_<P>::zeros(R, C);   }
	
};



// some invalid matrix configurations
template<typename P, int R>  struct DefaultValue<CvMatrixWrapper<P, R, -1> > {
  	// default is a 1x1 zero
	static inline const cv::Mat_<P> val() { cv::Mat_<P> ret(1, 1); ret(0,0) = 0; return ret; }
	
};
template<typename P, int R>  struct DefaultValue<CvMatrixWrapper<P, R, 0> > {
  	// default is a 1x1 zero
	static inline const cv::Mat_<P> val() { cv::Mat_<P> ret(1, 1); ret(0,0) = 0; return ret; }
	
	
};
template<typename P, int C>  struct DefaultValue<CvMatrixWrapper<P, 0, C> > {
  	// default is a 1x1 zero
	static inline const cv::Mat_<P> val() { cv::Mat_<P> ret(1, 1); ret(0,0) = 0; return ret; }
};
template<typename P, int C>  struct DefaultValue<CvMatrixWrapper<P, -1, C> > {
  	// default is a 1x1 zero
	static inline const cv::Mat_<P> val() { cv::Mat_<P> ret(1, 1); ret(0,0) = 0; return ret; }
};





// Ok, a few words about "iSAwkward". It essentially represents bad declaration of a class (for instance,
// a negative, zero or one length for a cv::Vec; TooN doesn't shout at you for such values, but openCV DOES).
// I would think that Rosten added this evaluated (based on declaration) template value, in order to produce valid defaults,
// in erroneous, yet compilable cases...

// If I am not mistaken the ValueHolder should work for openCV Vector templates, 
// BUT(!!!!): For OpenCV matrices we MUST use the CvMatrixWrapper struct!!!!!!!!
template<class C, int PainInTheNeck = IsAwkward<C>::is> struct ValueHolder {
  
	typedef typename DefaultValue<C>::Type T;
	
	T val;
	
	T& get() 
	{ 
		return val; 
	}
	
	const T& get() const 
	{ 
		return val; 
	}

	ValueHolder(const T& c) :val(c) {}

	void set(const T& c)
	{
		val = c;
	}

	T* ptr() { return &val; }
};

// So what do we do when this IS an awkward declaration?
template<class C> struct ValueHolder<C, 1>
{
	
	typedef typename DefaultValue<C>::Type T;
	
	std::unique_ptr<T> val; // pointer to the object (instead of a standard variable)

	T& get()
	{ 
		return *val; 
	}
	const T& get() const 
	{ 
		return *val; 
	}

	ValueHolder() : val( new T(DefaultValue<C>::val() )) 
	{}

	ValueHolder(const ValueHolder& c) :val( new T(c.get()) )
	{}

	ValueHolder(const T& c) :val(new T(c) )
	{}

	void set(const T& c)
	{
		val = std::unique_ptr<T>(new T(c));
	}

	T* ptr()
	{
		return val.get();
	}
};

}

#endif