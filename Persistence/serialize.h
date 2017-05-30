#ifndef PER_SERIALIZE_H
#define PER_SERIALIZE_H


#include "default.h"

#include <string>
#include <vector>
#include <sstream>
#include <cstdio>
#include <iomanip>
#include <iostream>
#include <ios>

using namespace std;

namespace Persistence
{
  
	
	namespace Serialize
	{
		/// Checks a stream and returns a status code
		int check_stream(std::istream& i);

		
		/// Function which sets a stream into "precise" mode. This will losslessly
		/// save any numbers up to and including double precision.
		inline void generic_setup(std::ostream& o, bool b)
		{
			if(b)
				o << std::setprecision(20) << std::scientific;
		}
			
		//Define a serializer for everything that works with iostreams - this means we can do this with OpenCV vectors
		//override to add new types with unusual serializers
		template<class T> std::string to_string(const T& val, bool precise)
		{
		         // output string stream
			std::ostringstream o;
			// set precision to high/non high
			generic_setup(o,precise);
			//just stream the object as a string (of course, overloading of << is implied... )
			o << val;
			return o.str();
		}

		// This overload takes care of string objects (there's basically special care for spcial characters such as \"...
		std::string to_string(const std::string &s, bool);

		// Basically this struct contains a static function that reads a value frok a stream into a preallocated space (with defaults)
		template<class T> struct FromStream
		{
			static T from(std::istream& i)
			{	
				T result = DefaultValue<T>::val();
				i >> result;
				return result;
			}
		};
		
		//Special reading of strings
		template<> struct FromStream<std::string>
		{
			static std::string from(std::istream& in); // defined in serialize.cpp
		};

		template<class T> struct FromStream<std::vector<T> >
		{
			static std::vector<T> from(std::istream& in)
			{
				std::vector<T> v;
				using std::ws;
				using std::ios;
				v.clear();
				in >> ws;
				int c;


				if((c = in.get()) == EOF) return v;

				bool bracket=1;

				if(c != '[')
				{
					bracket = 0;
					in.unget();
				}
					
				for(;;)
				{
					in >> ws;

					if(in.eof()) return v;
					
					c = in.get();
					
					if(c == EOF || (bracket && c == ']'))  return v;

					in.unget();

					T val =  FromStream<T>::from(in);

					if(!in.fail() && !in.bad())
						v.push_back(val);
					else
						return v;
				}
			}
		};

		// converting to string a std::vector of things...
		template<typename T> std::string to_string( const std::vector<T> & v, bool precise)
		{
			std::ostringstream o;
			generic_setup(o, precise);
			o << "[ ";
			for(unsigned i = 0; i < v.size(); ++i)
				o << to_string(v[i], precise) << " ";
			o << "]";
			return o.str();
		}

		// and the opposite...
		template<class T> struct FromStream<std::vector<std::vector<T> > >
		{
			static std::vector<std::vector<T> > from(std::istream &in)
			{
				std::vector<std::vector<T> > v;
				using std::ws;
				using std::ios;
				v.clear();
				in >> ws;
				int c;

				if((c = in.get()) == EOF) return v;

				if(c != '[')
				{
					in.setstate(std::ios::failbit);
					return v;
				}

				std::vector<T> current;

				for(;;)
				{
					in >> ws;
					
					if((c = in.get()) == EOF || c == ']') 
					{
						if(!current.empty())
							v.push_back(current);
						return v;
					}
					else if(c == ';')
					{
						v.push_back(current);
						current.clear();
					}
					else
						in.unget();

					T val = FromStream<T>::from(in);

					if(!in.fail() && !in.bad())
						current.push_back(val);
					else
						return v;
				}
			}
		};


	        // ******************* Ok, this is what we need. It used to be the TooN stuff. These days its about OpenCV matrices/vectors
	      
		// 1. Vector to string
		template<typename P, int N> std::string to_string(const cv::Vec<P, N> &m, bool precise) {
		
		  std::ostringstream o; // output stream
		  generic_setup(o, precise); // precision
		  o << "[ ";
		  for(int i=0; i<m.rows; i++) o << m[i] << " ";
		  o << "]";
		  return o.str();
	         }

		// 2. Matrix to string. Note here that OpenCV has overlaods for << (comma delinuted fashion)
		// but NOT for <<! So, I am going by Rosten's style ( space between elements).
		// Notice that i am avoiding the dimensions in the template... We make our way as we go...
		template<typename P> std::string to_string(const cv::Mat_<P> &m, bool precise) {
		  
		  std::ostringstream o;
		  generic_setup(o, precise);
		  o << "[ ";
		  for(int i=0; i<m.rows; i++) {
		    if(i != 0) o << "; "; // separating lines with ';'
      
		    for(int j=0; j<m.cols; j++) {		
		      if(j != 0) o << " "; // separating elements in row with space
		      o << m[i][j];
		    }
		  }
		  o << "]";
		  return o.str();
		}
			  
		// 3. Vector from stream
		template<typename P, int N> struct FromStream<cv::Vec<P, N> > {
		  static cv::Vec<P, N> from(std::istream& i) {
			std::vector<P> v = FromStream<std::vector<P> >::from(i);

			if(i.fail() || i.bad() || (N > 1 && (int)v.size() != N) || v.size() == 0) {
			 
			  i.setstate(std::ios::badbit);
			  i.setstate(std::ios::badbit);
			  return DefaultValue<cv::Vec<P, N> >::val(); // 
			}
			else {
			  //return TooN::wrapVector(&v[0], v.size());
			  cv::Vec<P, N> ret;
			  for (unsigned i = 0; i<v.size(); i++) ret[i] = v[i];
			    
			  return ret;
			}
		  }
		};

		// 4. Get a matrix from a stream.
		template<typename P> struct FromStream<cv::Mat_<P> > {
		  
		  static cv::Mat_<P> from(std::istream& i) {
		    using std::vector;
		    vector<vector<double> > v = FromStream<vector<vector<double> > >::from(i);

		    if(i.fail() || i.bad()) {
		      i.setstate(std::ios::failbit);
		      i.setstate(std::ios::badbit);
		      // this is an error,so we are free to choose an invalid dimension (e.g., -1 x 0 )...
		      return DefaultValue<CvMatrixWrapper<P, -1, 0> >::val(); // need a 1D default matrix ...
		    }

		    // taking care of the case in which a row is inconsistent with the others, or it has zero-size. 
		    bool badMatrix = false;
		    for(unsigned int r=1; r < v.size(); r++)
			if ( (v[r].size() != v[0].size() ) || (v[0].size() == 0) )  {
			  badMatrix = true;
			  break;
			}
		    if (badMatrix) {
			i.setstate(std::ios::failbit);
			i.setstate(std::ios::badbit);
			return DefaultValue<CvMatrixWrapper<P, -1, 0> >::val();
		    }
			
		    cv::Mat_<P> retval(v.size(), v[0].size());

		    for(int r=0; r < retval.rows; r++)
			for(int c=0; c < retval.cols; c++)
			    retval(r, c) = v[r][c];

		    return retval;
					
			
				
			      
		  }
		};

		// ******************** Done with OpenCV vectors and matrices! ***************************

		
		
		template<class T> T from_stream(std::istream& i)
		{
			return FromStream<T>::from(i);
		}

		template<class T> int from_string(const std::string& s, T& t)
		{
			std::istringstream is(s);
			t = from_stream<T>(is);
			return check_stream(is);
		}

	}
}


#endif
