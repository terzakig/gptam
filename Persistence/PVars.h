
#ifndef PER_PVARS_H
#define PER_PVARS_H
#include <map>
#include <set>
#include <string>
#include <list>
#include <vector>
#include <iostream>
#include <stdexcept>

#include "default.h"
#include "type_name.h"
#include "serialize.h"

namespace Persistence 
{
  
  void parse_warning(int e, std::string type, std::string name, std::string from);

  struct type_mismatch: public std::logic_error
  {
	  type_mismatch(const std::string &e)	
	  :std::logic_error("Persistence error getting " + e)
	  {
	  }
  };

struct pvar_was_not_defined: public std::runtime_error
{
	pvar_was_not_defined(const std::string& s)
	:std::runtime_error("Persistent variable " + s + " was not defined")
	{
	}
};


class PV3; // corresponds to GV3 in Gvars


class BaseMap
{
	public:
		virtual std::string get_as_string(const std::string& name, bool precise)=0;
		virtual int set_from_string(const std::string& name, const std::string& val)=0;
		virtual std::string name()=0;
		virtual std::vector<std::string> list_tags() = 0;
		virtual ~BaseMap(){};
};

template<class T> class pvar2 // correspondes to gvar2
{
	ValueHolder<T>* data;
	
	friend class PV3;
	
	public:
		pvar2() {data=NULL;}      // Debugging comfort feature, makes unregistered vars more obvious.
		T& operator*()
		{
			return data->get();
		}

		const T & operator*() const 
		{
			return data->get();
		}

		T* operator->()
		{
			return data->ptr();
		}

		const T * operator->() const 
		{
			return data->ptr();
		}

		bool IsRegistered() const
		{
			return data!=NULL;
		}

	
};

// Bit-masks for peristent variable registration:
// SILENT suppresses messages when using defaults;
// HIDDEN makes vars not appear in pvarlist unless used with -a

enum { SILENT = 1<<0, HIDDEN = 1<<1, FATAL_IF_NOT_DEFINED = 1<<2};
       

typedef pvar2<double> pvar_double;
typedef pvar2<int> pvar_int;
typedef pvar2<std::string> pvar_string;

template<class T> class pvar3: public pvar2<T>
{
	friend class PV3;

	public:
	inline pvar3(const std::string &name, const T &default_val = T(), int flags = 0);
	inline pvar3(const std::string &name, const std::string &default_val, int flags);
	inline pvar3(){};
};

template<> class pvar3<std::string>: public pvar2<std::string>
{
	friend class PV3;
	public:
	inline pvar3(const std::string &name, const std::string &default_val = "", int flags = 0);
	inline pvar3(){};
};

class PV3
{
	private:

		template<class C> class TypedMap: public BaseMap
		{
			typedef typename DefaultValue<C>::Type T;
			
			private:
				friend class PV3;

				//This gives us singletons
				static TypedMap& instance()
				{
					static TypedMap* inst=0;

					if(!inst)
					{
						inst = new TypedMap();
						//Register ourselves with GV3
						PV3::add_typemap(inst);
					}

					return *inst;
				}
		
				
				
				//Get a data member	
				ValueHolder<C>* get(const std::string& n)
				{
					DataIter i;

					i = data.find(n);

					if(i == data.end())
						return NULL;
					else
						return &(i->second);
				}
				
				ValueHolder<C>* safe_replace(const std::string &n, const T &t)
				{
					DataIter i, j;
					//Keep track of the neighboring point
					//to pass as a hint to insert.
					i = data.find(n);

					if(i == data.end())
					{
						return &(data.insert(std::make_pair(n, t)).first->second);

					}
					else
					{
						i->second.set(t);
						return &(i->second);
					}

				}

				//Create a data member
				ValueHolder<C>* create(const std::string &n)
				{
					return &(data.insert(std::make_pair(n, DefaultValue<C>::val()))->second);
				}
			
				virtual int set_from_string(const std::string &name, const std::string &val)
				{
					std::istringstream is(val);
					// Recall, that we need to pass to Serialize functions THE TYPE and not the class itself!!!
					T tmp = Serialize::from_stream<T>(is);
					int e = Serialize::check_stream(is);

					if(e == 0) safe_replace(name, tmp);
					
					return e;
				}

				virtual std::string get_as_string(const std::string &name, bool precise)
				{	
					DataIter i = data.find(name);

					if(i == data.end()) i = data.insert(std::make_pair(name, DefaultValue<C>::val())).first;

					return Serialize::to_string(i->second.get(), precise);
				}

				virtual std::string name()
				{
					return type_name<T>();
				}

				virtual std::vector<std::string> list_tags()
				{
					std::vector<std::string> l;
					for(DataIter i=data.begin(); i != data.end(); i++)
						l.push_back(i->first);
					
					return l;
				}

				std::map<std::string, ValueHolder<C> >	data;
				typedef typename std::map<std::string, ValueHolder<C> >::iterator DataIter;

		};

		template<class T> friend class TypedMap;

		template<class T> static ValueHolder<T>* attempt_get(const std::string &name)
		{
			ValueHolder<T>* d = TypedMap<T>::instance().get(name);
			
			if(!d)	 //Data not present in map of the correct type
			{
				//Does it exist with a different type?
				if(registered_type_and_trait.count(name))
				{		//Yes: programmer error.

					std::string err = type_name<T>() + " " + name + ": already registered "
							"as type " + registered_type_and_trait[name].first->name();

					std::cerr << "PV3:Error: type mismatch while getting " << err << ". Fix your code.\n";

					throw type_mismatch(err);
				}
				else
					return NULL;
			}

			return d;
		}

		template<class T> static ValueHolder<T>* safe_replace(const std::string &name, const T& t)
		{
			return TypedMap<T>::instance().safe_replace(name, t);
		}

		static void add_typemap(BaseMap* m);

		static std::map<std::string, std::string>		unmatched_tags;
		static std::map<std::string, std::pair<BaseMap*, int> >	registered_type_and_trait;
		static std::list<BaseMap*>				maps;

		
		template<class T> static ValueHolder<T>* get_by_val(const std::string& name, const T& default_val, int flags) {
	
		  ValueHolder<T>* d = attempt_get<T>(name);
		  if(d)
		    return d;
		  else
		    return  register_new_pvar(name, default_val, flags);
		}
		
		template<class T> static ValueHolder<T>* get_by_str(const std::string &name, const std::string &default_val, int flags) {
	
		  ValueHolder<T>* d = attempt_get<T>(name);
		  if(d) return d;
		  else {
		    std::istringstream is(default_val);
		    T def = Serialize::from_stream<T>(is);
		    int e = Serialize::check_stream(is);
		
		    //Don't bother to warn if FATAL_IF_NOT_DEFINED is defined, since
		    //the bad value will never be used.
		    if(!(flags & FATAL_IF_NOT_DEFINED))
			 parse_warning(e, type_name<T>(), name, default_val);

		  return register_new_pvar(name, def, flags);
		  }
		}
		
		 
		// George: I moved the standard implementation here from the gv3_implementation.hh. More convenient, fewer unnecessary files...
		// Now I need to do the specialization for OpenCV vector and matrix!
		template<class T> static ValueHolder<T>* register_new_pvar(const std::string& name, const T& default_val, int flags) {
	
		  std::map<std::string, std::string>::iterator i;

		  i = unmatched_tags.find(name);

		  ValueHolder<T>* d;

		  registered_type_and_trait[name] = std::pair<BaseMap*, int>(&TypedMap<T>::instance(), flags);

	
		  //Look to see if ``name'' has not already been set	
		  if(i == unmatched_tags.end()) {
		    
		    if(flags & FATAL_IF_NOT_DEFINED) {
			std::cerr << "!!PV3::Register: " << type_name<T>() << " " << name << " must be defined. Exception. " << std::endl;
			throw pvar_was_not_defined(name);
		    };

		  if(!(flags & SILENT))
			std::cerr << "? PV3::Register: " << type_name<T>() << " " << name << " undefined. Defaults to " << Serialize::to_string(default_val,0) << std::endl;

		  d = safe_replace(name, default_val);
		  }
		  else {
		
		    std::istringstream is(i->second);
		    T value = Serialize::from_stream<T>(is);
		    int e = Serialize::check_stream(is);

		    parse_warning(e, type_name<T>(), name, i->second);
		    if(e > 0 && flags & FATAL_IF_NOT_DEFINED) {
			std::cerr << "!!PV3::Register: " << type_name<T>() << " " << name << " must be defined. Exception. " << std::endl;
			throw pvar_was_not_defined(name);
		    }

		    d = safe_replace(name, value);

		  unmatched_tags.erase(i);
		  }

		  return d;
		}

		
		// ************* My specialization for openCV vectors! **********************
		// **************************************************************************
		template<typename T, int Sz> static ValueHolder<cv::Vec<T, Sz> >* register_new_pvar(const std::string& name, const cv::Vec<T, Sz> &default_val, int flags) {
	
		  std::map<std::string, std::string>::iterator i;

		  i = unmatched_tags.find(name);

		  ValueHolder<cv::Vec<T, Sz> >* d; // incredibly, this should be valid (but in the case of matrices we need to use the wrapper...)

		  registered_type_and_trait[name] = std::pair<BaseMap*, int>(&TypedMap<cv::Vec<T, Sz> >::instance(), flags);

	
		  //Look to see if "name" has not already been set	
		  if(i == unmatched_tags.end()) {
		    
		    if(flags & FATAL_IF_NOT_DEFINED) {
			std::cerr << "!!PV3::Register: " << type_name<cv::Vec<T, Sz> >() << " " << name << " must be defined. Exception. " << std::endl;
			throw pvar_was_not_defined(name);
		    };

		  if(!(flags & SILENT))
			std::cerr << "? PV3::Register: " << type_name<cv::Vec<T, Sz> >() << " " << name << " undefined. Defaults to " << Serialize::to_string(default_val,0) << std::endl;

		  d = safe_replace(name, default_val);
		  }
		  else {
		
		    std::istringstream is(i->second);
		    //T value = Serialize::from_stream<T>(is);
		    cv::Vec<T, Sz> value = Serialize::from_stream<cv::Vec<T, Sz> >(is);
		    
		    int e = Serialize::check_stream(is);

		    parse_warning(e, type_name<cv::Vec<T, Sz>>(), name, i->second);
		    if(e > 0 && flags & FATAL_IF_NOT_DEFINED) {
			std::cerr << "!!PV3::Register: " << type_name<cv::Vec<T, Sz> >() << " " << name << " must be defined. Exception. " << std::endl;
			throw pvar_was_not_defined(name);
		    }

		    d = safe_replace(name, value);

		  unmatched_tags.erase(i);
		  }

		  return d;
		}
		
		// ************* My specialization for openCV matrices! **********************
		// **************************************************************************
		template<typename T, int R, int C> static ValueHolder<CvMatrixWrapper<T, R, C> >* register_new_pvar(const std::string& name, const cv::Mat_<T> &default_val, int flags) {
	
		  std::map<std::string, std::string>::iterator i;

		  i = unmatched_tags.find(name);

		  ValueHolder<CvMatrixWrapper<T, R, C> >* d; // I am hoping my CvMatrixWrapper alchemy won't get stuck in some dark corner of the template side of the force....

		  registered_type_and_trait[name] = std::pair<BaseMap*, int>(&TypedMap<CvMatrixWrapper<T, R, C> >::instance(), flags);

	
		  //Look to see if "name" has not already been set	
		  if(i == unmatched_tags.end()) {
		    
		    if(flags & FATAL_IF_NOT_DEFINED) {
			std::cerr << "!!PV3::Register: " << type_name<CvMatrixWrapper<T, R, C> >() << " " << name << " must be defined. Exception. " << std::endl;
			throw pvar_was_not_defined(name);
		    };

		  if(!(flags & SILENT))
			std::cerr << "? PV3::Register: " << type_name<CvMatrixWrapper<T, R, C> >() << " " << name << " undefined. Defaults to " << Serialize::to_string(default_val,0) << std::endl;

		  d = safe_replace(name, default_val);
		  }
		  else {
		
		    std::istringstream is(i->second);
		    //T value = Serialize::from_stream<T>(is);
		    cv::Mat_<T> value = Serialize::from_stream<cv::Mat_<T> >(is);
		    
		    int e = Serialize::check_stream(is);

		    parse_warning(e, type_name<CvMatrixWrapper<T, R, C> >(), name, i->second);
		    if(e > 0 && flags & FATAL_IF_NOT_DEFINED) {
			std::cerr << "!!PV3::Register: " << type_name<CvMatrixWrapper<T, R, C> >() << " " << name << " must be defined. Exception. " << std::endl;
			throw pvar_was_not_defined(name);
		    }

		    d = safe_replace(name, value);

		  unmatched_tags.erase(i);
		  }

		  return d;
		}
	public:
	  // The value of the variable as a string (stole it from GV2...)
		static string StringValue(const string &name, bool no_quotes) {
		  if(no_quotes)
			  return PV3::get_var(name);
		  else
			return "\"" + PV3::get_var(name) + "\"";
		}

		//Get references by name
		template<class T> static T& get(const std::string &name, const T& default_val = Persistence::DefaultValue<T>::val(), int flags = 0) {
	
		  return get_by_val(name, default_val, flags)->get();
		}
		
		template<class T> static T& get(const std::string& name, const std::string &default_val, int flags = 0) {
	
		  return get_by_str<T>(name, default_val, flags)->get();
		}
		// the 3d one was not in the header originally...
		static std::string& get(const std::string& name, const std::string &default_val, int flags = 0) {
	
		  return get_by_val(name, default_val, flags)->get();
		}
		
		
		//Register persistent variables
		template<class T> static void Register(pvar2<T> &pv, const std::string &name, const T &default_val = Persistence::DefaultValue<T>::val(), int flags = 0) {
	
		  pv.data = get_by_val(name, default_val, flags);
		}

		
		
		template<class T> static void Register(pvar2<T> &pv, const std::string &name, const std::string &default_val, int flags = 0) {
	
		  pv.data = get_by_str<T>(name, default_val, flags);
		}

		static inline void Register(pvar2<std::string> &pv, const std::string &name, const std::string &default_val, int flags = 0) {
	
		  pv.data = get_by_val(name, default_val, flags);
		}
		
		//Get and set by string only
		static std::string get_var(std::string name);
		static bool set_var(std::string name, std::string val, bool silent=false);

		//Some helper functions
		static void print_var_list(std::ostream &o, std::string pattern = "", bool show_all = true);
		static std::vector<std::string> tag_list();
		// this here used to be in Gvars. Just porting thge code... 
		//static void printVar(const std::string &varName, std::ostream &os, bool bEndl = true);
		

		static void PrintVar(string s, ostream& os, bool bEndl = true) {
		  os << s << "=" << StringValue(s, true);
		  if(bEndl) os << endl;

		}
		
};



template<class T> pvar3<T>::pvar3(const std::string &name, const T &default_val, int flags)
{
	PV3::Register(*this, name, default_val, flags);
}

template<class T> pvar3<T>::pvar3(const std::string &name, const std::string &default_val, int flags)
{
	PV3::Register(*this, name, default_val, flags);
}
pvar3<std::string>::pvar3(const std::string &name, const std::string &default_val, int flags)
{
	PV3::Register(*this, name, default_val, flags);
}


// I am bringing it here from the original gv3_implementation.hh file
template<class T> inline void robust_set_var(std::string& name, const T& value) {
	    PV3::safe_replace<T>(name,value);
  
}


// **************************** Use the commented-out code to upgrade GVars2 snippets *****************************
/*
class GVars2
{
	public:
		template<class T> void Register(gvar2<T>& gv, const std::string& name, const T& default_val=T(), int flags=false)
		{ 
			GV3::Register(gv, name, default_val, flags);
		}

		template<class T> void Register(gvar2<T>& gv, const std::string& name, const std::string& default_val, int flags=false)
		{
			GV3::Register(gv, name, default_val, flags);
		}

		inline void Register(gvar2<std::string>& gv, const std::string& name, const std::string& default_val="", int flags=false)
		{
			GV3::Register(gv, name, default_val, flags);
		}

		template<class T> T& Get(const std::string& name, const T& default_val=T(),  int flags=false)
		{
			return GV3::get<T>(name, default_val, flags);
		}

		template<class T> T& Get(const std::string& name, const std::string& default_val="", int flags=false)
		{
			return GV3::get<T>(name, default_val, flags);
		}

		inline std::string& Get(const std::string& name, const std::string& default_val="", int flags=false)
		{
			return GV3::get<std::string>(name, default_val, flags);
		}

		void SetVar(std::string sVar, std::string sValue, bool s)
		{
		  GV3::set_var(sVar, sValue, s);
		}


		void SetVar(std::string s)
		{
		  // Expected format: "foo = bar"
		  // So crack the string in two at the equals sign. Easy.
		  // SetVar(string,string) is quite tolerant to whitespace.
	  
		  string::size_type n;
		  n=s.find("=");
		  if(n>=s.find("//"))  // Perhaps the line is a comment?
		      return;
		  if(n==s.npos)
		  {
		    cout << "? Gvars::SetVar(string): No equals sign found in \""<<s<<"\""<< endl;
		    return;
		  };
		  SetVar(s.substr(0,n),s.substr(n+1,s.npos-n));
		}	


		int& GetInt(const string& name, int default_val, int flags)
		{
		return Get<int>(name, default_val, flags);
		}
		
		
		double& GetDouble(const string& name, const string& default_val, int flags)
		{
		  return Get<double>(name, default_val, flags);
		  }

		string StringValue(const string &name, bool no_quotes)
		{
		  if(no_quotes)
			  return GV3::get_var(name);
		  else
			return "\"" + GV3::get_var(name) + "\"";
		}

		void PrintVarList(ostream& os)
		{
		  GV3::print_var_list(os);
		}

		void PrintVar(string s, ostream& os, bool bEndl)
		{
		  os << s << "=" << StringValue(s, true);
		  if(bEndl)
			  os << endl;

		 }
		
		
		int& GetInt(const string& name, int default_val, int flags)
		{
		  return Get<int>(name, default_val, flags);
		}

		double& GetDouble(const string& name, double default_val, int flags)
		{
		  return Get<double>(name, default_val, flags);
		}
		string& GetString(const string& name, const string& default_val, int flags)
		{
		  return Get<string>(name, default_val, flags);
		}


		
		
		//char* ReadlineCommandGenerator(const char *szText, int nState);

	private:
};

*/

}
#endif