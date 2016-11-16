#include "PVars.h"
#include <vector>
#include <algorithm>

#ifndef WIN32
#include <fnmatch.h>
#else 
// FIXME: empty dummy implementation for now
int fnmatch(const char *, const char *, int ){
    return -1;
}
#define FNM_CASEFOLD 0
#endif

using namespace std;

namespace Persistence
{
  
	 


	 std::map<std::string, std::string>		PV3::unmatched_tags;
         std::map<std::string, std::pair<BaseMap*,int> >	PV3::registered_type_and_trait;
	 std::list<BaseMap*>					PV3::maps;


	void PV3::add_typemap(BaseMap* m)
	{
		maps.push_back(m);
	}


	string PV3::get_var(string name)
	{
		if(registered_type_and_trait.count(name))
			return registered_type_and_trait[name].first->get_as_string(name, 0);
		else if(unmatched_tags.count(name))
			return unmatched_tags[name];
		else
			return "(Not present in GVar list.)";
	}

	bool PV3::set_var(string name, string val, bool silent)
	{
		if(registered_type_and_trait.count(name))
		{
			int e = registered_type_and_trait[name].first->set_from_string(name, val);
			if(!silent)
				parse_warning(e, registered_type_and_trait[name].first->name(), name, val);
			return e==0;
		}
		else
		{
			unmatched_tags[name]=val;
			return true;
		}
	}

        void PV3::print_var_list(ostream &o, string pattern, bool show_all)
	{
	        bool no_pattern = (pattern=="");

	        if(show_all) o << "//Registered persistent variables:" << endl;
		
		for(map<string, std::pair<BaseMap*,int> >::iterator i=registered_type_and_trait.begin(); i != registered_type_and_trait.end(); i++)
		  if(show_all || !(i->second.second & HIDDEN))
		    if(no_pattern || !fnmatch(pattern.c_str(), i->first.c_str(), FNM_CASEFOLD))
		      o << i->first << "=" << i->second.first->get_as_string(i->first, 1) << endl;

		if(show_all)
		  {
		    o << "//Unmatched tags:" << endl;
		    
		    for(map<string,string>::iterator i=unmatched_tags.begin(); i != unmatched_tags.end(); i++)
		      if(no_pattern || !fnmatch(pattern.c_str(), i->first.c_str(), FNM_CASEFOLD))
			o << i->first << "=" << i->second << endl;
		    
		    o << "// End of list of persistent variables." << endl;
		  };

	}

	
	
	/*void PV3::printVar(std::string varName, std::stream &os, bool bEndl) {
	  os << varName << "=" << StringValue(varName, true);
		if(bEndl)
			os << endl;
	}*/
	
	vector<string> PV3::tag_list()
	{
		vector<string> v;
		for(map<string, std::pair<BaseMap*, int> >::iterator i=registered_type_and_trait.begin(); i != registered_type_and_trait.end(); i++)
			v.push_back(i->first);

		return v;
	}

	void parse_warning(int e, string type, string name, string from)
	{
	if(e > 0)
		std::cerr << "! PV3:Parse error setting " << type << " " << name << " from " << from << std::endl;
	else if (e < 0)
		std::cerr << "! PV3:Parse warning setting " << type << " " << name << " from " << from << ": "
				  << "junk is -->" << from.c_str()-e  << "<--" << std::endl;
	}
};