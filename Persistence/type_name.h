/*                       
	This file is part of the GVars3 Library.
	Copyright (C) 2005 The Authors
	This library is free software; you can redistribute it and/or
	modify it under the terms of the GNU Lesser General Public
	License as published by the Free Software Foundation; either
	version 2.1 of the License, or (at your option) any later version.
	This library is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
	Lesser General Public License for more details.
	You should have received a copy of the GNU Lesser General Public
	License along with this library; if not, write to the Free Software
	Foundation, Inc., 
    51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
*/

#ifndef PER_TYPE_NAME_H
#define PER_TYPE_NAME_H

#include <string>


#include "default.h"

namespace Persistence
{

	template <class T> std::string type_name() 
	{
#ifdef _MSC_VER
      static std::string funcname = std::string(__FUNCSIG__);
      static std::string::size_type begin = funcname.find("type_name<")+10;
      static std::string name = funcname.substr(begin, funcname.rfind(">") - begin);
#elif defined __GNUC__
	//Output format of __PRETTY_FUNCTION__ changed at 4.5.0 to have the return 
	//type included.
	#if __GNUC__ * 100 + __GNUC_MINOR__ * 10 + __GNUC_PATCHLEVEL__ < 450
      std::string funcname = std::string(__PRETTY_FUNCTION__);
      std::string bname = funcname.substr(funcname.rfind(" = ")+3);
	  std::string name = bname.substr(0, bname.length()-1);
	#else
      std::string funcname = std::string(__PRETTY_FUNCTION__);
      std::string bname = funcname.substr(funcname.rfind("T = ")+3);
	  std::string name = bname.substr(0, bname.find(", "));
	#endif
#else
	#error no good way of getting a type name
#endif
	  return name;
	}
	
	template<> inline std::string type_name<std::string>()
	{
		return "string";
	}

	
	
	
	template <class T> std::string type_name(const T&)
	{
	  return type_name<T>(); // Maybe this thing works for the OpenCV vector and Matrix
	}
	
	// Just adding these two overloads and specializatioon to rturn OpenCV type names directly...
	/*template<typename P, int Sz> std::string type_name<cv::Vec<P, Sz> >(const cv::Vec<P, Sz> &v) {
	  
	  return "OpenCV vector";
	}*/
	
	
	
	


}



#endif