# - try to find TooN headers
#
# Users may optionally supply:
# TooN_ROOT_DIR - a prefix to start searching for the TooN headers.
#
# Cache Variables: (probably not for direct use in your scripts)
# TooN_INCLUDE_DIR
#
# Non-cache variables you might use in your CMakeLists.txt:
# TooN_FOUND
# TooN_INCLUDE_DIRS
# TooN_LIBRARIES
#
# Requires these CMake modules:
# FindPackageHandleStandardArgs (known included with CMake >=2.6.2)
#
# Original Author:
# 2009-2010 Ryan Pavlik <rpavlik@iastate.edu> <abiryan@ryand.net>
# http://academic.cleardefinition.com
# Iowa State University HCI Graduate Program/VRAC
#
# Copyright Iowa State University 2009-2010.
# Distributed under the Boost Software License, Version 1.0.
# (See accompanying file LICENSE_1_0.txt or copy at
# http://www.boost.org/LICENSE_1_0.txt)
set(TooN_ROOT_DIR
"${TooN_ROOT_DIR}"
CACHE
PATH
"Path to search for TooN")
###
# Dependencies
###
if(NOT LAPACKLIBS_ROOT_DIR)
set(LAPACKLIBS_ROOT_DIR "${TooN_ROOT_DIR}")
endif()
find_package(LAPACKLibs QUIET)
###
# Configure TooN
###
find_path(TooN_INCLUDE_DIR
NAMES
TooN/TooN.h
HINTS
"${TooN_ROOT_DIR}"
PATH_SUFFIXES
include)
mark_as_advanced(TooN_INCLUDE_DIR)
# handle the QUIETLY and REQUIRED arguments and set xxx_FOUND to TRUE if
# all listed variables are TRUE
SET(TooN_FOUND TRUE)
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(TooN
DEFAULT_MSG
TooN_INCLUDE_DIR
LAPACKLIBS_FOUND)
if(TooN_FOUND)
set(TooN_INCLUDE_DIRS "${TooN_INCLUDE_DIR}")
set(TooN_LIBRARIES ${LAPACKLIBS_LIBRARIES})
mark_as_advanced(TooN_ROOT_DIR)
endif()
