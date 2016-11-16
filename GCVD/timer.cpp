// This code is identical (except for a few names) to the one implementing the Timer (cvd_timer) class 
// in libCVD. Credit goes to Ed. Rosten and other libCVD developers.
// I should replace this class in time ...
///////////////////////////////////////////////////////
// 
// A timer class designed for dealing with timestamps
// CK Nov 2002
//
///////////////////////////////////////////////////////

#include "timer.h"

#include <iostream>
#include <chrono>

using namespace  std;
using namespace  std::chrono;

namespace CvUtils {


long long get_time_of_day_ns()
{
	auto time = high_resolution_clock::now();
	
	return time_point_cast<chrono::nanoseconds>(time).time_since_epoch().count();
}



Timer::Timer()
{
	start = high_resolution_clock::now();
}

double Timer::reset() 
{
	auto now = high_resolution_clock::now();
	double r = duration<float>(now - start).count();
	start = now;

	return r;
}

double Timer::get_time() 
{
  auto now = high_resolution_clock::now();
  return duration<float>(now - start).count();
}

double get_time_of_day() 
{
  return get_time_of_day_ns()/1e9;
}


double Timer::conv_ntime(const double & time) const 
{
	double start_seconds = time_point_cast<duration<float>>(start).time_since_epoch().count();
	return time - start_seconds;
}


Timer timer;

}