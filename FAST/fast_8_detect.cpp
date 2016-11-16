#include <vector>
//#include <cvd/image.h>
//#include <cvd/byte.h>

#include <cxcore.hpp>
#include <cv.hpp>
#include <highgui.hpp>

// This is mechanically generated code. 

using namespace std;
namespace FAST
{
//void fast_corner_detect_plain_8(const BasicImage<byte>& i, vector<ImageRef>& corners, int b)
void fast_corner_detect_plain_8(const cv::Mat_<uchar>& i, vector<cv::Point2i>& corners, int b)
  {
	int y, cb, c_b;
	const uchar *line_max, *line_min;
	const uchar* cache_0;

	int pixel[16] = {
		0 + i.cols * 3,
		1 + i.cols * 3,
		2 + i.cols * 2,
		3 + i.cols * 1,
		3 + i.cols * 0,
		3 + i.cols * -1,
		2 + i.cols * -2,
		1 + i.cols * -3,
		0 + i.cols * -3,
		-1 + i.cols * -3,
		-2 + i.cols * -2,
		-3 + i.cols * -1,
		-3 + i.cols * 0,
		-3 + i.cols * 1,
		-2 + i.cols * 2,
		-1 + i.cols * 3,
	};

	for(y = 3 ; y < i.rows - 3; y++)
	{
		//cache_0 = &i[y][3];
		cache_0 = i.ptr<uchar>(y, 3);
		line_min = cache_0 - 3;
		//line_max = &i[y][i.size().x - 3];
		line_max = i.ptr<uchar>(y, i.cols - 3);


		for(; cache_0 < line_max;cache_0++)
		{
			cb = *cache_0 + b;
			c_b= *cache_0 - b;
  
			if(*(cache_0 + pixel[0]) > cb)
			 if(*(cache_0 + pixel[8]) > cb)
			  if(*(cache_0 + pixel[3]) > cb)
			   if(*(cache_0 + pixel[5]) > cb)
			    if(*(cache_0 + pixel[6]) > cb)
			     if(*(cache_0 + pixel[2]) > cb)
			      if(*(cache_0 + 3) > cb)
			       if(*(cache_0 + pixel[7]) > cb)
			        if(*(cache_0 + pixel[1]) > cb)
			         goto success;
			        else
			         if(*(cache_0 + pixel[9]) > cb)
			          goto success;
			         else
			          continue;
			       else if(*(cache_0 + pixel[7]) < c_b)
			        if(*(cache_0 + pixel[15]) > cb)
			         if(*(cache_0 + pixel[1]) > cb)
			          goto success;
			         else
			          if(*(cache_0 + pixel[9]) > cb)
			           if(*(cache_0 + pixel[10]) > cb)
			            if(*(cache_0 + pixel[11]) > cb)
			             if(*(cache_0 + -3) > cb)
			              if(*(cache_0 + pixel[13]) > cb)
			               if(*(cache_0 + pixel[14]) > cb)
			                goto success;
			               else
			                continue;
			              else
			               continue;
			             else
			              continue;
			            else
			             continue;
			           else
			            continue;
			          else
			           continue;
			        else
			         continue;
			       else
			        if(*(cache_0 + pixel[15]) > cb)
			         if(*(cache_0 + pixel[1]) > cb)
			          goto success;
			         else if(*(cache_0 + pixel[1]) < c_b)
			          if(*(cache_0 + pixel[9]) > cb)
			           if(*(cache_0 + pixel[10]) > cb)
			            if(*(cache_0 + pixel[11]) > cb)
			             if(*(cache_0 + -3) > cb)
			              if(*(cache_0 + pixel[13]) > cb)
			               if(*(cache_0 + pixel[14]) > cb)
			                goto success;
			               else
			                continue;
			              else
			               continue;
			             else
			              continue;
			            else
			             continue;
			           else
			            continue;
			          else
			           continue;
			         else
			          if(*(cache_0 + pixel[11]) > cb)
			           if(*(cache_0 + pixel[14]) > cb)
			            if(*(cache_0 + pixel[9]) > cb)
			             if(*(cache_0 + -3) > cb)
			              if(*(cache_0 + pixel[13]) > cb)
			               if(*(cache_0 + pixel[10]) > cb)
			                goto success;
			               else
			                continue;
			              else
			               continue;
			             else
			              continue;
			            else
			             continue;
			           else
			            continue;
			          else
			           continue;
			        else
			         continue;
			      else if(*(cache_0 + 3) < c_b)
			       if(*(cache_0 + -3) > cb)
			        if(*(cache_0 + pixel[9]) > cb)
			         if(*(cache_0 + pixel[11]) > cb)
			          if(*(cache_0 + pixel[10]) > cb)
			           if(*(cache_0 + pixel[7]) > cb)
			            goto success;
			           else
			            if(*(cache_0 + pixel[13]) > cb)
			             if(*(cache_0 + pixel[14]) > cb)
			              if(*(cache_0 + pixel[15]) > cb)
			               goto success;
			              else
			               continue;
			             else
			              continue;
			            else
			             continue;
			          else
			           if(*(cache_0 + pixel[1]) > cb)
			            if(*(cache_0 + pixel[13]) > cb)
			             if(*(cache_0 + pixel[14]) > cb)
			              if(*(cache_0 + pixel[15]) > cb)
			               goto success;
			              else
			               continue;
			             else
			              continue;
			            else
			             continue;
			           else
			            continue;
			         else
			          if(*(cache_0 + pixel[1]) > cb)
			           if(*(cache_0 + pixel[13]) > cb)
			            if(*(cache_0 + pixel[14]) > cb)
			             if(*(cache_0 + pixel[15]) > cb)
			              goto success;
			             else
			              continue;
			            else
			             continue;
			           else
			            continue;
			          else
			           continue;
			        else
			         if(*(cache_0 + pixel[1]) > cb)
			          if(*(cache_0 + pixel[13]) > cb)
			           if(*(cache_0 + pixel[14]) > cb)
			            if(*(cache_0 + pixel[15]) > cb)
			             goto success;
			            else
			             continue;
			           else
			            continue;
			          else
			           continue;
			         else
			          continue;
			       else
			        continue;
			      else
			       if(*(cache_0 + -3) > cb)
			        if(*(cache_0 + pixel[1]) > cb)
			         if(*(cache_0 + pixel[14]) > cb)
			          if(*(cache_0 + pixel[13]) > cb)
			           if(*(cache_0 + pixel[15]) > cb)
			            goto success;
			           else if(*(cache_0 + pixel[15]) < c_b)
			            if(*(cache_0 + pixel[7]) > cb)
			             if(*(cache_0 + pixel[9]) > cb)
			              if(*(cache_0 + pixel[10]) > cb)
			               if(*(cache_0 + pixel[11]) > cb)
			                goto success;
			               else
			                continue;
			              else
			               continue;
			             else
			              continue;
			            else
			             continue;
			           else
			            if(*(cache_0 + pixel[10]) > cb)
			             if(*(cache_0 + pixel[11]) > cb)
			              if(*(cache_0 + pixel[7]) > cb)
			               if(*(cache_0 + pixel[9]) > cb)
			                goto success;
			               else
			                continue;
			              else
			               continue;
			             else
			              continue;
			            else
			             continue;
			          else if(*(cache_0 + pixel[13]) < c_b)
			           if(*(cache_0 + pixel[7]) > cb)
			            if(*(cache_0 + pixel[9]) > cb)
			             if(*(cache_0 + pixel[10]) > cb)
			              if(*(cache_0 + pixel[11]) > cb)
			               goto success;
			              else
			               continue;
			             else
			              continue;
			            else
			             continue;
			           else
			            continue;
			          else
			           if(*(cache_0 + pixel[11]) > cb)
			            if(*(cache_0 + pixel[9]) > cb)
			             if(*(cache_0 + pixel[7]) > cb)
			              if(*(cache_0 + pixel[10]) > cb)
			               goto success;
			              else
			               continue;
			             else
			              continue;
			            else
			             continue;
			           else
			            continue;
			         else if(*(cache_0 + pixel[14]) < c_b)
			          if(*(cache_0 + pixel[7]) > cb)
			           if(*(cache_0 + pixel[9]) > cb)
			            if(*(cache_0 + pixel[10]) > cb)
			             if(*(cache_0 + pixel[11]) > cb)
			              goto success;
			             else
			              continue;
			            else
			             continue;
			           else
			            continue;
			          else
			           continue;
			         else
			          if(*(cache_0 + pixel[9]) > cb)
			           if(*(cache_0 + pixel[7]) > cb)
			            if(*(cache_0 + pixel[11]) > cb)
			             if(*(cache_0 + pixel[10]) > cb)
			              goto success;
			             else
			              continue;
			            else
			             continue;
			           else
			            continue;
			          else
			           continue;
			        else if(*(cache_0 + pixel[1]) < c_b)
			         if(*(cache_0 + pixel[9]) > cb)
			          if(*(cache_0 + pixel[10]) > cb)
			           if(*(cache_0 + pixel[11]) > cb)
			            if(*(cache_0 + pixel[7]) > cb)
			             goto success;
			            else
			             if(*(cache_0 + pixel[13]) > cb)
			              if(*(cache_0 + pixel[14]) > cb)
			               if(*(cache_0 + pixel[15]) > cb)
			                goto success;
			               else
			                continue;
			              else
			               continue;
			             else
			              continue;
			           else
			            continue;
			          else
			           continue;
			         else
			          continue;
			        else
			         if(*(cache_0 + pixel[10]) > cb)
			          if(*(cache_0 + pixel[11]) > cb)
			           if(*(cache_0 + pixel[7]) > cb)
			            if(*(cache_0 + pixel[9]) > cb)
			             goto success;
			            else
			             continue;
			           else if(*(cache_0 + pixel[7]) < c_b)
			            if(*(cache_0 + pixel[9]) > cb)
			             if(*(cache_0 + pixel[13]) > cb)
			              if(*(cache_0 + pixel[14]) > cb)
			               if(*(cache_0 + pixel[15]) > cb)
			                goto success;
			               else
			                continue;
			              else
			               continue;
			             else
			              continue;
			            else
			             continue;
			           else
			            if(*(cache_0 + pixel[14]) > cb)
			             if(*(cache_0 + pixel[9]) > cb)
			              if(*(cache_0 + pixel[13]) > cb)
			               if(*(cache_0 + pixel[15]) > cb)
			                goto success;
			               else
			                continue;
			              else
			               continue;
			             else
			              continue;
			            else
			             continue;
			          else
			           continue;
			         else
			          continue;
			       else
			        continue;
			     else if(*(cache_0 + pixel[2]) < c_b)
			      if(*(cache_0 + pixel[10]) > cb)
			       if(*(cache_0 + pixel[9]) > cb)
			        if(*(cache_0 + pixel[7]) > cb)
			         if(*(cache_0 + 3) > cb)
			          goto success;
			         else
			          if(*(cache_0 + pixel[11]) > cb)
			           if(*(cache_0 + -3) > cb)
			            goto success;
			           else
			            continue;
			          else
			           continue;
			        else
			         if(*(cache_0 + pixel[11]) > cb)
			          if(*(cache_0 + -3) > cb)
			           if(*(cache_0 + pixel[13]) > cb)
			            if(*(cache_0 + pixel[14]) > cb)
			             if(*(cache_0 + pixel[15]) > cb)
			              goto success;
			             else
			              continue;
			            else
			             continue;
			           else
			            continue;
			          else
			           continue;
			         else
			          continue;
			       else
			        if(*(cache_0 + pixel[1]) > cb)
			         if(*(cache_0 + pixel[11]) > cb)
			          if(*(cache_0 + -3) > cb)
			           if(*(cache_0 + pixel[13]) > cb)
			            if(*(cache_0 + pixel[14]) > cb)
			             if(*(cache_0 + pixel[15]) > cb)
			              goto success;
			             else
			              continue;
			            else
			             continue;
			           else
			            continue;
			          else
			           continue;
			         else
			          continue;
			        else
			         continue;
			      else
			       continue;
			     else
			      if(*(cache_0 + pixel[10]) > cb)
			       if(*(cache_0 + pixel[7]) > cb)
			        if(*(cache_0 + pixel[9]) > cb)
			         if(*(cache_0 + 3) > cb)
			          goto success;
			         else if(*(cache_0 + 3) < c_b)
			          if(*(cache_0 + pixel[11]) > cb)
			           if(*(cache_0 + -3) > cb)
			            goto success;
			           else
			            continue;
			          else
			           continue;
			         else
			          if(*(cache_0 + -3) > cb)
			           if(*(cache_0 + pixel[11]) > cb)
			            goto success;
			           else
			            continue;
			          else
			           continue;
			        else if(*(cache_0 + pixel[9]) < c_b)
			         if(*(cache_0 + pixel[1]) > cb)
			          if(*(cache_0 + pixel[11]) > cb)
			           if(*(cache_0 + -3) > cb)
			            if(*(cache_0 + pixel[13]) > cb)
			             if(*(cache_0 + pixel[14]) > cb)
			              if(*(cache_0 + pixel[15]) > cb)
			               goto success;
			              else
			               continue;
			             else
			              continue;
			            else
			             continue;
			           else
			            continue;
			          else
			           continue;
			         else
			          continue;
			        else
			         if(*(cache_0 + pixel[1]) > cb)
			          if(*(cache_0 + pixel[14]) > cb)
			           if(*(cache_0 + pixel[11]) > cb)
			            if(*(cache_0 + -3) > cb)
			             if(*(cache_0 + pixel[13]) > cb)
			              if(*(cache_0 + pixel[15]) > cb)
			               goto success;
			              else
			               continue;
			             else
			              continue;
			            else
			             continue;
			           else
			            continue;
			          else
			           continue;
			         else
			          continue;
			       else if(*(cache_0 + pixel[7]) < c_b)
			        if(*(cache_0 + pixel[11]) > cb)
			         if(*(cache_0 + -3) > cb)
			          if(*(cache_0 + pixel[13]) > cb)
			           if(*(cache_0 + pixel[14]) > cb)
			            if(*(cache_0 + pixel[15]) > cb)
			             if(*(cache_0 + pixel[9]) > cb)
			              goto success;
			             else
			              if(*(cache_0 + pixel[1]) > cb)
			               goto success;
			              else
			               continue;
			            else
			             continue;
			           else
			            continue;
			          else
			           continue;
			         else
			          continue;
			        else
			         continue;
			       else
			        if(*(cache_0 + pixel[14]) > cb)
			         if(*(cache_0 + pixel[11]) > cb)
			          if(*(cache_0 + -3) > cb)
			           if(*(cache_0 + pixel[13]) > cb)
			            if(*(cache_0 + pixel[15]) > cb)
			             if(*(cache_0 + pixel[9]) > cb)
			              goto success;
			             else
			              if(*(cache_0 + pixel[1]) > cb)
			               goto success;
			              else
			               continue;
			            else
			             continue;
			           else
			            continue;
			          else
			           continue;
			         else
			          continue;
			        else
			         continue;
			      else
			       continue;
			    else if(*(cache_0 + pixel[6]) < c_b)
			     if(*(cache_0 + pixel[14]) > cb)
			      if(*(cache_0 + pixel[15]) > cb)
			       if(*(cache_0 + pixel[1]) > cb)
			        if(*(cache_0 + pixel[2]) > cb)
			         if(*(cache_0 + 3) > cb)
			          goto success;
			         else
			          if(*(cache_0 + -3) > cb)
			           if(*(cache_0 + pixel[13]) > cb)
			            goto success;
			           else
			            continue;
			          else
			           continue;
			        else
			         if(*(cache_0 + pixel[10]) > cb)
			          if(*(cache_0 + pixel[11]) > cb)
			           if(*(cache_0 + -3) > cb)
			            if(*(cache_0 + pixel[13]) > cb)
			             goto success;
			            else
			             continue;
			           else
			            continue;
			          else
			           continue;
			         else
			          continue;
			       else
			        if(*(cache_0 + pixel[9]) > cb)
			         if(*(cache_0 + pixel[10]) > cb)
			          if(*(cache_0 + pixel[11]) > cb)
			           if(*(cache_0 + -3) > cb)
			            if(*(cache_0 + pixel[13]) > cb)
			             goto success;
			            else
			             continue;
			           else
			            continue;
			          else
			           continue;
			         else
			          continue;
			        else
			         continue;
			      else
			       if(*(cache_0 + pixel[7]) > cb)
			        if(*(cache_0 + pixel[9]) > cb)
			         if(*(cache_0 + pixel[10]) > cb)
			          if(*(cache_0 + pixel[11]) > cb)
			           if(*(cache_0 + -3) > cb)
			            if(*(cache_0 + pixel[13]) > cb)
			             goto success;
			            else
			             continue;
			           else
			            continue;
			          else
			           continue;
			         else
			          continue;
			        else
			         continue;
			       else
			        continue;
			     else
			      continue;
			    else
			     if(*(cache_0 + pixel[14]) > cb)
			      if(*(cache_0 + pixel[2]) > cb)
			       if(*(cache_0 + pixel[15]) > cb)
			        if(*(cache_0 + pixel[1]) > cb)
			         if(*(cache_0 + 3) > cb)
			          goto success;
			         else
			          if(*(cache_0 + -3) > cb)
			           if(*(cache_0 + pixel[13]) > cb)
			            goto success;
			           else
			            continue;
			          else
			           continue;
			        else if(*(cache_0 + pixel[1]) < c_b)
			         if(*(cache_0 + pixel[9]) > cb)
			          if(*(cache_0 + pixel[10]) > cb)
			           if(*(cache_0 + pixel[11]) > cb)
			            if(*(cache_0 + -3) > cb)
			             if(*(cache_0 + pixel[13]) > cb)
			              goto success;
			             else
			              continue;
			            else
			             continue;
			           else
			            continue;
			          else
			           continue;
			         else
			          continue;
			        else
			         if(*(cache_0 + -3) > cb)
			          if(*(cache_0 + pixel[11]) > cb)
			           if(*(cache_0 + pixel[10]) > cb)
			            if(*(cache_0 + pixel[9]) > cb)
			             if(*(cache_0 + pixel[13]) > cb)
			              goto success;
			             else
			              continue;
			            else
			             continue;
			           else
			            continue;
			          else
			           continue;
			         else
			          continue;
			       else if(*(cache_0 + pixel[15]) < c_b)
			        if(*(cache_0 + pixel[7]) > cb)
			         if(*(cache_0 + pixel[9]) > cb)
			          if(*(cache_0 + pixel[10]) > cb)
			           if(*(cache_0 + pixel[11]) > cb)
			            if(*(cache_0 + -3) > cb)
			             if(*(cache_0 + pixel[13]) > cb)
			              goto success;
			             else
			              continue;
			            else
			             continue;
			           else
			            continue;
			          else
			           continue;
			         else
			          continue;
			        else
			         continue;
			       else
			        if(*(cache_0 + pixel[11]) > cb)
			         if(*(cache_0 + pixel[13]) > cb)
			          if(*(cache_0 + pixel[7]) > cb)
			           if(*(cache_0 + -3) > cb)
			            if(*(cache_0 + pixel[10]) > cb)
			             if(*(cache_0 + pixel[9]) > cb)
			              goto success;
			             else
			              continue;
			            else
			             continue;
			           else
			            continue;
			          else
			           continue;
			         else
			          continue;
			        else
			         continue;
			      else if(*(cache_0 + pixel[2]) < c_b)
			       if(*(cache_0 + pixel[10]) > cb)
			        if(*(cache_0 + pixel[11]) > cb)
			         if(*(cache_0 + -3) > cb)
			          if(*(cache_0 + pixel[13]) > cb)
			           if(*(cache_0 + pixel[9]) > cb)
			            if(*(cache_0 + pixel[7]) > cb)
			             goto success;
			            else
			             if(*(cache_0 + pixel[15]) > cb)
			              goto success;
			             else
			              continue;
			           else
			            if(*(cache_0 + pixel[1]) > cb)
			             if(*(cache_0 + pixel[15]) > cb)
			              goto success;
			             else
			              continue;
			            else
			             continue;
			          else
			           continue;
			         else
			          continue;
			        else
			         continue;
			       else
			        continue;
			      else
			       if(*(cache_0 + pixel[11]) > cb)
			        if(*(cache_0 + pixel[10]) > cb)
			         if(*(cache_0 + -3) > cb)
			          if(*(cache_0 + pixel[13]) > cb)
			           if(*(cache_0 + pixel[9]) > cb)
			            if(*(cache_0 + pixel[15]) > cb)
			             goto success;
			            else
			             if(*(cache_0 + pixel[7]) > cb)
			              goto success;
			             else
			              continue;
			           else
			            if(*(cache_0 + pixel[1]) > cb)
			             if(*(cache_0 + pixel[15]) > cb)
			              goto success;
			             else
			              continue;
			            else
			             continue;
			          else
			           continue;
			         else
			          continue;
			        else
			         continue;
			       else
			        continue;
			     else
			      continue;
			   else if(*(cache_0 + pixel[5]) < c_b)
			    if(*(cache_0 + pixel[13]) > cb)
			     if(*(cache_0 + pixel[14]) > cb)
			      if(*(cache_0 + pixel[15]) > cb)
			       if(*(cache_0 + pixel[1]) > cb)
			        if(*(cache_0 + pixel[2]) > cb)
			         if(*(cache_0 + -3) > cb)
			          goto success;
			         else
			          if(*(cache_0 + 3) > cb)
			           goto success;
			          else
			           continue;
			        else
			         if(*(cache_0 + pixel[10]) > cb)
			          if(*(cache_0 + pixel[11]) > cb)
			           if(*(cache_0 + -3) > cb)
			            goto success;
			           else
			            continue;
			          else
			           continue;
			         else
			          continue;
			       else if(*(cache_0 + pixel[1]) < c_b)
			        if(*(cache_0 + pixel[9]) > cb)
			         if(*(cache_0 + pixel[10]) > cb)
			          if(*(cache_0 + pixel[11]) > cb)
			           if(*(cache_0 + -3) > cb)
			            goto success;
			           else
			            continue;
			          else
			           continue;
			         else
			          continue;
			        else
			         continue;
			       else
			        if(*(cache_0 + pixel[10]) > cb)
			         if(*(cache_0 + pixel[9]) > cb)
			          if(*(cache_0 + pixel[11]) > cb)
			           if(*(cache_0 + -3) > cb)
			            goto success;
			           else
			            continue;
			          else
			           continue;
			         else
			          continue;
			        else
			         continue;
			      else
			       if(*(cache_0 + pixel[7]) > cb)
			        if(*(cache_0 + pixel[9]) > cb)
			         if(*(cache_0 + pixel[10]) > cb)
			          if(*(cache_0 + pixel[11]) > cb)
			           if(*(cache_0 + -3) > cb)
			            goto success;
			           else
			            continue;
			          else
			           continue;
			         else
			          continue;
			        else
			         continue;
			       else
			        continue;
			     else
			      if(*(cache_0 + pixel[6]) > cb)
			       if(*(cache_0 + pixel[7]) > cb)
			        if(*(cache_0 + pixel[9]) > cb)
			         if(*(cache_0 + pixel[10]) > cb)
			          if(*(cache_0 + pixel[11]) > cb)
			           if(*(cache_0 + -3) > cb)
			            goto success;
			           else
			            continue;
			          else
			           continue;
			         else
			          continue;
			        else
			         continue;
			       else
			        continue;
			      else
			       continue;
			    else
			     continue;
			   else
			    if(*(cache_0 + pixel[13]) > cb)
			     if(*(cache_0 + -3) > cb)
			      if(*(cache_0 + pixel[14]) > cb)
			       if(*(cache_0 + pixel[2]) > cb)
			        if(*(cache_0 + pixel[15]) > cb)
			         if(*(cache_0 + pixel[1]) > cb)
			          goto success;
			         else if(*(cache_0 + pixel[1]) < c_b)
			          if(*(cache_0 + pixel[9]) > cb)
			           if(*(cache_0 + pixel[10]) > cb)
			            if(*(cache_0 + pixel[11]) > cb)
			             goto success;
			            else
			             continue;
			           else
			            continue;
			          else
			           continue;
			         else
			          if(*(cache_0 + pixel[10]) > cb)
			           if(*(cache_0 + pixel[9]) > cb)
			            if(*(cache_0 + pixel[11]) > cb)
			             goto success;
			            else
			             continue;
			           else
			            continue;
			          else
			           continue;
			        else if(*(cache_0 + pixel[15]) < c_b)
			         if(*(cache_0 + pixel[7]) > cb)
			          if(*(cache_0 + pixel[9]) > cb)
			           if(*(cache_0 + pixel[10]) > cb)
			            if(*(cache_0 + pixel[11]) > cb)
			             goto success;
			            else
			             continue;
			           else
			            continue;
			          else
			           continue;
			         else
			          continue;
			        else
			         if(*(cache_0 + pixel[7]) > cb)
			          if(*(cache_0 + pixel[11]) > cb)
			           if(*(cache_0 + pixel[9]) > cb)
			            if(*(cache_0 + pixel[10]) > cb)
			             goto success;
			            else
			             continue;
			           else
			            continue;
			          else
			           continue;
			         else
			          continue;
			       else if(*(cache_0 + pixel[2]) < c_b)
			        if(*(cache_0 + pixel[10]) > cb)
			         if(*(cache_0 + pixel[11]) > cb)
			          if(*(cache_0 + pixel[15]) > cb)
			           if(*(cache_0 + pixel[1]) > cb)
			            goto success;
			           else
			            if(*(cache_0 + pixel[9]) > cb)
			             goto success;
			            else
			             continue;
			          else
			           if(*(cache_0 + pixel[7]) > cb)
			            if(*(cache_0 + pixel[9]) > cb)
			             goto success;
			            else
			             continue;
			           else
			            continue;
			         else
			          continue;
			        else
			         continue;
			       else
			        if(*(cache_0 + pixel[10]) > cb)
			         if(*(cache_0 + pixel[11]) > cb)
			          if(*(cache_0 + pixel[9]) > cb)
			           if(*(cache_0 + pixel[15]) > cb)
			            goto success;
			           else
			            if(*(cache_0 + pixel[7]) > cb)
			             goto success;
			            else
			             continue;
			          else
			           if(*(cache_0 + pixel[1]) > cb)
			            if(*(cache_0 + pixel[15]) > cb)
			             goto success;
			            else
			             continue;
			           else
			            continue;
			         else
			          continue;
			        else
			         continue;
			      else if(*(cache_0 + pixel[14]) < c_b)
			       if(*(cache_0 + pixel[7]) > cb)
			        if(*(cache_0 + pixel[6]) > cb)
			         if(*(cache_0 + pixel[9]) > cb)
			          if(*(cache_0 + pixel[10]) > cb)
			           if(*(cache_0 + pixel[11]) > cb)
			            goto success;
			           else
			            continue;
			          else
			           continue;
			         else
			          continue;
			        else
			         continue;
			       else
			        continue;
			      else
			       if(*(cache_0 + pixel[6]) > cb)
			        if(*(cache_0 + pixel[10]) > cb)
			         if(*(cache_0 + pixel[7]) > cb)
			          if(*(cache_0 + pixel[11]) > cb)
			           if(*(cache_0 + pixel[9]) > cb)
			            goto success;
			           else
			            continue;
			          else
			           continue;
			         else
			          continue;
			        else
			         continue;
			       else
			        continue;
			     else if(*(cache_0 + -3) < c_b)
			      if(*(cache_0 + pixel[1]) > cb)
			       if(*(cache_0 + pixel[2]) > cb)
			        if(*(cache_0 + 3) > cb)
			         if(*(cache_0 + pixel[14]) > cb)
			          if(*(cache_0 + pixel[15]) > cb)
			           goto success;
			          else
			           continue;
			         else
			          continue;
			        else
			         continue;
			       else
			        continue;
			      else
			       continue;
			     else
			      if(*(cache_0 + 3) > cb)
			       if(*(cache_0 + pixel[2]) > cb)
			        if(*(cache_0 + pixel[14]) > cb)
			         if(*(cache_0 + pixel[1]) > cb)
			          if(*(cache_0 + pixel[15]) > cb)
			           goto success;
			          else
			           continue;
			         else
			          continue;
			        else
			         continue;
			       else
			        continue;
			      else
			       continue;
			    else
			     continue;
			  else if(*(cache_0 + pixel[3]) < c_b)
			   if(*(cache_0 + pixel[11]) > cb)
			    if(*(cache_0 + -3) > cb)
			     if(*(cache_0 + pixel[13]) > cb)
			      if(*(cache_0 + pixel[14]) > cb)
			       if(*(cache_0 + pixel[15]) > cb)
			        if(*(cache_0 + pixel[10]) > cb)
			         if(*(cache_0 + pixel[9]) > cb)
			          goto success;
			         else
			          if(*(cache_0 + pixel[1]) > cb)
			           goto success;
			          else
			           continue;
			        else
			         if(*(cache_0 + pixel[1]) > cb)
			          if(*(cache_0 + pixel[2]) > cb)
			           goto success;
			          else
			           continue;
			         else
			          continue;
			       else
			        if(*(cache_0 + pixel[7]) > cb)
			         if(*(cache_0 + pixel[9]) > cb)
			          if(*(cache_0 + pixel[10]) > cb)
			           goto success;
			          else
			           continue;
			         else
			          continue;
			        else
			         continue;
			      else
			       if(*(cache_0 + pixel[6]) > cb)
			        if(*(cache_0 + pixel[7]) > cb)
			         if(*(cache_0 + pixel[9]) > cb)
			          if(*(cache_0 + pixel[10]) > cb)
			           goto success;
			          else
			           continue;
			         else
			          continue;
			        else
			         continue;
			       else
			        continue;
			     else
			      if(*(cache_0 + pixel[5]) > cb)
			       if(*(cache_0 + pixel[6]) > cb)
			        if(*(cache_0 + pixel[7]) > cb)
			         if(*(cache_0 + pixel[9]) > cb)
			          if(*(cache_0 + pixel[10]) > cb)
			           goto success;
			          else
			           continue;
			         else
			          continue;
			        else
			         continue;
			       else
			        continue;
			      else
			       continue;
			    else
			     if(*(cache_0 + 3) > cb)
			      if(*(cache_0 + pixel[5]) > cb)
			       if(*(cache_0 + pixel[6]) > cb)
			        if(*(cache_0 + pixel[7]) > cb)
			         if(*(cache_0 + pixel[9]) > cb)
			          if(*(cache_0 + pixel[10]) > cb)
			           goto success;
			          else
			           continue;
			         else
			          continue;
			        else
			         continue;
			       else
			        continue;
			      else
			       continue;
			     else
			      continue;
			   else
			    continue;
			  else
			   if(*(cache_0 + pixel[11]) > cb)
			    if(*(cache_0 + -3) > cb)
			     if(*(cache_0 + pixel[10]) > cb)
			      if(*(cache_0 + pixel[14]) > cb)
			       if(*(cache_0 + pixel[13]) > cb)
			        if(*(cache_0 + pixel[9]) > cb)
			         if(*(cache_0 + pixel[15]) > cb)
			          goto success;
			         else
			          if(*(cache_0 + pixel[7]) > cb)
			           goto success;
			          else
			           continue;
			        else
			         if(*(cache_0 + pixel[1]) > cb)
			          if(*(cache_0 + pixel[15]) > cb)
			           goto success;
			          else
			           continue;
			         else
			          continue;
			       else if(*(cache_0 + pixel[13]) < c_b)
			        if(*(cache_0 + pixel[5]) > cb)
			         if(*(cache_0 + pixel[6]) > cb)
			          if(*(cache_0 + pixel[7]) > cb)
			           if(*(cache_0 + pixel[9]) > cb)
			            goto success;
			           else
			            continue;
			          else
			           continue;
			         else
			          continue;
			        else
			         continue;
			       else
			        if(*(cache_0 + pixel[5]) > cb)
			         if(*(cache_0 + pixel[7]) > cb)
			          if(*(cache_0 + pixel[9]) > cb)
			           if(*(cache_0 + pixel[6]) > cb)
			            goto success;
			           else
			            continue;
			          else
			           continue;
			         else
			          continue;
			        else
			         continue;
			      else
			       if(*(cache_0 + pixel[6]) > cb)
			        if(*(cache_0 + pixel[7]) > cb)
			         if(*(cache_0 + pixel[9]) > cb)
			          if(*(cache_0 + pixel[5]) > cb)
			           goto success;
			          else
			           if(*(cache_0 + pixel[13]) > cb)
			            goto success;
			           else
			            continue;
			         else
			          continue;
			        else
			         continue;
			       else
			        continue;
			     else if(*(cache_0 + pixel[10]) < c_b)
			      if(*(cache_0 + pixel[1]) > cb)
			       if(*(cache_0 + pixel[2]) > cb)
			        if(*(cache_0 + pixel[13]) > cb)
			         if(*(cache_0 + pixel[14]) > cb)
			          if(*(cache_0 + pixel[15]) > cb)
			           goto success;
			          else
			           continue;
			         else
			          continue;
			        else
			         continue;
			       else
			        continue;
			      else
			       continue;
			     else
			      if(*(cache_0 + pixel[2]) > cb)
			       if(*(cache_0 + pixel[14]) > cb)
			        if(*(cache_0 + pixel[13]) > cb)
			         if(*(cache_0 + pixel[1]) > cb)
			          if(*(cache_0 + pixel[15]) > cb)
			           goto success;
			          else
			           continue;
			         else
			          continue;
			        else
			         continue;
			       else
			        continue;
			      else
			       continue;
			    else if(*(cache_0 + -3) < c_b)
			     if(*(cache_0 + 3) > cb)
			      if(*(cache_0 + pixel[5]) > cb)
			       if(*(cache_0 + pixel[6]) > cb)
			        if(*(cache_0 + pixel[7]) > cb)
			         if(*(cache_0 + pixel[9]) > cb)
			          if(*(cache_0 + pixel[10]) > cb)
			           goto success;
			          else
			           continue;
			         else
			          continue;
			        else
			         continue;
			       else
			        continue;
			      else
			       continue;
			     else
			      continue;
			    else
			     if(*(cache_0 + 3) > cb)
			      if(*(cache_0 + pixel[6]) > cb)
			       if(*(cache_0 + pixel[10]) > cb)
			        if(*(cache_0 + pixel[7]) > cb)
			         if(*(cache_0 + pixel[5]) > cb)
			          if(*(cache_0 + pixel[9]) > cb)
			           goto success;
			          else
			           continue;
			         else
			          continue;
			        else
			         continue;
			       else
			        continue;
			      else
			       continue;
			     else
			      continue;
			   else
			    continue;
			 else if(*(cache_0 + pixel[8]) < c_b)
			  if(*(cache_0 + pixel[9]) > cb)
			   if(*(cache_0 + pixel[15]) > cb)
			    if(*(cache_0 + pixel[14]) > cb)
			     if(*(cache_0 + pixel[13]) > cb)
			      if(*(cache_0 + -3) > cb)
			       if(*(cache_0 + pixel[11]) > cb)
			        if(*(cache_0 + pixel[10]) > cb)
			         goto success;
			        else
			         if(*(cache_0 + pixel[1]) > cb)
			          if(*(cache_0 + pixel[2]) > cb)
			           goto success;
			          else
			           continue;
			         else if(*(cache_0 + pixel[1]) < c_b)
			          if(*(cache_0 + pixel[2]) < c_b)
			           if(*(cache_0 + pixel[3]) < c_b)
			            if(*(cache_0 + 3) < c_b)
			             if(*(cache_0 + pixel[5]) < c_b)
			              if(*(cache_0 + pixel[6]) < c_b)
			               if(*(cache_0 + pixel[7]) < c_b)
			                goto success;
			               else
			                continue;
			              else
			               continue;
			             else
			              continue;
			            else
			             continue;
			           else
			            continue;
			          else
			           continue;
			         else
			          continue;
			       else
			        if(*(cache_0 + pixel[1]) > cb)
			         if(*(cache_0 + pixel[2]) > cb)
			          if(*(cache_0 + pixel[3]) > cb)
			           goto success;
			          else
			           continue;
			         else
			          continue;
			        else if(*(cache_0 + pixel[1]) < c_b)
			         if(*(cache_0 + pixel[2]) < c_b)
			          if(*(cache_0 + pixel[3]) < c_b)
			           if(*(cache_0 + 3) < c_b)
			            if(*(cache_0 + pixel[5]) < c_b)
			             if(*(cache_0 + pixel[6]) < c_b)
			              if(*(cache_0 + pixel[7]) < c_b)
			               goto success;
			              else
			               continue;
			             else
			              continue;
			            else
			             continue;
			           else
			            continue;
			          else
			           continue;
			         else
			          continue;
			        else
			         continue;
			      else
			       if(*(cache_0 + pixel[1]) > cb)
			        if(*(cache_0 + pixel[2]) > cb)
			         if(*(cache_0 + pixel[3]) > cb)
			          if(*(cache_0 + 3) > cb)
			           goto success;
			          else
			           continue;
			         else
			          continue;
			        else
			         continue;
			       else if(*(cache_0 + pixel[1]) < c_b)
			        if(*(cache_0 + pixel[2]) < c_b)
			         if(*(cache_0 + pixel[3]) < c_b)
			          if(*(cache_0 + 3) < c_b)
			           if(*(cache_0 + pixel[5]) < c_b)
			            if(*(cache_0 + pixel[6]) < c_b)
			             if(*(cache_0 + pixel[7]) < c_b)
			              goto success;
			             else
			              continue;
			            else
			             continue;
			           else
			            continue;
			          else
			           continue;
			         else
			          continue;
			        else
			         continue;
			       else
			        continue;
			     else
			      if(*(cache_0 + pixel[1]) > cb)
			       if(*(cache_0 + pixel[2]) > cb)
			        if(*(cache_0 + pixel[3]) > cb)
			         if(*(cache_0 + 3) > cb)
			          if(*(cache_0 + pixel[5]) > cb)
			           goto success;
			          else
			           continue;
			         else
			          continue;
			        else
			         continue;
			       else
			        continue;
			      else if(*(cache_0 + pixel[1]) < c_b)
			       if(*(cache_0 + pixel[2]) < c_b)
			        if(*(cache_0 + pixel[3]) < c_b)
			         if(*(cache_0 + 3) < c_b)
			          if(*(cache_0 + pixel[5]) < c_b)
			           if(*(cache_0 + pixel[6]) < c_b)
			            if(*(cache_0 + pixel[7]) < c_b)
			             goto success;
			            else
			             continue;
			           else
			            continue;
			          else
			           continue;
			         else
			          continue;
			        else
			         continue;
			       else
			        continue;
			      else
			       continue;
			    else
			     if(*(cache_0 + pixel[1]) > cb)
			      if(*(cache_0 + pixel[2]) > cb)
			       if(*(cache_0 + pixel[3]) > cb)
			        if(*(cache_0 + 3) > cb)
			         if(*(cache_0 + pixel[5]) > cb)
			          if(*(cache_0 + pixel[6]) > cb)
			           goto success;
			          else
			           continue;
			         else
			          continue;
			        else
			         continue;
			       else
			        continue;
			      else
			       continue;
			     else if(*(cache_0 + pixel[1]) < c_b)
			      if(*(cache_0 + pixel[2]) < c_b)
			       if(*(cache_0 + pixel[3]) < c_b)
			        if(*(cache_0 + 3) < c_b)
			         if(*(cache_0 + pixel[5]) < c_b)
			          if(*(cache_0 + pixel[6]) < c_b)
			           if(*(cache_0 + pixel[7]) < c_b)
			            goto success;
			           else
			            continue;
			          else
			           continue;
			         else
			          continue;
			        else
			         continue;
			       else
			        continue;
			      else
			       continue;
			     else
			      continue;
			   else
			    if(*(cache_0 + pixel[1]) > cb)
			     if(*(cache_0 + pixel[2]) > cb)
			      if(*(cache_0 + pixel[3]) > cb)
			       if(*(cache_0 + 3) > cb)
			        if(*(cache_0 + pixel[5]) > cb)
			         if(*(cache_0 + pixel[6]) > cb)
			          if(*(cache_0 + pixel[7]) > cb)
			           goto success;
			          else
			           continue;
			         else
			          continue;
			        else
			         continue;
			       else
			        continue;
			      else
			       continue;
			     else
			      continue;
			    else if(*(cache_0 + pixel[1]) < c_b)
			     if(*(cache_0 + pixel[2]) < c_b)
			      if(*(cache_0 + pixel[3]) < c_b)
			       if(*(cache_0 + 3) < c_b)
			        if(*(cache_0 + pixel[5]) < c_b)
			         if(*(cache_0 + pixel[6]) < c_b)
			          if(*(cache_0 + pixel[7]) < c_b)
			           goto success;
			          else
			           continue;
			         else
			          continue;
			        else
			         continue;
			       else
			        continue;
			      else
			       continue;
			     else
			      continue;
			    else
			     continue;
			  else if(*(cache_0 + pixel[9]) < c_b)
			   if(*(cache_0 + pixel[10]) > cb)
			    if(*(cache_0 + pixel[1]) > cb)
			     if(*(cache_0 + pixel[15]) > cb)
			      if(*(cache_0 + pixel[14]) > cb)
			       if(*(cache_0 + pixel[13]) > cb)
			        if(*(cache_0 + -3) > cb)
			         if(*(cache_0 + pixel[11]) > cb)
			          goto success;
			         else
			          if(*(cache_0 + pixel[2]) > cb)
			           if(*(cache_0 + pixel[3]) > cb)
			            goto success;
			           else
			            continue;
			          else if(*(cache_0 + pixel[2]) < c_b)
			           if(*(cache_0 + pixel[3]) < c_b)
			            if(*(cache_0 + 3) < c_b)
			             if(*(cache_0 + pixel[5]) < c_b)
			              if(*(cache_0 + pixel[6]) < c_b)
			               if(*(cache_0 + pixel[7]) < c_b)
			                goto success;
			               else
			                continue;
			              else
			               continue;
			             else
			              continue;
			            else
			             continue;
			           else
			            continue;
			          else
			           continue;
			        else
			         if(*(cache_0 + pixel[2]) > cb)
			          if(*(cache_0 + pixel[3]) > cb)
			           if(*(cache_0 + 3) > cb)
			            goto success;
			           else
			            continue;
			          else
			           continue;
			         else if(*(cache_0 + pixel[2]) < c_b)
			          if(*(cache_0 + pixel[3]) < c_b)
			           if(*(cache_0 + 3) < c_b)
			            if(*(cache_0 + pixel[5]) < c_b)
			             if(*(cache_0 + pixel[6]) < c_b)
			              if(*(cache_0 + pixel[7]) < c_b)
			               goto success;
			              else
			               continue;
			             else
			              continue;
			            else
			             continue;
			           else
			            continue;
			          else
			           continue;
			         else
			          continue;
			       else
			        if(*(cache_0 + pixel[2]) > cb)
			         if(*(cache_0 + pixel[3]) > cb)
			          if(*(cache_0 + 3) > cb)
			           if(*(cache_0 + pixel[5]) > cb)
			            goto success;
			           else
			            continue;
			          else
			           continue;
			         else
			          continue;
			        else if(*(cache_0 + pixel[2]) < c_b)
			         if(*(cache_0 + pixel[3]) < c_b)
			          if(*(cache_0 + 3) < c_b)
			           if(*(cache_0 + pixel[5]) < c_b)
			            if(*(cache_0 + pixel[6]) < c_b)
			             if(*(cache_0 + pixel[7]) < c_b)
			              goto success;
			             else
			              continue;
			            else
			             continue;
			           else
			            continue;
			          else
			           continue;
			         else
			          continue;
			        else
			         continue;
			      else
			       if(*(cache_0 + pixel[2]) > cb)
			        if(*(cache_0 + pixel[3]) > cb)
			         if(*(cache_0 + 3) > cb)
			          if(*(cache_0 + pixel[5]) > cb)
			           if(*(cache_0 + pixel[6]) > cb)
			            goto success;
			           else
			            continue;
			          else
			           continue;
			         else
			          continue;
			        else
			         continue;
			       else if(*(cache_0 + pixel[2]) < c_b)
			        if(*(cache_0 + pixel[3]) < c_b)
			         if(*(cache_0 + 3) < c_b)
			          if(*(cache_0 + pixel[5]) < c_b)
			           if(*(cache_0 + pixel[6]) < c_b)
			            if(*(cache_0 + pixel[7]) < c_b)
			             goto success;
			            else
			             continue;
			           else
			            continue;
			          else
			           continue;
			         else
			          continue;
			        else
			         continue;
			       else
			        continue;
			     else if(*(cache_0 + pixel[15]) < c_b)
			      if(*(cache_0 + pixel[2]) > cb)
			       if(*(cache_0 + pixel[3]) > cb)
			        if(*(cache_0 + 3) > cb)
			         if(*(cache_0 + pixel[5]) > cb)
			          if(*(cache_0 + pixel[6]) > cb)
			           if(*(cache_0 + pixel[7]) > cb)
			            goto success;
			           else
			            continue;
			          else
			           continue;
			         else
			          continue;
			        else
			         continue;
			       else
			        continue;
			      else if(*(cache_0 + pixel[2]) < c_b)
			       if(*(cache_0 + pixel[3]) < c_b)
			        if(*(cache_0 + 3) < c_b)
			         if(*(cache_0 + pixel[5]) < c_b)
			          if(*(cache_0 + pixel[6]) < c_b)
			           if(*(cache_0 + pixel[7]) < c_b)
			            goto success;
			           else
			            continue;
			          else
			           continue;
			         else
			          continue;
			        else
			         continue;
			       else
			        continue;
			      else
			       continue;
			     else
			      if(*(cache_0 + pixel[3]) > cb)
			       if(*(cache_0 + pixel[2]) > cb)
			        if(*(cache_0 + 3) > cb)
			         if(*(cache_0 + pixel[5]) > cb)
			          if(*(cache_0 + pixel[6]) > cb)
			           if(*(cache_0 + pixel[7]) > cb)
			            goto success;
			           else
			            continue;
			          else
			           continue;
			         else
			          continue;
			        else
			         continue;
			       else
			        continue;
			      else if(*(cache_0 + pixel[3]) < c_b)
			       if(*(cache_0 + pixel[2]) < c_b)
			        if(*(cache_0 + 3) < c_b)
			         if(*(cache_0 + pixel[5]) < c_b)
			          if(*(cache_0 + pixel[6]) < c_b)
			           if(*(cache_0 + pixel[7]) < c_b)
			            goto success;
			           else
			            continue;
			          else
			           continue;
			         else
			          continue;
			        else
			         continue;
			       else
			        continue;
			      else
			       continue;
			    else if(*(cache_0 + pixel[1]) < c_b)
			     if(*(cache_0 + pixel[2]) < c_b)
			      if(*(cache_0 + pixel[3]) < c_b)
			       if(*(cache_0 + 3) < c_b)
			        if(*(cache_0 + pixel[5]) < c_b)
			         if(*(cache_0 + pixel[6]) < c_b)
			          if(*(cache_0 + pixel[7]) < c_b)
			           goto success;
			          else
			           continue;
			         else
			          continue;
			        else
			         continue;
			       else
			        continue;
			      else
			       continue;
			     else
			      continue;
			    else
			     if(*(cache_0 + pixel[2]) < c_b)
			      if(*(cache_0 + 3) < c_b)
			       if(*(cache_0 + pixel[3]) < c_b)
			        if(*(cache_0 + pixel[5]) < c_b)
			         if(*(cache_0 + pixel[6]) < c_b)
			          if(*(cache_0 + pixel[7]) < c_b)
			           goto success;
			          else
			           continue;
			         else
			          continue;
			        else
			         continue;
			       else
			        continue;
			      else
			       continue;
			     else
			      continue;
			   else if(*(cache_0 + pixel[10]) < c_b)
			    if(*(cache_0 + pixel[7]) > cb)
			     if(*(cache_0 + pixel[11]) > cb)
			      if(*(cache_0 + pixel[1]) > cb)
			       if(*(cache_0 + pixel[2]) > cb)
			        if(*(cache_0 + pixel[3]) > cb)
			         if(*(cache_0 + 3) > cb)
			          if(*(cache_0 + pixel[5]) > cb)
			           if(*(cache_0 + pixel[6]) > cb)
			            goto success;
			           else
			            if(*(cache_0 + pixel[14]) > cb)
			             if(*(cache_0 + pixel[15]) > cb)
			              goto success;
			             else
			              continue;
			            else
			             continue;
			          else
			           if(*(cache_0 + pixel[13]) > cb)
			            if(*(cache_0 + pixel[14]) > cb)
			             if(*(cache_0 + pixel[15]) > cb)
			              goto success;
			             else
			              continue;
			            else
			             continue;
			           else
			            continue;
			         else
			          if(*(cache_0 + -3) > cb)
			           if(*(cache_0 + pixel[13]) > cb)
			            if(*(cache_0 + pixel[14]) > cb)
			             if(*(cache_0 + pixel[15]) > cb)
			              goto success;
			             else
			              continue;
			            else
			             continue;
			           else
			            continue;
			          else
			           continue;
			        else
			         if(*(cache_0 + -3) > cb)
			          if(*(cache_0 + pixel[13]) > cb)
			           if(*(cache_0 + pixel[14]) > cb)
			            if(*(cache_0 + pixel[15]) > cb)
			             goto success;
			            else
			             continue;
			           else
			            continue;
			          else
			           continue;
			         else
			          continue;
			       else
			        continue;
			      else
			       continue;
			     else if(*(cache_0 + pixel[11]) < c_b)
			      if(*(cache_0 + -3) > cb)
			       if(*(cache_0 + pixel[1]) > cb)
			        if(*(cache_0 + pixel[2]) > cb)
			         if(*(cache_0 + pixel[3]) > cb)
			          if(*(cache_0 + pixel[15]) > cb)
			           if(*(cache_0 + pixel[14]) > cb)
			            if(*(cache_0 + pixel[13]) > cb)
			             goto success;
			            else
			             if(*(cache_0 + 3) > cb)
			              if(*(cache_0 + pixel[5]) > cb)
			               goto success;
			              else
			               continue;
			             else
			              continue;
			           else
			            if(*(cache_0 + 3) > cb)
			             if(*(cache_0 + pixel[5]) > cb)
			              if(*(cache_0 + pixel[6]) > cb)
			               goto success;
			              else
			               continue;
			             else
			              continue;
			            else
			             continue;
			          else
			           if(*(cache_0 + 3) > cb)
			            if(*(cache_0 + pixel[5]) > cb)
			             if(*(cache_0 + pixel[6]) > cb)
			              goto success;
			             else
			              continue;
			            else
			             continue;
			           else
			            continue;
			         else
			          continue;
			        else
			         continue;
			       else
			        continue;
			      else if(*(cache_0 + -3) < c_b)
			       if(*(cache_0 + pixel[15]) > cb)
			        if(*(cache_0 + pixel[1]) > cb)
			         if(*(cache_0 + pixel[2]) > cb)
			          if(*(cache_0 + pixel[3]) > cb)
			           if(*(cache_0 + 3) > cb)
			            if(*(cache_0 + pixel[14]) > cb)
			             if(*(cache_0 + pixel[13]) > cb)
			              goto success;
			             else
			              if(*(cache_0 + pixel[5]) > cb)
			               goto success;
			              else
			               continue;
			            else
			             if(*(cache_0 + pixel[5]) > cb)
			              if(*(cache_0 + pixel[6]) > cb)
			               goto success;
			              else
			               continue;
			             else
			              continue;
			           else
			            continue;
			          else
			           continue;
			         else
			          continue;
			        else
			         continue;
			       else if(*(cache_0 + pixel[15]) < c_b)
			        if(*(cache_0 + pixel[13]) < c_b)
			         if(*(cache_0 + pixel[14]) < c_b)
			          goto success;
			         else
			          if(*(cache_0 + pixel[1]) > cb)
			           if(*(cache_0 + pixel[2]) > cb)
			            if(*(cache_0 + pixel[3]) > cb)
			             if(*(cache_0 + 3) > cb)
			              if(*(cache_0 + pixel[5]) > cb)
			               if(*(cache_0 + pixel[6]) > cb)
			                goto success;
			               else
			                continue;
			              else
			               continue;
			             else
			              continue;
			            else
			             continue;
			           else
			            continue;
			          else
			           continue;
			        else
			         if(*(cache_0 + pixel[1]) > cb)
			          if(*(cache_0 + pixel[2]) > cb)
			           if(*(cache_0 + pixel[3]) > cb)
			            if(*(cache_0 + 3) > cb)
			             if(*(cache_0 + pixel[5]) > cb)
			              if(*(cache_0 + pixel[6]) > cb)
			               goto success;
			              else
			               continue;
			             else
			              continue;
			            else
			             continue;
			           else
			            continue;
			          else
			           continue;
			         else
			          continue;
			       else
			        if(*(cache_0 + pixel[3]) > cb)
			         if(*(cache_0 + pixel[1]) > cb)
			          if(*(cache_0 + pixel[2]) > cb)
			           if(*(cache_0 + 3) > cb)
			            if(*(cache_0 + pixel[5]) > cb)
			             if(*(cache_0 + pixel[6]) > cb)
			              goto success;
			             else
			              continue;
			            else
			             continue;
			           else
			            continue;
			          else
			           continue;
			         else
			          continue;
			        else
			         continue;
			      else
			       if(*(cache_0 + pixel[1]) > cb)
			        if(*(cache_0 + pixel[2]) > cb)
			         if(*(cache_0 + pixel[3]) > cb)
			          if(*(cache_0 + 3) > cb)
			           if(*(cache_0 + pixel[5]) > cb)
			            if(*(cache_0 + pixel[6]) > cb)
			             goto success;
			            else
			             if(*(cache_0 + pixel[14]) > cb)
			              if(*(cache_0 + pixel[15]) > cb)
			               goto success;
			              else
			               continue;
			             else
			              continue;
			           else
			            if(*(cache_0 + pixel[13]) > cb)
			             if(*(cache_0 + pixel[14]) > cb)
			              if(*(cache_0 + pixel[15]) > cb)
			               goto success;
			              else
			               continue;
			             else
			              continue;
			            else
			             continue;
			          else
			           continue;
			         else
			          continue;
			        else
			         continue;
			       else
			        continue;
			     else
			      if(*(cache_0 + pixel[2]) > cb)
			       if(*(cache_0 + pixel[1]) > cb)
			        if(*(cache_0 + pixel[3]) > cb)
			         if(*(cache_0 + 3) > cb)
			          if(*(cache_0 + pixel[5]) > cb)
			           if(*(cache_0 + pixel[6]) > cb)
			            goto success;
			           else
			            if(*(cache_0 + pixel[14]) > cb)
			             if(*(cache_0 + pixel[15]) > cb)
			              goto success;
			             else
			              continue;
			            else
			             continue;
			          else
			           if(*(cache_0 + pixel[13]) > cb)
			            if(*(cache_0 + pixel[14]) > cb)
			             if(*(cache_0 + pixel[15]) > cb)
			              goto success;
			             else
			              continue;
			            else
			             continue;
			           else
			            continue;
			         else
			          if(*(cache_0 + -3) > cb)
			           if(*(cache_0 + pixel[13]) > cb)
			            if(*(cache_0 + pixel[14]) > cb)
			             if(*(cache_0 + pixel[15]) > cb)
			              goto success;
			             else
			              continue;
			            else
			             continue;
			           else
			            continue;
			          else
			           continue;
			        else
			         continue;
			       else
			        continue;
			      else
			       continue;
			    else if(*(cache_0 + pixel[7]) < c_b)
			     if(*(cache_0 + pixel[6]) > cb)
			      if(*(cache_0 + -3) > cb)
			       if(*(cache_0 + pixel[1]) > cb)
			        if(*(cache_0 + pixel[2]) > cb)
			         if(*(cache_0 + pixel[15]) > cb)
			          if(*(cache_0 + pixel[3]) > cb)
			           if(*(cache_0 + pixel[14]) > cb)
			            if(*(cache_0 + pixel[13]) > cb)
			             goto success;
			            else
			             if(*(cache_0 + 3) > cb)
			              if(*(cache_0 + pixel[5]) > cb)
			               goto success;
			              else
			               continue;
			             else
			              continue;
			           else
			            if(*(cache_0 + 3) > cb)
			             if(*(cache_0 + pixel[5]) > cb)
			              goto success;
			             else
			              continue;
			            else
			             continue;
			          else
			           if(*(cache_0 + pixel[11]) > cb)
			            if(*(cache_0 + pixel[13]) > cb)
			             if(*(cache_0 + pixel[14]) > cb)
			              goto success;
			             else
			              continue;
			            else
			             continue;
			           else
			            continue;
			         else
			          continue;
			        else
			         continue;
			       else
			        continue;
			      else if(*(cache_0 + -3) < c_b)
			       if(*(cache_0 + pixel[13]) > cb)
			        if(*(cache_0 + pixel[1]) > cb)
			         if(*(cache_0 + pixel[2]) > cb)
			          if(*(cache_0 + pixel[3]) > cb)
			           if(*(cache_0 + 3) > cb)
			            if(*(cache_0 + pixel[15]) > cb)
			             if(*(cache_0 + pixel[14]) > cb)
			              goto success;
			             else
			              if(*(cache_0 + pixel[5]) > cb)
			               goto success;
			              else
			               continue;
			            else
			             continue;
			           else
			            continue;
			          else
			           continue;
			         else
			          continue;
			        else
			         continue;
			       else if(*(cache_0 + pixel[13]) < c_b)
			        if(*(cache_0 + pixel[11]) > cb)
			         if(*(cache_0 + pixel[1]) > cb)
			          if(*(cache_0 + pixel[2]) > cb)
			           if(*(cache_0 + pixel[3]) > cb)
			            if(*(cache_0 + 3) > cb)
			             if(*(cache_0 + pixel[5]) > cb)
			              if(*(cache_0 + pixel[15]) > cb)
			               goto success;
			              else
			               continue;
			             else
			              continue;
			            else
			             continue;
			           else
			            continue;
			          else
			           continue;
			         else
			          continue;
			        else if(*(cache_0 + pixel[11]) < c_b)
			         if(*(cache_0 + pixel[14]) > cb)
			          if(*(cache_0 + pixel[1]) > cb)
			           if(*(cache_0 + pixel[2]) > cb)
			            if(*(cache_0 + pixel[3]) > cb)
			             if(*(cache_0 + 3) > cb)
			              if(*(cache_0 + pixel[5]) > cb)
			               if(*(cache_0 + pixel[15]) > cb)
			                goto success;
			               else
			                continue;
			              else
			               continue;
			             else
			              continue;
			            else
			             continue;
			           else
			            continue;
			          else
			           continue;
			         else if(*(cache_0 + pixel[14]) < c_b)
			          goto success;
			         else
			          if(*(cache_0 + pixel[3]) > cb)
			           if(*(cache_0 + pixel[1]) > cb)
			            if(*(cache_0 + pixel[2]) > cb)
			             if(*(cache_0 + 3) > cb)
			              if(*(cache_0 + pixel[5]) > cb)
			               if(*(cache_0 + pixel[15]) > cb)
			                goto success;
			               else
			                continue;
			              else
			               continue;
			             else
			              continue;
			            else
			             continue;
			           else
			            continue;
			          else
			           continue;
			        else
			         if(*(cache_0 + pixel[3]) > cb)
			          if(*(cache_0 + pixel[1]) > cb)
			           if(*(cache_0 + pixel[2]) > cb)
			            if(*(cache_0 + 3) > cb)
			             if(*(cache_0 + pixel[5]) > cb)
			              if(*(cache_0 + pixel[15]) > cb)
			               goto success;
			              else
			               continue;
			             else
			              continue;
			            else
			             continue;
			           else
			            continue;
			          else
			           continue;
			         else
			          continue;
			       else
			        if(*(cache_0 + pixel[3]) > cb)
			         if(*(cache_0 + pixel[1]) > cb)
			          if(*(cache_0 + pixel[2]) > cb)
			           if(*(cache_0 + 3) > cb)
			            if(*(cache_0 + pixel[5]) > cb)
			             if(*(cache_0 + pixel[15]) > cb)
			              goto success;
			             else
			              continue;
			            else
			             continue;
			           else
			            continue;
			          else
			           continue;
			         else
			          continue;
			        else
			         continue;
			      else
			       if(*(cache_0 + pixel[3]) > cb)
			        if(*(cache_0 + pixel[15]) > cb)
			         if(*(cache_0 + pixel[1]) > cb)
			          if(*(cache_0 + pixel[2]) > cb)
			           if(*(cache_0 + 3) > cb)
			            if(*(cache_0 + pixel[5]) > cb)
			             goto success;
			            else
			             if(*(cache_0 + pixel[13]) > cb)
			              if(*(cache_0 + pixel[14]) > cb)
			               goto success;
			              else
			               continue;
			             else
			              continue;
			           else
			            continue;
			          else
			           continue;
			         else
			          continue;
			        else
			         continue;
			       else
			        continue;
			     else if(*(cache_0 + pixel[6]) < c_b)
			      if(*(cache_0 + pixel[3]) > cb)
			       if(*(cache_0 + -3) > cb)
			        if(*(cache_0 + pixel[5]) > cb)
			         if(*(cache_0 + pixel[1]) > cb)
			          if(*(cache_0 + pixel[2]) > cb)
			           if(*(cache_0 + pixel[14]) > cb)
			            if(*(cache_0 + pixel[15]) > cb)
			             if(*(cache_0 + pixel[13]) > cb)
			              goto success;
			             else
			              if(*(cache_0 + 3) > cb)
			               goto success;
			              else
			               continue;
			            else
			             continue;
			           else
			            continue;
			          else
			           continue;
			         else
			          continue;
			        else if(*(cache_0 + pixel[5]) < c_b)
			         if(*(cache_0 + pixel[11]) < c_b)
			          if(*(cache_0 + 3) < c_b)
			           goto success;
			          else
			           if(*(cache_0 + pixel[1]) > cb)
			            if(*(cache_0 + pixel[2]) > cb)
			             if(*(cache_0 + pixel[13]) > cb)
			              if(*(cache_0 + pixel[14]) > cb)
			               if(*(cache_0 + pixel[15]) > cb)
			                goto success;
			               else
			                continue;
			              else
			               continue;
			             else
			              continue;
			            else
			             continue;
			           else
			            continue;
			         else
			          if(*(cache_0 + pixel[1]) > cb)
			           if(*(cache_0 + pixel[2]) > cb)
			            if(*(cache_0 + pixel[13]) > cb)
			             if(*(cache_0 + pixel[14]) > cb)
			              if(*(cache_0 + pixel[15]) > cb)
			               goto success;
			              else
			               continue;
			             else
			              continue;
			            else
			             continue;
			           else
			            continue;
			          else
			           continue;
			        else
			         if(*(cache_0 + pixel[1]) > cb)
			          if(*(cache_0 + pixel[2]) > cb)
			           if(*(cache_0 + pixel[13]) > cb)
			            if(*(cache_0 + pixel[14]) > cb)
			             if(*(cache_0 + pixel[15]) > cb)
			              goto success;
			             else
			              continue;
			            else
			             continue;
			           else
			            continue;
			          else
			           continue;
			         else
			          continue;
			       else if(*(cache_0 + -3) < c_b)
			        if(*(cache_0 + pixel[11]) > cb)
			         if(*(cache_0 + pixel[1]) > cb)
			          if(*(cache_0 + pixel[2]) > cb)
			           if(*(cache_0 + 3) > cb)
			            if(*(cache_0 + pixel[14]) > cb)
			             if(*(cache_0 + pixel[15]) > cb)
			              if(*(cache_0 + pixel[13]) > cb)
			               goto success;
			              else
			               if(*(cache_0 + pixel[5]) > cb)
			                goto success;
			               else
			                continue;
			             else
			              continue;
			            else
			             continue;
			           else
			            continue;
			          else
			           continue;
			         else
			          continue;
			        else if(*(cache_0 + pixel[11]) < c_b)
			         if(*(cache_0 + pixel[13]) > cb)
			          if(*(cache_0 + pixel[5]) < c_b)
			           goto success;
			          else
			           if(*(cache_0 + pixel[1]) > cb)
			            if(*(cache_0 + pixel[2]) > cb)
			             if(*(cache_0 + 3) > cb)
			              if(*(cache_0 + pixel[14]) > cb)
			               if(*(cache_0 + pixel[15]) > cb)
			                goto success;
			               else
			                continue;
			              else
			               continue;
			             else
			              continue;
			            else
			             continue;
			           else
			            continue;
			         else if(*(cache_0 + pixel[13]) < c_b)
			          goto success;
			         else
			          if(*(cache_0 + pixel[5]) > cb)
			           if(*(cache_0 + pixel[1]) > cb)
			            if(*(cache_0 + pixel[2]) > cb)
			             if(*(cache_0 + 3) > cb)
			              if(*(cache_0 + pixel[14]) > cb)
			               if(*(cache_0 + pixel[15]) > cb)
			                goto success;
			               else
			                continue;
			              else
			               continue;
			             else
			              continue;
			            else
			             continue;
			           else
			            continue;
			          else if(*(cache_0 + pixel[5]) < c_b)
			           goto success;
			          else
			           continue;
			        else
			         if(*(cache_0 + pixel[5]) > cb)
			          if(*(cache_0 + pixel[1]) > cb)
			           if(*(cache_0 + pixel[2]) > cb)
			            if(*(cache_0 + 3) > cb)
			             if(*(cache_0 + pixel[14]) > cb)
			              if(*(cache_0 + pixel[15]) > cb)
			               goto success;
			              else
			               continue;
			             else
			              continue;
			            else
			             continue;
			           else
			            continue;
			          else
			           continue;
			         else if(*(cache_0 + pixel[5]) < c_b)
			          if(*(cache_0 + pixel[1]) > cb)
			           if(*(cache_0 + pixel[2]) > cb)
			            if(*(cache_0 + 3) > cb)
			             if(*(cache_0 + pixel[13]) > cb)
			              if(*(cache_0 + pixel[14]) > cb)
			               if(*(cache_0 + pixel[15]) > cb)
			                goto success;
			               else
			                continue;
			              else
			               continue;
			             else
			              continue;
			            else
			             continue;
			           else
			            continue;
			          else
			           continue;
			         else
			          if(*(cache_0 + pixel[13]) > cb)
			           if(*(cache_0 + pixel[1]) > cb)
			            if(*(cache_0 + pixel[2]) > cb)
			             if(*(cache_0 + 3) > cb)
			              if(*(cache_0 + pixel[14]) > cb)
			               if(*(cache_0 + pixel[15]) > cb)
			                goto success;
			               else
			                continue;
			              else
			               continue;
			             else
			              continue;
			            else
			             continue;
			           else
			            continue;
			          else
			           continue;
			       else
			        if(*(cache_0 + 3) > cb)
			         if(*(cache_0 + pixel[14]) > cb)
			          if(*(cache_0 + pixel[1]) > cb)
			           if(*(cache_0 + pixel[2]) > cb)
			            if(*(cache_0 + pixel[15]) > cb)
			             if(*(cache_0 + pixel[5]) > cb)
			              goto success;
			             else
			              if(*(cache_0 + pixel[13]) > cb)
			               goto success;
			              else
			               continue;
			            else
			             continue;
			           else
			            continue;
			          else
			           continue;
			         else
			          continue;
			        else if(*(cache_0 + 3) < c_b)
			         if(*(cache_0 + pixel[11]) < c_b)
			          if(*(cache_0 + pixel[5]) < c_b)
			           goto success;
			          else
			           continue;
			         else
			          continue;
			        else
			         continue;
			      else if(*(cache_0 + pixel[3]) < c_b)
			       if(*(cache_0 + pixel[5]) > cb)
			        if(*(cache_0 + pixel[11]) > cb)
			         if(*(cache_0 + pixel[1]) > cb)
			          if(*(cache_0 + pixel[2]) > cb)
			           if(*(cache_0 + -3) > cb)
			            if(*(cache_0 + pixel[13]) > cb)
			             if(*(cache_0 + pixel[14]) > cb)
			              if(*(cache_0 + pixel[15]) > cb)
			               goto success;
			              else
			               continue;
			             else
			              continue;
			            else
			             continue;
			           else
			            continue;
			          else
			           continue;
			         else
			          continue;
			        else if(*(cache_0 + pixel[11]) < c_b)
			         if(*(cache_0 + -3) < c_b)
			          if(*(cache_0 + pixel[13]) < c_b)
			           goto success;
			          else
			           continue;
			         else
			          continue;
			        else
			         continue;
			       else if(*(cache_0 + pixel[5]) < c_b)
			        if(*(cache_0 + 3) > cb)
			         if(*(cache_0 + pixel[11]) > cb)
			          if(*(cache_0 + pixel[1]) > cb)
			           if(*(cache_0 + pixel[2]) > cb)
			            if(*(cache_0 + -3) > cb)
			             if(*(cache_0 + pixel[13]) > cb)
			              if(*(cache_0 + pixel[14]) > cb)
			               if(*(cache_0 + pixel[15]) > cb)
			                goto success;
			               else
			                continue;
			              else
			               continue;
			             else
			              continue;
			            else
			             continue;
			           else
			            continue;
			          else
			           continue;
			         else if(*(cache_0 + pixel[11]) < c_b)
			          if(*(cache_0 + -3) < c_b)
			           goto success;
			          else
			           continue;
			         else
			          continue;
			        else if(*(cache_0 + 3) < c_b)
			         goto success;
			        else
			         if(*(cache_0 + -3) > cb)
			          if(*(cache_0 + pixel[2]) > cb)
			           if(*(cache_0 + pixel[1]) > cb)
			            if(*(cache_0 + pixel[11]) > cb)
			             if(*(cache_0 + pixel[13]) > cb)
			              if(*(cache_0 + pixel[14]) > cb)
			               if(*(cache_0 + pixel[15]) > cb)
			                goto success;
			               else
			                continue;
			              else
			               continue;
			             else
			              continue;
			            else
			             continue;
			           else
			            continue;
			          else
			           continue;
			         else if(*(cache_0 + -3) < c_b)
			          if(*(cache_0 + pixel[11]) < c_b)
			           goto success;
			          else
			           continue;
			         else
			          continue;
			       else
			        if(*(cache_0 + pixel[13]) > cb)
			         if(*(cache_0 + pixel[2]) > cb)
			          if(*(cache_0 + pixel[1]) > cb)
			           if(*(cache_0 + pixel[11]) > cb)
			            if(*(cache_0 + -3) > cb)
			             if(*(cache_0 + pixel[14]) > cb)
			              if(*(cache_0 + pixel[15]) > cb)
			               goto success;
			              else
			               continue;
			             else
			              continue;
			            else
			             continue;
			           else
			            continue;
			          else
			           continue;
			         else
			          continue;
			        else if(*(cache_0 + pixel[13]) < c_b)
			         if(*(cache_0 + pixel[11]) < c_b)
			          if(*(cache_0 + -3) < c_b)
			           goto success;
			          else
			           continue;
			         else
			          continue;
			        else
			         continue;
			      else
			       if(*(cache_0 + pixel[11]) > cb)
			        if(*(cache_0 + pixel[1]) > cb)
			         if(*(cache_0 + pixel[2]) > cb)
			          if(*(cache_0 + -3) > cb)
			           if(*(cache_0 + pixel[13]) > cb)
			            if(*(cache_0 + pixel[14]) > cb)
			             if(*(cache_0 + pixel[15]) > cb)
			              goto success;
			             else
			              continue;
			            else
			             continue;
			           else
			            continue;
			          else
			           continue;
			         else
			          continue;
			        else
			         continue;
			       else if(*(cache_0 + pixel[11]) < c_b)
			        if(*(cache_0 + -3) < c_b)
			         if(*(cache_0 + pixel[5]) < c_b)
			          goto success;
			         else
			          if(*(cache_0 + pixel[13]) < c_b)
			           goto success;
			          else
			           continue;
			        else
			         if(*(cache_0 + 3) < c_b)
			          if(*(cache_0 + pixel[5]) < c_b)
			           goto success;
			          else
			           continue;
			         else
			          continue;
			       else
			        continue;
			     else
			      if(*(cache_0 + pixel[14]) > cb)
			       if(*(cache_0 + pixel[5]) > cb)
			        if(*(cache_0 + pixel[1]) > cb)
			         if(*(cache_0 + pixel[15]) > cb)
			          if(*(cache_0 + pixel[2]) > cb)
			           if(*(cache_0 + pixel[3]) > cb)
			            if(*(cache_0 + 3) > cb)
			             goto success;
			            else
			             if(*(cache_0 + -3) > cb)
			              if(*(cache_0 + pixel[13]) > cb)
			               goto success;
			              else
			               continue;
			             else
			              continue;
			           else
			            if(*(cache_0 + pixel[11]) > cb)
			             if(*(cache_0 + -3) > cb)
			              if(*(cache_0 + pixel[13]) > cb)
			               goto success;
			              else
			               continue;
			             else
			              continue;
			            else
			             continue;
			          else
			           continue;
			         else
			          continue;
			        else
			         continue;
			       else if(*(cache_0 + pixel[5]) < c_b)
			        if(*(cache_0 + pixel[13]) > cb)
			         if(*(cache_0 + pixel[15]) > cb)
			          if(*(cache_0 + pixel[11]) > cb)
			           if(*(cache_0 + pixel[1]) > cb)
			            if(*(cache_0 + pixel[2]) > cb)
			             if(*(cache_0 + -3) > cb)
			              goto success;
			             else
			              if(*(cache_0 + pixel[3]) > cb)
			               if(*(cache_0 + 3) > cb)
			                goto success;
			               else
			                continue;
			              else
			               continue;
			            else
			             continue;
			           else
			            continue;
			          else if(*(cache_0 + pixel[11]) < c_b)
			           if(*(cache_0 + 3) > cb)
			            if(*(cache_0 + pixel[1]) > cb)
			             if(*(cache_0 + pixel[2]) > cb)
			              if(*(cache_0 + pixel[3]) > cb)
			               goto success;
			              else
			               continue;
			             else
			              continue;
			            else
			             continue;
			           else if(*(cache_0 + 3) < c_b)
			            if(*(cache_0 + pixel[1]) > cb)
			             if(*(cache_0 + pixel[2]) > cb)
			              if(*(cache_0 + pixel[3]) > cb)
			               if(*(cache_0 + -3) > cb)
			                goto success;
			               else
			                continue;
			              else
			               continue;
			             else
			              continue;
			            else
			             continue;
			           else
			            if(*(cache_0 + -3) > cb)
			             if(*(cache_0 + pixel[1]) > cb)
			              if(*(cache_0 + pixel[2]) > cb)
			               if(*(cache_0 + pixel[3]) > cb)
			                goto success;
			               else
			                continue;
			              else
			               continue;
			             else
			              continue;
			            else
			             continue;
			          else
			           if(*(cache_0 + pixel[3]) > cb)
			            if(*(cache_0 + -3) > cb)
			             if(*(cache_0 + pixel[1]) > cb)
			              if(*(cache_0 + pixel[2]) > cb)
			               goto success;
			              else
			               continue;
			             else
			              continue;
			            else if(*(cache_0 + -3) < c_b)
			             if(*(cache_0 + pixel[1]) > cb)
			              if(*(cache_0 + pixel[2]) > cb)
			               if(*(cache_0 + 3) > cb)
			                goto success;
			               else
			                continue;
			              else
			               continue;
			             else
			              continue;
			            else
			             if(*(cache_0 + 3) > cb)
			              if(*(cache_0 + pixel[1]) > cb)
			               if(*(cache_0 + pixel[2]) > cb)
			                goto success;
			               else
			                continue;
			              else
			               continue;
			             else
			              continue;
			           else
			            continue;
			         else
			          continue;
			        else
			         continue;
			       else
			        if(*(cache_0 + pixel[13]) > cb)
			         if(*(cache_0 + pixel[2]) > cb)
			          if(*(cache_0 + pixel[11]) > cb)
			           if(*(cache_0 + pixel[1]) > cb)
			            if(*(cache_0 + pixel[15]) > cb)
			             if(*(cache_0 + -3) > cb)
			              goto success;
			             else
			              if(*(cache_0 + pixel[3]) > cb)
			               if(*(cache_0 + 3) > cb)
			                goto success;
			               else
			                continue;
			              else
			               continue;
			            else
			             continue;
			           else
			            continue;
			          else if(*(cache_0 + pixel[11]) < c_b)
			           if(*(cache_0 + pixel[1]) > cb)
			            if(*(cache_0 + pixel[3]) > cb)
			             if(*(cache_0 + pixel[15]) > cb)
			              if(*(cache_0 + -3) > cb)
			               goto success;
			              else
			               if(*(cache_0 + 3) > cb)
			                goto success;
			               else
			                continue;
			             else
			              continue;
			            else
			             continue;
			           else
			            continue;
			          else
			           if(*(cache_0 + pixel[3]) > cb)
			            if(*(cache_0 + 3) > cb)
			             if(*(cache_0 + pixel[1]) > cb)
			              if(*(cache_0 + pixel[15]) > cb)
			               goto success;
			              else
			               continue;
			             else
			              continue;
			            else if(*(cache_0 + 3) < c_b)
			             if(*(cache_0 + pixel[1]) > cb)
			              if(*(cache_0 + -3) > cb)
			               if(*(cache_0 + pixel[15]) > cb)
			                goto success;
			               else
			                continue;
			              else
			               continue;
			             else
			              continue;
			            else
			             if(*(cache_0 + -3) > cb)
			              if(*(cache_0 + pixel[1]) > cb)
			               if(*(cache_0 + pixel[15]) > cb)
			                goto success;
			               else
			                continue;
			              else
			               continue;
			             else
			              continue;
			           else
			            continue;
			         else
			          continue;
			        else
			         continue;
			      else if(*(cache_0 + pixel[14]) < c_b)
			       if(*(cache_0 + -3) < c_b)
			        if(*(cache_0 + pixel[11]) < c_b)
			         if(*(cache_0 + pixel[13]) < c_b)
			          goto success;
			         else
			          continue;
			        else
			         continue;
			       else
			        continue;
			      else
			       continue;
			    else
			     if(*(cache_0 + pixel[5]) > cb)
			      if(*(cache_0 + pixel[3]) > cb)
			       if(*(cache_0 + pixel[15]) > cb)
			        if(*(cache_0 + pixel[1]) > cb)
			         if(*(cache_0 + pixel[6]) > cb)
			          if(*(cache_0 + pixel[2]) > cb)
			           if(*(cache_0 + 3) > cb)
			            goto success;
			           else
			            if(*(cache_0 + -3) > cb)
			             if(*(cache_0 + pixel[13]) > cb)
			              if(*(cache_0 + pixel[14]) > cb)
			               goto success;
			              else
			               continue;
			             else
			              continue;
			            else
			             continue;
			          else
			           continue;
			         else
			          if(*(cache_0 + pixel[14]) > cb)
			           if(*(cache_0 + pixel[2]) > cb)
			            if(*(cache_0 + 3) > cb)
			             goto success;
			            else
			             if(*(cache_0 + -3) > cb)
			              if(*(cache_0 + pixel[13]) > cb)
			               goto success;
			              else
			               continue;
			             else
			              continue;
			           else
			            continue;
			          else
			           continue;
			        else
			         continue;
			       else if(*(cache_0 + pixel[15]) < c_b)
			        if(*(cache_0 + pixel[14]) < c_b)
			         if(*(cache_0 + -3) < c_b)
			          if(*(cache_0 + pixel[11]) < c_b)
			           if(*(cache_0 + pixel[13]) < c_b)
			            goto success;
			           else
			            continue;
			          else
			           continue;
			         else
			          continue;
			        else
			         continue;
			       else
			        continue;
			      else if(*(cache_0 + pixel[3]) < c_b)
			       if(*(cache_0 + pixel[13]) > cb)
			        if(*(cache_0 + pixel[1]) > cb)
			         if(*(cache_0 + pixel[2]) > cb)
			          if(*(cache_0 + pixel[11]) > cb)
			           if(*(cache_0 + -3) > cb)
			            if(*(cache_0 + pixel[14]) > cb)
			             if(*(cache_0 + pixel[15]) > cb)
			              goto success;
			             else
			              continue;
			            else
			             continue;
			           else
			            continue;
			          else
			           continue;
			         else
			          continue;
			        else
			         continue;
			       else if(*(cache_0 + pixel[13]) < c_b)
			        if(*(cache_0 + pixel[11]) < c_b)
			         if(*(cache_0 + -3) < c_b)
			          if(*(cache_0 + pixel[14]) < c_b)
			           if(*(cache_0 + pixel[15]) < c_b)
			            goto success;
			           else
			            continue;
			          else
			           continue;
			         else
			          continue;
			        else
			         continue;
			       else
			        continue;
			      else
			       if(*(cache_0 + pixel[15]) > cb)
			        if(*(cache_0 + pixel[11]) > cb)
			         if(*(cache_0 + pixel[1]) > cb)
			          if(*(cache_0 + pixel[2]) > cb)
			           if(*(cache_0 + -3) > cb)
			            if(*(cache_0 + pixel[13]) > cb)
			             if(*(cache_0 + pixel[14]) > cb)
			              goto success;
			             else
			              continue;
			            else
			             continue;
			           else
			            continue;
			          else
			           continue;
			         else
			          continue;
			        else
			         continue;
			       else if(*(cache_0 + pixel[15]) < c_b)
			        if(*(cache_0 + pixel[11]) < c_b)
			         if(*(cache_0 + -3) < c_b)
			          if(*(cache_0 + pixel[13]) < c_b)
			           if(*(cache_0 + pixel[14]) < c_b)
			            goto success;
			           else
			            continue;
			          else
			           continue;
			         else
			          continue;
			        else
			         continue;
			       else
			        continue;
			     else if(*(cache_0 + pixel[5]) < c_b)
			      if(*(cache_0 + pixel[14]) > cb)
			       if(*(cache_0 + pixel[13]) > cb)
			        if(*(cache_0 + pixel[15]) > cb)
			         if(*(cache_0 + pixel[2]) > cb)
			          if(*(cache_0 + 3) > cb)
			           if(*(cache_0 + pixel[1]) > cb)
			            if(*(cache_0 + pixel[3]) > cb)
			             goto success;
			            else
			             if(*(cache_0 + pixel[11]) > cb)
			              if(*(cache_0 + -3) > cb)
			               goto success;
			              else
			               continue;
			             else
			              continue;
			           else
			            continue;
			          else
			           if(*(cache_0 + -3) > cb)
			            if(*(cache_0 + pixel[3]) > cb)
			             if(*(cache_0 + pixel[1]) > cb)
			              goto success;
			             else
			              continue;
			            else if(*(cache_0 + pixel[3]) < c_b)
			             if(*(cache_0 + pixel[1]) > cb)
			              if(*(cache_0 + pixel[11]) > cb)
			               goto success;
			              else
			               continue;
			             else
			              continue;
			            else
			             if(*(cache_0 + pixel[11]) > cb)
			              if(*(cache_0 + pixel[1]) > cb)
			               goto success;
			              else
			               continue;
			             else
			              continue;
			           else
			            continue;
			         else
			          continue;
			        else
			         continue;
			       else
			        continue;
			      else if(*(cache_0 + pixel[14]) < c_b)
			       if(*(cache_0 + pixel[15]) < c_b)
			        if(*(cache_0 + pixel[11]) < c_b)
			         if(*(cache_0 + -3) < c_b)
			          if(*(cache_0 + pixel[13]) < c_b)
			           goto success;
			          else
			           continue;
			         else
			          continue;
			        else
			         continue;
			       else
			        continue;
			      else
			       continue;
			     else
			      if(*(cache_0 + pixel[14]) > cb)
			       if(*(cache_0 + pixel[13]) > cb)
			        if(*(cache_0 + pixel[11]) > cb)
			         if(*(cache_0 + pixel[1]) > cb)
			          if(*(cache_0 + pixel[2]) > cb)
			           if(*(cache_0 + pixel[15]) > cb)
			            if(*(cache_0 + -3) > cb)
			             goto success;
			            else
			             if(*(cache_0 + pixel[3]) > cb)
			              if(*(cache_0 + 3) > cb)
			               goto success;
			              else
			               continue;
			             else
			              continue;
			           else
			            continue;
			          else
			           continue;
			         else
			          continue;
			        else if(*(cache_0 + pixel[11]) < c_b)
			         if(*(cache_0 + pixel[3]) > cb)
			          if(*(cache_0 + pixel[1]) > cb)
			           if(*(cache_0 + pixel[2]) > cb)
			            if(*(cache_0 + -3) > cb)
			             if(*(cache_0 + pixel[15]) > cb)
			              goto success;
			             else
			              continue;
			            else
			             if(*(cache_0 + 3) > cb)
			              if(*(cache_0 + pixel[15]) > cb)
			               goto success;
			              else
			               continue;
			             else
			              continue;
			           else
			            continue;
			          else
			           continue;
			         else
			          continue;
			        else
			         if(*(cache_0 + pixel[3]) > cb)
			          if(*(cache_0 + 3) > cb)
			           if(*(cache_0 + pixel[1]) > cb)
			            if(*(cache_0 + pixel[2]) > cb)
			             if(*(cache_0 + pixel[15]) > cb)
			              goto success;
			             else
			              continue;
			            else
			             continue;
			           else
			            continue;
			          else if(*(cache_0 + 3) < c_b)
			           if(*(cache_0 + pixel[1]) > cb)
			            if(*(cache_0 + pixel[2]) > cb)
			             if(*(cache_0 + -3) > cb)
			              if(*(cache_0 + pixel[15]) > cb)
			               goto success;
			              else
			               continue;
			             else
			              continue;
			            else
			             continue;
			           else
			            continue;
			          else
			           if(*(cache_0 + -3) > cb)
			            if(*(cache_0 + pixel[1]) > cb)
			             if(*(cache_0 + pixel[2]) > cb)
			              if(*(cache_0 + pixel[15]) > cb)
			               goto success;
			              else
			               continue;
			             else
			              continue;
			            else
			             continue;
			           else
			            continue;
			         else
			          continue;
			       else
			        continue;
			      else if(*(cache_0 + pixel[14]) < c_b)
			       if(*(cache_0 + pixel[15]) < c_b)
			        if(*(cache_0 + -3) < c_b)
			         if(*(cache_0 + pixel[11]) < c_b)
			          if(*(cache_0 + pixel[13]) < c_b)
			           goto success;
			          else
			           continue;
			         else
			          continue;
			        else
			         continue;
			       else
			        continue;
			      else
			       continue;
			   else
			    if(*(cache_0 + pixel[2]) > cb)
			     if(*(cache_0 + pixel[15]) > cb)
			      if(*(cache_0 + pixel[3]) > cb)
			       if(*(cache_0 + pixel[14]) > cb)
			        if(*(cache_0 + pixel[5]) > cb)
			         if(*(cache_0 + pixel[1]) > cb)
			          if(*(cache_0 + 3) > cb)
			           goto success;
			          else
			           if(*(cache_0 + -3) > cb)
			            if(*(cache_0 + pixel[13]) > cb)
			             goto success;
			            else
			             continue;
			           else
			            continue;
			         else
			          continue;
			        else if(*(cache_0 + pixel[5]) < c_b)
			         if(*(cache_0 + pixel[1]) > cb)
			          if(*(cache_0 + pixel[13]) > cb)
			           if(*(cache_0 + 3) > cb)
			            goto success;
			           else
			            if(*(cache_0 + -3) > cb)
			             goto success;
			            else
			             continue;
			          else
			           continue;
			         else
			          continue;
			        else
			         if(*(cache_0 + -3) > cb)
			          if(*(cache_0 + pixel[13]) > cb)
			           if(*(cache_0 + pixel[1]) > cb)
			            goto success;
			           else
			            continue;
			          else
			           continue;
			         else if(*(cache_0 + -3) < c_b)
			          if(*(cache_0 + pixel[13]) > cb)
			           if(*(cache_0 + pixel[1]) > cb)
			            if(*(cache_0 + 3) > cb)
			             goto success;
			            else
			             continue;
			           else
			            continue;
			          else
			           continue;
			         else
			          if(*(cache_0 + 3) > cb)
			           if(*(cache_0 + pixel[13]) > cb)
			            if(*(cache_0 + pixel[1]) > cb)
			             goto success;
			            else
			             continue;
			           else
			            continue;
			          else
			           continue;
			       else if(*(cache_0 + pixel[14]) < c_b)
			        if(*(cache_0 + pixel[6]) > cb)
			         if(*(cache_0 + pixel[1]) > cb)
			          if(*(cache_0 + 3) > cb)
			           if(*(cache_0 + pixel[5]) > cb)
			            goto success;
			           else
			            continue;
			          else
			           continue;
			         else
			          continue;
			        else
			         continue;
			       else
			        if(*(cache_0 + pixel[6]) > cb)
			         if(*(cache_0 + 3) > cb)
			          if(*(cache_0 + pixel[5]) > cb)
			           if(*(cache_0 + pixel[1]) > cb)
			            goto success;
			           else
			            continue;
			          else
			           continue;
			         else
			          continue;
			        else
			         continue;
			      else if(*(cache_0 + pixel[3]) < c_b)
			       if(*(cache_0 + pixel[11]) > cb)
			        if(*(cache_0 + pixel[1]) > cb)
			         if(*(cache_0 + -3) > cb)
			          if(*(cache_0 + pixel[13]) > cb)
			           if(*(cache_0 + pixel[14]) > cb)
			            goto success;
			           else
			            continue;
			          else
			           continue;
			         else
			          continue;
			        else
			         continue;
			       else
			        continue;
			      else
			       if(*(cache_0 + pixel[11]) > cb)
			        if(*(cache_0 + pixel[1]) > cb)
			         if(*(cache_0 + pixel[13]) > cb)
			          if(*(cache_0 + -3) > cb)
			           if(*(cache_0 + pixel[14]) > cb)
			            goto success;
			           else
			            continue;
			          else
			           continue;
			         else
			          continue;
			        else
			         continue;
			       else
			        continue;
			     else if(*(cache_0 + pixel[15]) < c_b)
			      if(*(cache_0 + 3) > cb)
			       if(*(cache_0 + pixel[1]) > cb)
			        if(*(cache_0 + pixel[3]) > cb)
			         if(*(cache_0 + pixel[5]) > cb)
			          if(*(cache_0 + pixel[6]) > cb)
			           if(*(cache_0 + pixel[7]) > cb)
			            goto success;
			           else
			            continue;
			          else
			           continue;
			         else
			          continue;
			        else
			         continue;
			       else
			        continue;
			      else
			       continue;
			     else
			      if(*(cache_0 + pixel[7]) > cb)
			       if(*(cache_0 + pixel[3]) > cb)
			        if(*(cache_0 + pixel[1]) > cb)
			         if(*(cache_0 + 3) > cb)
			          if(*(cache_0 + pixel[5]) > cb)
			           if(*(cache_0 + pixel[6]) > cb)
			            goto success;
			           else
			            continue;
			          else
			           continue;
			         else
			          continue;
			        else
			         continue;
			       else
			        continue;
			      else
			       continue;
			    else if(*(cache_0 + pixel[2]) < c_b)
			     if(*(cache_0 + 3) < c_b)
			      if(*(cache_0 + pixel[6]) < c_b)
			       if(*(cache_0 + pixel[7]) < c_b)
			        if(*(cache_0 + pixel[3]) < c_b)
			         if(*(cache_0 + pixel[5]) < c_b)
			          goto success;
			         else
			          continue;
			        else
			         continue;
			       else
			        continue;
			      else
			       continue;
			     else
			      continue;
			    else
			     continue;
			  else
			   if(*(cache_0 + pixel[1]) > cb)
			    if(*(cache_0 + pixel[2]) > cb)
			     if(*(cache_0 + pixel[15]) > cb)
			      if(*(cache_0 + pixel[3]) > cb)
			       if(*(cache_0 + pixel[5]) > cb)
			        if(*(cache_0 + 3) > cb)
			         if(*(cache_0 + pixel[14]) > cb)
			          goto success;
			         else
			          if(*(cache_0 + pixel[6]) > cb)
			           goto success;
			          else
			           continue;
			        else
			         if(*(cache_0 + -3) > cb)
			          if(*(cache_0 + pixel[13]) > cb)
			           if(*(cache_0 + pixel[14]) > cb)
			            goto success;
			           else
			            continue;
			          else
			           continue;
			         else
			          continue;
			       else if(*(cache_0 + pixel[5]) < c_b)
			        if(*(cache_0 + pixel[13]) > cb)
			         if(*(cache_0 + pixel[14]) > cb)
			          if(*(cache_0 + 3) > cb)
			           goto success;
			          else
			           if(*(cache_0 + -3) > cb)
			            goto success;
			           else
			            continue;
			         else
			          continue;
			        else
			         continue;
			       else
			        if(*(cache_0 + -3) > cb)
			         if(*(cache_0 + pixel[13]) > cb)
			          if(*(cache_0 + pixel[14]) > cb)
			           goto success;
			          else
			           continue;
			         else
			          continue;
			        else if(*(cache_0 + -3) < c_b)
			         if(*(cache_0 + pixel[13]) > cb)
			          if(*(cache_0 + 3) > cb)
			           if(*(cache_0 + pixel[14]) > cb)
			            goto success;
			           else
			            continue;
			          else
			           continue;
			         else
			          continue;
			        else
			         if(*(cache_0 + 3) > cb)
			          if(*(cache_0 + pixel[13]) > cb)
			           if(*(cache_0 + pixel[14]) > cb)
			            goto success;
			           else
			            continue;
			          else
			           continue;
			         else
			          continue;
			      else
			       if(*(cache_0 + pixel[11]) > cb)
			        if(*(cache_0 + -3) > cb)
			         if(*(cache_0 + pixel[13]) > cb)
			          if(*(cache_0 + pixel[14]) > cb)
			           goto success;
			          else
			           continue;
			         else
			          continue;
			        else
			         continue;
			       else
			        continue;
			     else
			      if(*(cache_0 + pixel[7]) > cb)
			       if(*(cache_0 + pixel[3]) > cb)
			        if(*(cache_0 + 3) > cb)
			         if(*(cache_0 + pixel[5]) > cb)
			          if(*(cache_0 + pixel[6]) > cb)
			           goto success;
			          else
			           continue;
			         else
			          continue;
			        else
			         continue;
			       else
			        continue;
			      else
			       continue;
			    else if(*(cache_0 + pixel[2]) < c_b)
			     if(*(cache_0 + pixel[10]) > cb)
			      if(*(cache_0 + pixel[11]) > cb)
			       if(*(cache_0 + -3) > cb)
			        if(*(cache_0 + pixel[13]) > cb)
			         if(*(cache_0 + pixel[14]) > cb)
			          if(*(cache_0 + pixel[15]) > cb)
			           goto success;
			          else
			           continue;
			         else
			          continue;
			        else
			         continue;
			       else
			        continue;
			      else
			       continue;
			     else
			      continue;
			    else
			     if(*(cache_0 + pixel[10]) > cb)
			      if(*(cache_0 + pixel[13]) > cb)
			       if(*(cache_0 + pixel[11]) > cb)
			        if(*(cache_0 + pixel[15]) > cb)
			         if(*(cache_0 + -3) > cb)
			          if(*(cache_0 + pixel[14]) > cb)
			           goto success;
			          else
			           continue;
			         else
			          continue;
			        else
			         continue;
			       else
			        continue;
			      else
			       continue;
			     else
			      continue;
			   else if(*(cache_0 + pixel[1]) < c_b)
			    if(*(cache_0 + pixel[2]) < c_b)
			     if(*(cache_0 + pixel[6]) < c_b)
			      if(*(cache_0 + pixel[3]) < c_b)
			       if(*(cache_0 + 3) < c_b)
			        if(*(cache_0 + pixel[5]) < c_b)
			         if(*(cache_0 + pixel[7]) < c_b)
			          goto success;
			         else
			          continue;
			        else
			         continue;
			       else
			        continue;
			      else
			       continue;
			     else
			      continue;
			    else
			     continue;
			   else
			    continue;
			 else
			  if(*(cache_0 + pixel[2]) > cb)
			   if(*(cache_0 + 3) > cb)
			    if(*(cache_0 + pixel[15]) > cb)
			     if(*(cache_0 + pixel[6]) > cb)
			      if(*(cache_0 + pixel[3]) > cb)
			       if(*(cache_0 + pixel[1]) > cb)
			        if(*(cache_0 + pixel[5]) > cb)
			         goto success;
			        else
			         if(*(cache_0 + pixel[13]) > cb)
			          if(*(cache_0 + pixel[14]) > cb)
			           goto success;
			          else
			           continue;
			         else
			          continue;
			       else if(*(cache_0 + pixel[1]) < c_b)
			        if(*(cache_0 + pixel[9]) > cb)
			         if(*(cache_0 + pixel[10]) > cb)
			          if(*(cache_0 + pixel[11]) > cb)
			           if(*(cache_0 + -3) > cb)
			            if(*(cache_0 + pixel[13]) > cb)
			             if(*(cache_0 + pixel[14]) > cb)
			              goto success;
			             else
			              continue;
			            else
			             continue;
			           else
			            continue;
			          else
			           continue;
			         else
			          continue;
			        else
			         continue;
			       else
			        if(*(cache_0 + pixel[9]) > cb)
			         if(*(cache_0 + -3) > cb)
			          if(*(cache_0 + pixel[13]) > cb)
			           if(*(cache_0 + pixel[10]) > cb)
			            if(*(cache_0 + pixel[11]) > cb)
			             if(*(cache_0 + pixel[14]) > cb)
			              goto success;
			             else
			              continue;
			            else
			             continue;
			           else
			            continue;
			          else
			           continue;
			         else
			          continue;
			        else
			         continue;
			      else if(*(cache_0 + pixel[3]) < c_b)
			       if(*(cache_0 + pixel[11]) > cb)
			        if(*(cache_0 + -3) > cb)
			         if(*(cache_0 + pixel[13]) > cb)
			          if(*(cache_0 + pixel[14]) > cb)
			           if(*(cache_0 + pixel[1]) > cb)
			            goto success;
			           else
			            if(*(cache_0 + pixel[9]) > cb)
			             if(*(cache_0 + pixel[10]) > cb)
			              goto success;
			             else
			              continue;
			            else
			             continue;
			          else
			           continue;
			         else
			          continue;
			        else
			         continue;
			       else
			        continue;
			      else
			       if(*(cache_0 + pixel[11]) > cb)
			        if(*(cache_0 + pixel[14]) > cb)
			         if(*(cache_0 + -3) > cb)
			          if(*(cache_0 + pixel[13]) > cb)
			           if(*(cache_0 + pixel[1]) > cb)
			            goto success;
			           else
			            if(*(cache_0 + pixel[9]) > cb)
			             if(*(cache_0 + pixel[10]) > cb)
			              goto success;
			             else
			              continue;
			            else
			             continue;
			          else
			           continue;
			         else
			          continue;
			        else
			         continue;
			       else
			        continue;
			     else if(*(cache_0 + pixel[6]) < c_b)
			      if(*(cache_0 + pixel[14]) > cb)
			       if(*(cache_0 + pixel[1]) > cb)
			        if(*(cache_0 + pixel[3]) > cb)
			         if(*(cache_0 + pixel[5]) > cb)
			          goto success;
			         else
			          if(*(cache_0 + pixel[13]) > cb)
			           goto success;
			          else
			           continue;
			        else
			         if(*(cache_0 + pixel[11]) > cb)
			          if(*(cache_0 + -3) > cb)
			           if(*(cache_0 + pixel[13]) > cb)
			            goto success;
			           else
			            continue;
			          else
			           continue;
			         else
			          continue;
			       else
			        if(*(cache_0 + pixel[9]) > cb)
			         if(*(cache_0 + pixel[10]) > cb)
			          if(*(cache_0 + pixel[11]) > cb)
			           if(*(cache_0 + -3) > cb)
			            if(*(cache_0 + pixel[13]) > cb)
			             goto success;
			            else
			             continue;
			           else
			            continue;
			          else
			           continue;
			         else
			          continue;
			        else
			         continue;
			      else
			       continue;
			     else
			      if(*(cache_0 + pixel[14]) > cb)
			       if(*(cache_0 + pixel[1]) > cb)
			        if(*(cache_0 + pixel[5]) > cb)
			         if(*(cache_0 + pixel[3]) > cb)
			          goto success;
			         else
			          if(*(cache_0 + pixel[11]) > cb)
			           if(*(cache_0 + -3) > cb)
			            if(*(cache_0 + pixel[13]) > cb)
			             goto success;
			            else
			             continue;
			           else
			            continue;
			          else
			           continue;
			        else
			         if(*(cache_0 + pixel[13]) > cb)
			          if(*(cache_0 + pixel[3]) > cb)
			           goto success;
			          else
			           if(*(cache_0 + pixel[11]) > cb)
			            if(*(cache_0 + -3) > cb)
			             goto success;
			            else
			             continue;
			           else
			            continue;
			         else
			          continue;
			       else if(*(cache_0 + pixel[1]) < c_b)
			        if(*(cache_0 + pixel[9]) > cb)
			         if(*(cache_0 + pixel[10]) > cb)
			          if(*(cache_0 + pixel[11]) > cb)
			           if(*(cache_0 + -3) > cb)
			            if(*(cache_0 + pixel[13]) > cb)
			             goto success;
			            else
			             continue;
			           else
			            continue;
			          else
			           continue;
			         else
			          continue;
			        else
			         continue;
			       else
			        if(*(cache_0 + pixel[9]) > cb)
			         if(*(cache_0 + -3) > cb)
			          if(*(cache_0 + pixel[10]) > cb)
			           if(*(cache_0 + pixel[11]) > cb)
			            if(*(cache_0 + pixel[13]) > cb)
			             goto success;
			            else
			             continue;
			           else
			            continue;
			          else
			           continue;
			         else
			          continue;
			        else
			         continue;
			      else
			       continue;
			    else if(*(cache_0 + pixel[15]) < c_b)
			     if(*(cache_0 + pixel[7]) > cb)
			      if(*(cache_0 + pixel[1]) > cb)
			       if(*(cache_0 + pixel[3]) > cb)
			        if(*(cache_0 + pixel[5]) > cb)
			         if(*(cache_0 + pixel[6]) > cb)
			          goto success;
			         else
			          continue;
			        else
			         continue;
			       else
			        continue;
			      else
			       continue;
			     else
			      continue;
			    else
			     if(*(cache_0 + pixel[7]) > cb)
			      if(*(cache_0 + pixel[6]) > cb)
			       if(*(cache_0 + pixel[3]) > cb)
			        if(*(cache_0 + pixel[5]) > cb)
			         if(*(cache_0 + pixel[1]) > cb)
			          goto success;
			         else
			          continue;
			        else
			         continue;
			       else
			        continue;
			      else
			       continue;
			     else
			      continue;
			   else if(*(cache_0 + 3) < c_b)
			    if(*(cache_0 + -3) > cb)
			     if(*(cache_0 + pixel[15]) > cb)
			      if(*(cache_0 + pixel[14]) > cb)
			       if(*(cache_0 + pixel[13]) > cb)
			        if(*(cache_0 + pixel[11]) > cb)
			         if(*(cache_0 + pixel[1]) > cb)
			          goto success;
			         else
			          if(*(cache_0 + pixel[9]) > cb)
			           if(*(cache_0 + pixel[10]) > cb)
			            goto success;
			           else
			            continue;
			          else
			           continue;
			        else if(*(cache_0 + pixel[11]) < c_b)
			         if(*(cache_0 + pixel[1]) > cb)
			          if(*(cache_0 + pixel[3]) > cb)
			           goto success;
			          else
			           continue;
			         else
			          continue;
			        else
			         if(*(cache_0 + pixel[3]) > cb)
			          if(*(cache_0 + pixel[1]) > cb)
			           goto success;
			          else
			           continue;
			         else
			          continue;
			       else
			        continue;
			      else
			       continue;
			     else
			      continue;
			    else
			     continue;
			   else
			    if(*(cache_0 + -3) > cb)
			     if(*(cache_0 + pixel[14]) > cb)
			      if(*(cache_0 + pixel[11]) > cb)
			       if(*(cache_0 + pixel[15]) > cb)
			        if(*(cache_0 + pixel[13]) > cb)
			         if(*(cache_0 + pixel[1]) > cb)
			          goto success;
			         else
			          if(*(cache_0 + pixel[9]) > cb)
			           if(*(cache_0 + pixel[10]) > cb)
			            goto success;
			           else
			            continue;
			          else
			           continue;
			        else
			         continue;
			       else
			        continue;
			      else if(*(cache_0 + pixel[11]) < c_b)
			       if(*(cache_0 + pixel[1]) > cb)
			        if(*(cache_0 + pixel[3]) > cb)
			         if(*(cache_0 + pixel[13]) > cb)
			          if(*(cache_0 + pixel[15]) > cb)
			           goto success;
			          else
			           continue;
			         else
			          continue;
			        else
			         continue;
			       else
			        continue;
			      else
			       if(*(cache_0 + pixel[3]) > cb)
			        if(*(cache_0 + pixel[15]) > cb)
			         if(*(cache_0 + pixel[13]) > cb)
			          if(*(cache_0 + pixel[1]) > cb)
			           goto success;
			          else
			           continue;
			         else
			          continue;
			        else
			         continue;
			       else
			        continue;
			     else
			      continue;
			    else
			     continue;
			  else if(*(cache_0 + pixel[2]) < c_b)
			   if(*(cache_0 + pixel[10]) > cb)
			    if(*(cache_0 + pixel[14]) > cb)
			     if(*(cache_0 + -3) > cb)
			      if(*(cache_0 + pixel[11]) > cb)
			       if(*(cache_0 + pixel[13]) > cb)
			        if(*(cache_0 + pixel[1]) > cb)
			         if(*(cache_0 + pixel[15]) > cb)
			          goto success;
			         else
			          continue;
			        else
			         if(*(cache_0 + pixel[9]) > cb)
			          if(*(cache_0 + pixel[15]) > cb)
			           goto success;
			          else
			           continue;
			         else
			          continue;
			       else
			        continue;
			      else
			       continue;
			     else
			      continue;
			    else
			     continue;
			   else
			    continue;
			  else
			   if(*(cache_0 + pixel[10]) > cb)
			    if(*(cache_0 + pixel[13]) > cb)
			     if(*(cache_0 + pixel[14]) > cb)
			      if(*(cache_0 + pixel[11]) > cb)
			       if(*(cache_0 + pixel[1]) > cb)
			        if(*(cache_0 + pixel[15]) > cb)
			         if(*(cache_0 + -3) > cb)
			          goto success;
			         else
			          continue;
			        else
			         continue;
			       else if(*(cache_0 + pixel[1]) < c_b)
			        if(*(cache_0 + pixel[9]) > cb)
			         if(*(cache_0 + -3) > cb)
			          if(*(cache_0 + pixel[15]) > cb)
			           goto success;
			          else
			           continue;
			         else
			          continue;
			        else
			         continue;
			       else
			        if(*(cache_0 + pixel[9]) > cb)
			         if(*(cache_0 + pixel[15]) > cb)
			          if(*(cache_0 + -3) > cb)
			           goto success;
			          else
			           continue;
			         else
			          continue;
			        else
			         continue;
			      else
			       continue;
			     else
			      continue;
			    else
			     continue;
			   else
			    continue;
			else if(*(cache_0 + pixel[0]) < c_b)
			 if(*(cache_0 + pixel[8]) > cb)
			  if(*(cache_0 + pixel[1]) > cb)
			   if(*(cache_0 + pixel[7]) > cb)
			    if(*(cache_0 + pixel[6]) > cb)
			     if(*(cache_0 + pixel[5]) > cb)
			      if(*(cache_0 + 3) > cb)
			       if(*(cache_0 + pixel[3]) > cb)
			        if(*(cache_0 + pixel[2]) > cb)
			         goto success;
			        else if(*(cache_0 + pixel[2]) < c_b)
			         if(*(cache_0 + pixel[9]) > cb)
			          if(*(cache_0 + pixel[10]) > cb)
			           goto success;
			          else
			           continue;
			         else if(*(cache_0 + pixel[9]) < c_b)
			          if(*(cache_0 + pixel[10]) < c_b)
			           if(*(cache_0 + pixel[11]) < c_b)
			            if(*(cache_0 + -3) < c_b)
			             if(*(cache_0 + pixel[13]) < c_b)
			              if(*(cache_0 + pixel[14]) < c_b)
			               if(*(cache_0 + pixel[15]) < c_b)
			                goto success;
			               else
			                continue;
			              else
			               continue;
			             else
			              continue;
			            else
			             continue;
			           else
			            continue;
			          else
			           continue;
			         else
			          continue;
			        else
			         if(*(cache_0 + pixel[10]) > cb)
			          if(*(cache_0 + pixel[9]) > cb)
			           goto success;
			          else
			           continue;
			         else if(*(cache_0 + pixel[10]) < c_b)
			          if(*(cache_0 + pixel[9]) < c_b)
			           if(*(cache_0 + pixel[11]) < c_b)
			            if(*(cache_0 + -3) < c_b)
			             if(*(cache_0 + pixel[13]) < c_b)
			              if(*(cache_0 + pixel[14]) < c_b)
			               if(*(cache_0 + pixel[15]) < c_b)
			                goto success;
			               else
			                continue;
			              else
			               continue;
			             else
			              continue;
			            else
			             continue;
			           else
			            continue;
			          else
			           continue;
			         else
			          continue;
			       else
			        if(*(cache_0 + pixel[9]) > cb)
			         if(*(cache_0 + pixel[10]) > cb)
			          if(*(cache_0 + pixel[11]) > cb)
			           goto success;
			          else
			           continue;
			         else
			          continue;
			        else if(*(cache_0 + pixel[9]) < c_b)
			         if(*(cache_0 + pixel[10]) < c_b)
			          if(*(cache_0 + pixel[11]) < c_b)
			           if(*(cache_0 + -3) < c_b)
			            if(*(cache_0 + pixel[13]) < c_b)
			             if(*(cache_0 + pixel[14]) < c_b)
			              if(*(cache_0 + pixel[15]) < c_b)
			               goto success;
			              else
			               continue;
			             else
			              continue;
			            else
			             continue;
			           else
			            continue;
			          else
			           continue;
			         else
			          continue;
			        else
			         continue;
			      else
			       if(*(cache_0 + pixel[9]) > cb)
			        if(*(cache_0 + pixel[10]) > cb)
			         if(*(cache_0 + pixel[11]) > cb)
			          if(*(cache_0 + -3) > cb)
			           goto success;
			          else
			           continue;
			         else
			          continue;
			        else
			         continue;
			       else if(*(cache_0 + pixel[9]) < c_b)
			        if(*(cache_0 + pixel[10]) < c_b)
			         if(*(cache_0 + pixel[11]) < c_b)
			          if(*(cache_0 + -3) < c_b)
			           if(*(cache_0 + pixel[13]) < c_b)
			            if(*(cache_0 + pixel[14]) < c_b)
			             if(*(cache_0 + pixel[15]) < c_b)
			              goto success;
			             else
			              continue;
			            else
			             continue;
			           else
			            continue;
			          else
			           continue;
			         else
			          continue;
			        else
			         continue;
			       else
			        continue;
			     else if(*(cache_0 + pixel[5]) < c_b)
			      if(*(cache_0 + pixel[9]) > cb)
			       if(*(cache_0 + pixel[10]) > cb)
			        if(*(cache_0 + pixel[11]) > cb)
			         if(*(cache_0 + -3) > cb)
			          if(*(cache_0 + pixel[13]) > cb)
			           goto success;
			          else
			           continue;
			         else
			          continue;
			        else
			         continue;
			       else
			        continue;
			      else if(*(cache_0 + pixel[9]) < c_b)
			       if(*(cache_0 + pixel[10]) < c_b)
			        if(*(cache_0 + pixel[11]) < c_b)
			         if(*(cache_0 + -3) < c_b)
			          if(*(cache_0 + pixel[13]) < c_b)
			           if(*(cache_0 + pixel[14]) < c_b)
			            if(*(cache_0 + pixel[15]) < c_b)
			             goto success;
			            else
			             continue;
			           else
			            continue;
			          else
			           continue;
			         else
			          continue;
			        else
			         continue;
			       else
			        continue;
			      else
			       continue;
			     else
			      if(*(cache_0 + -3) > cb)
			       if(*(cache_0 + pixel[9]) > cb)
			        if(*(cache_0 + pixel[10]) > cb)
			         if(*(cache_0 + pixel[11]) > cb)
			          if(*(cache_0 + pixel[13]) > cb)
			           goto success;
			          else
			           continue;
			         else
			          continue;
			        else
			         continue;
			       else
			        continue;
			      else if(*(cache_0 + -3) < c_b)
			       if(*(cache_0 + pixel[9]) < c_b)
			        if(*(cache_0 + pixel[10]) < c_b)
			         if(*(cache_0 + pixel[11]) < c_b)
			          if(*(cache_0 + pixel[13]) < c_b)
			           if(*(cache_0 + pixel[14]) < c_b)
			            if(*(cache_0 + pixel[15]) < c_b)
			             goto success;
			            else
			             continue;
			           else
			            continue;
			          else
			           continue;
			         else
			          continue;
			        else
			         continue;
			       else
			        continue;
			      else
			       continue;
			    else
			     if(*(cache_0 + pixel[9]) > cb)
			      if(*(cache_0 + pixel[10]) > cb)
			       if(*(cache_0 + pixel[11]) > cb)
			        if(*(cache_0 + -3) > cb)
			         if(*(cache_0 + pixel[13]) > cb)
			          if(*(cache_0 + pixel[14]) > cb)
			           goto success;
			          else
			           continue;
			         else
			          continue;
			        else
			         continue;
			       else
			        continue;
			      else
			       continue;
			     else if(*(cache_0 + pixel[9]) < c_b)
			      if(*(cache_0 + pixel[10]) < c_b)
			       if(*(cache_0 + pixel[11]) < c_b)
			        if(*(cache_0 + -3) < c_b)
			         if(*(cache_0 + pixel[13]) < c_b)
			          if(*(cache_0 + pixel[14]) < c_b)
			           if(*(cache_0 + pixel[15]) < c_b)
			            goto success;
			           else
			            continue;
			          else
			           continue;
			         else
			          continue;
			        else
			         continue;
			       else
			        continue;
			      else
			       continue;
			     else
			      continue;
			   else
			    if(*(cache_0 + pixel[9]) > cb)
			     if(*(cache_0 + pixel[10]) > cb)
			      if(*(cache_0 + pixel[11]) > cb)
			       if(*(cache_0 + -3) > cb)
			        if(*(cache_0 + pixel[13]) > cb)
			         if(*(cache_0 + pixel[14]) > cb)
			          if(*(cache_0 + pixel[15]) > cb)
			           goto success;
			          else
			           continue;
			         else
			          continue;
			        else
			         continue;
			       else
			        continue;
			      else
			       continue;
			     else
			      continue;
			    else if(*(cache_0 + pixel[9]) < c_b)
			     if(*(cache_0 + pixel[10]) < c_b)
			      if(*(cache_0 + pixel[11]) < c_b)
			       if(*(cache_0 + -3) < c_b)
			        if(*(cache_0 + pixel[13]) < c_b)
			         if(*(cache_0 + pixel[14]) < c_b)
			          if(*(cache_0 + pixel[15]) < c_b)
			           goto success;
			          else
			           continue;
			         else
			          continue;
			        else
			         continue;
			       else
			        continue;
			      else
			       continue;
			     else
			      continue;
			    else
			     continue;
			  else if(*(cache_0 + pixel[1]) < c_b)
			   if(*(cache_0 + pixel[15]) > cb)
			    if(*(cache_0 + pixel[9]) > cb)
			     if(*(cache_0 + pixel[10]) > cb)
			      if(*(cache_0 + pixel[11]) > cb)
			       if(*(cache_0 + -3) > cb)
			        if(*(cache_0 + pixel[13]) > cb)
			         if(*(cache_0 + pixel[14]) > cb)
			          goto success;
			         else
			          if(*(cache_0 + pixel[6]) > cb)
			           if(*(cache_0 + pixel[7]) > cb)
			            goto success;
			           else
			            continue;
			          else if(*(cache_0 + pixel[6]) < c_b)
			           if(*(cache_0 + pixel[2]) < c_b)
			            if(*(cache_0 + pixel[3]) < c_b)
			             if(*(cache_0 + 3) < c_b)
			              if(*(cache_0 + pixel[5]) < c_b)
			               if(*(cache_0 + pixel[7]) < c_b)
			                goto success;
			               else
			                continue;
			              else
			               continue;
			             else
			              continue;
			            else
			             continue;
			           else
			            continue;
			          else
			           continue;
			        else
			         if(*(cache_0 + pixel[5]) > cb)
			          if(*(cache_0 + pixel[6]) > cb)
			           if(*(cache_0 + pixel[7]) > cb)
			            goto success;
			           else
			            continue;
			          else
			           continue;
			         else if(*(cache_0 + pixel[5]) < c_b)
			          if(*(cache_0 + pixel[2]) < c_b)
			           if(*(cache_0 + pixel[3]) < c_b)
			            if(*(cache_0 + 3) < c_b)
			             if(*(cache_0 + pixel[6]) < c_b)
			              if(*(cache_0 + pixel[7]) < c_b)
			               goto success;
			              else
			               continue;
			             else
			              continue;
			            else
			             continue;
			           else
			            continue;
			          else
			           continue;
			         else
			          continue;
			       else
			        if(*(cache_0 + 3) > cb)
			         if(*(cache_0 + pixel[5]) > cb)
			          if(*(cache_0 + pixel[6]) > cb)
			           if(*(cache_0 + pixel[7]) > cb)
			            goto success;
			           else
			            continue;
			          else
			           continue;
			         else
			          continue;
			        else if(*(cache_0 + 3) < c_b)
			         if(*(cache_0 + pixel[2]) < c_b)
			          if(*(cache_0 + pixel[3]) < c_b)
			           if(*(cache_0 + pixel[5]) < c_b)
			            if(*(cache_0 + pixel[6]) < c_b)
			             if(*(cache_0 + pixel[7]) < c_b)
			              goto success;
			             else
			              continue;
			            else
			             continue;
			           else
			            continue;
			          else
			           continue;
			         else
			          continue;
			        else
			         continue;
			      else
			       if(*(cache_0 + pixel[3]) > cb)
			        if(*(cache_0 + 3) > cb)
			         if(*(cache_0 + pixel[5]) > cb)
			          if(*(cache_0 + pixel[6]) > cb)
			           if(*(cache_0 + pixel[7]) > cb)
			            goto success;
			           else
			            continue;
			          else
			           continue;
			         else
			          continue;
			        else
			         continue;
			       else if(*(cache_0 + pixel[3]) < c_b)
			        if(*(cache_0 + pixel[2]) < c_b)
			         if(*(cache_0 + 3) < c_b)
			          if(*(cache_0 + pixel[5]) < c_b)
			           if(*(cache_0 + pixel[6]) < c_b)
			            if(*(cache_0 + pixel[7]) < c_b)
			             goto success;
			            else
			             continue;
			           else
			            continue;
			          else
			           continue;
			         else
			          continue;
			        else
			         continue;
			       else
			        continue;
			     else
			      if(*(cache_0 + pixel[2]) > cb)
			       if(*(cache_0 + pixel[3]) > cb)
			        if(*(cache_0 + 3) > cb)
			         if(*(cache_0 + pixel[5]) > cb)
			          if(*(cache_0 + pixel[6]) > cb)
			           if(*(cache_0 + pixel[7]) > cb)
			            goto success;
			           else
			            continue;
			          else
			           continue;
			         else
			          continue;
			        else
			         continue;
			       else
			        continue;
			      else if(*(cache_0 + pixel[2]) < c_b)
			       if(*(cache_0 + pixel[3]) < c_b)
			        if(*(cache_0 + 3) < c_b)
			         if(*(cache_0 + pixel[5]) < c_b)
			          if(*(cache_0 + pixel[6]) < c_b)
			           if(*(cache_0 + pixel[7]) < c_b)
			            goto success;
			           else
			            continue;
			          else
			           continue;
			         else
			          continue;
			        else
			         continue;
			       else
			        continue;
			      else
			       continue;
			    else
			     if(*(cache_0 + pixel[2]) < c_b)
			      if(*(cache_0 + pixel[3]) < c_b)
			       if(*(cache_0 + 3) < c_b)
			        if(*(cache_0 + pixel[5]) < c_b)
			         if(*(cache_0 + pixel[6]) < c_b)
			          if(*(cache_0 + pixel[7]) < c_b)
			           goto success;
			          else
			           continue;
			         else
			          continue;
			        else
			         continue;
			       else
			        continue;
			      else
			       continue;
			     else
			      continue;
			   else if(*(cache_0 + pixel[15]) < c_b)
			    if(*(cache_0 + pixel[2]) > cb)
			     if(*(cache_0 + pixel[7]) > cb)
			      if(*(cache_0 + pixel[9]) > cb)
			       if(*(cache_0 + pixel[6]) > cb)
			        if(*(cache_0 + pixel[5]) > cb)
			         if(*(cache_0 + pixel[3]) > cb)
			          if(*(cache_0 + 3) > cb)
			           goto success;
			          else
			           if(*(cache_0 + pixel[10]) > cb)
			            if(*(cache_0 + pixel[11]) > cb)
			             if(*(cache_0 + -3) > cb)
			              goto success;
			             else
			              continue;
			            else
			             continue;
			           else if(*(cache_0 + pixel[10]) < c_b)
			            if(*(cache_0 + pixel[11]) < c_b)
			             if(*(cache_0 + -3) < c_b)
			              if(*(cache_0 + pixel[13]) < c_b)
			               if(*(cache_0 + pixel[14]) < c_b)
			                goto success;
			               else
			                continue;
			              else
			               continue;
			             else
			              continue;
			            else
			             continue;
			           else
			            continue;
			         else if(*(cache_0 + pixel[3]) < c_b)
			          if(*(cache_0 + pixel[10]) > cb)
			           if(*(cache_0 + pixel[11]) > cb)
			            if(*(cache_0 + 3) > cb)
			             goto success;
			            else
			             if(*(cache_0 + -3) > cb)
			              goto success;
			             else
			              continue;
			           else
			            continue;
			          else if(*(cache_0 + pixel[10]) < c_b)
			           if(*(cache_0 + pixel[11]) < c_b)
			            if(*(cache_0 + -3) < c_b)
			             if(*(cache_0 + pixel[13]) < c_b)
			              if(*(cache_0 + pixel[14]) < c_b)
			               goto success;
			              else
			               continue;
			             else
			              continue;
			            else
			             continue;
			           else
			            continue;
			          else
			           continue;
			         else
			          if(*(cache_0 + pixel[11]) > cb)
			           if(*(cache_0 + pixel[10]) > cb)
			            if(*(cache_0 + 3) > cb)
			             goto success;
			            else
			             if(*(cache_0 + -3) > cb)
			              goto success;
			             else
			              continue;
			           else
			            continue;
			          else if(*(cache_0 + pixel[11]) < c_b)
			           if(*(cache_0 + pixel[10]) < c_b)
			            if(*(cache_0 + -3) < c_b)
			             if(*(cache_0 + pixel[13]) < c_b)
			              if(*(cache_0 + pixel[14]) < c_b)
			               goto success;
			              else
			               continue;
			             else
			              continue;
			            else
			             continue;
			           else
			            continue;
			          else
			           continue;
			        else
			         if(*(cache_0 + pixel[10]) > cb)
			          if(*(cache_0 + pixel[11]) > cb)
			           if(*(cache_0 + -3) > cb)
			            if(*(cache_0 + pixel[13]) > cb)
			             goto success;
			            else
			             continue;
			           else
			            continue;
			          else
			           continue;
			         else if(*(cache_0 + pixel[10]) < c_b)
			          if(*(cache_0 + pixel[11]) < c_b)
			           if(*(cache_0 + -3) < c_b)
			            if(*(cache_0 + pixel[13]) < c_b)
			             if(*(cache_0 + pixel[14]) < c_b)
			              goto success;
			             else
			              continue;
			            else
			             continue;
			           else
			            continue;
			          else
			           continue;
			         else
			          continue;
			       else if(*(cache_0 + pixel[6]) < c_b)
			        if(*(cache_0 + pixel[10]) > cb)
			         if(*(cache_0 + pixel[11]) > cb)
			          if(*(cache_0 + -3) > cb)
			           if(*(cache_0 + pixel[13]) > cb)
			            if(*(cache_0 + pixel[14]) > cb)
			             goto success;
			            else
			             continue;
			           else
			            continue;
			          else
			           continue;
			         else
			          continue;
			        else if(*(cache_0 + pixel[10]) < c_b)
			         if(*(cache_0 + pixel[11]) < c_b)
			          if(*(cache_0 + -3) < c_b)
			           if(*(cache_0 + pixel[13]) < c_b)
			            if(*(cache_0 + pixel[14]) < c_b)
			             goto success;
			            else
			             continue;
			           else
			            continue;
			          else
			           continue;
			         else
			          continue;
			        else
			         continue;
			       else
			        if(*(cache_0 + pixel[11]) > cb)
			         if(*(cache_0 + pixel[10]) > cb)
			          if(*(cache_0 + -3) > cb)
			           if(*(cache_0 + pixel[13]) > cb)
			            if(*(cache_0 + pixel[14]) > cb)
			             goto success;
			            else
			             continue;
			           else
			            continue;
			          else
			           continue;
			         else
			          continue;
			        else if(*(cache_0 + pixel[11]) < c_b)
			         if(*(cache_0 + pixel[10]) < c_b)
			          if(*(cache_0 + -3) < c_b)
			           if(*(cache_0 + pixel[13]) < c_b)
			            if(*(cache_0 + pixel[14]) < c_b)
			             goto success;
			            else
			             continue;
			           else
			            continue;
			          else
			           continue;
			         else
			          continue;
			        else
			         continue;
			      else
			       if(*(cache_0 + pixel[10]) < c_b)
			        if(*(cache_0 + pixel[11]) < c_b)
			         if(*(cache_0 + -3) < c_b)
			          if(*(cache_0 + pixel[13]) < c_b)
			           if(*(cache_0 + pixel[14]) < c_b)
			            goto success;
			           else
			            continue;
			          else
			           continue;
			         else
			          continue;
			        else
			         continue;
			       else
			        continue;
			     else
			      if(*(cache_0 + pixel[10]) < c_b)
			       if(*(cache_0 + pixel[11]) < c_b)
			        if(*(cache_0 + -3) < c_b)
			         if(*(cache_0 + pixel[13]) < c_b)
			          if(*(cache_0 + pixel[14]) < c_b)
			           goto success;
			          else
			           continue;
			         else
			          continue;
			        else
			         continue;
			       else
			        continue;
			      else
			       continue;
			    else if(*(cache_0 + pixel[2]) < c_b)
			     if(*(cache_0 + pixel[3]) > cb)
			      if(*(cache_0 + pixel[13]) > cb)
			       if(*(cache_0 + pixel[10]) > cb)
			        if(*(cache_0 + pixel[7]) > cb)
			         if(*(cache_0 + pixel[9]) > cb)
			          if(*(cache_0 + pixel[5]) > cb)
			           if(*(cache_0 + pixel[6]) > cb)
			            if(*(cache_0 + 3) > cb)
			             goto success;
			            else
			             if(*(cache_0 + pixel[11]) > cb)
			              if(*(cache_0 + -3) > cb)
			               goto success;
			              else
			               continue;
			             else
			              continue;
			           else
			            if(*(cache_0 + pixel[11]) > cb)
			             if(*(cache_0 + -3) > cb)
			              if(*(cache_0 + pixel[14]) > cb)
			               goto success;
			              else
			               continue;
			             else
			              continue;
			            else
			             continue;
			          else
			           if(*(cache_0 + pixel[11]) > cb)
			            if(*(cache_0 + -3) > cb)
			             if(*(cache_0 + pixel[14]) > cb)
			              goto success;
			             else
			              if(*(cache_0 + pixel[6]) > cb)
			               goto success;
			              else
			               continue;
			            else
			             continue;
			           else
			            continue;
			         else
			          continue;
			        else
			         continue;
			       else
			        continue;
			      else if(*(cache_0 + pixel[13]) < c_b)
			       if(*(cache_0 + pixel[11]) > cb)
			        if(*(cache_0 + pixel[5]) > cb)
			         if(*(cache_0 + pixel[6]) > cb)
			          if(*(cache_0 + pixel[7]) > cb)
			           if(*(cache_0 + pixel[9]) > cb)
			            if(*(cache_0 + pixel[10]) > cb)
			             if(*(cache_0 + -3) > cb)
			              goto success;
			             else
			              if(*(cache_0 + 3) > cb)
			               goto success;
			              else
			               continue;
			            else
			             continue;
			           else
			            continue;
			          else
			           continue;
			         else
			          continue;
			        else
			         continue;
			       else if(*(cache_0 + pixel[11]) < c_b)
			        if(*(cache_0 + pixel[14]) > cb)
			         if(*(cache_0 + 3) > cb)
			          if(*(cache_0 + pixel[5]) > cb)
			           if(*(cache_0 + pixel[6]) > cb)
			            if(*(cache_0 + pixel[7]) > cb)
			             if(*(cache_0 + pixel[9]) > cb)
			              if(*(cache_0 + pixel[10]) > cb)
			               goto success;
			              else
			               continue;
			             else
			              continue;
			            else
			             continue;
			           else
			            continue;
			          else
			           continue;
			         else
			          continue;
			        else if(*(cache_0 + pixel[14]) < c_b)
			         if(*(cache_0 + -3) < c_b)
			          goto success;
			         else
			          if(*(cache_0 + 3) > cb)
			           if(*(cache_0 + pixel[5]) > cb)
			            if(*(cache_0 + pixel[6]) > cb)
			             if(*(cache_0 + pixel[7]) > cb)
			              if(*(cache_0 + pixel[9]) > cb)
			               if(*(cache_0 + pixel[10]) > cb)
			                goto success;
			               else
			                continue;
			              else
			               continue;
			             else
			              continue;
			            else
			             continue;
			           else
			            continue;
			          else
			           continue;
			        else
			         if(*(cache_0 + pixel[6]) > cb)
			          if(*(cache_0 + 3) > cb)
			           if(*(cache_0 + pixel[5]) > cb)
			            if(*(cache_0 + pixel[7]) > cb)
			             if(*(cache_0 + pixel[9]) > cb)
			              if(*(cache_0 + pixel[10]) > cb)
			               goto success;
			              else
			               continue;
			             else
			              continue;
			            else
			             continue;
			           else
			            continue;
			          else
			           continue;
			         else
			          continue;
			       else
			        if(*(cache_0 + pixel[10]) > cb)
			         if(*(cache_0 + pixel[7]) > cb)
			          if(*(cache_0 + 3) > cb)
			           if(*(cache_0 + pixel[5]) > cb)
			            if(*(cache_0 + pixel[6]) > cb)
			             if(*(cache_0 + pixel[9]) > cb)
			              goto success;
			             else
			              continue;
			            else
			             continue;
			           else
			            continue;
			          else
			           continue;
			         else
			          continue;
			        else
			         continue;
			      else
			       if(*(cache_0 + pixel[10]) > cb)
			        if(*(cache_0 + pixel[5]) > cb)
			         if(*(cache_0 + pixel[6]) > cb)
			          if(*(cache_0 + pixel[7]) > cb)
			           if(*(cache_0 + pixel[9]) > cb)
			            if(*(cache_0 + 3) > cb)
			             goto success;
			            else
			             if(*(cache_0 + pixel[11]) > cb)
			              if(*(cache_0 + -3) > cb)
			               goto success;
			              else
			               continue;
			             else
			              continue;
			           else
			            continue;
			          else
			           continue;
			         else
			          continue;
			        else
			         continue;
			       else
			        continue;
			     else if(*(cache_0 + pixel[3]) < c_b)
			      if(*(cache_0 + pixel[14]) > cb)
			       if(*(cache_0 + pixel[6]) > cb)
			        if(*(cache_0 + pixel[7]) > cb)
			         if(*(cache_0 + pixel[9]) > cb)
			          if(*(cache_0 + pixel[10]) > cb)
			           if(*(cache_0 + pixel[11]) > cb)
			            if(*(cache_0 + -3) > cb)
			             if(*(cache_0 + pixel[13]) > cb)
			              goto success;
			             else
			              if(*(cache_0 + pixel[5]) > cb)
			               goto success;
			              else
			               continue;
			            else
			             if(*(cache_0 + 3) > cb)
			              if(*(cache_0 + pixel[5]) > cb)
			               goto success;
			              else
			               continue;
			             else
			              continue;
			           else
			            continue;
			          else
			           continue;
			         else
			          continue;
			        else
			         continue;
			       else if(*(cache_0 + pixel[6]) < c_b)
			        if(*(cache_0 + pixel[5]) < c_b)
			         if(*(cache_0 + 3) < c_b)
			          goto success;
			         else
			          if(*(cache_0 + pixel[7]) > cb)
			           if(*(cache_0 + pixel[9]) > cb)
			            if(*(cache_0 + pixel[10]) > cb)
			             if(*(cache_0 + pixel[11]) > cb)
			              if(*(cache_0 + -3) > cb)
			               if(*(cache_0 + pixel[13]) > cb)
			                goto success;
			               else
			                continue;
			              else
			               continue;
			             else
			              continue;
			            else
			             continue;
			           else
			            continue;
			          else
			           continue;
			        else
			         if(*(cache_0 + pixel[7]) > cb)
			          if(*(cache_0 + pixel[9]) > cb)
			           if(*(cache_0 + pixel[10]) > cb)
			            if(*(cache_0 + pixel[11]) > cb)
			             if(*(cache_0 + -3) > cb)
			              if(*(cache_0 + pixel[13]) > cb)
			               goto success;
			              else
			               continue;
			             else
			              continue;
			            else
			             continue;
			           else
			            continue;
			          else
			           continue;
			         else
			          continue;
			       else
			        if(*(cache_0 + pixel[11]) > cb)
			         if(*(cache_0 + pixel[7]) > cb)
			          if(*(cache_0 + pixel[9]) > cb)
			           if(*(cache_0 + pixel[10]) > cb)
			            if(*(cache_0 + -3) > cb)
			             if(*(cache_0 + pixel[13]) > cb)
			              goto success;
			             else
			              continue;
			            else
			             continue;
			           else
			            continue;
			          else
			           continue;
			         else
			          continue;
			        else
			         continue;
			      else if(*(cache_0 + pixel[14]) < c_b)
			       if(*(cache_0 + -3) > cb)
			        if(*(cache_0 + 3) > cb)
			         if(*(cache_0 + pixel[6]) > cb)
			          if(*(cache_0 + pixel[7]) > cb)
			           if(*(cache_0 + pixel[9]) > cb)
			            if(*(cache_0 + pixel[10]) > cb)
			             if(*(cache_0 + pixel[11]) > cb)
			              if(*(cache_0 + pixel[13]) > cb)
			               goto success;
			              else
			               if(*(cache_0 + pixel[5]) > cb)
			                goto success;
			               else
			                continue;
			             else
			              continue;
			            else
			             continue;
			           else
			            continue;
			          else
			           continue;
			         else
			          continue;
			        else if(*(cache_0 + 3) < c_b)
			         if(*(cache_0 + pixel[5]) > cb)
			          if(*(cache_0 + pixel[13]) < c_b)
			           goto success;
			          else
			           if(*(cache_0 + pixel[6]) > cb)
			            if(*(cache_0 + pixel[7]) > cb)
			             if(*(cache_0 + pixel[9]) > cb)
			              if(*(cache_0 + pixel[10]) > cb)
			               if(*(cache_0 + pixel[11]) > cb)
			                goto success;
			               else
			                continue;
			              else
			               continue;
			             else
			              continue;
			            else
			             continue;
			           else
			            continue;
			         else if(*(cache_0 + pixel[5]) < c_b)
			          goto success;
			         else
			          if(*(cache_0 + pixel[13]) > cb)
			           if(*(cache_0 + pixel[6]) > cb)
			            if(*(cache_0 + pixel[7]) > cb)
			             if(*(cache_0 + pixel[9]) > cb)
			              if(*(cache_0 + pixel[10]) > cb)
			               if(*(cache_0 + pixel[11]) > cb)
			                goto success;
			               else
			                continue;
			              else
			               continue;
			             else
			              continue;
			            else
			             continue;
			           else
			            continue;
			          else if(*(cache_0 + pixel[13]) < c_b)
			           goto success;
			          else
			           continue;
			        else
			         if(*(cache_0 + pixel[6]) > cb)
			          if(*(cache_0 + pixel[7]) > cb)
			           if(*(cache_0 + pixel[5]) > cb)
			            if(*(cache_0 + pixel[9]) > cb)
			             if(*(cache_0 + pixel[10]) > cb)
			              if(*(cache_0 + pixel[11]) > cb)
			               goto success;
			              else
			               continue;
			             else
			              continue;
			            else
			             continue;
			           else if(*(cache_0 + pixel[5]) < c_b)
			            if(*(cache_0 + pixel[9]) > cb)
			             if(*(cache_0 + pixel[10]) > cb)
			              if(*(cache_0 + pixel[11]) > cb)
			               if(*(cache_0 + pixel[13]) > cb)
			                goto success;
			               else
			                continue;
			              else
			               continue;
			             else
			              continue;
			            else
			             continue;
			           else
			            if(*(cache_0 + pixel[13]) > cb)
			             if(*(cache_0 + pixel[9]) > cb)
			              if(*(cache_0 + pixel[10]) > cb)
			               if(*(cache_0 + pixel[11]) > cb)
			                goto success;
			               else
			                continue;
			              else
			               continue;
			             else
			              continue;
			            else
			             continue;
			          else
			           continue;
			         else
			          continue;
			       else if(*(cache_0 + -3) < c_b)
			        if(*(cache_0 + pixel[13]) > cb)
			         if(*(cache_0 + 3) > cb)
			          if(*(cache_0 + pixel[5]) > cb)
			           if(*(cache_0 + pixel[6]) > cb)
			            if(*(cache_0 + pixel[7]) > cb)
			             if(*(cache_0 + pixel[9]) > cb)
			              if(*(cache_0 + pixel[10]) > cb)
			               if(*(cache_0 + pixel[11]) > cb)
			                goto success;
			               else
			                continue;
			              else
			               continue;
			             else
			              continue;
			            else
			             continue;
			           else
			            continue;
			          else
			           continue;
			         else if(*(cache_0 + 3) < c_b)
			          if(*(cache_0 + pixel[5]) < c_b)
			           goto success;
			          else
			           continue;
			         else
			          continue;
			        else if(*(cache_0 + pixel[13]) < c_b)
			         goto success;
			        else
			         if(*(cache_0 + pixel[5]) > cb)
			          if(*(cache_0 + 3) > cb)
			           if(*(cache_0 + pixel[6]) > cb)
			            if(*(cache_0 + pixel[7]) > cb)
			             if(*(cache_0 + pixel[9]) > cb)
			              if(*(cache_0 + pixel[10]) > cb)
			               if(*(cache_0 + pixel[11]) > cb)
			                goto success;
			               else
			                continue;
			              else
			               continue;
			             else
			              continue;
			            else
			             continue;
			           else
			            continue;
			          else
			           continue;
			         else if(*(cache_0 + pixel[5]) < c_b)
			          if(*(cache_0 + 3) < c_b)
			           goto success;
			          else
			           continue;
			         else
			          continue;
			       else
			        if(*(cache_0 + 3) > cb)
			         if(*(cache_0 + pixel[6]) > cb)
			          if(*(cache_0 + pixel[5]) > cb)
			           if(*(cache_0 + pixel[7]) > cb)
			            if(*(cache_0 + pixel[9]) > cb)
			             if(*(cache_0 + pixel[10]) > cb)
			              if(*(cache_0 + pixel[11]) > cb)
			               goto success;
			              else
			               continue;
			             else
			              continue;
			            else
			             continue;
			           else
			            continue;
			          else
			           continue;
			         else
			          continue;
			        else if(*(cache_0 + 3) < c_b)
			         if(*(cache_0 + pixel[13]) < c_b)
			          goto success;
			         else
			          if(*(cache_0 + pixel[5]) < c_b)
			           goto success;
			          else
			           continue;
			        else
			         continue;
			      else
			       if(*(cache_0 + pixel[6]) > cb)
			        if(*(cache_0 + pixel[11]) > cb)
			         if(*(cache_0 + pixel[7]) > cb)
			          if(*(cache_0 + pixel[9]) > cb)
			           if(*(cache_0 + pixel[10]) > cb)
			            if(*(cache_0 + pixel[5]) > cb)
			             if(*(cache_0 + -3) > cb)
			              goto success;
			             else
			              if(*(cache_0 + 3) > cb)
			               goto success;
			              else
			               continue;
			            else
			             if(*(cache_0 + -3) > cb)
			              if(*(cache_0 + pixel[13]) > cb)
			               goto success;
			              else
			               continue;
			             else
			              continue;
			           else
			            continue;
			          else
			           continue;
			         else
			          continue;
			        else
			         continue;
			       else if(*(cache_0 + pixel[6]) < c_b)
			        if(*(cache_0 + pixel[5]) < c_b)
			         if(*(cache_0 + 3) < c_b)
			          goto success;
			         else
			          continue;
			        else
			         continue;
			       else
			        continue;
			     else
			      if(*(cache_0 + pixel[11]) > cb)
			       if(*(cache_0 + pixel[6]) > cb)
			        if(*(cache_0 + pixel[5]) > cb)
			         if(*(cache_0 + pixel[7]) > cb)
			          if(*(cache_0 + -3) > cb)
			           if(*(cache_0 + pixel[9]) > cb)
			            if(*(cache_0 + pixel[10]) > cb)
			             goto success;
			            else
			             continue;
			           else
			            continue;
			          else
			           if(*(cache_0 + 3) > cb)
			            if(*(cache_0 + pixel[9]) > cb)
			             if(*(cache_0 + pixel[10]) > cb)
			              goto success;
			             else
			              continue;
			            else
			             continue;
			           else
			            continue;
			         else
			          continue;
			        else if(*(cache_0 + pixel[5]) < c_b)
			         if(*(cache_0 + pixel[7]) > cb)
			          if(*(cache_0 + pixel[9]) > cb)
			           if(*(cache_0 + pixel[10]) > cb)
			            if(*(cache_0 + -3) > cb)
			             if(*(cache_0 + pixel[13]) > cb)
			              goto success;
			             else
			              continue;
			            else
			             continue;
			           else
			            continue;
			          else
			           continue;
			         else
			          continue;
			        else
			         if(*(cache_0 + pixel[13]) > cb)
			          if(*(cache_0 + pixel[9]) > cb)
			           if(*(cache_0 + pixel[7]) > cb)
			            if(*(cache_0 + pixel[10]) > cb)
			             if(*(cache_0 + -3) > cb)
			              goto success;
			             else
			              continue;
			            else
			             continue;
			           else
			            continue;
			          else
			           continue;
			         else
			          continue;
			       else
			        if(*(cache_0 + pixel[14]) > cb)
			         if(*(cache_0 + pixel[7]) > cb)
			          if(*(cache_0 + pixel[9]) > cb)
			           if(*(cache_0 + pixel[10]) > cb)
			            if(*(cache_0 + -3) > cb)
			             if(*(cache_0 + pixel[13]) > cb)
			              goto success;
			             else
			              continue;
			            else
			             continue;
			           else
			            continue;
			          else
			           continue;
			         else
			          continue;
			        else
			         continue;
			      else if(*(cache_0 + pixel[11]) < c_b)
			       if(*(cache_0 + -3) < c_b)
			        if(*(cache_0 + pixel[14]) < c_b)
			         if(*(cache_0 + pixel[13]) < c_b)
			          goto success;
			         else
			          continue;
			        else
			         continue;
			       else
			        continue;
			      else
			       continue;
			    else
			     if(*(cache_0 + pixel[10]) > cb)
			      if(*(cache_0 + pixel[5]) > cb)
			       if(*(cache_0 + pixel[7]) > cb)
			        if(*(cache_0 + 3) > cb)
			         if(*(cache_0 + pixel[9]) > cb)
			          if(*(cache_0 + pixel[3]) > cb)
			           if(*(cache_0 + pixel[6]) > cb)
			            goto success;
			           else if(*(cache_0 + pixel[6]) < c_b)
			            if(*(cache_0 + pixel[11]) > cb)
			             if(*(cache_0 + -3) > cb)
			              if(*(cache_0 + pixel[13]) > cb)
			               if(*(cache_0 + pixel[14]) > cb)
			                goto success;
			               else
			                continue;
			              else
			               continue;
			             else
			              continue;
			            else
			             continue;
			           else
			            if(*(cache_0 + -3) > cb)
			             if(*(cache_0 + pixel[11]) > cb)
			              if(*(cache_0 + pixel[13]) > cb)
			               if(*(cache_0 + pixel[14]) > cb)
			                goto success;
			               else
			                continue;
			              else
			               continue;
			             else
			              continue;
			            else
			             continue;
			          else
			           if(*(cache_0 + pixel[11]) > cb)
			            if(*(cache_0 + pixel[6]) > cb)
			             goto success;
			            else
			             if(*(cache_0 + -3) > cb)
			              if(*(cache_0 + pixel[13]) > cb)
			               if(*(cache_0 + pixel[14]) > cb)
			                goto success;
			               else
			                continue;
			              else
			               continue;
			             else
			              continue;
			           else
			            continue;
			         else
			          continue;
			        else if(*(cache_0 + 3) < c_b)
			         if(*(cache_0 + pixel[9]) > cb)
			          if(*(cache_0 + pixel[11]) > cb)
			           if(*(cache_0 + -3) > cb)
			            if(*(cache_0 + pixel[6]) > cb)
			             goto success;
			            else
			             if(*(cache_0 + pixel[13]) > cb)
			              if(*(cache_0 + pixel[14]) > cb)
			               goto success;
			              else
			               continue;
			             else
			              continue;
			           else
			            continue;
			          else
			           continue;
			         else
			          continue;
			        else
			         if(*(cache_0 + -3) > cb)
			          if(*(cache_0 + pixel[11]) > cb)
			           if(*(cache_0 + pixel[9]) > cb)
			            if(*(cache_0 + pixel[6]) > cb)
			             goto success;
			            else
			             if(*(cache_0 + pixel[13]) > cb)
			              if(*(cache_0 + pixel[14]) > cb)
			               goto success;
			              else
			               continue;
			             else
			              continue;
			           else
			            continue;
			          else
			           continue;
			         else
			          continue;
			       else
			        continue;
			      else if(*(cache_0 + pixel[5]) < c_b)
			       if(*(cache_0 + pixel[14]) > cb)
			        if(*(cache_0 + pixel[7]) > cb)
			         if(*(cache_0 + pixel[9]) > cb)
			          if(*(cache_0 + pixel[11]) > cb)
			           if(*(cache_0 + -3) > cb)
			            if(*(cache_0 + pixel[13]) > cb)
			             goto success;
			            else
			             continue;
			           else
			            continue;
			          else
			           continue;
			         else
			          continue;
			        else
			         continue;
			       else
			        if(*(cache_0 + pixel[6]) > cb)
			         if(*(cache_0 + pixel[7]) > cb)
			          if(*(cache_0 + pixel[9]) > cb)
			           if(*(cache_0 + pixel[11]) > cb)
			            if(*(cache_0 + -3) > cb)
			             if(*(cache_0 + pixel[13]) > cb)
			              goto success;
			             else
			              continue;
			            else
			             continue;
			           else
			            continue;
			          else
			           continue;
			         else
			          continue;
			        else
			         continue;
			      else
			       if(*(cache_0 + pixel[13]) > cb)
			        if(*(cache_0 + pixel[7]) > cb)
			         if(*(cache_0 + pixel[14]) > cb)
			          if(*(cache_0 + pixel[9]) > cb)
			           if(*(cache_0 + pixel[11]) > cb)
			            if(*(cache_0 + -3) > cb)
			             goto success;
			            else
			             continue;
			           else
			            continue;
			          else
			           continue;
			         else if(*(cache_0 + pixel[14]) < c_b)
			          if(*(cache_0 + pixel[6]) > cb)
			           if(*(cache_0 + pixel[9]) > cb)
			            if(*(cache_0 + pixel[11]) > cb)
			             if(*(cache_0 + -3) > cb)
			              goto success;
			             else
			              continue;
			            else
			             continue;
			           else
			            continue;
			          else
			           continue;
			         else
			          if(*(cache_0 + pixel[6]) > cb)
			           if(*(cache_0 + pixel[11]) > cb)
			            if(*(cache_0 + pixel[9]) > cb)
			             if(*(cache_0 + -3) > cb)
			              goto success;
			             else
			              continue;
			            else
			             continue;
			           else
			            continue;
			          else
			           continue;
			        else
			         continue;
			       else
			        continue;
			     else if(*(cache_0 + pixel[10]) < c_b)
			      if(*(cache_0 + pixel[11]) < c_b)
			       if(*(cache_0 + pixel[13]) < c_b)
			        if(*(cache_0 + pixel[14]) < c_b)
			         if(*(cache_0 + -3) < c_b)
			          goto success;
			         else
			          continue;
			        else
			         continue;
			       else
			        continue;
			      else
			       continue;
			     else
			      continue;
			   else
			    if(*(cache_0 + pixel[7]) > cb)
			     if(*(cache_0 + pixel[9]) > cb)
			      if(*(cache_0 + pixel[11]) > cb)
			       if(*(cache_0 + pixel[13]) > cb)
			        if(*(cache_0 + pixel[10]) > cb)
			         if(*(cache_0 + pixel[14]) > cb)
			          if(*(cache_0 + -3) > cb)
			           goto success;
			          else
			           if(*(cache_0 + 3) > cb)
			            if(*(cache_0 + pixel[5]) > cb)
			             if(*(cache_0 + pixel[6]) > cb)
			              goto success;
			             else
			              continue;
			            else
			             continue;
			           else
			            continue;
			         else
			          if(*(cache_0 + pixel[6]) > cb)
			           if(*(cache_0 + -3) > cb)
			            goto success;
			           else
			            if(*(cache_0 + 3) > cb)
			             if(*(cache_0 + pixel[5]) > cb)
			              goto success;
			             else
			              continue;
			            else
			             continue;
			          else
			           continue;
			        else
			         if(*(cache_0 + pixel[2]) > cb)
			          if(*(cache_0 + pixel[3]) > cb)
			           if(*(cache_0 + 3) > cb)
			            if(*(cache_0 + pixel[5]) > cb)
			             if(*(cache_0 + pixel[6]) > cb)
			              goto success;
			             else
			              continue;
			            else
			             continue;
			           else
			            continue;
			          else
			           continue;
			         else
			          continue;
			       else if(*(cache_0 + pixel[13]) < c_b)
			        if(*(cache_0 + pixel[5]) > cb)
			         if(*(cache_0 + pixel[6]) > cb)
			          if(*(cache_0 + pixel[10]) > cb)
			           if(*(cache_0 + -3) > cb)
			            goto success;
			           else
			            if(*(cache_0 + 3) > cb)
			             goto success;
			            else
			             continue;
			          else
			           if(*(cache_0 + pixel[2]) > cb)
			            if(*(cache_0 + pixel[3]) > cb)
			             if(*(cache_0 + 3) > cb)
			              goto success;
			             else
			              continue;
			            else
			             continue;
			           else
			            continue;
			         else
			          continue;
			        else
			         continue;
			       else
			        if(*(cache_0 + pixel[5]) > cb)
			         if(*(cache_0 + pixel[6]) > cb)
			          if(*(cache_0 + 3) > cb)
			           if(*(cache_0 + pixel[10]) > cb)
			            goto success;
			           else
			            if(*(cache_0 + pixel[2]) > cb)
			             if(*(cache_0 + pixel[3]) > cb)
			              goto success;
			             else
			              continue;
			            else
			             continue;
			          else if(*(cache_0 + 3) < c_b)
			           if(*(cache_0 + pixel[10]) > cb)
			            if(*(cache_0 + -3) > cb)
			             goto success;
			            else
			             continue;
			           else
			            continue;
			          else
			           if(*(cache_0 + -3) > cb)
			            if(*(cache_0 + pixel[10]) > cb)
			             goto success;
			            else
			             continue;
			           else
			            continue;
			         else
			          continue;
			        else
			         continue;
			      else
			       if(*(cache_0 + pixel[3]) > cb)
			        if(*(cache_0 + 3) > cb)
			         if(*(cache_0 + pixel[5]) > cb)
			          if(*(cache_0 + pixel[6]) > cb)
			           if(*(cache_0 + pixel[2]) > cb)
			            goto success;
			           else
			            if(*(cache_0 + pixel[10]) > cb)
			             goto success;
			            else
			             continue;
			          else
			           continue;
			         else
			          continue;
			        else
			         continue;
			       else
			        continue;
			     else
			      continue;
			    else if(*(cache_0 + pixel[7]) < c_b)
			     if(*(cache_0 + pixel[3]) < c_b)
			      if(*(cache_0 + pixel[2]) < c_b)
			       if(*(cache_0 + 3) < c_b)
			        if(*(cache_0 + pixel[5]) < c_b)
			         if(*(cache_0 + pixel[6]) < c_b)
			          goto success;
			         else
			          continue;
			        else
			         continue;
			       else
			        continue;
			      else
			       continue;
			     else
			      continue;
			    else
			     continue;
			  else
			   if(*(cache_0 + pixel[9]) > cb)
			    if(*(cache_0 + pixel[7]) > cb)
			     if(*(cache_0 + pixel[10]) > cb)
			      if(*(cache_0 + pixel[11]) > cb)
			       if(*(cache_0 + pixel[6]) > cb)
			        if(*(cache_0 + pixel[13]) > cb)
			         if(*(cache_0 + -3) > cb)
			          goto success;
			         else
			          if(*(cache_0 + 3) > cb)
			           if(*(cache_0 + pixel[5]) > cb)
			            goto success;
			           else
			            continue;
			          else
			           continue;
			        else
			         if(*(cache_0 + pixel[5]) > cb)
			          if(*(cache_0 + 3) > cb)
			           goto success;
			          else
			           if(*(cache_0 + -3) > cb)
			            goto success;
			           else
			            continue;
			         else
			          continue;
			       else
			        if(*(cache_0 + pixel[14]) > cb)
			         if(*(cache_0 + -3) > cb)
			          if(*(cache_0 + pixel[13]) > cb)
			           goto success;
			          else
			           continue;
			         else
			          continue;
			        else
			         continue;
			      else if(*(cache_0 + pixel[11]) < c_b)
			       if(*(cache_0 + pixel[3]) > cb)
			        if(*(cache_0 + 3) > cb)
			         if(*(cache_0 + pixel[5]) > cb)
			          if(*(cache_0 + pixel[6]) > cb)
			           goto success;
			          else
			           continue;
			         else
			          continue;
			        else
			         continue;
			       else
			        continue;
			      else
			       if(*(cache_0 + pixel[3]) > cb)
			        if(*(cache_0 + pixel[5]) > cb)
			         if(*(cache_0 + 3) > cb)
			          if(*(cache_0 + pixel[6]) > cb)
			           goto success;
			          else
			           continue;
			         else
			          continue;
			        else
			         continue;
			       else
			        continue;
			     else if(*(cache_0 + pixel[10]) < c_b)
			      if(*(cache_0 + 3) > cb)
			       if(*(cache_0 + pixel[2]) > cb)
			        if(*(cache_0 + pixel[3]) > cb)
			         if(*(cache_0 + pixel[5]) > cb)
			          if(*(cache_0 + pixel[6]) > cb)
			           goto success;
			          else
			           continue;
			         else
			          continue;
			        else
			         continue;
			       else
			        continue;
			      else
			       continue;
			     else
			      if(*(cache_0 + pixel[2]) > cb)
			       if(*(cache_0 + pixel[5]) > cb)
			        if(*(cache_0 + 3) > cb)
			         if(*(cache_0 + pixel[3]) > cb)
			          if(*(cache_0 + pixel[6]) > cb)
			           goto success;
			          else
			           continue;
			         else
			          continue;
			        else
			         continue;
			       else
			        continue;
			      else
			       continue;
			    else if(*(cache_0 + pixel[7]) < c_b)
			     if(*(cache_0 + pixel[14]) > cb)
			      if(*(cache_0 + pixel[10]) > cb)
			       if(*(cache_0 + pixel[11]) > cb)
			        if(*(cache_0 + -3) > cb)
			         if(*(cache_0 + pixel[13]) > cb)
			          if(*(cache_0 + pixel[15]) > cb)
			           goto success;
			          else
			           continue;
			         else
			          continue;
			        else
			         continue;
			       else
			        continue;
			      else
			       continue;
			     else
			      continue;
			    else
			     if(*(cache_0 + pixel[15]) > cb)
			      if(*(cache_0 + pixel[10]) > cb)
			       if(*(cache_0 + pixel[11]) > cb)
			        if(*(cache_0 + -3) > cb)
			         if(*(cache_0 + pixel[13]) > cb)
			          if(*(cache_0 + pixel[14]) > cb)
			           goto success;
			          else
			           continue;
			         else
			          continue;
			        else
			         continue;
			       else
			        continue;
			      else
			       continue;
			     else
			      continue;
			   else if(*(cache_0 + pixel[9]) < c_b)
			    if(*(cache_0 + pixel[11]) < c_b)
			     if(*(cache_0 + pixel[13]) < c_b)
			      if(*(cache_0 + pixel[10]) < c_b)
			       if(*(cache_0 + -3) < c_b)
			        if(*(cache_0 + pixel[14]) < c_b)
			         if(*(cache_0 + pixel[15]) < c_b)
			          goto success;
			         else
			          continue;
			        else
			         continue;
			       else
			        continue;
			      else
			       continue;
			     else
			      continue;
			    else
			     continue;
			   else
			    continue;
			 else if(*(cache_0 + pixel[8]) < c_b)
			  if(*(cache_0 + 3) > cb)
			   if(*(cache_0 + -3) < c_b)
			    if(*(cache_0 + pixel[11]) > cb)
			     if(*(cache_0 + pixel[1]) < c_b)
			      if(*(cache_0 + pixel[2]) < c_b)
			       if(*(cache_0 + pixel[3]) < c_b)
			        if(*(cache_0 + pixel[13]) < c_b)
			         if(*(cache_0 + pixel[14]) < c_b)
			          if(*(cache_0 + pixel[15]) < c_b)
			           goto success;
			          else
			           continue;
			         else
			          continue;
			        else
			         continue;
			       else
			        continue;
			      else
			       continue;
			     else
			      continue;
			    else if(*(cache_0 + pixel[11]) < c_b)
			     if(*(cache_0 + pixel[13]) < c_b)
			      if(*(cache_0 + pixel[14]) > cb)
			       if(*(cache_0 + pixel[6]) < c_b)
			        if(*(cache_0 + pixel[7]) < c_b)
			         if(*(cache_0 + pixel[9]) < c_b)
			          if(*(cache_0 + pixel[10]) < c_b)
			           goto success;
			          else
			           continue;
			         else
			          continue;
			        else
			         continue;
			       else
			        continue;
			      else if(*(cache_0 + pixel[14]) < c_b)
			       if(*(cache_0 + pixel[10]) > cb)
			        if(*(cache_0 + pixel[1]) < c_b)
			         if(*(cache_0 + pixel[2]) < c_b)
			          if(*(cache_0 + pixel[15]) < c_b)
			           goto success;
			          else
			           continue;
			         else
			          continue;
			        else
			         continue;
			       else if(*(cache_0 + pixel[10]) < c_b)
			        if(*(cache_0 + pixel[15]) < c_b)
			         if(*(cache_0 + pixel[9]) < c_b)
			          goto success;
			         else
			          if(*(cache_0 + pixel[1]) < c_b)
			           goto success;
			          else
			           continue;
			        else
			         if(*(cache_0 + pixel[7]) < c_b)
			          if(*(cache_0 + pixel[9]) < c_b)
			           goto success;
			          else
			           continue;
			         else
			          continue;
			       else
			        if(*(cache_0 + pixel[2]) < c_b)
			         if(*(cache_0 + pixel[15]) < c_b)
			          if(*(cache_0 + pixel[1]) < c_b)
			           goto success;
			          else
			           continue;
			         else
			          continue;
			        else
			         continue;
			      else
			       if(*(cache_0 + pixel[6]) < c_b)
			        if(*(cache_0 + pixel[10]) < c_b)
			         if(*(cache_0 + pixel[7]) < c_b)
			          if(*(cache_0 + pixel[9]) < c_b)
			           goto success;
			          else
			           continue;
			         else
			          continue;
			        else
			         continue;
			       else
			        continue;
			     else
			      if(*(cache_0 + pixel[5]) < c_b)
			       if(*(cache_0 + pixel[6]) < c_b)
			        if(*(cache_0 + pixel[7]) < c_b)
			         if(*(cache_0 + pixel[9]) < c_b)
			          if(*(cache_0 + pixel[10]) < c_b)
			           goto success;
			          else
			           continue;
			         else
			          continue;
			        else
			         continue;
			       else
			        continue;
			      else
			       continue;
			    else
			     if(*(cache_0 + pixel[3]) < c_b)
			      if(*(cache_0 + pixel[15]) < c_b)
			       if(*(cache_0 + pixel[1]) < c_b)
			        if(*(cache_0 + pixel[2]) < c_b)
			         if(*(cache_0 + pixel[13]) < c_b)
			          if(*(cache_0 + pixel[14]) < c_b)
			           goto success;
			          else
			           continue;
			         else
			          continue;
			        else
			         continue;
			       else
			        continue;
			      else
			       continue;
			     else
			      continue;
			   else
			    continue;
			  else if(*(cache_0 + 3) < c_b)
			   if(*(cache_0 + pixel[2]) > cb)
			    if(*(cache_0 + pixel[10]) < c_b)
			     if(*(cache_0 + pixel[9]) < c_b)
			      if(*(cache_0 + pixel[7]) > cb)
			       if(*(cache_0 + pixel[11]) < c_b)
			        if(*(cache_0 + -3) < c_b)
			         if(*(cache_0 + pixel[13]) < c_b)
			          if(*(cache_0 + pixel[14]) < c_b)
			           if(*(cache_0 + pixel[15]) < c_b)
			            goto success;
			           else
			            continue;
			          else
			           continue;
			         else
			          continue;
			        else
			         continue;
			       else
			        continue;
			      else if(*(cache_0 + pixel[7]) < c_b)
			       if(*(cache_0 + pixel[6]) < c_b)
			        if(*(cache_0 + pixel[5]) < c_b)
			         if(*(cache_0 + pixel[11]) < c_b)
			          goto success;
			         else
			          if(*(cache_0 + pixel[3]) < c_b)
			           goto success;
			          else
			           continue;
			        else
			         if(*(cache_0 + pixel[11]) < c_b)
			          if(*(cache_0 + -3) < c_b)
			           if(*(cache_0 + pixel[13]) < c_b)
			            goto success;
			           else
			            continue;
			          else
			           continue;
			         else
			          continue;
			       else
			        if(*(cache_0 + pixel[11]) < c_b)
			         if(*(cache_0 + -3) < c_b)
			          if(*(cache_0 + pixel[13]) < c_b)
			           if(*(cache_0 + pixel[14]) < c_b)
			            goto success;
			           else
			            continue;
			          else
			           continue;
			         else
			          continue;
			        else
			         continue;
			      else
			       if(*(cache_0 + -3) < c_b)
			        if(*(cache_0 + pixel[11]) < c_b)
			         if(*(cache_0 + pixel[13]) < c_b)
			          if(*(cache_0 + pixel[14]) < c_b)
			           if(*(cache_0 + pixel[15]) < c_b)
			            goto success;
			           else
			            continue;
			          else
			           continue;
			         else
			          continue;
			        else
			         continue;
			       else
			        continue;
			     else
			      if(*(cache_0 + pixel[1]) < c_b)
			       if(*(cache_0 + pixel[11]) < c_b)
			        if(*(cache_0 + -3) < c_b)
			         if(*(cache_0 + pixel[13]) < c_b)
			          if(*(cache_0 + pixel[14]) < c_b)
			           if(*(cache_0 + pixel[15]) < c_b)
			            goto success;
			           else
			            continue;
			          else
			           continue;
			         else
			          continue;
			        else
			         continue;
			       else
			        continue;
			      else
			       continue;
			    else
			     continue;
			   else if(*(cache_0 + pixel[2]) < c_b)
			    if(*(cache_0 + pixel[6]) > cb)
			     if(*(cache_0 + pixel[14]) < c_b)
			      if(*(cache_0 + pixel[1]) > cb)
			       if(*(cache_0 + pixel[9]) < c_b)
			        if(*(cache_0 + pixel[10]) < c_b)
			         if(*(cache_0 + pixel[11]) < c_b)
			          if(*(cache_0 + -3) < c_b)
			           if(*(cache_0 + pixel[13]) < c_b)
			            if(*(cache_0 + pixel[7]) < c_b)
			             goto success;
			            else
			             if(*(cache_0 + pixel[15]) < c_b)
			              goto success;
			             else
			              continue;
			           else
			            continue;
			          else
			           continue;
			         else
			          continue;
			        else
			         continue;
			       else
			        continue;
			      else if(*(cache_0 + pixel[1]) < c_b)
			       if(*(cache_0 + pixel[3]) < c_b)
			        if(*(cache_0 + pixel[15]) > cb)
			         if(*(cache_0 + pixel[7]) < c_b)
			          if(*(cache_0 + pixel[9]) < c_b)
			           if(*(cache_0 + pixel[10]) < c_b)
			            if(*(cache_0 + pixel[11]) < c_b)
			             if(*(cache_0 + -3) < c_b)
			              if(*(cache_0 + pixel[13]) < c_b)
			               goto success;
			              else
			               continue;
			             else
			              continue;
			            else
			             continue;
			           else
			            continue;
			          else
			           continue;
			         else
			          continue;
			        else if(*(cache_0 + pixel[15]) < c_b)
			         if(*(cache_0 + pixel[13]) < c_b)
			          goto success;
			         else
			          if(*(cache_0 + pixel[5]) < c_b)
			           goto success;
			          else
			           continue;
			        else
			         if(*(cache_0 + pixel[11]) < c_b)
			          if(*(cache_0 + pixel[7]) < c_b)
			           if(*(cache_0 + pixel[9]) < c_b)
			            if(*(cache_0 + pixel[10]) < c_b)
			             if(*(cache_0 + pixel[13]) < c_b)
			              if(*(cache_0 + -3) < c_b)
			               goto success;
			              else
			               continue;
			             else
			              continue;
			            else
			             continue;
			           else
			            continue;
			          else
			           continue;
			         else
			          continue;
			       else
			        if(*(cache_0 + pixel[11]) < c_b)
			         if(*(cache_0 + -3) < c_b)
			          if(*(cache_0 + pixel[13]) < c_b)
			           if(*(cache_0 + pixel[15]) < c_b)
			            goto success;
			           else
			            if(*(cache_0 + pixel[7]) < c_b)
			             if(*(cache_0 + pixel[9]) < c_b)
			              if(*(cache_0 + pixel[10]) < c_b)
			               goto success;
			              else
			               continue;
			             else
			              continue;
			            else
			             continue;
			          else
			           continue;
			         else
			          continue;
			        else
			         continue;
			      else
			       if(*(cache_0 + pixel[11]) < c_b)
			        if(*(cache_0 + pixel[9]) < c_b)
			         if(*(cache_0 + pixel[10]) < c_b)
			          if(*(cache_0 + -3) < c_b)
			           if(*(cache_0 + pixel[13]) < c_b)
			            if(*(cache_0 + pixel[7]) < c_b)
			             goto success;
			            else
			             if(*(cache_0 + pixel[15]) < c_b)
			              goto success;
			             else
			              continue;
			           else
			            continue;
			          else
			           continue;
			         else
			          continue;
			        else
			         continue;
			       else
			        continue;
			     else
			      continue;
			    else if(*(cache_0 + pixel[6]) < c_b)
			     if(*(cache_0 + pixel[3]) > cb)
			      if(*(cache_0 + pixel[11]) < c_b)
			       if(*(cache_0 + pixel[10]) < c_b)
			        if(*(cache_0 + pixel[9]) < c_b)
			         if(*(cache_0 + pixel[7]) < c_b)
			          if(*(cache_0 + pixel[5]) < c_b)
			           goto success;
			          else
			           if(*(cache_0 + -3) < c_b)
			            if(*(cache_0 + pixel[13]) < c_b)
			             goto success;
			            else
			             continue;
			           else
			            continue;
			         else
			          if(*(cache_0 + -3) < c_b)
			           if(*(cache_0 + pixel[13]) < c_b)
			            if(*(cache_0 + pixel[14]) < c_b)
			             if(*(cache_0 + pixel[15]) < c_b)
			              goto success;
			             else
			              continue;
			            else
			             continue;
			           else
			            continue;
			          else
			           continue;
			        else
			         if(*(cache_0 + pixel[1]) < c_b)
			          if(*(cache_0 + -3) < c_b)
			           if(*(cache_0 + pixel[13]) < c_b)
			            if(*(cache_0 + pixel[14]) < c_b)
			             if(*(cache_0 + pixel[15]) < c_b)
			              goto success;
			             else
			              continue;
			            else
			             continue;
			           else
			            continue;
			          else
			           continue;
			         else
			          continue;
			       else
			        if(*(cache_0 + pixel[1]) < c_b)
			         if(*(cache_0 + -3) < c_b)
			          if(*(cache_0 + pixel[13]) < c_b)
			           if(*(cache_0 + pixel[14]) < c_b)
			            if(*(cache_0 + pixel[15]) < c_b)
			             goto success;
			            else
			             continue;
			           else
			            continue;
			          else
			           continue;
			         else
			          continue;
			        else
			         continue;
			      else
			       continue;
			     else if(*(cache_0 + pixel[3]) < c_b)
			      if(*(cache_0 + pixel[5]) > cb)
			       if(*(cache_0 + pixel[13]) < c_b)
			        if(*(cache_0 + pixel[14]) < c_b)
			         if(*(cache_0 + pixel[15]) < c_b)
			          if(*(cache_0 + pixel[1]) < c_b)
			           goto success;
			          else
			           if(*(cache_0 + pixel[9]) < c_b)
			            if(*(cache_0 + pixel[10]) < c_b)
			             if(*(cache_0 + pixel[11]) < c_b)
			              if(*(cache_0 + -3) < c_b)
			               goto success;
			              else
			               continue;
			             else
			              continue;
			            else
			             continue;
			           else
			            continue;
			         else
			          if(*(cache_0 + pixel[7]) < c_b)
			           if(*(cache_0 + pixel[9]) < c_b)
			            if(*(cache_0 + pixel[10]) < c_b)
			             if(*(cache_0 + pixel[11]) < c_b)
			              if(*(cache_0 + -3) < c_b)
			               goto success;
			              else
			               continue;
			             else
			              continue;
			            else
			             continue;
			           else
			            continue;
			          else
			           continue;
			        else
			         if(*(cache_0 + pixel[7]) < c_b)
			          if(*(cache_0 + pixel[9]) < c_b)
			           if(*(cache_0 + pixel[10]) < c_b)
			            if(*(cache_0 + pixel[11]) < c_b)
			             if(*(cache_0 + -3) < c_b)
			              goto success;
			             else
			              continue;
			            else
			             continue;
			           else
			            continue;
			          else
			           continue;
			         else
			          continue;
			       else
			        continue;
			      else if(*(cache_0 + pixel[5]) < c_b)
			       if(*(cache_0 + pixel[7]) > cb)
			        if(*(cache_0 + pixel[15]) < c_b)
			         if(*(cache_0 + pixel[1]) < c_b)
			          goto success;
			         else
			          if(*(cache_0 + pixel[9]) < c_b)
			           if(*(cache_0 + pixel[10]) < c_b)
			            if(*(cache_0 + pixel[11]) < c_b)
			             if(*(cache_0 + -3) < c_b)
			              if(*(cache_0 + pixel[13]) < c_b)
			               if(*(cache_0 + pixel[14]) < c_b)
			                goto success;
			               else
			                continue;
			              else
			               continue;
			             else
			              continue;
			            else
			             continue;
			           else
			            continue;
			          else
			           continue;
			        else
			         continue;
			       else if(*(cache_0 + pixel[7]) < c_b)
			        if(*(cache_0 + pixel[1]) < c_b)
			         goto success;
			        else
			         if(*(cache_0 + pixel[9]) < c_b)
			          goto success;
			         else
			          continue;
			       else
			        if(*(cache_0 + pixel[15]) < c_b)
			         if(*(cache_0 + pixel[1]) > cb)
			          if(*(cache_0 + pixel[9]) < c_b)
			           if(*(cache_0 + pixel[10]) < c_b)
			            if(*(cache_0 + pixel[11]) < c_b)
			             if(*(cache_0 + -3) < c_b)
			              if(*(cache_0 + pixel[13]) < c_b)
			               if(*(cache_0 + pixel[14]) < c_b)
			                goto success;
			               else
			                continue;
			              else
			               continue;
			             else
			              continue;
			            else
			             continue;
			           else
			            continue;
			          else
			           continue;
			         else if(*(cache_0 + pixel[1]) < c_b)
			          goto success;
			         else
			          if(*(cache_0 + pixel[11]) < c_b)
			           if(*(cache_0 + pixel[14]) < c_b)
			            if(*(cache_0 + pixel[9]) < c_b)
			             if(*(cache_0 + -3) < c_b)
			              if(*(cache_0 + pixel[13]) < c_b)
			               if(*(cache_0 + pixel[10]) < c_b)
			                goto success;
			               else
			                continue;
			              else
			               continue;
			             else
			              continue;
			            else
			             continue;
			           else
			            continue;
			          else
			           continue;
			        else
			         continue;
			      else
			       if(*(cache_0 + pixel[13]) < c_b)
			        if(*(cache_0 + pixel[14]) > cb)
			         if(*(cache_0 + pixel[7]) < c_b)
			          if(*(cache_0 + pixel[9]) < c_b)
			           if(*(cache_0 + pixel[10]) < c_b)
			            if(*(cache_0 + pixel[11]) < c_b)
			             if(*(cache_0 + -3) < c_b)
			              goto success;
			             else
			              continue;
			            else
			             continue;
			           else
			            continue;
			          else
			           continue;
			         else
			          continue;
			        else if(*(cache_0 + pixel[14]) < c_b)
			         if(*(cache_0 + pixel[1]) > cb)
			          if(*(cache_0 + pixel[9]) < c_b)
			           if(*(cache_0 + pixel[10]) < c_b)
			            if(*(cache_0 + pixel[11]) < c_b)
			             if(*(cache_0 + -3) < c_b)
			              if(*(cache_0 + pixel[15]) < c_b)
			               goto success;
			              else
			               if(*(cache_0 + pixel[7]) < c_b)
			                goto success;
			               else
			                continue;
			             else
			              continue;
			            else
			             continue;
			           else
			            continue;
			          else
			           continue;
			         else if(*(cache_0 + pixel[1]) < c_b)
			          if(*(cache_0 + pixel[15]) > cb)
			           if(*(cache_0 + pixel[7]) < c_b)
			            if(*(cache_0 + pixel[9]) < c_b)
			             if(*(cache_0 + pixel[10]) < c_b)
			              if(*(cache_0 + pixel[11]) < c_b)
			               if(*(cache_0 + -3) < c_b)
			                goto success;
			               else
			                continue;
			              else
			               continue;
			             else
			              continue;
			            else
			             continue;
			           else
			            continue;
			          else if(*(cache_0 + pixel[15]) < c_b)
			           goto success;
			          else
			           if(*(cache_0 + pixel[11]) < c_b)
			            if(*(cache_0 + pixel[7]) < c_b)
			             if(*(cache_0 + -3) < c_b)
			              if(*(cache_0 + pixel[9]) < c_b)
			               if(*(cache_0 + pixel[10]) < c_b)
			                goto success;
			               else
			                continue;
			              else
			               continue;
			             else
			              continue;
			            else
			             continue;
			           else
			            continue;
			         else
			          if(*(cache_0 + pixel[11]) < c_b)
			           if(*(cache_0 + pixel[9]) < c_b)
			            if(*(cache_0 + -3) < c_b)
			             if(*(cache_0 + pixel[10]) < c_b)
			              if(*(cache_0 + pixel[15]) < c_b)
			               goto success;
			              else
			               if(*(cache_0 + pixel[7]) < c_b)
			                goto success;
			               else
			                continue;
			             else
			              continue;
			            else
			             continue;
			           else
			            continue;
			          else
			           continue;
			        else
			         if(*(cache_0 + pixel[11]) < c_b)
			          if(*(cache_0 + pixel[7]) < c_b)
			           if(*(cache_0 + pixel[10]) < c_b)
			            if(*(cache_0 + -3) < c_b)
			             if(*(cache_0 + pixel[9]) < c_b)
			              goto success;
			             else
			              continue;
			            else
			             continue;
			           else
			            continue;
			          else
			           continue;
			         else
			          continue;
			       else
			        continue;
			     else
			      if(*(cache_0 + pixel[11]) < c_b)
			       if(*(cache_0 + pixel[10]) > cb)
			        if(*(cache_0 + pixel[1]) < c_b)
			         if(*(cache_0 + -3) < c_b)
			          if(*(cache_0 + pixel[13]) < c_b)
			           if(*(cache_0 + pixel[14]) < c_b)
			            if(*(cache_0 + pixel[15]) < c_b)
			             goto success;
			            else
			             continue;
			           else
			            continue;
			          else
			           continue;
			         else
			          continue;
			        else
			         continue;
			       else if(*(cache_0 + pixel[10]) < c_b)
			        if(*(cache_0 + pixel[9]) > cb)
			         if(*(cache_0 + pixel[1]) < c_b)
			          if(*(cache_0 + -3) < c_b)
			           if(*(cache_0 + pixel[13]) < c_b)
			            if(*(cache_0 + pixel[14]) < c_b)
			             if(*(cache_0 + pixel[15]) < c_b)
			              goto success;
			             else
			              continue;
			            else
			             continue;
			           else
			            continue;
			          else
			           continue;
			         else
			          continue;
			        else if(*(cache_0 + pixel[9]) < c_b)
			         if(*(cache_0 + pixel[7]) > cb)
			          if(*(cache_0 + -3) < c_b)
			           if(*(cache_0 + pixel[13]) < c_b)
			            if(*(cache_0 + pixel[14]) < c_b)
			             if(*(cache_0 + pixel[15]) < c_b)
			              goto success;
			             else
			              continue;
			            else
			             continue;
			           else
			            continue;
			          else
			           continue;
			         else if(*(cache_0 + pixel[7]) < c_b)
			          if(*(cache_0 + pixel[5]) > cb)
			           if(*(cache_0 + -3) < c_b)
			            if(*(cache_0 + pixel[13]) < c_b)
			             goto success;
			            else
			             continue;
			           else
			            continue;
			          else if(*(cache_0 + pixel[5]) < c_b)
			           goto success;
			          else
			           if(*(cache_0 + pixel[13]) < c_b)
			            if(*(cache_0 + -3) < c_b)
			             goto success;
			            else
			             continue;
			           else
			            continue;
			         else
			          if(*(cache_0 + pixel[13]) < c_b)
			           if(*(cache_0 + pixel[15]) < c_b)
			            if(*(cache_0 + -3) < c_b)
			             if(*(cache_0 + pixel[14]) < c_b)
			              goto success;
			             else
			              continue;
			            else
			             continue;
			           else
			            continue;
			          else
			           continue;
			        else
			         if(*(cache_0 + pixel[13]) < c_b)
			          if(*(cache_0 + pixel[14]) < c_b)
			           if(*(cache_0 + -3) < c_b)
			            if(*(cache_0 + pixel[15]) < c_b)
			             if(*(cache_0 + pixel[1]) < c_b)
			              goto success;
			             else
			              continue;
			            else
			             continue;
			           else
			            continue;
			          else
			           continue;
			         else
			          continue;
			       else
			        if(*(cache_0 + pixel[13]) < c_b)
			         if(*(cache_0 + pixel[15]) < c_b)
			          if(*(cache_0 + -3) < c_b)
			           if(*(cache_0 + pixel[14]) < c_b)
			            if(*(cache_0 + pixel[1]) < c_b)
			             goto success;
			            else
			             continue;
			           else
			            continue;
			          else
			           continue;
			         else
			          continue;
			        else
			         continue;
			      else
			       continue;
			    else
			     if(*(cache_0 + pixel[14]) < c_b)
			      if(*(cache_0 + pixel[13]) > cb)
			       if(*(cache_0 + pixel[5]) < c_b)
			        if(*(cache_0 + pixel[1]) < c_b)
			         if(*(cache_0 + pixel[3]) < c_b)
			          if(*(cache_0 + pixel[15]) < c_b)
			           goto success;
			          else
			           continue;
			         else
			          continue;
			        else
			         continue;
			       else
			        continue;
			      else if(*(cache_0 + pixel[13]) < c_b)
			       if(*(cache_0 + pixel[15]) > cb)
			        if(*(cache_0 + pixel[7]) < c_b)
			         if(*(cache_0 + pixel[9]) < c_b)
			          if(*(cache_0 + pixel[10]) < c_b)
			           if(*(cache_0 + pixel[11]) < c_b)
			            if(*(cache_0 + -3) < c_b)
			             goto success;
			            else
			             continue;
			           else
			            continue;
			          else
			           continue;
			         else
			          continue;
			        else
			         continue;
			       else if(*(cache_0 + pixel[15]) < c_b)
			        if(*(cache_0 + pixel[3]) > cb)
			         if(*(cache_0 + pixel[11]) < c_b)
			          if(*(cache_0 + -3) < c_b)
			           if(*(cache_0 + pixel[1]) < c_b)
			            goto success;
			           else
			            if(*(cache_0 + pixel[9]) < c_b)
			             if(*(cache_0 + pixel[10]) < c_b)
			              goto success;
			             else
			              continue;
			            else
			             continue;
			          else
			           continue;
			         else
			          continue;
			        else if(*(cache_0 + pixel[3]) < c_b)
			         if(*(cache_0 + pixel[1]) > cb)
			          if(*(cache_0 + pixel[9]) < c_b)
			           if(*(cache_0 + pixel[10]) < c_b)
			            if(*(cache_0 + pixel[11]) < c_b)
			             if(*(cache_0 + -3) < c_b)
			              goto success;
			             else
			              continue;
			            else
			             continue;
			           else
			            continue;
			          else
			           continue;
			         else if(*(cache_0 + pixel[1]) < c_b)
			          goto success;
			         else
			          if(*(cache_0 + pixel[10]) < c_b)
			           if(*(cache_0 + -3) < c_b)
			            if(*(cache_0 + pixel[9]) < c_b)
			             if(*(cache_0 + pixel[11]) < c_b)
			              goto success;
			             else
			              continue;
			            else
			             continue;
			           else
			            continue;
			          else
			           continue;
			        else
			         if(*(cache_0 + pixel[11]) < c_b)
			          if(*(cache_0 + -3) < c_b)
			           if(*(cache_0 + pixel[1]) > cb)
			            if(*(cache_0 + pixel[9]) < c_b)
			             if(*(cache_0 + pixel[10]) < c_b)
			              goto success;
			             else
			              continue;
			            else
			             continue;
			           else if(*(cache_0 + pixel[1]) < c_b)
			            goto success;
			           else
			            if(*(cache_0 + pixel[10]) < c_b)
			             if(*(cache_0 + pixel[9]) < c_b)
			              goto success;
			             else
			              continue;
			            else
			             continue;
			          else
			           continue;
			         else
			          continue;
			       else
			        if(*(cache_0 + pixel[7]) < c_b)
			         if(*(cache_0 + pixel[11]) < c_b)
			          if(*(cache_0 + pixel[9]) < c_b)
			           if(*(cache_0 + -3) < c_b)
			            if(*(cache_0 + pixel[10]) < c_b)
			             goto success;
			            else
			             continue;
			           else
			            continue;
			          else
			           continue;
			         else
			          continue;
			        else
			         continue;
			      else
			       if(*(cache_0 + pixel[5]) < c_b)
			        if(*(cache_0 + pixel[15]) < c_b)
			         if(*(cache_0 + pixel[3]) < c_b)
			          if(*(cache_0 + pixel[1]) < c_b)
			           goto success;
			          else
			           continue;
			         else
			          continue;
			        else
			         continue;
			       else
			        continue;
			     else
			      continue;
			   else
			    if(*(cache_0 + pixel[10]) < c_b)
			     if(*(cache_0 + pixel[11]) > cb)
			      if(*(cache_0 + pixel[3]) < c_b)
			       if(*(cache_0 + pixel[5]) < c_b)
			        if(*(cache_0 + pixel[6]) < c_b)
			         if(*(cache_0 + pixel[7]) < c_b)
			          if(*(cache_0 + pixel[9]) < c_b)
			           goto success;
			          else
			           continue;
			         else
			          continue;
			        else
			         continue;
			       else
			        continue;
			      else
			       continue;
			     else if(*(cache_0 + pixel[11]) < c_b)
			      if(*(cache_0 + pixel[6]) > cb)
			       if(*(cache_0 + pixel[13]) < c_b)
			        if(*(cache_0 + pixel[14]) < c_b)
			         if(*(cache_0 + -3) < c_b)
			          if(*(cache_0 + pixel[15]) < c_b)
			           if(*(cache_0 + pixel[1]) < c_b)
			            goto success;
			           else
			            if(*(cache_0 + pixel[9]) < c_b)
			             goto success;
			            else
			             continue;
			          else
			           if(*(cache_0 + pixel[7]) < c_b)
			            if(*(cache_0 + pixel[9]) < c_b)
			             goto success;
			            else
			             continue;
			           else
			            continue;
			         else
			          continue;
			        else
			         continue;
			       else
			        continue;
			      else if(*(cache_0 + pixel[6]) < c_b)
			       if(*(cache_0 + pixel[9]) > cb)
			        if(*(cache_0 + pixel[1]) < c_b)
			         if(*(cache_0 + -3) < c_b)
			          if(*(cache_0 + pixel[13]) < c_b)
			           if(*(cache_0 + pixel[14]) < c_b)
			            if(*(cache_0 + pixel[15]) < c_b)
			             goto success;
			            else
			             continue;
			           else
			            continue;
			          else
			           continue;
			         else
			          continue;
			        else
			         continue;
			       else if(*(cache_0 + pixel[9]) < c_b)
			        if(*(cache_0 + pixel[7]) > cb)
			         if(*(cache_0 + -3) < c_b)
			          if(*(cache_0 + pixel[13]) < c_b)
			           if(*(cache_0 + pixel[14]) < c_b)
			            if(*(cache_0 + pixel[15]) < c_b)
			             goto success;
			            else
			             continue;
			           else
			            continue;
			          else
			           continue;
			         else
			          continue;
			        else if(*(cache_0 + pixel[7]) < c_b)
			         if(*(cache_0 + pixel[5]) > cb)
			          if(*(cache_0 + -3) < c_b)
			           if(*(cache_0 + pixel[13]) < c_b)
			            goto success;
			           else
			            continue;
			          else
			           continue;
			         else if(*(cache_0 + pixel[5]) < c_b)
			          goto success;
			         else
			          if(*(cache_0 + pixel[13]) < c_b)
			           if(*(cache_0 + -3) < c_b)
			            goto success;
			           else
			            continue;
			          else
			           continue;
			        else
			         if(*(cache_0 + pixel[14]) < c_b)
			          if(*(cache_0 + pixel[13]) < c_b)
			           if(*(cache_0 + pixel[15]) < c_b)
			            if(*(cache_0 + -3) < c_b)
			             goto success;
			            else
			             continue;
			           else
			            continue;
			          else
			           continue;
			         else
			          continue;
			       else
			        if(*(cache_0 + pixel[13]) < c_b)
			         if(*(cache_0 + pixel[1]) < c_b)
			          if(*(cache_0 + pixel[14]) < c_b)
			           if(*(cache_0 + pixel[15]) < c_b)
			            if(*(cache_0 + -3) < c_b)
			             goto success;
			            else
			             continue;
			           else
			            continue;
			          else
			           continue;
			         else
			          continue;
			        else
			         continue;
			      else
			       if(*(cache_0 + pixel[14]) < c_b)
			        if(*(cache_0 + pixel[13]) < c_b)
			         if(*(cache_0 + -3) < c_b)
			          if(*(cache_0 + pixel[15]) < c_b)
			           if(*(cache_0 + pixel[9]) < c_b)
			            goto success;
			           else
			            if(*(cache_0 + pixel[1]) < c_b)
			             goto success;
			            else
			             continue;
			          else
			           if(*(cache_0 + pixel[7]) < c_b)
			            if(*(cache_0 + pixel[9]) < c_b)
			             goto success;
			            else
			             continue;
			           else
			            continue;
			         else
			          continue;
			        else
			         continue;
			       else
			        continue;
			     else
			      if(*(cache_0 + pixel[3]) < c_b)
			       if(*(cache_0 + pixel[6]) < c_b)
			        if(*(cache_0 + pixel[5]) < c_b)
			         if(*(cache_0 + pixel[9]) < c_b)
			          if(*(cache_0 + pixel[7]) < c_b)
			           goto success;
			          else
			           continue;
			         else
			          continue;
			        else
			         continue;
			       else
			        continue;
			      else
			       continue;
			    else
			     continue;
			  else
			   if(*(cache_0 + -3) < c_b)
			    if(*(cache_0 + pixel[11]) > cb)
			     if(*(cache_0 + pixel[1]) < c_b)
			      if(*(cache_0 + pixel[2]) < c_b)
			       if(*(cache_0 + pixel[3]) < c_b)
			        if(*(cache_0 + pixel[13]) < c_b)
			         if(*(cache_0 + pixel[14]) < c_b)
			          if(*(cache_0 + pixel[15]) < c_b)
			           goto success;
			          else
			           continue;
			         else
			          continue;
			        else
			         continue;
			       else
			        continue;
			      else
			       continue;
			     else
			      continue;
			    else if(*(cache_0 + pixel[11]) < c_b)
			     if(*(cache_0 + pixel[14]) > cb)
			      if(*(cache_0 + pixel[6]) < c_b)
			       if(*(cache_0 + pixel[7]) < c_b)
			        if(*(cache_0 + pixel[9]) < c_b)
			         if(*(cache_0 + pixel[10]) < c_b)
			          if(*(cache_0 + pixel[5]) < c_b)
			           goto success;
			          else
			           if(*(cache_0 + pixel[13]) < c_b)
			            goto success;
			           else
			            continue;
			         else
			          continue;
			        else
			         continue;
			       else
			        continue;
			      else
			       continue;
			     else if(*(cache_0 + pixel[14]) < c_b)
			      if(*(cache_0 + pixel[13]) > cb)
			       if(*(cache_0 + pixel[5]) < c_b)
			        if(*(cache_0 + pixel[6]) < c_b)
			         if(*(cache_0 + pixel[7]) < c_b)
			          if(*(cache_0 + pixel[9]) < c_b)
			           if(*(cache_0 + pixel[10]) < c_b)
			            goto success;
			           else
			            continue;
			          else
			           continue;
			         else
			          continue;
			        else
			         continue;
			       else
			        continue;
			      else if(*(cache_0 + pixel[13]) < c_b)
			       if(*(cache_0 + pixel[10]) > cb)
			        if(*(cache_0 + pixel[2]) < c_b)
			         if(*(cache_0 + pixel[1]) < c_b)
			          if(*(cache_0 + pixel[15]) < c_b)
			           goto success;
			          else
			           continue;
			         else
			          continue;
			        else
			         continue;
			       else if(*(cache_0 + pixel[10]) < c_b)
			        if(*(cache_0 + pixel[15]) < c_b)
			         if(*(cache_0 + pixel[9]) < c_b)
			          goto success;
			         else
			          if(*(cache_0 + pixel[1]) < c_b)
			           goto success;
			          else
			           continue;
			        else
			         if(*(cache_0 + pixel[7]) < c_b)
			          if(*(cache_0 + pixel[9]) < c_b)
			           goto success;
			          else
			           continue;
			         else
			          continue;
			       else
			        if(*(cache_0 + pixel[2]) < c_b)
			         if(*(cache_0 + pixel[15]) < c_b)
			          if(*(cache_0 + pixel[1]) < c_b)
			           goto success;
			          else
			           continue;
			         else
			          continue;
			        else
			         continue;
			      else
			       if(*(cache_0 + pixel[5]) < c_b)
			        if(*(cache_0 + pixel[10]) < c_b)
			         if(*(cache_0 + pixel[6]) < c_b)
			          if(*(cache_0 + pixel[7]) < c_b)
			           if(*(cache_0 + pixel[9]) < c_b)
			            goto success;
			           else
			            continue;
			          else
			           continue;
			         else
			          continue;
			        else
			         continue;
			       else
			        continue;
			     else
			      if(*(cache_0 + pixel[6]) < c_b)
			       if(*(cache_0 + pixel[10]) < c_b)
			        if(*(cache_0 + pixel[5]) > cb)
			         if(*(cache_0 + pixel[7]) < c_b)
			          if(*(cache_0 + pixel[9]) < c_b)
			           if(*(cache_0 + pixel[13]) < c_b)
			            goto success;
			           else
			            continue;
			          else
			           continue;
			         else
			          continue;
			        else if(*(cache_0 + pixel[5]) < c_b)
			         if(*(cache_0 + pixel[7]) < c_b)
			          if(*(cache_0 + pixel[9]) < c_b)
			           goto success;
			          else
			           continue;
			         else
			          continue;
			        else
			         if(*(cache_0 + pixel[13]) < c_b)
			          if(*(cache_0 + pixel[7]) < c_b)
			           if(*(cache_0 + pixel[9]) < c_b)
			            goto success;
			           else
			            continue;
			          else
			           continue;
			         else
			          continue;
			       else
			        continue;
			      else
			       continue;
			    else
			     if(*(cache_0 + pixel[3]) < c_b)
			      if(*(cache_0 + pixel[2]) < c_b)
			       if(*(cache_0 + pixel[14]) < c_b)
			        if(*(cache_0 + pixel[13]) < c_b)
			         if(*(cache_0 + pixel[1]) < c_b)
			          if(*(cache_0 + pixel[15]) < c_b)
			           goto success;
			          else
			           continue;
			         else
			          continue;
			        else
			         continue;
			       else
			        continue;
			      else
			       continue;
			     else
			      continue;
			   else
			    continue;
			 else
			  if(*(cache_0 + -3) > cb)
			   if(*(cache_0 + 3) < c_b)
			    if(*(cache_0 + pixel[1]) < c_b)
			     if(*(cache_0 + pixel[2]) < c_b)
			      if(*(cache_0 + pixel[5]) < c_b)
			       if(*(cache_0 + pixel[3]) < c_b)
			        if(*(cache_0 + pixel[6]) < c_b)
			         if(*(cache_0 + pixel[15]) < c_b)
			          goto success;
			         else
			          if(*(cache_0 + pixel[7]) < c_b)
			           goto success;
			          else
			           continue;
			        else
			         if(*(cache_0 + pixel[14]) < c_b)
			          if(*(cache_0 + pixel[15]) < c_b)
			           goto success;
			          else
			           continue;
			         else
			          continue;
			       else
			        continue;
			      else
			       if(*(cache_0 + pixel[13]) < c_b)
			        if(*(cache_0 + pixel[3]) < c_b)
			         if(*(cache_0 + pixel[14]) < c_b)
			          if(*(cache_0 + pixel[15]) < c_b)
			           goto success;
			          else
			           continue;
			         else
			          continue;
			        else
			         continue;
			       else
			        continue;
			     else
			      continue;
			    else
			     continue;
			   else
			    continue;
			  else if(*(cache_0 + -3) < c_b)
			   if(*(cache_0 + pixel[2]) > cb)
			    if(*(cache_0 + pixel[10]) < c_b)
			     if(*(cache_0 + pixel[14]) < c_b)
			      if(*(cache_0 + pixel[13]) < c_b)
			       if(*(cache_0 + pixel[11]) < c_b)
			        if(*(cache_0 + pixel[15]) < c_b)
			         if(*(cache_0 + pixel[1]) < c_b)
			          goto success;
			         else
			          if(*(cache_0 + pixel[9]) < c_b)
			           goto success;
			          else
			           continue;
			        else
			         continue;
			       else
			        continue;
			      else
			       continue;
			     else
			      continue;
			    else
			     continue;
			   else if(*(cache_0 + pixel[2]) < c_b)
			    if(*(cache_0 + pixel[15]) > cb)
			     if(*(cache_0 + pixel[1]) < c_b)
			      if(*(cache_0 + pixel[3]) < c_b)
			       if(*(cache_0 + 3) < c_b)
			        if(*(cache_0 + pixel[5]) < c_b)
			         if(*(cache_0 + pixel[6]) < c_b)
			          if(*(cache_0 + pixel[7]) < c_b)
			           goto success;
			          else
			           continue;
			         else
			          continue;
			        else
			         continue;
			       else
			        continue;
			      else
			       continue;
			     else
			      continue;
			    else if(*(cache_0 + pixel[15]) < c_b)
			     if(*(cache_0 + pixel[11]) > cb)
			      if(*(cache_0 + pixel[3]) < c_b)
			       if(*(cache_0 + pixel[1]) < c_b)
			        if(*(cache_0 + pixel[14]) < c_b)
			         if(*(cache_0 + pixel[13]) < c_b)
			          goto success;
			         else
			          if(*(cache_0 + 3) < c_b)
			           if(*(cache_0 + pixel[5]) < c_b)
			            goto success;
			           else
			            continue;
			          else
			           continue;
			        else
			         if(*(cache_0 + 3) < c_b)
			          if(*(cache_0 + pixel[5]) < c_b)
			           if(*(cache_0 + pixel[6]) < c_b)
			            goto success;
			           else
			            continue;
			          else
			           continue;
			         else
			          continue;
			       else
			        continue;
			      else
			       continue;
			     else if(*(cache_0 + pixel[11]) < c_b)
			      if(*(cache_0 + pixel[14]) > cb)
			       if(*(cache_0 + pixel[1]) < c_b)
			        if(*(cache_0 + 3) < c_b)
			         if(*(cache_0 + pixel[3]) < c_b)
			          if(*(cache_0 + pixel[5]) < c_b)
			           if(*(cache_0 + pixel[6]) < c_b)
			            goto success;
			           else
			            continue;
			          else
			           continue;
			         else
			          continue;
			        else
			         continue;
			       else
			        continue;
			      else if(*(cache_0 + pixel[14]) < c_b)
			       if(*(cache_0 + pixel[1]) < c_b)
			        if(*(cache_0 + pixel[13]) > cb)
			         if(*(cache_0 + pixel[3]) < c_b)
			          if(*(cache_0 + 3) < c_b)
			           if(*(cache_0 + pixel[5]) < c_b)
			            goto success;
			           else
			            continue;
			          else
			           continue;
			         else
			          continue;
			        else if(*(cache_0 + pixel[13]) < c_b)
			         goto success;
			        else
			         if(*(cache_0 + pixel[5]) < c_b)
			          if(*(cache_0 + pixel[3]) < c_b)
			           if(*(cache_0 + 3) < c_b)
			            goto success;
			           else
			            continue;
			          else
			           continue;
			         else
			          continue;
			       else
			        if(*(cache_0 + pixel[9]) < c_b)
			         if(*(cache_0 + pixel[10]) < c_b)
			          if(*(cache_0 + pixel[13]) < c_b)
			           goto success;
			          else
			           continue;
			         else
			          continue;
			        else
			         continue;
			      else
			       if(*(cache_0 + pixel[6]) < c_b)
			        if(*(cache_0 + 3) < c_b)
			         if(*(cache_0 + pixel[3]) < c_b)
			          if(*(cache_0 + pixel[1]) < c_b)
			           if(*(cache_0 + pixel[5]) < c_b)
			            goto success;
			           else
			            continue;
			          else
			           continue;
			         else
			          continue;
			        else
			         continue;
			       else
			        continue;
			     else
			      if(*(cache_0 + pixel[3]) < c_b)
			       if(*(cache_0 + pixel[14]) > cb)
			        if(*(cache_0 + pixel[1]) < c_b)
			         if(*(cache_0 + 3) < c_b)
			          if(*(cache_0 + pixel[5]) < c_b)
			           if(*(cache_0 + pixel[6]) < c_b)
			            goto success;
			           else
			            continue;
			          else
			           continue;
			         else
			          continue;
			        else
			         continue;
			       else if(*(cache_0 + pixel[14]) < c_b)
			        if(*(cache_0 + pixel[1]) < c_b)
			         if(*(cache_0 + pixel[13]) > cb)
			          if(*(cache_0 + 3) < c_b)
			           if(*(cache_0 + pixel[5]) < c_b)
			            goto success;
			           else
			            continue;
			          else
			           continue;
			         else if(*(cache_0 + pixel[13]) < c_b)
			          goto success;
			         else
			          if(*(cache_0 + pixel[5]) < c_b)
			           if(*(cache_0 + 3) < c_b)
			            goto success;
			           else
			            continue;
			          else
			           continue;
			        else
			         continue;
			       else
			        if(*(cache_0 + pixel[6]) < c_b)
			         if(*(cache_0 + pixel[5]) < c_b)
			          if(*(cache_0 + pixel[1]) < c_b)
			           if(*(cache_0 + 3) < c_b)
			            goto success;
			           else
			            continue;
			          else
			           continue;
			         else
			          continue;
			        else
			         continue;
			      else
			       continue;
			    else
			     if(*(cache_0 + pixel[7]) < c_b)
			      if(*(cache_0 + pixel[6]) < c_b)
			       if(*(cache_0 + 3) < c_b)
			        if(*(cache_0 + pixel[1]) < c_b)
			         if(*(cache_0 + pixel[5]) < c_b)
			          if(*(cache_0 + pixel[3]) < c_b)
			           goto success;
			          else
			           continue;
			         else
			          continue;
			        else
			         continue;
			       else
			        continue;
			      else
			       continue;
			     else
			      continue;
			   else
			    if(*(cache_0 + pixel[10]) < c_b)
			     if(*(cache_0 + pixel[14]) < c_b)
			      if(*(cache_0 + pixel[11]) < c_b)
			       if(*(cache_0 + pixel[1]) > cb)
			        if(*(cache_0 + pixel[9]) < c_b)
			         if(*(cache_0 + pixel[13]) < c_b)
			          if(*(cache_0 + pixel[15]) < c_b)
			           goto success;
			          else
			           continue;
			         else
			          continue;
			        else
			         continue;
			       else if(*(cache_0 + pixel[1]) < c_b)
			        if(*(cache_0 + pixel[13]) < c_b)
			         if(*(cache_0 + pixel[15]) < c_b)
			          goto success;
			         else
			          continue;
			        else
			         continue;
			       else
			        if(*(cache_0 + pixel[9]) < c_b)
			         if(*(cache_0 + pixel[15]) < c_b)
			          if(*(cache_0 + pixel[13]) < c_b)
			           goto success;
			          else
			           continue;
			         else
			          continue;
			        else
			         continue;
			      else
			       continue;
			     else
			      continue;
			    else
			     continue;
			  else
			   if(*(cache_0 + 3) < c_b)
			    if(*(cache_0 + pixel[2]) < c_b)
			     if(*(cache_0 + pixel[14]) > cb)
			      if(*(cache_0 + pixel[6]) < c_b)
			       if(*(cache_0 + pixel[1]) < c_b)
			        if(*(cache_0 + pixel[3]) < c_b)
			         if(*(cache_0 + pixel[15]) > cb)
			          if(*(cache_0 + pixel[5]) < c_b)
			           if(*(cache_0 + pixel[7]) < c_b)
			            goto success;
			           else
			            continue;
			          else
			           continue;
			         else if(*(cache_0 + pixel[15]) < c_b)
			          if(*(cache_0 + pixel[5]) < c_b)
			           goto success;
			          else
			           continue;
			         else
			          if(*(cache_0 + pixel[7]) < c_b)
			           if(*(cache_0 + pixel[5]) < c_b)
			            goto success;
			           else
			            continue;
			          else
			           continue;
			        else
			         continue;
			       else
			        continue;
			      else
			       continue;
			     else if(*(cache_0 + pixel[14]) < c_b)
			      if(*(cache_0 + pixel[3]) < c_b)
			       if(*(cache_0 + pixel[1]) < c_b)
			        if(*(cache_0 + pixel[5]) < c_b)
			         if(*(cache_0 + pixel[15]) > cb)
			          if(*(cache_0 + pixel[6]) < c_b)
			           if(*(cache_0 + pixel[7]) < c_b)
			            goto success;
			           else
			            continue;
			          else
			           continue;
			         else if(*(cache_0 + pixel[15]) < c_b)
			          goto success;
			         else
			          if(*(cache_0 + pixel[7]) < c_b)
			           if(*(cache_0 + pixel[6]) < c_b)
			            goto success;
			           else
			            continue;
			          else
			           continue;
			        else
			         if(*(cache_0 + pixel[13]) < c_b)
			          if(*(cache_0 + pixel[15]) < c_b)
			           goto success;
			          else
			           continue;
			         else
			          continue;
			       else
			        continue;
			      else
			       continue;
			     else
			      if(*(cache_0 + pixel[6]) < c_b)
			       if(*(cache_0 + pixel[15]) > cb)
			        if(*(cache_0 + pixel[1]) < c_b)
			         if(*(cache_0 + pixel[3]) < c_b)
			          if(*(cache_0 + pixel[5]) < c_b)
			           if(*(cache_0 + pixel[7]) < c_b)
			            goto success;
			           else
			            continue;
			          else
			           continue;
			         else
			          continue;
			        else
			         continue;
			       else if(*(cache_0 + pixel[15]) < c_b)
			        if(*(cache_0 + pixel[1]) < c_b)
			         if(*(cache_0 + pixel[5]) < c_b)
			          if(*(cache_0 + pixel[3]) < c_b)
			           goto success;
			          else
			           continue;
			         else
			          continue;
			        else
			         continue;
			       else
			        if(*(cache_0 + pixel[7]) < c_b)
			         if(*(cache_0 + pixel[1]) < c_b)
			          if(*(cache_0 + pixel[3]) < c_b)
			           if(*(cache_0 + pixel[5]) < c_b)
			            goto success;
			           else
			            continue;
			          else
			           continue;
			         else
			          continue;
			        else
			         continue;
			      else
			       continue;
			    else
			     continue;
			   else
			    continue;
			else
			 if(*(cache_0 + pixel[8]) > cb)
			  if(*(cache_0 + 3) > cb)
			   if(*(cache_0 + pixel[3]) > cb)
			    if(*(cache_0 + pixel[6]) > cb)
			     if(*(cache_0 + pixel[10]) > cb)
			      if(*(cache_0 + pixel[7]) > cb)
			       if(*(cache_0 + pixel[5]) > cb)
			        if(*(cache_0 + pixel[9]) > cb)
			         goto success;
			        else
			         if(*(cache_0 + pixel[1]) > cb)
			          if(*(cache_0 + pixel[2]) > cb)
			           goto success;
			          else
			           continue;
			         else
			          continue;
			       else if(*(cache_0 + pixel[5]) < c_b)
			        if(*(cache_0 + pixel[9]) > cb)
			         if(*(cache_0 + pixel[11]) > cb)
			          if(*(cache_0 + -3) > cb)
			           if(*(cache_0 + pixel[13]) > cb)
			            goto success;
			           else
			            continue;
			          else
			           continue;
			         else
			          continue;
			        else
			         continue;
			       else
			        if(*(cache_0 + -3) > cb)
			         if(*(cache_0 + pixel[13]) > cb)
			          if(*(cache_0 + pixel[11]) > cb)
			           if(*(cache_0 + pixel[9]) > cb)
			            goto success;
			           else
			            continue;
			          else
			           continue;
			         else
			          continue;
			        else
			         continue;
			      else if(*(cache_0 + pixel[7]) < c_b)
			       if(*(cache_0 + pixel[9]) > cb)
			        if(*(cache_0 + pixel[11]) > cb)
			         if(*(cache_0 + -3) > cb)
			          if(*(cache_0 + pixel[13]) > cb)
			           if(*(cache_0 + pixel[14]) > cb)
			            if(*(cache_0 + pixel[15]) > cb)
			             goto success;
			            else
			             continue;
			           else
			            continue;
			          else
			           continue;
			         else
			          continue;
			        else
			         continue;
			       else
			        continue;
			      else
			       if(*(cache_0 + pixel[15]) > cb)
			        if(*(cache_0 + -3) > cb)
			         if(*(cache_0 + pixel[11]) > cb)
			          if(*(cache_0 + pixel[14]) > cb)
			           if(*(cache_0 + pixel[9]) > cb)
			            if(*(cache_0 + pixel[13]) > cb)
			             goto success;
			            else
			             continue;
			           else
			            continue;
			          else
			           continue;
			         else
			          continue;
			        else
			         continue;
			       else
			        continue;
			     else if(*(cache_0 + pixel[10]) < c_b)
			      if(*(cache_0 + pixel[2]) > cb)
			       if(*(cache_0 + pixel[5]) > cb)
			        if(*(cache_0 + pixel[7]) > cb)
			         if(*(cache_0 + pixel[1]) > cb)
			          goto success;
			         else
			          if(*(cache_0 + pixel[9]) > cb)
			           goto success;
			          else
			           continue;
			        else
			         continue;
			       else
			        continue;
			      else
			       continue;
			     else
			      if(*(cache_0 + pixel[2]) > cb)
			       if(*(cache_0 + pixel[7]) > cb)
			        if(*(cache_0 + pixel[9]) > cb)
			         if(*(cache_0 + pixel[5]) > cb)
			          goto success;
			         else
			          continue;
			        else
			         if(*(cache_0 + pixel[1]) > cb)
			          if(*(cache_0 + pixel[5]) > cb)
			           goto success;
			          else
			           continue;
			         else
			          continue;
			       else
			        continue;
			      else
			       continue;
			    else if(*(cache_0 + pixel[6]) < c_b)
			     if(*(cache_0 + pixel[11]) > cb)
			      if(*(cache_0 + pixel[9]) > cb)
			       if(*(cache_0 + pixel[10]) > cb)
			        if(*(cache_0 + -3) > cb)
			         if(*(cache_0 + pixel[13]) > cb)
			          if(*(cache_0 + pixel[14]) > cb)
			           if(*(cache_0 + pixel[7]) > cb)
			            goto success;
			           else
			            if(*(cache_0 + pixel[15]) > cb)
			             goto success;
			            else
			             continue;
			          else
			           continue;
			         else
			          continue;
			        else
			         continue;
			       else
			        continue;
			      else
			       continue;
			     else
			      continue;
			    else
			     if(*(cache_0 + pixel[14]) > cb)
			      if(*(cache_0 + pixel[11]) > cb)
			       if(*(cache_0 + pixel[10]) > cb)
			        if(*(cache_0 + pixel[13]) > cb)
			         if(*(cache_0 + pixel[7]) > cb)
			          if(*(cache_0 + -3) > cb)
			           if(*(cache_0 + pixel[9]) > cb)
			            goto success;
			           else
			            continue;
			          else
			           continue;
			         else if(*(cache_0 + pixel[7]) < c_b)
			          if(*(cache_0 + pixel[9]) > cb)
			           if(*(cache_0 + -3) > cb)
			            if(*(cache_0 + pixel[15]) > cb)
			             goto success;
			            else
			             continue;
			           else
			            continue;
			          else
			           continue;
			         else
			          if(*(cache_0 + pixel[15]) > cb)
			           if(*(cache_0 + -3) > cb)
			            if(*(cache_0 + pixel[9]) > cb)
			             goto success;
			            else
			             continue;
			           else
			            continue;
			          else
			           continue;
			        else
			         continue;
			       else
			        continue;
			      else
			       continue;
			     else
			      continue;
			   else if(*(cache_0 + pixel[3]) < c_b)
			    if(*(cache_0 + pixel[10]) > cb)
			     if(*(cache_0 + pixel[11]) > cb)
			      if(*(cache_0 + pixel[9]) > cb)
			       if(*(cache_0 + pixel[7]) > cb)
			        if(*(cache_0 + pixel[6]) > cb)
			         if(*(cache_0 + pixel[5]) > cb)
			          goto success;
			         else
			          if(*(cache_0 + -3) > cb)
			           if(*(cache_0 + pixel[13]) > cb)
			            goto success;
			           else
			            continue;
			          else
			           continue;
			        else
			         if(*(cache_0 + -3) > cb)
			          if(*(cache_0 + pixel[13]) > cb)
			           if(*(cache_0 + pixel[14]) > cb)
			            goto success;
			           else
			            continue;
			          else
			           continue;
			         else
			          continue;
			       else
			        if(*(cache_0 + -3) > cb)
			         if(*(cache_0 + pixel[13]) > cb)
			          if(*(cache_0 + pixel[14]) > cb)
			           if(*(cache_0 + pixel[15]) > cb)
			            goto success;
			           else
			            continue;
			          else
			           continue;
			         else
			          continue;
			        else
			         continue;
			      else
			       continue;
			     else
			      continue;
			    else
			     continue;
			   else
			    if(*(cache_0 + pixel[11]) > cb)
			     if(*(cache_0 + pixel[10]) > cb)
			      if(*(cache_0 + pixel[6]) > cb)
			       if(*(cache_0 + pixel[7]) > cb)
			        if(*(cache_0 + pixel[9]) > cb)
			         if(*(cache_0 + pixel[5]) > cb)
			          goto success;
			         else if(*(cache_0 + pixel[5]) < c_b)
			          if(*(cache_0 + -3) > cb)
			           if(*(cache_0 + pixel[13]) > cb)
			            goto success;
			           else
			            continue;
			          else
			           continue;
			         else
			          if(*(cache_0 + pixel[13]) > cb)
			           if(*(cache_0 + -3) > cb)
			            goto success;
			           else
			            continue;
			          else
			           continue;
			        else
			         continue;
			       else if(*(cache_0 + pixel[7]) < c_b)
			        if(*(cache_0 + pixel[9]) > cb)
			         if(*(cache_0 + -3) > cb)
			          if(*(cache_0 + pixel[13]) > cb)
			           if(*(cache_0 + pixel[14]) > cb)
			            if(*(cache_0 + pixel[15]) > cb)
			             goto success;
			            else
			             continue;
			           else
			            continue;
			          else
			           continue;
			         else
			          continue;
			        else
			         continue;
			       else
			        if(*(cache_0 + pixel[15]) > cb)
			         if(*(cache_0 + pixel[13]) > cb)
			          if(*(cache_0 + pixel[14]) > cb)
			           if(*(cache_0 + pixel[9]) > cb)
			            if(*(cache_0 + -3) > cb)
			             goto success;
			            else
			             continue;
			           else
			            continue;
			          else
			           continue;
			         else
			          continue;
			        else
			         continue;
			      else if(*(cache_0 + pixel[6]) < c_b)
			       if(*(cache_0 + pixel[9]) > cb)
			        if(*(cache_0 + -3) > cb)
			         if(*(cache_0 + pixel[13]) > cb)
			          if(*(cache_0 + pixel[14]) > cb)
			           if(*(cache_0 + pixel[7]) > cb)
			            goto success;
			           else
			            if(*(cache_0 + pixel[15]) > cb)
			             goto success;
			            else
			             continue;
			          else
			           continue;
			         else
			          continue;
			        else
			         continue;
			       else
			        continue;
			      else
			       if(*(cache_0 + pixel[14]) > cb)
			        if(*(cache_0 + -3) > cb)
			         if(*(cache_0 + pixel[7]) > cb)
			          if(*(cache_0 + pixel[13]) > cb)
			           if(*(cache_0 + pixel[9]) > cb)
			            goto success;
			           else
			            continue;
			          else
			           continue;
			         else if(*(cache_0 + pixel[7]) < c_b)
			          if(*(cache_0 + pixel[9]) > cb)
			           if(*(cache_0 + pixel[13]) > cb)
			            if(*(cache_0 + pixel[15]) > cb)
			             goto success;
			            else
			             continue;
			           else
			            continue;
			          else
			           continue;
			         else
			          if(*(cache_0 + pixel[15]) > cb)
			           if(*(cache_0 + pixel[9]) > cb)
			            if(*(cache_0 + pixel[13]) > cb)
			             goto success;
			            else
			             continue;
			           else
			            continue;
			          else
			           continue;
			        else
			         continue;
			       else
			        continue;
			     else
			      continue;
			    else
			     continue;
			  else if(*(cache_0 + 3) < c_b)
			   if(*(cache_0 + -3) > cb)
			    if(*(cache_0 + pixel[10]) > cb)
			     if(*(cache_0 + pixel[9]) > cb)
			      if(*(cache_0 + pixel[7]) > cb)
			       if(*(cache_0 + pixel[14]) > cb)
			        if(*(cache_0 + pixel[11]) > cb)
			         if(*(cache_0 + pixel[13]) > cb)
			          goto success;
			         else
			          if(*(cache_0 + pixel[5]) > cb)
			           if(*(cache_0 + pixel[6]) > cb)
			            goto success;
			           else
			            continue;
			          else
			           continue;
			        else
			         continue;
			       else
			        if(*(cache_0 + pixel[6]) > cb)
			         if(*(cache_0 + pixel[11]) > cb)
			          if(*(cache_0 + pixel[13]) > cb)
			           goto success;
			          else
			           if(*(cache_0 + pixel[5]) > cb)
			            goto success;
			           else
			            continue;
			         else
			          continue;
			        else
			         continue;
			      else if(*(cache_0 + pixel[7]) < c_b)
			       if(*(cache_0 + pixel[13]) > cb)
			        if(*(cache_0 + pixel[11]) > cb)
			         if(*(cache_0 + pixel[14]) > cb)
			          if(*(cache_0 + pixel[15]) > cb)
			           goto success;
			          else
			           continue;
			         else
			          continue;
			        else
			         continue;
			       else
			        continue;
			      else
			       if(*(cache_0 + pixel[15]) > cb)
			        if(*(cache_0 + pixel[14]) > cb)
			         if(*(cache_0 + pixel[11]) > cb)
			          if(*(cache_0 + pixel[13]) > cb)
			           goto success;
			          else
			           continue;
			         else
			          continue;
			        else
			         continue;
			       else
			        continue;
			     else
			      continue;
			    else
			     continue;
			   else
			    continue;
			  else
			   if(*(cache_0 + -3) > cb)
			    if(*(cache_0 + pixel[10]) > cb)
			     if(*(cache_0 + pixel[6]) > cb)
			      if(*(cache_0 + pixel[13]) > cb)
			       if(*(cache_0 + pixel[11]) > cb)
			        if(*(cache_0 + pixel[9]) > cb)
			         if(*(cache_0 + pixel[7]) > cb)
			          goto success;
			         else if(*(cache_0 + pixel[7]) < c_b)
			          if(*(cache_0 + pixel[14]) > cb)
			           if(*(cache_0 + pixel[15]) > cb)
			            goto success;
			           else
			            continue;
			          else
			           continue;
			         else
			          if(*(cache_0 + pixel[15]) > cb)
			           if(*(cache_0 + pixel[14]) > cb)
			            goto success;
			           else
			            continue;
			          else
			           continue;
			        else
			         continue;
			       else
			        continue;
			      else if(*(cache_0 + pixel[13]) < c_b)
			       if(*(cache_0 + pixel[5]) > cb)
			        if(*(cache_0 + pixel[7]) > cb)
			         if(*(cache_0 + pixel[9]) > cb)
			          if(*(cache_0 + pixel[11]) > cb)
			           goto success;
			          else
			           continue;
			         else
			          continue;
			        else
			         continue;
			       else
			        continue;
			      else
			       if(*(cache_0 + pixel[5]) > cb)
			        if(*(cache_0 + pixel[11]) > cb)
			         if(*(cache_0 + pixel[9]) > cb)
			          if(*(cache_0 + pixel[7]) > cb)
			           goto success;
			          else
			           continue;
			         else
			          continue;
			        else
			         continue;
			       else
			        continue;
			     else if(*(cache_0 + pixel[6]) < c_b)
			      if(*(cache_0 + pixel[14]) > cb)
			       if(*(cache_0 + pixel[11]) > cb)
			        if(*(cache_0 + pixel[13]) > cb)
			         if(*(cache_0 + pixel[9]) > cb)
			          if(*(cache_0 + pixel[15]) > cb)
			           goto success;
			          else
			           if(*(cache_0 + pixel[7]) > cb)
			            goto success;
			           else
			            continue;
			         else
			          continue;
			        else
			         continue;
			       else
			        continue;
			      else
			       continue;
			     else
			      if(*(cache_0 + pixel[14]) > cb)
			       if(*(cache_0 + pixel[7]) > cb)
			        if(*(cache_0 + pixel[11]) > cb)
			         if(*(cache_0 + pixel[13]) > cb)
			          if(*(cache_0 + pixel[9]) > cb)
			           goto success;
			          else
			           continue;
			         else
			          continue;
			        else
			         continue;
			       else if(*(cache_0 + pixel[7]) < c_b)
			        if(*(cache_0 + pixel[9]) > cb)
			         if(*(cache_0 + pixel[11]) > cb)
			          if(*(cache_0 + pixel[13]) > cb)
			           if(*(cache_0 + pixel[15]) > cb)
			            goto success;
			           else
			            continue;
			          else
			           continue;
			         else
			          continue;
			        else
			         continue;
			       else
			        if(*(cache_0 + pixel[15]) > cb)
			         if(*(cache_0 + pixel[9]) > cb)
			          if(*(cache_0 + pixel[13]) > cb)
			           if(*(cache_0 + pixel[11]) > cb)
			            goto success;
			           else
			            continue;
			          else
			           continue;
			         else
			          continue;
			        else
			         continue;
			      else
			       continue;
			    else
			     continue;
			   else
			    continue;
			 else if(*(cache_0 + pixel[8]) < c_b)
			  if(*(cache_0 + 3) > cb)
			   if(*(cache_0 + -3) < c_b)
			    if(*(cache_0 + pixel[10]) < c_b)
			     if(*(cache_0 + pixel[9]) < c_b)
			      if(*(cache_0 + pixel[14]) > cb)
			       if(*(cache_0 + pixel[6]) < c_b)
			        if(*(cache_0 + pixel[11]) < c_b)
			         if(*(cache_0 + pixel[7]) < c_b)
			          if(*(cache_0 + pixel[5]) < c_b)
			           goto success;
			          else
			           if(*(cache_0 + pixel[13]) < c_b)
			            goto success;
			           else
			            continue;
			         else
			          continue;
			        else
			         continue;
			       else
			        continue;
			      else if(*(cache_0 + pixel[14]) < c_b)
			       if(*(cache_0 + pixel[11]) < c_b)
			        if(*(cache_0 + pixel[13]) < c_b)
			         if(*(cache_0 + pixel[7]) < c_b)
			          goto success;
			         else
			          if(*(cache_0 + pixel[15]) < c_b)
			           goto success;
			          else
			           continue;
			        else
			         if(*(cache_0 + pixel[5]) < c_b)
			          if(*(cache_0 + pixel[6]) < c_b)
			           if(*(cache_0 + pixel[7]) < c_b)
			            goto success;
			           else
			            continue;
			          else
			           continue;
			         else
			          continue;
			       else
			        continue;
			      else
			       if(*(cache_0 + pixel[6]) < c_b)
			        if(*(cache_0 + pixel[13]) > cb)
			         if(*(cache_0 + pixel[5]) < c_b)
			          if(*(cache_0 + pixel[7]) < c_b)
			           if(*(cache_0 + pixel[11]) < c_b)
			            goto success;
			           else
			            continue;
			          else
			           continue;
			         else
			          continue;
			        else if(*(cache_0 + pixel[13]) < c_b)
			         if(*(cache_0 + pixel[7]) < c_b)
			          if(*(cache_0 + pixel[11]) < c_b)
			           goto success;
			          else
			           continue;
			         else
			          continue;
			        else
			         if(*(cache_0 + pixel[5]) < c_b)
			          if(*(cache_0 + pixel[11]) < c_b)
			           if(*(cache_0 + pixel[7]) < c_b)
			            goto success;
			           else
			            continue;
			          else
			           continue;
			         else
			          continue;
			       else
			        continue;
			     else
			      continue;
			    else
			     continue;
			   else
			    continue;
			  else if(*(cache_0 + 3) < c_b)
			   if(*(cache_0 + pixel[10]) > cb)
			    if(*(cache_0 + pixel[2]) < c_b)
			     if(*(cache_0 + pixel[5]) < c_b)
			      if(*(cache_0 + pixel[3]) < c_b)
			       if(*(cache_0 + pixel[6]) < c_b)
			        if(*(cache_0 + pixel[9]) < c_b)
			         if(*(cache_0 + pixel[7]) < c_b)
			          goto success;
			         else
			          continue;
			        else
			         if(*(cache_0 + pixel[1]) < c_b)
			          if(*(cache_0 + pixel[7]) < c_b)
			           goto success;
			          else
			           continue;
			         else
			          continue;
			       else
			        continue;
			      else
			       continue;
			     else
			      continue;
			    else
			     continue;
			   else if(*(cache_0 + pixel[10]) < c_b)
			    if(*(cache_0 + pixel[6]) > cb)
			     if(*(cache_0 + pixel[14]) < c_b)
			      if(*(cache_0 + pixel[11]) < c_b)
			       if(*(cache_0 + pixel[9]) < c_b)
			        if(*(cache_0 + -3) < c_b)
			         if(*(cache_0 + pixel[13]) < c_b)
			          if(*(cache_0 + pixel[7]) < c_b)
			           goto success;
			          else
			           if(*(cache_0 + pixel[15]) < c_b)
			            goto success;
			           else
			            continue;
			         else
			          continue;
			        else
			         continue;
			       else
			        continue;
			      else
			       continue;
			     else
			      continue;
			    else if(*(cache_0 + pixel[6]) < c_b)
			     if(*(cache_0 + pixel[3]) > cb)
			      if(*(cache_0 + pixel[11]) < c_b)
			       if(*(cache_0 + pixel[9]) < c_b)
			        if(*(cache_0 + pixel[7]) < c_b)
			         if(*(cache_0 + pixel[5]) < c_b)
			          goto success;
			         else
			          if(*(cache_0 + -3) < c_b)
			           if(*(cache_0 + pixel[13]) < c_b)
			            goto success;
			           else
			            continue;
			          else
			           continue;
			        else
			         if(*(cache_0 + -3) < c_b)
			          if(*(cache_0 + pixel[13]) < c_b)
			           if(*(cache_0 + pixel[14]) < c_b)
			            if(*(cache_0 + pixel[15]) < c_b)
			             goto success;
			            else
			             continue;
			           else
			            continue;
			          else
			           continue;
			         else
			          continue;
			       else
			        continue;
			      else
			       continue;
			     else if(*(cache_0 + pixel[3]) < c_b)
			      if(*(cache_0 + pixel[7]) > cb)
			       if(*(cache_0 + pixel[9]) < c_b)
			        if(*(cache_0 + pixel[11]) < c_b)
			         if(*(cache_0 + -3) < c_b)
			          if(*(cache_0 + pixel[13]) < c_b)
			           if(*(cache_0 + pixel[14]) < c_b)
			            if(*(cache_0 + pixel[15]) < c_b)
			             goto success;
			            else
			             continue;
			           else
			            continue;
			          else
			           continue;
			         else
			          continue;
			        else
			         continue;
			       else
			        continue;
			      else if(*(cache_0 + pixel[7]) < c_b)
			       if(*(cache_0 + pixel[5]) > cb)
			        if(*(cache_0 + pixel[9]) < c_b)
			         if(*(cache_0 + pixel[11]) < c_b)
			          if(*(cache_0 + -3) < c_b)
			           if(*(cache_0 + pixel[13]) < c_b)
			            goto success;
			           else
			            continue;
			          else
			           continue;
			         else
			          continue;
			        else
			         continue;
			       else if(*(cache_0 + pixel[5]) < c_b)
			        if(*(cache_0 + pixel[9]) < c_b)
			         goto success;
			        else
			         if(*(cache_0 + pixel[1]) < c_b)
			          if(*(cache_0 + pixel[2]) < c_b)
			           goto success;
			          else
			           continue;
			         else
			          continue;
			       else
			        if(*(cache_0 + pixel[13]) < c_b)
			         if(*(cache_0 + pixel[11]) < c_b)
			          if(*(cache_0 + -3) < c_b)
			           if(*(cache_0 + pixel[9]) < c_b)
			            goto success;
			           else
			            continue;
			          else
			           continue;
			         else
			          continue;
			        else
			         continue;
			      else
			       if(*(cache_0 + pixel[15]) < c_b)
			        if(*(cache_0 + -3) < c_b)
			         if(*(cache_0 + pixel[14]) < c_b)
			          if(*(cache_0 + pixel[9]) < c_b)
			           if(*(cache_0 + pixel[11]) < c_b)
			            if(*(cache_0 + pixel[13]) < c_b)
			             goto success;
			            else
			             continue;
			           else
			            continue;
			          else
			           continue;
			         else
			          continue;
			        else
			         continue;
			       else
			        continue;
			     else
			      if(*(cache_0 + pixel[11]) < c_b)
			       if(*(cache_0 + pixel[9]) < c_b)
			        if(*(cache_0 + pixel[7]) > cb)
			         if(*(cache_0 + -3) < c_b)
			          if(*(cache_0 + pixel[13]) < c_b)
			           if(*(cache_0 + pixel[14]) < c_b)
			            if(*(cache_0 + pixel[15]) < c_b)
			             goto success;
			            else
			             continue;
			           else
			            continue;
			          else
			           continue;
			         else
			          continue;
			        else if(*(cache_0 + pixel[7]) < c_b)
			         if(*(cache_0 + pixel[5]) > cb)
			          if(*(cache_0 + -3) < c_b)
			           if(*(cache_0 + pixel[13]) < c_b)
			            goto success;
			           else
			            continue;
			          else
			           continue;
			         else if(*(cache_0 + pixel[5]) < c_b)
			          goto success;
			         else
			          if(*(cache_0 + pixel[13]) < c_b)
			           if(*(cache_0 + -3) < c_b)
			            goto success;
			           else
			            continue;
			          else
			           continue;
			        else
			         if(*(cache_0 + pixel[15]) < c_b)
			          if(*(cache_0 + pixel[14]) < c_b)
			           if(*(cache_0 + pixel[13]) < c_b)
			            if(*(cache_0 + -3) < c_b)
			             goto success;
			            else
			             continue;
			           else
			            continue;
			          else
			           continue;
			         else
			          continue;
			       else
			        continue;
			      else
			       continue;
			    else
			     if(*(cache_0 + pixel[14]) < c_b)
			      if(*(cache_0 + -3) < c_b)
			       if(*(cache_0 + pixel[7]) > cb)
			        if(*(cache_0 + pixel[9]) < c_b)
			         if(*(cache_0 + pixel[11]) < c_b)
			          if(*(cache_0 + pixel[13]) < c_b)
			           if(*(cache_0 + pixel[15]) < c_b)
			            goto success;
			           else
			            continue;
			          else
			           continue;
			         else
			          continue;
			        else
			         continue;
			       else if(*(cache_0 + pixel[7]) < c_b)
			        if(*(cache_0 + pixel[9]) < c_b)
			         if(*(cache_0 + pixel[13]) < c_b)
			          if(*(cache_0 + pixel[11]) < c_b)
			           goto success;
			          else
			           continue;
			         else
			          continue;
			        else
			         continue;
			       else
			        if(*(cache_0 + pixel[15]) < c_b)
			         if(*(cache_0 + pixel[9]) < c_b)
			          if(*(cache_0 + pixel[13]) < c_b)
			           if(*(cache_0 + pixel[11]) < c_b)
			            goto success;
			           else
			            continue;
			          else
			           continue;
			         else
			          continue;
			        else
			         continue;
			      else
			       continue;
			     else
			      continue;
			   else
			    if(*(cache_0 + pixel[2]) < c_b)
			     if(*(cache_0 + pixel[6]) < c_b)
			      if(*(cache_0 + pixel[3]) < c_b)
			       if(*(cache_0 + pixel[9]) > cb)
			        if(*(cache_0 + pixel[1]) < c_b)
			         if(*(cache_0 + pixel[5]) < c_b)
			          if(*(cache_0 + pixel[7]) < c_b)
			           goto success;
			          else
			           continue;
			         else
			          continue;
			        else
			         continue;
			       else if(*(cache_0 + pixel[9]) < c_b)
			        if(*(cache_0 + pixel[7]) < c_b)
			         if(*(cache_0 + pixel[5]) < c_b)
			          goto success;
			         else
			          continue;
			        else
			         continue;
			       else
			        if(*(cache_0 + pixel[1]) < c_b)
			         if(*(cache_0 + pixel[7]) < c_b)
			          if(*(cache_0 + pixel[5]) < c_b)
			           goto success;
			          else
			           continue;
			         else
			          continue;
			        else
			         continue;
			      else
			       continue;
			     else
			      continue;
			    else
			     continue;
			  else
			   if(*(cache_0 + -3) < c_b)
			    if(*(cache_0 + pixel[10]) < c_b)
			     if(*(cache_0 + pixel[6]) > cb)
			      if(*(cache_0 + pixel[14]) < c_b)
			       if(*(cache_0 + pixel[11]) < c_b)
			        if(*(cache_0 + pixel[9]) < c_b)
			         if(*(cache_0 + pixel[15]) < c_b)
			          if(*(cache_0 + pixel[13]) < c_b)
			           goto success;
			          else
			           continue;
			         else
			          if(*(cache_0 + pixel[7]) < c_b)
			           if(*(cache_0 + pixel[13]) < c_b)
			            goto success;
			           else
			            continue;
			          else
			           continue;
			        else
			         continue;
			       else
			        continue;
			      else
			       continue;
			     else if(*(cache_0 + pixel[6]) < c_b)
			      if(*(cache_0 + pixel[11]) < c_b)
			       if(*(cache_0 + pixel[13]) > cb)
			        if(*(cache_0 + pixel[5]) < c_b)
			         if(*(cache_0 + pixel[7]) < c_b)
			          if(*(cache_0 + pixel[9]) < c_b)
			           goto success;
			          else
			           continue;
			         else
			          continue;
			        else
			         continue;
			       else if(*(cache_0 + pixel[13]) < c_b)
			        if(*(cache_0 + pixel[7]) > cb)
			         if(*(cache_0 + pixel[9]) < c_b)
			          if(*(cache_0 + pixel[14]) < c_b)
			           if(*(cache_0 + pixel[15]) < c_b)
			            goto success;
			           else
			            continue;
			          else
			           continue;
			         else
			          continue;
			        else if(*(cache_0 + pixel[7]) < c_b)
			         if(*(cache_0 + pixel[9]) < c_b)
			          goto success;
			         else
			          continue;
			        else
			         if(*(cache_0 + pixel[15]) < c_b)
			          if(*(cache_0 + pixel[14]) < c_b)
			           if(*(cache_0 + pixel[9]) < c_b)
			            goto success;
			           else
			            continue;
			          else
			           continue;
			         else
			          continue;
			       else
			        if(*(cache_0 + pixel[5]) < c_b)
			         if(*(cache_0 + pixel[9]) < c_b)
			          if(*(cache_0 + pixel[7]) < c_b)
			           goto success;
			          else
			           continue;
			         else
			          continue;
			        else
			         continue;
			      else
			       continue;
			     else
			      if(*(cache_0 + pixel[14]) < c_b)
			       if(*(cache_0 + pixel[7]) < c_b)
			        if(*(cache_0 + pixel[13]) < c_b)
			         if(*(cache_0 + pixel[9]) < c_b)
			          if(*(cache_0 + pixel[11]) < c_b)
			           goto success;
			          else
			           continue;
			         else
			          continue;
			        else
			         continue;
			       else
			        if(*(cache_0 + pixel[15]) < c_b)
			         if(*(cache_0 + pixel[9]) < c_b)
			          if(*(cache_0 + pixel[11]) < c_b)
			           if(*(cache_0 + pixel[13]) < c_b)
			            goto success;
			           else
			            continue;
			          else
			           continue;
			         else
			          continue;
			        else
			         continue;
			      else
			       continue;
			    else
			     continue;
			   else
			    continue;
			 else
			  continue;

			success:
				//corners.push_back(ImageRef(cache_0-line_min, y));
				corners.push_back(cv::Point2i(cache_0 - line_min, y) );
		
			
		} // end chache (col) for
	} // end y for (rows)
}
}
