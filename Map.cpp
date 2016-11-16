// ********* George Terzakis 2016 *************
// *
// 	University of Portsmouth 
//
// This code was based on  PTAM by Klein and Murray (Copyright 2008 Isis Innovation Limited)



#include "Map.h"
#include "MapPoint.h"

Map::Map()
{
  Reset();
}

void Map::Reset()
{
  // delet ALL mappoints and clear the vector
  //for(unsigned int i=0; i<vpPoints.size(); i++) delete vpPoints[i];
  vpPoints.clear(); // clearing all references should do the job!
  // nothing good in the map
  bGood = false; 
  // destroy trashed points list
  //EmptyTrash();
}

// This is not necessary, but could be useful for statistics... We'll see ...
/*void Map::MoveBadPointsToTrash() {
  
  int nBad = 0;
  for(int i = vpPoints.size()-1; i>=0; i--) {
    
      if(vpPoints[i]->bBad) {
	  vpPointsTrash.push_back(vpPoints[i]);
	  vpPoints.erase(vpPoints.begin() + i);
	  nBad++;
      }
  }
}*/

/*void Map::EmptyTrash() {
  
  // destroy all the trashed points and cleatr the vector
  //for(unsigned int i=0; i<vpPointsTrash.size(); i++) delete vpPointsTrash[i];
  // although the above should not be necessary, we just need to clear the references...
  vpPointsTrash.clear();
}*/




