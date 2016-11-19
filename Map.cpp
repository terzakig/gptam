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
  EmptyTrash();
}



void Map::deleteBadPoints() {
    // clean the map
  
  for (unsigned int i = 0; i<vpPoints.size(); i++) {
    if (vpPoints.end() - (vpPoints.begin() + i) <= 0) continue; // see if somehow we got 
								// out of the vector's range (veeery unlikely)...
    MapPoint::Ptr pMP = vpPoints[i]; 
    
    if (pMP.use_count() == 0) {               // checking if the mappoint has been dereferenced (veery unlikely)
      vpPoints.erase( vpPoints.begin() + i );
      continue;
    }
    // And finally, if bad erase as well...
    if (pMP->bBad) vpPoints.erase( vpPoints.begin() + i );
  }
  
   

  
}

void Map::EmptyTrash() {
  
  vpPointsTrash.clear();
  cout <<"DEBUG: Ditched "<<vpPointsTrash.size()<<" trashed map points! I hope..."<<endl;

}


