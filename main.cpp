// ********** George Terzakis 2016 ***************
//
//         University of Portsmouth 
//
// Code is based on PTAM by Klein and Murray (Copyright 2008 Isis Innovation Limited)
//
//
#include <stdlib.h>
#include <iostream>

#include "Persistence/instances.h"
#include "System.h"

using namespace std;
using namespace Persistence;

int main(int argc, char** argv)
{
  
  cout << "  Visual Tracking and Mapping with OpenCV" << endl;
  cout << "  --------------- " << endl;
  cout <<endl<<endl<<" University of Portsmouth 2016";
  cout << "  Based on PTAM by Klein and Murray (Isis Innovation Limited 2008)" << endl;
  cout << "   " << endl;
  cout << endl;

  cout << "  Parsing settings.cfg ...." << endl;
  GUI.LoadFile("settings.cfg");
  
  GUI.StartParserThread(); // Start parsing of the console input
  atexit(GUI.StopParserThread);

  try
  {
    System s;
    s.Run();
  }
  catch(cv::Exception e) {
    
      cout << endl;
      cout << "!! Failed to run System; got exception. " << endl;
      cout << "   Exception was: " << endl;
      //cout << e.what << endl;
      cout <<"At line : " << e.line << endl << e.msg << endl;
    }
    
}










