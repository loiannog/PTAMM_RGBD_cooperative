// Copyright 2009 Isis Innovation Limited
// This is the main extry point for PTAMM
#include <stdlib.h>
#include <iostream>
#include <gvars3/instances.h>
#include "System.h"
#include <ParamsPTAMM.h>

using namespace std;
using namespace GVars3;

int main(int argc, char** argv)
{
  ////////////////////////////modified Loianno/////////////////////////////////////
  ros::init(argc, argv, "PTAMM_RGBD_cooperative");

  cout << "  ... got video source." << endl;
  ////////////////////////////////////////////////////////////////////////////////
  cout << "  Welcome to PTAMM " << endl;
  cout << "  ---------------- " << endl;
  cout << "  Parallel tracking and multiple mapping" << endl;
  cout << "  Copyright (C) Isis Innovation Limited 2009 " << endl;
  cout << endl;
  cout << "  Parsing camera.cfg ...." << endl;
  std::string path_file;
  ros::NodeHandle n("~");
  n.param<string>("path_file", path_file, "/home/odroid/git/PTAMM_RGBD_cooperative/");
  ///////////////////////////modified Loianno////////////////////////////////////////////////////
  GUI.LoadFile(path_file+"camera.cfg");
  //GUI.LoadFile(path_file+"settings.cfg");
  //////////////////////////////////////////////////////////////////////////////////////////////////

  GUI.StartParserThread(); // Start parsing of the console input
  atexit(GUI.StopParserThread);



  try
  {
    PtammParameters mPtammParameters;//load static and dynamic parameters
    System s;
    //load parameters to build the global map grid
    s.Run();
  }
  catch(CVD::Exceptions::All e)
  {
    cout << endl;
    cout << "!! Failed to run system; got exception. " << endl;
    cout << "   Exception was: " << endl;
    cout << e.what << endl;
  }

  return 0;
}








