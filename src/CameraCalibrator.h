// -*- c++ -*-
// Copyright 2008 Isis Innovation Limited

#ifndef __CAMERACALIBRATOR_H
#define __CAMERACALIBRATOR_H
#include "CalibImage.h"
#include "VideoSource_ros.h"//modified Loianno
#include <gvars3/gvars3.h>
#include <vector>
#include "GLWindow2.h"

namespace PTAMM {

class CameraCalibrator
{
public:
  CameraCalibrator();
  void Run();
  //////////////////modified Loianno////////////////
  void imageCallback(const sensor_msgs::ImageConstPtr & img);
  ros::NodeHandle nh;
  /////////////////////////////////////////////////
  
protected:
  void Reset();
  void HandleFrame(CVD::Image<CVD::byte> imFrame);
  static void MainLoopCallback(void* pvUserData);
  void MainLoopStep();
  VideoSource mVideoSource;
  
  GLWindow2 mGLWindow;
  ATANCamera mCamera;
  bool mbDone;

  std::vector<CalibImage> mvCalibImgs;
  void OptimizeOneStep();
  
  bool mbGrabNextFrame;
  GVars3::gvar3<int> mgvnOptimizing;
  GVars3::gvar3<int> mgvnShowImage;
  GVars3::gvar3<int> mgvnDisableDistortion;
  double mdMeanPixelError;

  void GUICommandHandler(std::string sCommand, std::string sParams);
  static void GUICommandCallBack(void* ptr, std::string sCommand, std::string sParams);
};

}

#endif
