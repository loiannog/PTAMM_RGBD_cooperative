//-*- C++ -*-
// Copyright 2008 Isis Innovation Limited
// 
// This header declares the Tracker class.
// The Tracker is one of main components of the system,
// and is responsible for determining the pose of a camera
// from a video feed. It uses the Map to track, and communicates 
// with the MapMaker (which runs in a different thread)
// to help construct this map.
//
// Initially there is no map, so the Tracker also has a mode to 
// do simple patch tracking across a stereo pair. This is handled 
// by the TrackForInitialMap() method and associated sub-methods. 
// Once there is a map, TrackMap() is used.
//
// Externally, the tracker should be used by calling TrackFrame()
// with every new input video frame. This then calls either 
// TrackForInitialMap() or TrackMap() as appropriate.
//

#ifndef __TRACKER_H
#define __TRACKER_H

#include "MapMaker.h"
#include "ATANCamera.h"
#include "MiniPatch.h"
#include "Relocaliser.h"

#include <sstream>
#include <vector>
#include <list>
/////////////////////////////modified Loianno/////////////////////////////////
#include <ParamsPTAMM.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/image_encodings.h>
#include <TooN/SVD.h>
#include <boost/thread/locks.hpp>
#include <boost/thread.hpp>
#include <boost/date_time.hpp>
#include <boost/bind.hpp>
#include <boost/array.hpp>
#include <boost/thread/thread.hpp>
//////////////////////////////////////////////////////////////////////////////
#define NUM_LOST_FRAMES 3

namespace PTAMM {

class TrackerData;
struct Trail    // This struct is used for initial correspondences of the first stereo pair.
{
  MiniPatch mPatch;
  CVD::ImageRef irCurrentPos;
  CVD::ImageRef irInitialPos;
};

class Tracker
{
public:
	Tracker(CVD::ImageRef irVideoSize, const ATANCamera &c, std::vector<Map*> &maps, Map *m, MapMaker &mm);
  
  // TrackFrame is the main working part of the tracker: call this every frame.
  void TrackFrame(CVD::Image<CVD::byte> &imFrame, bool bDraw, const sensor_msgs::ImageConstPtr & depth_msg);

  inline SE3<> GetCurrentPose() { return mse3CamFromWorld; }
  inline bool IsLost() { return (mnLostFrames > NUM_LOST_FRAMES); }
  
  // Gets messages to be printed on-screen for the user.
  std::string GetMessageForUser();

  bool SwitchMap(Map *map);
  void SetNewMap(Map * map);
  void ForceRecovery() { if(mnLostFrames < NUM_LOST_FRAMES) mnLostFrames = NUM_LOST_FRAMES; }
  void Reset();                   // Restart from scratch. Also tells the mapmaker to reset itself.

  bool HandleKeyPress( std::string sKey );    // act on a key press (new addition for PTAMM)
  bool trail_sempahore;
///////////////////////////////////modified Loianno///////////////////////////////
  KeyFrame mCurrentKF;            // The current working frame as a keyframe struct//modified Loianno
  bool current_key_Frame_added_to_map;//modified Loianno
  enum {BAD, DODGY, GOOD} mTrackingQuality;//modified Loianno
  Map *mpMap;         //modified Loianno
  inline Matrix<6> GetCurrentCov() { return mmCovariances;}
  TooN::Matrix<6>	mmCovariances;		// covariance of current converged pose estimate
  CVD::Image<TooN::Vector<2> > & ComputeGrid();             // Computes the reference grid;
  std::list<Trail> & getTrails(){return  mlTrails;};
  bool getTrailTrackingComplete(){return mnInitialStage == TRAIL_TRACKING_COMPLETE;};
  bool getTrailTrackingStarted(){return mnInitialStage == TRAIL_TRACKING_STARTED;};
  CVD::Image<TooN::Vector<2> > mProjVertices;
  bool mbUserPressedSpacebar;
  boost::shared_mutex access;
  int num_tracker;
  double scale_factor;//modified Loianno
int counter;
int getTrackingQuality(){return mTrackingQuality;}
void  mean_variance(std::vector <double> scale_fact, int size);
std::vector <double> scale_factor_final;
double scale_factor_final_scalar_tracker;
bool depth_initialization;
////////////////////////////////////////////////////////////////////////////////
protected:
  
  // The major components to which the tracker needs access:
  std::vector<Map*> & mvpMaps;     // Reference to all of the maps
  //Map *mpMap;         //modified Loianno            // The map, consisting of points and keyframes
  MapMaker &mMapMaker;            // The class which maintains the map
  ATANCamera mCamera;             // Projection model
  Relocaliser mRelocaliser;       // Relocalisation module

  CVD::ImageRef mirSize;          // Image size of whole image

  void ResetCommon();              // Common reset code for the following two functions
  void RenderGrid();              // Draws the reference grid

  // The following members are used for initial map tracking (to get the first stereo pair and correspondences):
  void TrackForInitialDepth(const sensor_msgs::ImageConstPtr & depth_msg);//modified Loianno
  void TrackForInitialMap();      // This is called by TrackFrame if there is not a map yet.
  enum {TRAIL_TRACKING_NOT_STARTED,
	TRAIL_TRACKING_STARTED,
	TRAIL_TRACKING_COMPLETE} mnInitialStage;  // How far are we towards making the initial map?
  void TrailTracking_Start();     // First frame of initial trail tracking. Called by TrackForInitialMap.
  int  TrailTracking_Advance();   // Steady-state of initial trail tracking. Called by TrackForInitialMap.
  std::list<Trail> mlTrails;      // Used by trail tracking
  KeyFrame mPreviousFrameKF;      // Used by trail tracking to check married matches
  KeyFrame mFirstKF;              // First of the stereo pair

  // Methods for tracking the map once it has been made:
  void TrackMap();                // Called by TrackFrame if there is a map.
  void AssessTrackingQuality();   // Heuristics to choose between good, poor, bad.
  void ApplyMotionModel();        // Decaying velocity motion model applied prior to TrackMap
  void UpdateMotionModel();       // Motion model is updated after TrackMap
  int SearchForPoints(std::vector<TrackerData*> &vTD, 
		      int nRange, 
		      int nFineIts);  // Finds points in the image
  Vector<6> CalcPoseUpdate(std::vector<TrackerData*> vTD, 
			   double dOverrideSigma = 0.0, 
			   bool bMarkOutliers = false); // Updates pose from found points.
  SE3<> mse3CamFromWorld;           // Camera pose: this is what the tracker updates every frame.
  SE3<> mse3StartPos;               // What the camera pose was at the start of the frame.
  Vector<6> mv6CameraVelocity;    // Motion model
  double mdVelocityMagnitude;     // Used to decide on coarse tracking 
  double mdMSDScaledVelocityMagnitude; // Velocity magnitude scaled by relative scene depth.
  bool mbDidCoarse;               // Did tracking use the coarse tracking stage?
  
  bool mbDraw;                    // Should the tracker draw anything to OpenGL?
  
  // Interface with map maker:
  int mnFrame;                    // Frames processed since last reset
  int mnLastKeyFrameDropped;      // Counter of last keyframe inserted.
  void AddNewKeyFrame();          // Gives the current frame to the mapmaker to use as a keyframe
  
  // Tracking quality control:
  int manMeasAttempted[LEVELS];
  int manMeasFound[LEVELS];
  //enum {BAD, DODGY, GOOD} mTrackingQuality;
  int mnLostFrames;
  
  // Relocalisation functions:
  bool AttemptRecovery();         // Called by TrackFrame if tracking is lost.
  bool mbJustRecoveredSoUseCoarse;// Always use coarse tracking after recovery!

  // Frame-to-frame motion init:
  SmallBlurryImage *mpSBILastFrame;
  SmallBlurryImage *mpSBIThisFrame;
  void CalcSBIRotation();
  Vector<6> mv6SBIRot;
  bool mbUseSBIInit;
  
  // User interaction for initial tracking:

  std::ostringstream mMessageForUser;
  
  // GUI interface:
  void GUICommandHandler(std::string sCommand, std::string sParams);
  static void GUICommandCallBack(void* ptr, std::string sCommand, std::string sParams);
  struct Command {std::string sCommand; std::string sParams; };
  std::vector<Command> mvQueuedCommands;

};


}


#endif






