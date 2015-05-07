// Copyright 2009 Isis Innovation Limited
#include "System.h"
#include "OpenGL.h"
#include <gvars3/GStringUtil.h>
#include <stdlib.h>
#include "ATANCamera.h"
#include "MapMaker.h"
#include "Tracker.h"
#include "MapViewer.h"
#include "MapSerializer.h"
#include <opencv/cv.h>
#include <pcl/filters/voxel_grid.h>

#ifdef _LINUX
#include <fcntl.h>
#endif
#include <boost/thread.hpp>
/////////////////////////////////////modified Loianno/////////////////////////////////////

#include <iomanip>
#include <locale>
#include <sstream>
#include <cvd/image_ref.h>
#include "LevelHelpers.h"
#include <string> // this should be already included in <sstream>
#include "depth_traits.h"
#include <Eigen/Geometry>
#include <pcl/common/transforms.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <nav_msgs/Odometry.h>
#include <TooN/SymEigen.h>
#include <utility>

TooN::Vector<3> offset_translation;
TooN::SO3<double> offset_rotation;
ofstream scale_factor_txt;
bool initialization_main;
typedef sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;
class Init_thread_image
{
public:
Init_thread_image(ros::NodeHandle &_nh, message_filters::Subscriber < sensor_msgs::Image > *_pleft,
		message_filters::Subscriber < sensor_msgs::Image > *_pright, System* _point)
{
	n = _nh;
	pleft = _pleft;
	pright = _pright;
	point = _point;
}
void operator()() {


	 Synchronizer<MySyncPolicy> sync1(MySyncPolicy(2), *pleft, *pright);
				 sync1.registerCallback(boost::bind(&System::imageCallback, point, _1, _2));

}
private:
ros::NodeHandle n;
bool f;
System* point;
message_filters::Subscriber < sensor_msgs::Image > *pleft;
message_filters::Subscriber < sensor_msgs::Image > *pright;
};

char c;

typedef union
{
struct /*anonymous*/
{
unsigned char Blue;
unsigned char Green;
unsigned char Red;
unsigned char Alpha;
};
float float_value;
long long_value;
} RGBValue;

Vector<4> MatToQuat(Matrix<3> Rot){
	Vector<4> Quat;
	double tr = Rot[0][0]+ Rot[1][1]+ Rot[2][2];
	int ii;
	ii=0;
	if (Rot[1][1] > Rot[0][0]) ii=1;
	if (Rot[2][2] > Rot[ii][ii]) ii=2;
	double s;
	if (tr >= 0){
		s = sqrt((tr + 1));
		Quat[0] = s * 0.5;
		s = 0.5 / s;
		Quat[1] = (Rot[2][1] - Rot[1][2]) * s;
		Quat[2] = (Rot[0][2] - Rot[2][0]) * s;
		Quat[3] = (Rot[1][0] - Rot[0][1]) * s;
	} else {
		switch(ii) {
		 case 0:
				s = sqrt((Rot[0][0]-Rot[1][1]-Rot[2][2]+1));
				Quat[1] = s * 0.5;
				s = 0.5 / s;

				Quat[2] = (Rot[1][0] + Rot[0][1]) * s;//Update pose estimation

				Quat[3] = (Rot[2][0] + Rot[0][2]) * s;
				Quat[0] = (Rot[2][1] - Rot[1][2]) * s;
				break;
		 case 1:
				s = sqrt((Rot[1][1]-Rot[2][2]-Rot[0][0]+1));
				Quat[2] = s * 0.5;
				s = 0.5 / s;

				Quat[3] = (Rot[2][1] + Rot[1][2]) * s;
				Quat[1] = (Rot[0][1] + Rot[1][0]) * s;
				Quat[0] = (Rot[0][2] - Rot[2][0]) * s;
				break;
		 case 2:
				s = sqrt((Rot[2][2]-Rot[0][0]-Rot[1][1]+1));
				Quat[3] = s * 0.5;
				s = 0.5 / s;

				Quat[1] = (Rot[0][2] + Rot[2][0]) * s;
				Quat[2] = (Rot[1][2] + Rot[2][1]) * s;
				Quat[0] = (Rot[1][0] - Rot[0][1]) * s;
				break;
		 }
	}
	return Quat;
}

Matrix<3> QuatToMat(Vector<4> Quat){
	Matrix<3> Rot;
	double s = Quat[0];
	double x = Quat[1];
	double y = Quat[2];
	double z = Quat[3];
	Fill(Rot) = 1-2*(y*y+z*z),2*(x*y-s*z),2*(x*z+s*y),
			    2*(x*y+s*z),1-2*(x*x+z*z),2*(y*z-s*x),
			    2*(x*z-s*y),2*(y*z+s*x),1-2*(x*x+y*y);
	return Rot;
 }

/////////////////////////////////////////////////////////////////////////////////////////////////



namespace PTAMM {

using namespace CVD;
using namespace std;
using namespace GVars3;


System::System()
  //: mGLWindow(mVideoSource.Size(), "PTAMM")
//: mGLWindow(CVD::ImageRef(ParamsAccess::fixParams->image_width, ParamsAccess::fixParams->image_height), "PTAMM")//modified Loianno
{

	///////////////////////modified Loianno///////////////////////////////
	  scale_factor_final_filtered.resize(2);
	  counter_scale_factor.resize(1);
	  counter_scale_factor[0] = 0;
	  size_old.resize(1);
	  size_old[0] = 0;
	  scale_factor_final_scalar.resize(1);
	  drawfirsttracker = 1;
	  is_relocalized =  false;
	semaphore_dense_srv = false;
	semaphore_copy = true;
	counter_frames = 0;
	TooN::Matrix<3> temp = TooN::Identity;
	Best_estimation = TooN::SE3<double>(temp,TooN::makeVector(0,0,0));
	best_score = 99999999999;
scan_completed = false;
	////////////////////////////////////////////////////////////////////////
  GUI.RegisterCommand("exit", GUICommandCallBack, this);
  GUI.RegisterCommand("quit", GUICommandCallBack, this);
  tracker2_active = false;



  // First, check if the camera is calibrated.
  // If not, we need to run the calibration widget.
  Vector<NUMTRACKERCAMPARAMETERS> vTest;

  vTest = GV3::get<Vector<NUMTRACKERCAMPARAMETERS> >("Camera.Parameters", ATANCamera::mvDefaultParams, HIDDEN);
  mpCamera = new ATANCamera("Camera");
  mpCamera->SetImageSize(CVD::ImageRef(ParamsAccess::fixParams->image_width, ParamsAccess::fixParams->image_height));

  if(vTest == ATANCamera::mvDefaultParams)
  {
    cout << endl;
    cout << "! Camera.Parameters is not set, need to run the CameraCalibrator tool" << endl;
    cout << "  and/or put the Camera.Parameters= line into the appropriate .cfg file." << endl;
    exit(1);
  }

  if(ParamsAccess::fixParams->gui){
    mGLWindow = new GLWindow2(CVD::ImageRef(ParamsAccess::fixParams->image_width, ParamsAccess::fixParams->image_height), "PTAMM");

    //PTAMM commands
    GUI.RegisterCommand("SwitchMap", GUICommandCallBack, this);
    GUI.RegisterCommand("NewMap", GUICommandCallBack, this);
    GUI.RegisterCommand("DeleteMap", GUICommandCallBack, this);
    GUI.RegisterCommand("ResetAll", GUICommandCallBack, this);

    GUI.RegisterCommand("LoadMap", GUICommandCallBack, this);
    GUI.RegisterCommand("SaveMap", GUICommandCallBack, this);
    GUI.RegisterCommand("SaveMaps", GUICommandCallBack, this);

    GV2.Register(mgvnLockMap, "LockMap", 0, SILENT);
    GV2.Register(mgvnDrawMapInfo, "MapInfo", 0, SILENT);
    GV2.Register(mgvnDrawNewTracker, "NewTracker", 0, SILENT);

  }
  //create the first map
  mpMap = new Map();
  mvpMaps.push_back( mpMap );
  mpMap->mapLockManager.Register(this);//register current object to the map

  mpMapMaker = new MapMaker( mvpMaps, mpMap );
  mpTracker = new Tracker(CVD::ImageRef(ParamsAccess::fixParams->image_width, ParamsAccess::fixParams->image_height), *mpCamera, mvpMaps, mpMap, *mpMapMaker);
  //Tracker *g;

  mvpTracker.push_back(mpTracker);//update the list of trackers
  mpMapViewer = new MapViewer(mvpMaps, mpMap, *mGLWindow);

  mpMapSerializer = new MapSerializer( mvpMaps );
  //mpTracker2 = new Tracker(CVD::ImageRef(ParamsAccess::fixParams->image_width, ParamsAccess::fixParams->image_height), *mpCamera, mvpMaps, mpMap, *mpMapMaker);
  //mvpTracker.push_back( mpTracker2 );


#ifdef _LINUX
  GV2.Register(mgvnSaveFIFO, "SaveFIFO", 0, SILENT);
  GV2.Register(mgvnBitrate, "Bitrate", 15000, SILENT);
#endif
  if(ParamsAccess::fixParams->gui){
  GUI.RegisterCommand("KeyPress", GUICommandCallBack, this);
  //These commands have to be registered here as they call the classes created above
  GUI.RegisterCommand("NextMap", GUICommandCallBack, mpMapViewer);
  GUI.RegisterCommand("PrevMap", GUICommandCallBack, mpMapViewer);
  GUI.RegisterCommand("CurrentMap", GUICommandCallBack, mpMapViewer);


  //create the menus
  GUI.ParseLine("GLWindow.AddMenu Menu Menu");
  GUI.ParseLine("Menu.ShowMenu Root");
  GUI.ParseLine("Menu.AddMenuButton Root \"Reset All\" ResetAll Root");
  GUI.ParseLine("Menu.AddMenuButton Root Reset Reset Root");
  GUI.ParseLine("Menu.AddMenuButton Root Spacebar PokeTracker Root");
  GUI.ParseLine("NewTracker=0");
  GUI.ParseLine("Menu.AddMenuToggle Root \"New Tracker\" NewTracker Root");

  GUI.ParseLine("DrawMap=0");

  GUI.ParseLine("GLWindow.AddMenu MapsMenu Maps");
  GUI.ParseLine("MapsMenu.AddMenuButton Root \"New Map\" NewMap Root");
  GUI.ParseLine("MapsMenu.AddMenuButton Root \"Serialize\" \"\" Serial");
  GUI.ParseLine("MapsMenu.AddMenuButton Serial \"Save Maps\" SaveMaps Root");
  GUI.ParseLine("MapsMenu.AddMenuButton Serial \"Save Map\" SaveMap Root");
  GUI.ParseLine("MapsMenu.AddMenuButton Serial \"Load Map\" LoadMap Root");
#ifdef _LINUX
  GUI.ParseLine("MapsMenu.AddMenuToggle Serial \"Save Video\" SaveFIFO Serial");
  GUI.ParseLine("MapsMenu.AddMenuSlider Serial Bitrate Bitrate 100 20000 Serial");
#endif
  GUI.ParseLine("LockMap=0");
  GUI.ParseLine("MapsMenu.AddMenuToggle Root \"Lock Map\" LockMap Root");
  GUI.ParseLine("MapsMenu.AddMenuButton Root \"Delete Map\" DeleteMap Root");
  GUI.ParseLine("MapInfo=0");
  GUI.ParseLine("MapsMenu.AddMenuToggle Root \"Map Info\" MapInfo Root");

  GUI.ParseLine("GLWindow.AddMenu MapViewerMenu Viewer");
  GUI.ParseLine("MapViewerMenu.AddMenuToggle Root \"View Map\" DrawMap Root");
  GUI.ParseLine("MapViewerMenu.AddMenuButton Root Next NextMap Root");
  GUI.ParseLine("MapViewerMenu.AddMenuButton Root Previous PrevMap Root");
  GUI.ParseLine("MapViewerMenu.AddMenuButton Root Current CurrentMap Root");

  }
  mbDone = false;
  /////////////////////modified Loianno////////////////////////////
  mpTracker->trail_sempahore = false;
  ////////////////////////////////////////////////////////////////
}

/**
 * Destructor
 */
System::~System()
{
  if( mpMap != NULL )  {
    mpMap->mapLockManager.UnRegister( this );
  }

}


/**
 * Run the main system thread.
 * This handles the tracker and the map viewer.
 */

  void System::Run()
  {

        ros::NodeHandle nh("~");
    message_filters::Subscriber<sensor_msgs::Image> left_sub(nh, "rgb/image_raw", 2);
    message_filters::Subscriber<sensor_msgs::Image> right_sub(nh, "depth_registered/image_raw", 2);

    Synchronizer<MySyncPolicy> sync1(MySyncPolicy(2), left_sub, right_sub);
    sync1.registerCallback(boost::bind(&System::imageCallback, this, _1, _2));

    pub_pose = nh.advertise<geometry_msgs::PoseWithCovarianceStamped> ("pose", 1);
    //pub_posetrack2 = nh.advertise<geometry_msgs::PoseWithCovarianceStamped> ("pose_track2", 1);
    pub_pose_overlap = nh.advertise<geometry_msgs::PoseWithCovarianceStamped> ("/pose_overlap", 1);
    srvKF = nh.advertiseService("keyframes", &System::keyframesservice,this);
    srvPC = nh.advertiseService("pointcloud_ptamm", &System::pointcloudservice,this);
    srvdensePC = nh.advertiseService("pointcloud_dense", &System::densepointcloudservice,this);
    sub_kb_input = nh.subscribe("/key_pressed", 1, &System::keyboardCallback, this);
    num_map = nh.advertise < std_msgs::Int8 > ("num_map", 1);//publish map number
    //tot_map_points = nh.subscribe("/global_map", 1, &System::readmapPointscallback, this);
    tot_keyframes = nh.subscribe("/Oscar/ptamm_visualizer/vslam/kfs", 1, &System::readkeyframecallback, this);
    //image_transport::ImageTransport it(nh);
    //pub_preview_image = it.advertise("/preview", 1);
    ros::spin();

  }

void System::imageCallback(const sensor_msgs::ImageConstPtr & img, const sensor_msgs::ImageConstPtr & depth_msg)
{
  
  
  
		if(scan_completed){
		tot_keyframes.shutdown();//shutdown the relocalization callback and send the pose of the overlapping
		//cout<<"relocalization completed sending roation for overlapping"<<endl;
		geometry_msgs::PoseWithCovarianceStamped overlap_pub_pose;
		overlap_pub_pose.pose.pose.position.x = Best_estimation.get_translation()[0];
		overlap_pub_pose.pose.pose.position.y = Best_estimation.get_translation()[1];
		overlap_pub_pose.pose.pose.position.z = Best_estimation.get_translation()[2];
		TooN::Vector<4> ori_overlap = MatToQuat(Best_estimation.get_rotation().get_matrix());
		overlap_pub_pose.pose.pose.orientation.w = ori_overlap[0];
		overlap_pub_pose.pose.pose.orientation.x = ori_overlap[1];
		overlap_pub_pose.pose.pose.orientation.y = ori_overlap[2];
		overlap_pub_pose.pose.pose.orientation.z = ori_overlap[3];
		pub_pose_overlap.publish(overlap_pub_pose);
		is_relocalized =  false;
		}

	/////////////////////////modified Loianno////////////////////////////
	CVD::BasicImage<CVD::Rgb <CVD::byte> > img_tmp((CVD::Rgb<CVD::byte> *)&img->data[0], CVD::ImageRef(img->width, img->height));
	//CVD::Image<CVD::Rgb<CVD::byte> > mCurrentImage(CVD::ImageRef(img->width, img->height));
	//CVD::copy(img_tmp, mCurrentImage);
	mpTracker->current_key_Frame_added_to_map = false;
	mimFrameBW.resize(CVD::ImageRef(img->width, img->height));
	mimFrameRGB.resize(CVD::ImageRef(img->width, img->height));
	/////////////////////////////////////////////////////////////////////

	      //Check if the map has been locked by another thread, and wait for release.
	      //bool bWasLocked = mpMap->mapLockManager.CheckLockAndWait( this, 0 );


	      /* This is a rather hacky way of getting this feedback,
	         but GVars cannot be assigned to different variables
	         and each map has its own edit lock bool.
	         A button could be used instead, but the visual
	         feedback would not be as obvious.
	      */
	      ////////////////////////////////////modified Loianno/////////////////////////////////
	      if(ParamsAccess::fixParams->gui){
	      mpMap->bEditLocked = *mgvnLockMap; //sync up the maps edit lock with the gvar bool.
	      }
	      ////////////////////////////////////////////////////////////////////////////////////

	      // We use two versions of each video frame:
	      // One black and white (for processing by the tracker etc)
	      // and one RGB, for drawing.
	      // Grab new video frame...
	      CVD::copy(img_tmp, mimFrameBW);
	      static bool bFirstFrame = true;
	      if(bFirstFrame)
	      {
	        bFirstFrame = false;
	      }
////////////////////////////////////modified Loianno/////////////////////////////////
	      if(ParamsAccess::fixParams->gui){
              CVD::copy(img_tmp, mimFrameRGB);
	      mGLWindow->SetupViewport();
	      mGLWindow->SetupVideoOrtho();
	      mGLWindow->SetupVideoRasterPosAndZoom();
	      }

////////////////////////////////////////////////////////////////////////////////////

	      //if(bWasLocked)  {
	        //mpTracker->ForceRecovery();
	      //}


////////////////////////////modified Loianno////////////////////////////////////////
	      bool bDrawMap = true;
	   	  bool bDrawNewTracker = true;
	   	if(ParamsAccess::fixParams->gui){
	      static gvar3<int> gvnDrawMap("DrawMap", 0, HIDDEN|SILENT);
	      static gvar3<int> gvnDrawNewTracker("NewTracker", 0, HIDDEN|SILENT);

	      bDrawMap = mpMap->IsGood() && *gvnDrawMap;
	      bDrawNewTracker = mpMap->IsGood() && *gvnDrawNewTracker;
	   	}

	   	mpTracker->num_tracker = 0;
	   	if(!tracker2_active)
	   		bDrawNewTracker = false;

	      mpTracker->TrackFrame(mimFrameBW, !bDrawMap && !bDrawNewTracker, depth_msg);
	      //publishPreviewImage(mimFrameBW, img->header);
/////////////////////////////////////////////////////////////////////////////
    	  //////////////////////modified Loianno compute scale factor////////////////////////////////////
	     if(mpMap->IsGood() && mpTracker->getTrackingQuality()){

	      counter_frames++;
	      scale_factor_computation(depth_msg, mpTracker);

	      //////////////////////////////////////////////////////////////////////////////////////////////
	      /////////////////////////modified Loianno////////////////////////////



	    /////////////////////////////publish the map////////////////////////////////////////////7


	    	    publishPoseAndInfo(img->header, mpTracker);
                               //double init_time = ros::Time::now().toSec();
                 	               if(semaphore_dense_srv == false){
                                 	semaphore_copy = true;
					globalimg = img;
                                        globaldepth_msg = depth_msg;                                       
                                        TooN::Vector<3> translation = mpTracker->GetCurrentPose().inverse().get_translation();
                                         TooN::SO3<double> rotation = mpTracker->GetCurrentPose().inverse().get_rotation();
                                        TooN::SE3<> pose_temp(rotation, translation*mpTracker->scale_factor_final_scalar_tracker);           // Camera pose: this is what the tracker updates every frame.
                                        pose_global_img = &pose_temp;                                       
					 semaphore_copy = false;
  					}

	   }

	      /////////////////////////////////////////////////////////////////////


//////////////////////////////modified Loianno////////////////////////////////////
	      if(ParamsAccess::fixParams->gui){
		      if(bDrawMap) {
		    //TooN::SE3<> pose_tracker1(mpTracker->GetCurrentPose().get_rotation(), scale_factor_final_scalar[mpMap->MapID()]*mpTracker->GetCurrentPose().get_translation());           // Camera pose: this is what the tracker updates every frame.
			mpMapViewer->DrawMap(mpTracker->GetCurrentPose());
		      }

		      if(*mgvnDrawMapInfo) {
		        DrawMapInfo();
		      }

		      string sCaption;
		      if(bDrawMap) {
			sCaption = mpMapViewer->GetMessageForUser();
		      }
		      else {
			sCaption = mpTracker->GetMessageForUser();
		      }

		      mGLWindow->DrawCaption(sCaption);
		      mGLWindow->DrawMenus();
		      }
/////////////////////////////////////////////////////////////////////////////////

		#ifdef _LINUX
		      if( *mgvnSaveFIFO )
		      {
		        SaveFIFO();
		      }
		#endif
         /////////////modified Loianno/////////////////////////
		      if(ParamsAccess::fixParams->gui && !bDrawNewTracker){
		      mGLWindow->swap_buffers();
		      mGLWindow->HandlePendingEvents();
		      }
		/////////////////////////////////////////////////

}


////////////////////////////////////modified Loianno/////////////////////////////////////////////////

void System::scale_factor_computation(const sensor_msgs::ImageConstPtr & depth_msg, Tracker *curr_tracker){
	 vector <double> scale_factor;
	  vector <double> norm_ptam_3D_points;
	  vector <double> norm_depth_points;
	  int alfa;

	    /////////////////////modified Loianno scale factor estimation////////////////////////////////
	 if(curr_tracker->mCurrentKF.mMeasurements.size()>0)
	  {

   	  // Combine unit conversion (if necessary) with scaling by focal length for computing (X,Y)
		 float constant_x_rgb = 1 / ParamsAccess::fixParams->fx_rgb;
		 float constant_y_rgb = 1 / ParamsAccess::fixParams->fy_rgb;
	    	  float bad_point = std::numeric_limits<float>::quiet_NaN ();
	    	  const float* depth_row;
	    	  const float* depth_row_init = reinterpret_cast<const float*>(&depth_msg->data[0]);
	    	  int row_step = depth_msg->step / sizeof(float);
	    	  pcl::PointCloud<pcl::PointXYZ> pt;

		  		int p = 0;
	  			int k = 0;
	    	int counter_feature_level = 0;
	    	//rotate all points in PTAMM
	    	TooN::Matrix<Dynamic,3> v3cam_features(curr_tracker->mCurrentKF.mMeasurements.size(),3);
	    	int c_cam = 0;



for(std::map<MapPoint*, Measurement>::iterator it = curr_tracker->mCurrentKF.mMeasurements.begin();  mpMap->IsGood()==true  && it != curr_tracker->mCurrentKF.mMeasurements.end(); it++) {
	    	pt.resize(counter_feature_level+1);
	  	    scale_factor.resize(counter_feature_level+1);
	  	    norm_ptam_3D_points.resize(counter_feature_level+1);
	  	    norm_depth_points.resize(counter_feature_level+1);
	  		int u = it->second.v2RootPos[0];
	  		int v = it->second.v2RootPos[1];
	  		depth_row = depth_row_init + v*row_step;
	  		float depth = depth_row[u];
	  		// Check for invalid measurements

	  		if (!depth_image_proc::DepthTraits<float>::valid(depth) || it->first->bBad==true)// || sqrt(((u - center_x) * depth * constant_x)*((u - center_x) * depth * constant_x)+((v - center_y) * depth * constant_y)*((v - center_y) * depth * constant_y)+depth_image_proc::DepthTraits<float>::toMeters(depth)*depth_image_proc::DepthTraits<float>::toMeters(depth))<0.4 || sqrt(((u - center_x) * depth * constant_x)*((u - center_x) * depth * constant_x)+((v - center_y) * depth * constant_y)*((v - center_y) * depth * constant_y)+depth_image_proc::DepthTraits<float>::toMeters(depth)*depth_image_proc::DepthTraits<float>::toMeters(depth))>1.5)
	  		{
	  			pt[counter_feature_level].x = pt[counter_feature_level].y = pt[counter_feature_level].z = bad_point;
	  		}
	  		else
	  		{
	  		// Find XYZ
	  		pt[counter_feature_level].x = (u - ParamsAccess::fixParams->cx_rgb) * depth * constant_x_rgb;scale_factor_final_scalar[mpMap->MapID()];
	  		pt[counter_feature_level].y = (v - ParamsAccess::fixParams->cy_rgb) * depth * constant_y_rgb;
	  		pt[counter_feature_level].z = depth_image_proc::DepthTraits<float>::toMeters(depth);
	  		Vector<3> v3Cam = curr_tracker->GetCurrentPose()*it->first->v3WorldPos;
	  		p++;//counter for good points
	  		}
		  counter_feature_level++;
	    	}





/*for(std::map<MapPoint*, Measurement>::iterator it3 = curr_tracker->mCurrentKF.mMeasurements.begin(); mpMap->IsGood()==true  && it3 != curr_tracker->mCurrentKF.mMeasurements.end(); it3++) {

						if((!isnan(pt[s].x) || !isnan(pt[s].y) || !isnan(pt[s].z))){
						Vector<3> v3Cam = curr_tracker->GetCurrentPose()*it3->first->v3WorldPos;
	  			  		//myfile_ptam<<v3Cam[0]<<" "<<v3Cam[1]<<" "<<v3Cam[2]<<endl;
	  			  		//myfile_depth<<pt[s].x<<" "<<pt[s].y<<" "<<pt[s].z<<endl;
	  			  		}
	  			  	    s++;
	  			  		}*/



	  //////////////////////////points distance from center of mass////////////////////////////////
	  		TooN::Vector<> dist_ptam(p);
	  		TooN::Vector<> dist_depth(p);
	  		double mean_dist_ptam = 0;
	  		double mean_dist_depth = 0;
	  		int counter_mean = 0;

	  		int v = 0;
			std::vector<double> SF (p);
			for(std::map<MapPoint*, Measurement>::iterator it = curr_tracker->mCurrentKF.mMeasurements.begin(); mpMap->IsGood()==true  && it != curr_tracker->mCurrentKF.mMeasurements.end(); it++) {

			 if((!std::isnan(pt[v].x) && !std::isnan(pt[v].y) && !std::isnan(pt[v].z))){
					Vector<3> v3Cam = curr_tracker->GetCurrentPose()*it->first->v3WorldPos;
					dist_ptam[counter_mean] = sqrt((v3Cam[0])*(v3Cam[0])+(v3Cam[1])*(v3Cam[1])+(v3Cam[2])*(v3Cam[2]));
					dist_depth[counter_mean] = sqrt((pt[v].x)*(pt[v].x)+(pt[v].y)*(pt[v].y)+(pt[v].z)*(pt[v].z));
					SF[counter_mean] = dist_depth[counter_mean]/dist_ptam[counter_mean];//norm rate
					counter_mean++;
					}
					v++;
				}

	  		////////////////////erase points outside the standard deviation//////////////////////////////////////////////

                        curr_tracker->mean_variance(SF, SF.size());
		        curr_tracker->scale_factor_final_scalar_tracker = 0;	
                       for (uint z = 0; z<curr_tracker->scale_factor_final.size(); z++){
				curr_tracker->scale_factor_final_scalar_tracker = curr_tracker->scale_factor_final_scalar_tracker + curr_tracker->scale_factor_final[z];
				        	}
			curr_tracker->scale_factor_final_scalar_tracker = curr_tracker->scale_factor_final_scalar_tracker/curr_tracker->scale_factor_final.size();
	  		////////////////////////////////////////////////////////////////////////////////////////////////////////////


	  		}
}



void System::transformPoints(TooN::Matrix<3> RotCurr,
		      TooN::Vector<3> TransCurr,
		      pcl::PointCloud <pcl::PointXYZRGB>& depth_points)
{
    int dim_ = depth_points.size();

    for (int i = 0; i < dim_; i++)
    {
	TooN::Vector<3> temp_depth_points;
	TooN::Vector<3> temp_depth_points2;
    temp_depth_points = TooN::makeVector( depth_points[i].x,
											 depth_points[i].y,
										     depth_points[i].z );


    temp_depth_points2 = RotCurr*temp_depth_points + TransCurr;

    depth_points[i].x = temp_depth_points2[0];
    depth_points[i].y = temp_depth_points2[1];
    depth_points[i].z = temp_depth_points2[2];

    }

}
int counter_output = 0;
void System::publishPoseAndInfo(const std_msgs::Header & header, Tracker* curr_tracker)
{


		                  TooN::Vector<3> translation = curr_tracker->GetCurrentPose().inverse().get_translation();
		    		  TooN::SO3<double> rotation = curr_tracker->GetCurrentPose().inverse().get_rotation();
		    		  TooN::SE3<double> H_R(rotation, translation);
//cout<<"pose:"<<curr_tracker->GetCurrentPose().get_translation()<<endl;

		                  TooN::Vector<4> quat = MatToQuat(H_R.get_rotation().get_matrix());

		    			static tf::TransformBroadcaster br_rescaled;
		    			tf::Transform transform_rescaled;
		    			static tf::TransformBroadcaster br_origin;
		    			tf::Transform transform_origin;

		    		  transform_rescaled.setOrigin( tf::Vector3(curr_tracker->scale_factor_final_scalar_tracker *translation[0], curr_tracker->scale_factor_final_scalar_tracker *translation[1], curr_tracker->scale_factor_final_scalar_tracker *translation[2]) );
		    		  transform_rescaled.setRotation( tf::Quaternion(quat[1], quat[2], quat[3], quat[0]) );
		    		  //br_rescaled.sendTransform(tf::StampedTransform(transform_rescaled, ros::Time::now(), "/world", "/PTAMM_ros_track1"));



		    		  //TooN::Vector<3> translation_not_scaled = curr_tracker->GetCurrentPose().inverse().get_translation();
		    		  //TooN::SO3<double> rotation_not_scaled = curr_tracker->GetCurrentPose().inverse().get_rotation();
		    		  //TooN::SE3<double> H_not_scaled(rotation_not_scaled.get_matrix(),translation_not_scaled);
		    		 // TooN::SE3<double> delta_H = H.inverse()*H_not_scaled;
		    		  //cout<<"delta_H"<<delta_H<<endl;
		    		 // Vector<4> quat_scaled = MatToQuat(delta_H.get_rotation().get_matrix());

		    		  //transform_origin.setOrigin( tf::Vector3(translation[0], translation[1], translation[2]) );
		    		  //transform_origin.setRotation( tf::Quaternion(quat[1], quat[2], quat[3], quat[0]) );
		    		  //br_origin.sendTransform(tf::StampedTransform(transform_origin, ros::Time::now(), "/world", "/PTAMM_ros_not_scaled"));

		    		  /*tf::StampedTransform transform;
		    			  tf::TransformListener listener;

		    			  try{
		    		      //ros::Time now = ros::Time::now();
		    				listener.waitForTransform("/world", "/openni_camera", ros::Time(0), ros::Duration(0.2));
		    		      listener.lookupTransform("/world", "/openni_camera", ros::Time(0), transform);
		    		      myfile_rgbd_slam_pose.precision(15);
			    		  myfile_rgbd_slam_pose<<transform.stamp_.toSec()<<" "<<transform.getOrigin().x()<<" "<<transform.getOrigin().y()<<" "<<transform.getOrigin().z()<<" "<<transform.getRotation().x()<<" "<<transform.getRotation().y()<<" "<<transform.getRotation().z()<<" "<<transform.getRotation().w()<<endl;
		    			  }
		    			  catch(tf::TransformException ex){
		    			  cout<<"error_trasnform"<<endl;
		    			  }*/

		    		  //ParamsAccess::varParams->Scale = scale_factor_final_scalar[mpMap->MapID()];

		    		    ParamsAccess::varParams->Scale = curr_tracker->scale_factor_final_scalar_tracker;

		    		    //TooN::SE3<double> pose = H_R;
		    		    //world in the camera frame
		    		    //TooN::Matrix<3, 3, double> r_ptam = pose.get_rotation().get_matrix();
		    		    //TooN::Vector<3, double> t_ptam =  pose.get_translation()*curr_tracker->scale_factor_final_scalar_tracker;
		    		    //camera in the world frame
		    		    //TooN::Matrix<3, 3, double> r_world = pose.get_rotation().get_matrix().T();
		    		    //TooN::Vector<3, double> t_world =  - curr_tracker->scale_factor_final_scalar_tracker*(r_world * pose.get_translation());
		    		    //tf::StampedTransform transform_world(tf::Transform(tf::Matrix3x3(r_world(0, 0), r_world(0, 1), r_world(0, 2), r_world(1, 0), r_world(1, 1), r_world(1, 2), r_world(2, 0), r_world(2, 1), r_world(2, 2))
		    		      //  , tf::Vector3(t_world[0], t_world[1], t_world[2])), ros::Time::now(),  "/world", "/PTAMM_ros_pub");


  		    if (pub_pose.getNumSubscribers() > 0 || pub_pose_world.getNumSubscribers() > 0)
	    		    {

	    		      //world in the camera frame
	    		      geometry_msgs::PoseWithCovarianceStampedPtr msg_pose(new geometry_msgs::PoseWithCovarianceStamped);
	    		      const tf::Quaternion & q_tf_ptam = transform_rescaled.getRotation();
	    		      const tf::Vector3 & t_tf_ptam = transform_rescaled.getOrigin();
	    		      msg_pose->pose.pose.orientation.w = q_tf_ptam.w();
	    		      msg_pose->pose.pose.orientation.x = q_tf_ptam.x();
	    		      msg_pose->pose.pose.orientation.y = q_tf_ptam.y();
	    		      msg_pose->pose.pose.orientation.z = q_tf_ptam.z();
	    		      msg_pose->pose.pose.position.x = t_tf_ptam.x();
	    		      msg_pose->pose.pose.position.y = t_tf_ptam.y();
	    		      msg_pose->pose.pose.position.z = t_tf_ptam.z();
	    		      TooN::Matrix<6> CovRotIMU;
	    		      //CovRotIMU.slice(0,0,3,3) = H_tot.get_rotation().get_matrix();
	    		      //CovRotIMU.slice(3,3,3,3) = CovRotIMU.slice(0,0,3,3);
	    		      TooN::Matrix<6> covar = mpTracker->GetCurrentCov();
	    		      covar.slice(0,0,3,3) = 100*(curr_tracker->scale_factor_final_scalar_tracker*curr_tracker->scale_factor_final_scalar_tracker)*covar.slice(0,0,3,3);
	    		      for (unsigned int i = 0; i < msg_pose->pose.covariance.size(); i++)
	    		        msg_pose->pose.covariance[i] = ((covar[i % 6][i / 6]));

	    		      msg_pose->header.seq = header.seq;
	    		      msg_pose->header.stamp = header.stamp;
	    		      msg_pose->header.frame_id = "/world";
	    		      pub_pose.publish(msg_pose);
	    		      //TooN::SymEigen<6> eigM(covar);

	    		    }


}

void System::PointCloud_convert(const sensor_msgs::ImageConstPtr depth_msg,
                                      const sensor_msgs::ImageConstPtr rgb_msg,
                                      pcl::PointCloud<pcl::PointXYZRGB>& cloud_msg,
                                       sensor_msgs::PointCloud2& msg_pub,
                                      int red_offset, int green_offset, int blue_offset, int color_step)
{
	//double begin = ros::Time::now().toSec();
   int index = 0;
  // Use correct principal point from calibration
  // Combine unit conversion (if necessary) with scaling by focal length for computing (X,Y)
  double unit_scaling = 1;
  float constant_x = unit_scaling / ParamsAccess::fixParams->fx_rgb;
  float constant_y = unit_scaling / ParamsAccess::fixParams->fy_rgb;
  float bad_point = std::numeric_limits<float>::quiet_NaN ();

  const float* depth_row = reinterpret_cast<const float*>(&depth_msg->data[0]);
  int row_step = depth_msg->step / sizeof(float);
  const uint8_t* rgb = &rgb_msg->data[0];
  int rgb_skip = rgb_msg->step - rgb_msg->width * color_step;


  pcl::PointCloud <pcl::PointXYZRGB> depth_points_grid;

  for (int v = 0; v < (int)depth_msg->height; ++v, depth_row += row_step, rgb += rgb_skip)
  {
    for (int u = 0; u < (int)depth_msg->width; ++u, rgb += color_step)
    {
      cloud_msg.resize(index+1);
      float depth = depth_row[u];

      // Check for invalid measurements
      if (!depth_image_proc::DepthTraits<float>::valid(depth))
      {
    	  cloud_msg.points[index].x = cloud_msg.points[index].y = cloud_msg.points[index].z = bad_point;
      }
      else
      {
        // Fill in XYZ
    	  cloud_msg.points[index].x = (u - ParamsAccess::fixParams->cx_rgb) * depth * constant_x;
    	  cloud_msg.points[index].y = (v - ParamsAccess::fixParams->cy_rgb) * depth * constant_y;
    	  cloud_msg.points[index].z = depth_image_proc::DepthTraits<float>::toMeters(depth);

      }

      // Fill in color
      RGBValue color;
      color.Red   = rgb[red_offset];
      color.Green = rgb[green_offset];
      color.Blue  = rgb[blue_offset];
      color.Alpha = 0;
      cloud_msg.points[index].rgb = color.float_value;
      index++;

    }

  }

  //srv_publishdepth(cloud_msg);//grid and rotation
}


void System::extract_PointCloud(const sensor_msgs::ImageConstPtr depth_msg,
                                      const sensor_msgs::ImageConstPtr rgb_msg,
                                      pcl::PointCloud<pcl::PointXYZRGB>& cloud_msg,
                                       sensor_msgs::PointCloud2& msg_pub,
                                      int red_offset, int green_offset, int blue_offset, int color_step)
{
  
    int index = 0; 
  double unit_scaling = 1;
  float constant_x = unit_scaling / ParamsAccess::fixParams->fx_rgb;
  float constant_y = unit_scaling / ParamsAccess::fixParams->fy_rgb;
  float bad_point = std::numeric_limits<float>::quiet_NaN ();

  const float* depth_row = reinterpret_cast<const float*>(&depth_msg->data[0]);
  int row_step = depth_msg->step / sizeof(float);
   const uint8_t* rgb = &rgb_msg->data[0];
   int rgb_skip = rgb_msg->step - rgb_msg->width * color_step;
  TooN::Vector<3> translation = pose_global_img->get_translation();
	TooN::SO3<double> rotation = pose_global_img->get_rotation();
		TooN::Matrix<3> rot_mat =  rotation.get_matrix();

		pcl::PointCloud <pcl::PointXYZRGB> depth_points_grid;
		int ir = 0;

		while (ir < r_max) {
			int ic = 0;
			while (ic < c_max) {
			    int index_c = floor((ir+0.5*ParamsAccess::fixParams->DR)*c_max + ic+0.5*ParamsAccess::fixParams->DC);
				double zc =  depth_image_proc::DepthTraits<float>::toMeters(depth_row[index_c]);//depth_points[ index_c ].z;
				if (std::isnan(zc)) zc = floor((ParamsAccess::fixParams->Z_MAX-ParamsAccess::fixParams->Z_MIN)/2);
				zc = fmax(ParamsAccess::fixParams->Z_MIN,fmin(ParamsAccess::fixParams->Z_MAX,zc));
				int step_r = floor(ParamsAccess::fixParams->STEP_R_MIN + (ParamsAccess::fixParams->STEP_R_MAX-ParamsAccess::fixParams->STEP_R_MIN)/(ParamsAccess::fixParams->Z_MAX-ParamsAccess::fixParams->Z_MIN) * (ParamsAccess::fixParams->Z_MAX-zc));
				int step_c = floor(ParamsAccess::fixParams->STEP_C_MIN + (ParamsAccess::fixParams->STEP_C_MAX-ParamsAccess::fixParams->STEP_C_MIN)/(ParamsAccess::fixParams->Z_MAX-ParamsAccess::fixParams->Z_MIN) * (ParamsAccess::fixParams->Z_MAX-zc));

					for (int r = ir; r<fmin(r_max,ir+ParamsAccess::fixParams->DR); r+=step_r) {
				
					for (int c = ic; c < fmin(c_max,ic+ParamsAccess::fixParams->DC); c += step_c) {  
				        int index_u = c_max*r+c;//find the index jumping all raws and gettig the column element	
                                         cloud_msg.points.resize(index+1);
					 cloud_msg.points[index].x = (c - ParamsAccess::fixParams->cx_rgb) * depth_image_proc::DepthTraits<float>::toMeters(depth_row[index_c]) * constant_x;
                                         cloud_msg.points[index].y = (r - ParamsAccess::fixParams->cy_rgb) * depth_image_proc::DepthTraits<float>::toMeters(depth_row[index_c]) * constant_y;
					 cloud_msg.points[index].z = depth_image_proc::DepthTraits<float>::toMeters(depth_row[index_c]);
  					 
          // Fill in color
					      RGBValue color;
					      color.Red   = rgb[3*index_u + red_offset];
					      color.Green = rgb[3*index_u + green_offset];
					      color.Blue  = rgb[3*index_u + blue_offset];
					      color.Alpha = 0;
					      cloud_msg.points[index].rgb = color.float_value;
					      index++;

					}
				}
				ic += ParamsAccess::fixParams->DC;
			}                        
			ir += ParamsAccess::fixParams->DR;
                        //depth_row+=ParamsAccess::fixParams->DR*row_step;

		} 

                Eigen::Matrix< float, 4, 4 > m;
                Eigen::Matrix< float, 4, 4 > m1;
		m1 << 0, 0, 1, 0,
                      -1, 0, 0, 0.02,
                      0, -1, 0, 0.105,
                0, 0, 0, 1;
                m << rot_mat[0][0], rot_mat[0][1], rot_mat[0][2], translation[0],
                                rot_mat[1][0], rot_mat[1][1], rot_mat[1][2], translation[1],
                                rot_mat[2][0], rot_mat[2][1], rot_mat[2][2], translation[2],
                0, 0, 0, 1;

                pcl::transformPointCloud (cloud_msg,cloud_msg, m);
                pcl::transformPointCloud (cloud_msg,cloud_msg, m1);
  
  
  
}  





void System::srv_publishdepth(pcl::PointCloud<pcl::PointXYZRGB> depth_points, sensor_msgs::PointCloud2 & resp)
{
		// Supported color encodings: RGB8, BGR8, MONO8

	TooN::Vector<3> translation = pose_global_img->get_translation();
	TooN::SO3<double> rotation = pose_global_img->get_rotation();
		TooN::Matrix<3> rot_mat =  rotation.get_matrix();

		pcl::PointCloud <pcl::PointXYZRGB> depth_points_grid;
		int ir = 0;
		while (ir < r_max) {
			int ic = 0;
			while (ic < c_max) {
				int index_c = floor((ir+0.5*ParamsAccess::fixParams->DR) * c_max + ic+0.5*ParamsAccess::fixParams->DC);
				double zc =  depth_points[ index_c ].z;
				if (std::isnan(zc)) zc = floor((ParamsAccess::fixParams->Z_MAX-ParamsAccess::fixParams->Z_MIN)/2);
				zc = fmax(ParamsAccess::fixParams->Z_MIN,fmin(ParamsAccess::fixParams->Z_MAX,zc));
				int step_r = floor(ParamsAccess::fixParams->STEP_R_MIN + (ParamsAccess::fixParams->STEP_R_MAX-ParamsAccess::fixParams->STEP_R_MIN)/(ParamsAccess::fixParams->Z_MAX-ParamsAccess::fixParams->Z_MIN) * (ParamsAccess::fixParams->Z_MAX-zc));
				int step_c = floor(ParamsAccess::fixParams->STEP_C_MIN + (ParamsAccess::fixParams->STEP_C_MAX-ParamsAccess::fixParams->STEP_C_MIN)/(ParamsAccess::fixParams->Z_MAX-ParamsAccess::fixParams->Z_MIN) * (ParamsAccess::fixParams->Z_MAX-zc));
				for (int r = ir; r<fmin(r_max,ir+ParamsAccess::fixParams->DR); r+=step_r) {
					int rb = r*c_max;
					for (int c = ic; c < fmin(c_max,ic+ParamsAccess::fixParams->DC); c += step_c) {
						depth_points_grid.push_back(depth_points[rb+c]);
					}
				}
				ic += ParamsAccess::fixParams->DC;
			}
			ir += ParamsAccess::fixParams->DR;
		}
		cout << "depth_points_grid.size = " << depth_points_grid.size() << endl;

		//System::transformPoints(rot_mat, scale_factor_final_scalar*translation, depth_points_grid);

		Eigen::Matrix< float, 4, 4 > m;
		m << rot_mat[0][0], rot_mat[0][1], rot_mat[0][2], translation[0],
				rot_mat[1][0], rot_mat[1][1], rot_mat[1][2], translation[1],
				rot_mat[2][0], rot_mat[2][1], rot_mat[2][2], translation[2],
		0, 0, 0, 1;
		//pcl::transformPointCloud (depth_points_grid,depth_points_grid, m);
		pcl::toROSMsg(depth_points_grid, resp);
	        //resp.header.frame_id = "/world";
                //resp.header.stamp = ros::Time::now();
	
               /*sensor_msgs::PointCloud2 out_points;
		pcl::toROSMsg(depth_points_grid, out_points);
		out_points.header.frame_id = "/world";
		out_points.header.stamp = ros::Time::now();



		  // Publish the data
		 points_depth.publish(out_points);*/



}



///////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////modified Loianno////////////////////////////////////////////
int counter_scale = 0;

void System::readkeyframecallback(const PTAMM_RGBD_cooperative::KeyFrame_msg & resp)
{
    if(mpMap->IsGood()==true)
{
	// flags: 	negative number = send newest N KeyFrames
	//			zero = send all available KeyFrames
	//			positive number = send all KeyFrames with ID>N
    geometry_msgs::PoseWithCovarianceStamped buffpose;
    std::vector<Map*> mpMapreloc;
    mpMapreloc.clear();
   mpMapreloc.resize(1);
   mpMapreloc[0] = new Map(); //mpMap->vpKeyFrames.resize(resp.KFs.size());
   ATANCamera *mpCamera_reloc = new ATANCamera("Camera");

   for(int i = 0; i<resp.KFs.size(); i++){
			mpMapreloc[0]->vpKeyFrames.resize(i+1);
	                mpMapreloc[0]->vpKeyFrames[i] =  new KeyFrame(ATANCamera("Camera"));
			mpMapreloc[0]->vpKeyFrames[i]->pSBI = new SmallBlurryImage();
			//take each keyframe and insert into the map its position and its desciptors
     			buffpose = resp.KFs[i];
			TooN::SO3<double> buffpose_mat(QuatToMat(TooN::makeVector(buffpose.pose.pose.orientation.w,buffpose.pose.pose.orientation.x,buffpose.pose.pose.orientation.y,buffpose.pose.pose.orientation.z)));
			TooN::Vector<3> buffpose_trans = TooN::makeVector(buffpose.pose.pose.position.x, buffpose.pose.pose.position.y, buffpose.pose.pose.position.z);
			mpMapreloc[0]->vpKeyFrames[i]->se3CfromW = TooN::SE3<double>(buffpose_mat,buffpose_trans);//position of each keyframe
			mpMapreloc[0]->vpKeyFrames[i]->pSBI->mimTemplate.resize(CVD::ImageRef(resp.descriptors_KFs[i].layout.dim[1].size,resp.descriptors_KFs[i].layout.dim[0].size));
			mpMapreloc[0]->vpKeyFrames[i]->pSBI->mimImageJacs.resize(CVD::ImageRef(resp.jacobians_KFs[i].layout.dim[1].size,resp.jacobians_KFs[i].layout.dim[0].size));
			mpMapreloc[0]->vpKeyFrames[i]->pSBI->mbMadeJacs = true;
			memcpy(&(mpMapreloc[0]->vpKeyFrames[i]->pSBI->mimTemplate.data()[0]), &(resp.descriptors_KFs[i].data[0]), resp.descriptors_KFs[i].layout.dim[0].stride*sizeof(float));
			int k = 0;
			for(int p = 0; p < resp.descriptors_KFs[i].data.size(); p = p+2){
			 mpMapreloc[0]->vpKeyFrames[i]->pSBI->mimImageJacs.data()[k][0] = resp.jacobians_KFs[i].data[p];
			 mpMapreloc[0]->vpKeyFrames[i]->pSBI->mimImageJacs.data()[k][1] = resp.jacobians_KFs[i].data[p+1];
			 k++;
			  }
			
	      }

            //Scroll all the keyframe of the current map and select the one with good score
            std::vector<std::pair<TooN::SE3<>,double>> score_pair;
	    std::pair<TooN::SE3<>,double> best_score_pair;
	   is_relocalized = false;
             double best_scale = 1;
	     TooN::SE3<double> T_C20_C2;
	    int j = 0;
            bool scan_compelted = false;
	    for(int c = 0; c < mpMap->vpKeyFrames.size(); c++){
            cout<<"current_kf:"<<c<<endl;
	    Relocaliser relocate_on_map(mpMapreloc, *mpCamera_reloc);
            //is_relocalized = relocate_on_map.AttemptRecovery(*mpMapreloc[0], *mpMap->vpKeyFrames[c]);
	     if(relocate_on_map.AttemptRecovery(*mpMapreloc[0], *mpMap->vpKeyFrames[c])){
		score_pair.resize(j+1);
		score_pair[j] = std::make_pair(relocate_on_map.BestPose(), relocate_on_map.mdBestScore);
		if(best_score > relocate_on_map.mdBestScore){
		  T_C20_C2 = mpMap->vpKeyFrames[c]->se3CfromW;
		  best_score_pair = std::make_pair(score_pair[j].first, score_pair[j].second);//choose the transormation with the best score
		  best_score = relocate_on_map.mdBestScore;
		  best_scale =  mpMap->vpKeyFrames[c]->scale_factor_key_frame/(mpMapreloc[0]->vpKeyFrames[relocate_on_map.mnBest]->scale_factor_key_frame);  
                 is_relocalized = true;
                 }
		j++;
		}
	}
	    if(is_relocalized){
            TooN::Vector<3> translation = best_scale*std::get<0>(best_score_pair).get_translation();
	    //reset the translation
	   TooN::SE3<double> best_result = TooN::SE3<double>(std::get<0>(best_score_pair).get_rotation(),TooN::makeVector(translation[0], translation[1], translation[2]));
	    Best_estimation = T_C20_C2.inverse()*best_result;//std::get<0>(best_score_pair);//the estimation represents T_C10_C2 so it is necessary to use T_C2_C20 to obatin the world transformations
	    cout<<"T_C10_T_C20:"<<Best_estimation<<endl;
            cout<<"T_C20_C2.inverse():"<<T_C20_C2.inverse();	
            cout<<"overlap:"<<std::get<0>(best_score_pair)<<endl;    
            cout<<"best_scale"<<best_scale<<endl;
	    cout<<"size:"<<score_pair.size()<<endl;
	  scan_completed = true;
	    }
cout<<"scan_completed"<<scan_completed<<endl;
	   
}
	    //cout<<"relocalized:"<<relocalized<<endl;
}


void System::readmapPointscallback(const PTAMM_RGBD_cooperative::PointCloud_msg & resp)
{


      mpMap->vpPoints.resize(resp.pointcloud.width*resp.pointcloud.height);
      MapPoint curr_point;
      int point_counter = 0;
      int dimension   = 6;
      unsigned char* dat = (unsigned char*) &(resp.pointcloud.data[0]);
      for(int i=0; i<mpMap->vpPoints.size(); i++)
		      {
			memcpy(&curr_point.v3WorldPos,dat, 3*sizeof(float));
			memcpy(&curr_point.nSourceLevel,dat+7,sizeof(uint32_t));
			mpMap->vpPoints.push_back(&curr_point);
		        dat+=dimension*sizeof(uint32_t);
			}
}

bool System::keyframesservice(PTAMM_RGBD_cooperative::KeyFrame_srvRequest & req, PTAMM_RGBD_cooperative::KeyFrame_srvResponse & resp)
{
	// flags: 	negative number = send newest N KeyFrames
	//			zero = send all available KeyFrames
	//			positive number = send all KeyFrames with ID>N



	TooN::SE3<double> pose;
	TooN::Matrix<3, 3, double> rot;
	TooN::Vector<3, double> trans;
	tf::Quaternion q;
	tf::Vector3 t;
	geometry_msgs::PoseWithCovarianceStamped buffpose;
	std_msgs::Float32MultiArray buffdes;
	std_msgs::Float32MultiArray buffdes_jac;

	int k=0;
	static unsigned int seq=0;

	if(!(mpMap->vpKeyFrames.size()>0) | !mpMap->bGood)
		return false;
  
        resp.KF_scale.resize(mpMap->vpKeyFrames.size());
	resp.KFids.reserve(mpMap->vpKeyFrames.size());
	resp.KFs.reserve(mpMap->vpKeyFrames.size());
	resp.descriptors_KFs.reserve(mpMap->vpKeyFrames.size());
	  
        //now knowing the corresponding keyframe do a for loop to remap all of them in the same MapID

	for(std::vector<KeyFrame*>::iterator rit=mpMap->vpKeyFrames.begin(); rit!=mpMap->vpKeyFrames.end();++rit)
	{
		//if(mpMap->vpKeyFrames.size()<3)
		resp.KF_scale[k] = (*rit)->scale_factor_key_frame;
		double scale = 1;
		//scale = ParamsAccess::varParams->Scale;
			pose = (*rit)->se3CfromW;//world in the camera frame
			rot =pose.get_rotation().get_matrix();
			trans = pose.get_translation();
			tf::Transform transform(tf::Matrix3x3(rot(0, 0), rot(0, 1), rot(0, 2),
							      rot(1, 0), rot(1, 1), rot(1, 2),
							      rot(2, 0), rot(2, 1), rot(2, 2)),
			tf::Vector3(trans[0] * scale, trans[1]* scale, trans[2] * scale));
			q = transform.getRotation();
			t = transform.getOrigin();
			buffpose.header.seq=seq;
			buffpose.header.stamp=ros::Time::now();
			buffpose.pose.pose.position.x=t[0];
			buffpose.pose.pose.position.y=t[1];
			buffpose.pose.pose.position.z=t[2];
			buffpose.pose.pose.orientation.w=q.w();
			buffpose.pose.pose.orientation.x=q.x();
			buffpose.pose.pose.orientation.y=q.y();
			buffpose.pose.pose.orientation.z=q.z();
			memset(&(buffpose.pose.covariance[0]),0,sizeof(double)*6*6);
			resp.KFs.push_back(buffpose);
			//set the current keyframe image
			buffdes.layout.dim.resize(3);
			buffdes.layout.dim[0].label="height";
			buffdes.layout.dim[0].size=(uint32_t)(*rit)->pSBI->mimTemplate.size().y;
			buffdes.layout.dim[0].stride=(uint32_t)(*rit)->pSBI->mimTemplate.size().x*(uint32_t)(*rit)->pSBI->mimTemplate.size().y;
			buffdes.layout.dim[1].label="width";
		        buffdes.layout.dim[1].size=(uint32_t)(*rit)->pSBI->mimTemplate.size().x;
			buffdes.layout.dim[1].stride=(uint32_t)(*rit)->pSBI->mimTemplate.size().x;
			buffdes.layout.dim[2].label="channel";
			buffdes.layout.dim[2].size=1;
			buffdes.layout.dim[2].stride=1;
			buffdes.layout.data_offset=0;
			buffdes.data.resize((uint32_t)(*rit)->pSBI->mimTemplate.size().x*(uint32_t)(*rit)->pSBI->mimTemplate.size().y);
		        memcpy(&(buffdes.data[0]), &((*rit)->pSBI->mimTemplate.data()[0]),(uint32_t)(*rit)->pSBI->mimTemplate.size().x*(uint32_t)(*rit)->pSBI->mimTemplate.size().y*sizeof(float));//copy the actual descriptor
			resp.descriptors_KFs.push_back(buffdes);
			//set the current keyfram Jacobians
			buffdes_jac.layout.dim.resize(3);
			buffdes_jac.layout.dim[0].label="height";
			buffdes_jac.layout.dim[0].size=(uint32_t)(*rit)->pSBI->mimImageJacs.size().y;
			buffdes_jac.layout.dim[0].stride=(uint32_t)2*(*rit)->pSBI->mimImageJacs.size().x*(uint32_t)(*rit)->pSBI->mimImageJacs.size().y;
			buffdes_jac.layout.dim[1].label="width";
			buffdes_jac.layout.dim[1].size=(uint32_t)(*rit)->pSBI->mimImageJacs.size().x;
			buffdes_jac.layout.dim[1].stride=(uint32_t)2*(*rit)->pSBI->mimImageJacs.size().x;
			buffdes_jac.layout.dim[2].label="channel";
			buffdes_jac.layout.dim[2].size=2;
			buffdes_jac.layout.dim[2].stride=2;
			buffdes_jac.layout.data_offset=0;
			buffdes_jac.data.resize(2*(uint32_t)(*rit)->pSBI->mimImageJacs.size().x*(uint32_t)(*rit)->pSBI->mimImageJacs.size().y);
		        //memcpy(&(buffdes_jac.data[0]), &((*rit)->pSBI->mimImageJacs.data()[0]),2*(*rit)->pSBI->mimImageJacs.size().x*(*rit)->pSBI->mimImageJacs.size().y*sizeof(float));//copy the actual descriptor
			int p = 0;
			for(int i = 0; i < (uint32_t)(*rit)->pSBI->mimImageJacs.size().x*(uint32_t)(*rit)->pSBI->mimImageJacs.size().y; i++){
			  buffdes_jac.data[p] = (*rit)->pSBI->mimImageJacs.data()[i][0];
			  buffdes_jac.data[p+1] = (*rit)->pSBI->mimImageJacs.data()[i][1];
			  p = p + 2;
			  }
			resp.jacobians_KFs.push_back(buffdes_jac);
			seq++;

		k++;
	}
	return true;

}

bool System::pointcloudservice(PTAMM_RGBD_cooperative::PointCloudRequest & req, PTAMM_RGBD_cooperative::PointCloudResponse & resp)
{
	static unsigned int seq=0;
	int dimension   = 5;

    if(!mpMap->bGood)
                return false;

       // static tf::TransformBroadcaster br;
        //tf::Transform transform_sparse;

        //transform_sparse.setOrigin( tf::Vector3(0, -0.02, 0.08) );
        //transform_sparse.setRotation( tf::Quaternion(0.5, -0.5, 0.5, -0.5) );
        //br.sendTransform(tf::StampedTransform(transform_sparse, ros::Time::now(), "/world", "/camera_origin"));
        TooN::SE3<double> T_C_W(QuatToMat(TooN::makeVector(0.5, -0.5, 0.5, -0.5)), TooN::makeVector(0, -0.02, 0.105));
	resp.pointcloud.header.seq=seq;
	seq++;
	resp.pointcloud.header.stamp = ros::Time::now();
	resp.pointcloud.height = 1;
	resp.pointcloud.header.frame_id = "/world";
	if(mpMap->bGood)
	{
		resp.pointcloud.width = mpMap->vpPoints.size();
		resp.pointcloud.fields.resize(dimension);
		resp.pointcloud.fields[0].name = "x";
		resp.pointcloud.fields[0].offset = 0*sizeof(float);
		resp.pointcloud.fields[0].datatype = sensor_msgs::PointField::FLOAT32;
		resp.pointcloud.fields[0].count = 1;
		resp.pointcloud.fields[1].name = "y";
		resp.pointcloud.fields[1].offset = 1*sizeof(float);
		resp.pointcloud.fields[1].datatype = sensor_msgs::PointField::FLOAT32;
		resp.pointcloud.fields[1].count = 1;
		resp.pointcloud.fields[2].name = "z";
		resp.pointcloud.fields[2].offset = 2*sizeof(float);
		resp.pointcloud.fields[2].datatype = sensor_msgs::PointField::FLOAT32;
		resp.pointcloud.fields[2].count = 1;
		resp.pointcloud.fields[3].name = "rgba";
		resp.pointcloud.fields[3].offset = 3*sizeof(uint32_t);
		resp.pointcloud.fields[3].datatype = sensor_msgs::PointField::UINT32;
		resp.pointcloud.fields[3].count = 1;
		//resp.pointcloud.fields[4].name = "KF";
		//resp.pointcloud.fields[4].offset = 4*sizeof(uint32_t);
		//resp.pointcloud.fields[4].datatype = sensor_msgs::PointField::INT32;
		//resp.pointcloud.fields[4].count = 1;
		resp.pointcloud.fields[4].name = "lvl";
		resp.pointcloud.fields[4].offset = 4*sizeof(uint32_t);
		resp.pointcloud.fields[4].datatype = sensor_msgs::PointField::UINT32;
		resp.pointcloud.fields[4].count = 1;

		resp.pointcloud.point_step = dimension*sizeof(uint32_t);
		resp.pointcloud.row_step = resp.pointcloud.point_step * resp.pointcloud.width;
		resp.pointcloud.data.resize(resp.pointcloud.row_step * resp.pointcloud.height);
		resp.pointcloud.is_dense = false;


		unsigned char* dat = &(resp.pointcloud.data[0]);
		unsigned int n=0;
		for(std::vector<MapPoint*>::iterator it=mpMap->vpPoints.begin(); it!=mpMap->vpPoints.end(); ++it,++n)
		{
			if(n>resp.pointcloud.width-1) break;
			MapPoint p = *(*it);

			Vector<3,float> fvec = T_C_W*p.v3WorldPos;
			uint32_t colorlvl = 0xff<<((3-p.nSourceLevel)*8);
			uint32_t lvl = p.nSourceLevel;
			//uint32_t KF = p.pPatchSourceKF->ID;

			memcpy(dat, &(fvec),3*sizeof(float));
			memcpy(dat+3*sizeof(uint32_t),&colorlvl,sizeof(uint32_t));
			memcpy(dat+4*sizeof(uint32_t),&lvl,sizeof(uint32_t));
			//memcpy(dat+5*sizeof(uint32_t),&KF,sizeof(uint32_t));
			dat+=resp.pointcloud.point_step;
		}
	}
	return true;
}

bool System::densepointcloudservice(PTAMM_RGBD_cooperative::PointCloudRequest & req, PTAMM_RGBD_cooperative::PointCloudResponse & resp)
//void System::publish_depth(const sensor_msgs::ImageConstPtr & globaldepth_msg, const sensor_msgs::ImageConstPtr&  globalimg)

{
	//depth points
    int red_offset   = 0;
    int green_offset = 1;
    int blue_offset  = 2;
    int color_step   = 3;
    if(semaphore_copy == false && mpMap->IsGood() && mpTracker->mTrackingQuality==2){
          semaphore_dense_srv = true;
    pcl::PointCloud<pcl::PointXYZRGB> cloud_msg;
    sensor_msgs::PointCloud2 msg_pub;    
     //  PointCloud_convert(globaldepth_msg, globalimg, cloud_msg, msg_pub, red_offset, green_offset, blue_offset, color_step);
//	srv_publishdepth(cloud_msg, resp.pointcloud);//grid and rotation
        extract_PointCloud(globaldepth_msg, globalimg, cloud_msg, msg_pub, red_offset, green_offset, blue_offset, color_step);
	semaphore_dense_srv = false;
                pcl::toROSMsg(cloud_msg, resp.pointcloud);
                resp.pointcloud.header.frame_id = "/world";
                resp.pointcloud.header.stamp = ros::Time::now();
	return true; 
        }
   else if (semaphore_copy == true){
   	return false;
       }
else
return false;
//    points_depth.publish(resp);



}
//////////////////////////////////////////////////////////////////////////////////////////////////////////




////////////////////////////////////////modified Loianno///////////////////////////////////////////////////
void System::publishPreviewImage(CVD::Image<CVD::byte> & img, const std_msgs::Header & header)
{
  CVD::Image<TooN::Vector<2> > & grid = mpTracker->ComputeGrid();
  std::list<Trail> & trails = mpTracker->getTrails();
   bool drawGrid = mpTracker->getTrailTrackingComplete();
   //cout<<"drawGrid"<<drawGrid<<endl;
  bool drawTrails = mpTracker->getTrailTrackingStarted();
  if (pub_preview_image.getNumSubscribers() > 0)
  {
    CVD::ImageRef sub_size(img.size()/2);
    sensor_msgs::ImagePtr img_msg(new sensor_msgs::Image);
    img_msg->header = header;
    img_msg->encoding = sensor_msgs::image_encodings::MONO8;
    img_msg->width = sub_size.x;
    img_msg->height = sub_size.y;
    img_msg->step = sub_size.x;
    img_msg->is_bigendian = 0;
    img_msg->data.resize(sub_size.x * sub_size.y);
    // subsample image
    CVD::BasicImage<CVD::byte> img_sub((CVD::byte *)&img_msg->data[0], sub_size);
    CVD::halfSample(img, img_sub);

    // set opencv pointer to image

    cv::Mat* ocv_img =  new cv::Mat(cv::Size(img_sub.size().x, img_sub.size().y), CV_8UC1, img_msg->data[0]);
    ocv_img->data = (uchar*)&img_msg->data[0];
    int dim0 = grid.size().x;
    int dim1 = grid.size().y;
    if (drawGrid)
    {
      for (int i = 0; i < dim0; i++)
      {
        for (int j = 0; j < dim1 - 1; j++)
          cv::line(*ocv_img, cv::Point(grid[i][j][0]/2, grid[i][j][1]/2), cv::Point(grid[i][j + 1][0]/2, grid[i][j + 1][1]/2),
        		  CV_RGB(50, 50, 50));

        for (int j = 0; j < dim1 - 1; j++)
          cv::line(*ocv_img, cv::Point(grid[j][i][0]/2, grid[j][i][1]/2), cv::Point(grid[j + 1][i][0]/2, grid[j + 1][i][1]/2),
                CV_RGB(50, 50, 50));
      }
    }
    if (drawTrails)
    {

      ParamsAccess Params;
      int level = Params.fixParams->InitLevel;

      for (std::list<Trail>::iterator i = trails.begin(); i != trails.end(); i++)
      {
        cv:line(*ocv_img, cv::Point(LevelZeroPos(i->irCurrentPos.x, level)/2, LevelZeroPos(i->irCurrentPos.y, level)/2),
               cv::Point(LevelZeroPos(i->irInitialPos.x, level)/2, LevelZeroPos(i->irInitialPos.y, level)/2),
               CV_RGB(0, 0, 0), 2);
      }
    }
    pub_preview_image.publish(img_msg);
    ocv_img->release();
  }
}


void System::keyboardCallback(const std_msgs::StringConstPtr & kb_input){
	command(kb_input->data);
}
void System::command(const std::string & cmd){

  if(!ParamsAccess::fixParams->gui)
   this->GUICommandCallBack(this, "KeyPress", cmd);
   else
    GUICommandHandler("KeyPress", cmd);
}
// This is called in the tracker's own thread.
void System::GUICommandHandler(string sCommand, string sParams)  // Called by the callback func..
{
	if(sCommand=="Reset")
	{
		mpTracker->Reset();
		return;
	}

	// KeyPress commands are issued by GLWindow
	if(sCommand=="KeyPress")
	{
		if(sParams == "Space")
		{
			mpTracker->mbUserPressedSpacebar = true;
		}
		else if(sParams == "r")
		{
			mAutoreset=false;
			ROS_WARN_STREAM("Forcing map reset because of user input! Autoreset set to " << mAutoreset);
			mpTracker->Reset();
		}
		else if(sParams == "e")
		{
			mAutoreset=false;
			ROS_WARN_STREAM("Forcing all maps to reset because of user input!" << mAutoreset);
			GUICommandCallBack(this, "ResetAll", sParams);
		}

		else if(sParams == "a")	// autoreset test button
		{
			mAutoreset=true;
			ROS_WARN_STREAM("Forcing map reset because of user input! Autoreset set to " << mAutoreset);

		}
		else if(sParams == "n")
		{
			mAutoreset=true;
			ROS_WARN_STREAM("Forcing new map generate!" << mAutoreset);
			GUICommandCallBack(this, "NewMap", sParams);

		}

		else if(sParams == "q" || sParams == "Escape")
		{
			GUI.ParseLine("quit");
		}
		return;
	}
	if((sCommand=="PokeTracker"))
	{

		mpTracker->mbUserPressedSpacebar = true;
		return;
	}


	cout << "! Tracker::GUICommandHandler: unhandled command "<< sCommand << endl;
	exit(1);
};
//////////////////////////////////////////////////////////////////////////////////////////////////////////

/**
 * Parse commands sent via the GVars command system.
 * @param ptr Object callback
 * @param sCommand command string
 * @param sParams parameters
 */
void System::GUICommandCallBack(void *ptr, string sCommand, string sParams)
{
  if(sCommand=="quit" || sCommand == "exit") {
    static_cast<System*>(ptr)->mbDone = true;
  }
  else if( sCommand == "SwitchMap")
  {
    int nMapNum = -1;
    if( static_cast<System*>(ptr)->GetSingleParam(nMapNum, sCommand, sParams) ) {
      static_cast<System*>(ptr)->SwitchMap( nMapNum );
    }
  }
  else if(sCommand == "ResetAll")
  {


    static_cast<System*>(ptr)->ResetAll();
    return;
  }
  else if( sCommand == "NewMap")
  {
    cout << "Making new map..." << endl;
    static_cast<System*>(ptr)->NewMap();
  }
  else if( sCommand == "DeleteMap")
  {
    int nMapNum = -1;
    if( sParams.empty() ) {
        static_cast<System*>(ptr)->DeleteMap( static_cast<System*>(ptr)->mpMap->MapID() );
    }
    else if( static_cast<System*>(ptr)->GetSingleParam(nMapNum, sCommand, sParams) ) {
      static_cast<System*>(ptr)->DeleteMap( nMapNum );
    }
  }
  else if( sCommand == "NextMap")  {
    static_cast<MapViewer*>(ptr)->ViewNextMap();
  }
  else if( sCommand == "PrevMap")  {
    static_cast<MapViewer*>(ptr)->ViewPrevMap();
  }
  else if( sCommand == "CurrentMap")  {
    static_cast<MapViewer*>(ptr)->ViewCurrentMap();
  }
  else if( sCommand == "SaveMap" || sCommand == "SaveMaps" || sCommand == "LoadMap")  {
    static_cast<System*>(ptr)->StartMapSerialization( sCommand, sParams );
  }
 // else if( sCommand == "LoadGame" )
  //{
    //static_cast<ARDriver*>(ptr)->LoadGame(sParams);
  //}
  else if( sCommand == "Mouse.Click" ) {
    vector<string> vs = ChopAndUnquoteString(sParams);
    
    if( vs.size() != 3 ) {
      return;
    }

    istringstream is(sParams);
    int nButton;
    ImageRef irWin;
    is >> nButton >> irWin.x >> irWin.y;
    //static_cast<ARDriver*>(ptr)->HandleClick( nButton, irWin );
    
  }
  else if( sCommand == "KeyPress" )
  {
    if(sParams == "q" || sParams == "Escape")
    {
      GUI.ParseLine("quit");
      return;
    }

    bool bUsed = static_cast<System*>(ptr)->mpTracker->HandleKeyPress( sParams );//spacebar

  }
    

}


/**
 * Parse and allocate a single integer variable from a string parameter
 * @param nAnswer the result
 * @param sCommand the command (used to display usage info)
 * @param sParams  the parameters to parse
 * @return success or failure.
 */
bool System::GetSingleParam(int &nAnswer, string sCommand, string sParams)
{
  vector<string> vs = ChopAndUnquoteString(sParams);
    
  if(vs.size() == 1)
  {
    //is param a number?
    bool bIsNum = true;
    for( size_t i = 0; i < vs[0].size(); i++ ) {
      bIsNum = isdigit( vs[0][i] ) && bIsNum;
    }
      
    if( !bIsNum )
    {
      return false;
    }

    int *pN = ParseAndAllocate<int>(vs[0]);
    if( pN )
    {
      nAnswer = *pN;
      delete pN;
      return true;
    }
  }

  cout << sCommand << " usage: " << sCommand << " value" << endl;

  return false;
}


/**
 * Switch to the map with ID nMapNum
 * @param  nMapNum Map ID
 * @param bForce This is only used by DeleteMap and ResetAll, and is
 * to ensure that MapViewer is looking at a safe map.
 */
bool System::SwitchMap( int nMapNum, bool bForce )
{

  //same map, do nothing. This should not actually occur
  if(mpMap->MapID() == nMapNum) {
    return true;
  }
  if( (nMapNum < 0) )
  {
    cerr << "Invalid map number: " << nMapNum << ". Not changing." << endl;
    return false;
  }

  
  for( size_t ii = 0; ii < mvpMaps.size(); ii++ )
  {
    Map * pcMap = mvpMaps[ ii ];
    if( pcMap->MapID() == nMapNum ) {
      mpMap->mapLockManager.UnRegister( this );
      mpMap = pcMap;
      mpMap->mapLockManager.Register( this );
    }
  }
  if(mpMap->MapID() != nMapNum)
  {
    cerr << "Failed to switch to " << nMapNum << ". Does not exist." << endl;
    return false;
  }
  /*  Map was found and switched to for system.
      Now update the rest of the system.
      Order is important. Do not want keyframes added or
      points deleted from the wrong map.
  
      MapMaker is in its own thread.
      System,Tracker, and MapViewer are all in this thread.
  */
  if(ParamsAccess::fixParams->gui)
  *mgvnLockMap = mpMap->bEditLocked;

 
  //update the map maker thread
  if( !mpMapMaker->RequestSwitch( mpMap ) ) {
    return false;
  }
  
  while( !mpMapMaker->SwitchDone() ) {
#ifdef WIN32
    Sleep(1);
#else
    usleep(10);
#endif
  }

  //update the map viewer object
  mpMapViewer->SwitchMap(mpMap, bForce);

  
  if( !mpTracker->SwitchMap( mpMap ) ) {
    return false;
  }

  return true;
}
///////////////////////////////modified Loianno///////////////////////////////////////

void System::DeleteTracker(int numtracker)
{
if(mvpTracker.size()==1){
	cout<<"cannot erase tracker since only one tracker is defined"<<endl;
}
else if (numtracker>mvpTracker.size())
	cout<<"the tracker number to erase is not defined"<<endl;
else
	mvpTracker.erase(mvpTracker.begin()+numtracker);
}
//////////////////////////////////////////////////////////////////////////////////
/**
 * Create a new map and switch all
 * threads and objects to it.
 */
void System::NewMap()
{
if(ParamsAccess::fixParams->gui)
  *mgvnLockMap = false;
  mpMap->mapLockManager.UnRegister( this );
  mpMap = new Map();
  mpMap->mapLockManager.Register( this );
  mvpMaps.push_back( mpMap );
  //update the map maker thread
  mpMapMaker->RequestReInit( mpMap );
  while( !mpMapMaker->ReInitDone() ) {
#ifdef WIN32
    Sleep(1);
#else
    usleep(10);
#endif
  }

  //update the map viewer object
  mpMapViewer->SwitchMap(mpMap);

  //update the tracker object
  mpTracker->SetNewMap( mpMap );

  cout << "New map created (" << mpMap->MapID() << ")" << endl;
	///////////////////////modified Loianno///////////////////////////////
	  scale_factor_final_filtered.clear();
	  scale_factor_final_filtered.resize(2);
	  //counter_scale_factor = 0;
	  scale_factor_final_scalar.resize(mpMap->MapID()+1);//resize vector to increase the number of scale factor according to maps
	  counter_scale_factor.resize(mpMap->MapID()+1);
	  counter_scale_factor[mpMap->MapID()] = 0;
	  size_old.resize(mpMap->MapID()+1);
	  size_old[mpMap->MapID()] = 0;
	  mpTracker->trail_sempahore = false;
	////////////////////////////////////////////////////////////////////////
}


/**
 * Moves all objects and threads to the first map, and resets it.
 * Then deletes the rest of the maps, placing PTAMM in its
 * original state.
 * This reset ignores the edit lock status on all maps
 */
void System::ResetAll()
{
	cout<<"size"<<mvpMaps.size()<<endl;
  //move all maps to first map.
  if( mpMap != mvpMaps.front() )
  {

    if( !SwitchMap( mvpMaps.front()->MapID(), true ) ) {
      cerr << "Reset All: Failed to switch to first map" << endl;
    }
  }
  mpMap->bEditLocked = false;

  //reset map.
  mpTracker->Reset();

  //lock and delete all remaining maps
  while( mvpMaps.size() > 1 )
  {
    DeleteMap( mvpMaps.back()->MapID() );
  }

}


/**
 * Delete a specified map.
 * @param nMapNum map to delete
 */
bool System::DeleteMap( int nMapNum )
{

  if( mvpMaps.size() <= 1 )
  {
    cout << "Cannot delete the only map. Use Reset instead." << endl;
    return false;
  }


  //if the specified map is the current map, move threads to another map
  if( nMapNum == mpMap->MapID() )
  {
    int nNewMap = -1;
    
    if( mpMap == mvpMaps.front() ) {
      nNewMap = mvpMaps.back()->MapID();
    }
    else {
      nNewMap = mvpMaps.front()->MapID();
    }
    
    // move the current map users elsewhere
    if( !SwitchMap( nNewMap, true ) ) {
      cerr << "Delete Map: Failed to move threads to another map." << endl;
      return false;
    }
  }

  
 
  // find and delete the map
  for( size_t ii = 0; ii < mvpMaps.size(); ii++ )
  {
    Map * pDelMap = mvpMaps[ ii ];
    if( pDelMap->MapID() == nMapNum ) {

      pDelMap->mapLockManager.Register( this );
      pDelMap->mapLockManager.LockMap( this );
      delete pDelMap;
      mvpMaps.erase( mvpMaps.begin() + ii );

      ///@TODO Possible bug. If another thread (eg serialization) was using this
      /// and waiting for unlock, would become stuck or seg fault.
    }
  }
  
  return true;
}


/**
 * Set up the map serialization thread for saving/loading and the start the thread
 * @param sCommand the function that was called (eg. SaveMap)
 * @param sParams the params string, which may contain a filename and/or a map number
 */
void System::StartMapSerialization(std::string sCommand, std::string sParams)
{
  if( mpMapSerializer->Init( sCommand, sParams, *mpMap) ) {
    mpMapSerializer->start();
  }
}


/**
 * Draw a box with information about the maps.
 */
void System::DrawMapInfo()
{
  int nLines = static_cast<int>(mvpMaps.size()) + 2;
  int x = 5, y = 120, w = 160, nBorder = 5;
  ////////////////////modiefied Loianno//////////////////
  mGLWindow->DrawBox( x, y, w, nLines, 0.7f );
////////////////////////////////////////////////////////
  y += 17;

  glColor3f(1,1,1);
  std::ostringstream os;
  os << "Maps " << mvpMaps.size();
  mGLWindow->PrintString( ImageRef(x + nBorder,y + nBorder), os.str() );
  os.str("");
      
  for( size_t i = 0; i < mvpMaps.size(); i++ )
  {
    Map * pMap = mvpMaps[i];
    if( pMap == mpMap ) {
      glColor3f(1,1,0);
    }
    else if( pMap->bEditLocked ) {
      glColor3f(1,0,0);
    }
    else {
      glColor3f(1,1,1);
    }
    
    os << "M: " << pMap->MapID() << "  P: " << pMap->vpPoints.size() << "  K: " << pMap->vpKeyFrames.size();
    mGLWindow->PrintString( ImageRef( x + nBorder , y + nBorder + (i+1)*17), os.str() );
    os.str("");
  }
}


/**
 * Save the current frame to a FIFO.
 * This function is called on each frame to create a video.
 * The GVar SaveFIFO starts and stops the saving, and the GVar
 * Bitrate sets the quality.
 * Bitrate can only be set before the first call of SaveFIFO.
 */
void System::SaveFIFO()
{
#ifdef _LINUX
  //Some static variables
  static CVD::byte* pcImage = NULL;
  static int fd = 0;
  static bool bFIFOInitDone = false;
  static ImageRef irWindowSize;

  if( !bFIFOInitDone )
  {
    irWindowSize = mGLWindow.size();

    ostringstream os;
    os << /*"/bin/bash\n" <<*/
        "file=\"`date '+%Y-%m-%d_%H-%M-%S'`.avi\"; " <<
        "if [ ! -e FIFO ]; then mkfifo FIFO; echo Made FIFO...; fi; " <<
        "echo Mencoding to $file....; " <<
        "cat FIFO |nice mencoder -flip -demuxer rawvideo -rawvideo fps=30:w=" <<
        irWindowSize.x << ":h=" << irWindowSize.y <<
        ":format=rgb24 -o $file -ovc lavc -lavcopts vcodec=mpeg4:vbitrate=" << *mgvnBitrate <<
        ":keyint=45 -ofps 30 -ffourcc DIVX - &";

    cout << "::" << os.str()<< "::" << endl;
    int i = system( os.str().c_str() );
    if( i != 0 ) {
      cerr << "ERROR: could not set up the FIFO!" << endl;
      return;
    }

    posix_memalign((void**)(&pcImage), 16, irWindowSize.x*irWindowSize.y*3);
    string s = "FIFO";
    fd = open(s.c_str(), O_RDWR | O_ASYNC);

    bFIFOInitDone = true;
  }
  
  if( irWindowSize != mGLWindow.size() )
  {
    cerr << "ERROR: Aborting FIFO as window size has changed!!" << endl;
    *mgvnSaveFIFO = 0;
    return;
  }

  glReadBuffer(GL_BACK);
  glReadPixels(0,0,irWindowSize.x,irWindowSize.y,GL_RGB, GL_UNSIGNED_BYTE, pcImage);
  write(fd, (char*) pcImage, irWindowSize.x*irWindowSize.y*3);

#else
  cout << "Video Saving using FIFOs is only available under Linux" << endl;
#endif

}


}

