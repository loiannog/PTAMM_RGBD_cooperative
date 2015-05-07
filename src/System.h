// -*- c++ -*-
// Copyright 2009 Isis Innovation Limited
//
// System.h
//
// Defines the System class
//
// This stores the main functional classes of the system, like the
// mapmaker, map, tracker etc, and spawns the working threads.
//
#ifndef __SYSTEM_H
#define __SYSTEM_H
#include "GLWindow2.h"

#include <gvars3/instances.h>

#include <cvd/image.h>
#include <cvd/rgb.h>
#include <cvd/byte.h>
#include <cvd/image_convert.h>
//////////////////////////////////////modified Loianno///////////////////////////////////
#include "MapPoint.h"
using namespace std;
using namespace GVars3;
using namespace PTAMM;
#include <fstream>
#include <TooN/TooN.h>
#include <TooN/se3.h>
//#include </opt/ros/fuerte/stacks/image_pipeline-1.8/depth_image_proc/src/nodelets/depth_traits.h>
#include "MapSerializer.h"
#include <boost/thread.hpp>
#include <boost/date_time.hpp>
#include <boost/bind.hpp>
#include <boost/array.hpp>
#include <tf/tfMessage.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/TransformStamped.h>
#include <cvd/utility.h>
#include <cvd/convolution.h>
#include <cvd/vision.h>


#include <sensor_msgs/PointCloud2.h>


#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/image_encodings.h>
//#include <cv_bridge/CvBridge.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <std_msgs/Int8.h>

// PCL specific includes
//#include <pcl/ros/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/impl/point_types.hpp>
#include "sensor_msgs/PointCloud.h"
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <nav_msgs/Odometry.h>
#include <PTAMM_RGBD_cooperative/KeyFrame_srv.h>
#include <PTAMM_RGBD_cooperative/KeyFrame_msg.h>
#include <std_msgs/Float32MultiArray.h>

using namespace std;
using namespace TooN;

using namespace message_filters;


#include <ParamsPTAMM.h>
#include <PTAMM_RGBD_cooperative/PointCloud.h>
#include <PTAMM_RGBD_cooperative/PointCloud_msg.h>
#include <std_msgs/Float32.h>
#include "Relocaliser.h"

#include <std_msgs/String.h>
#define r_max 480
#define c_max 640
//////////////////////////////////////////////////////////////////////////////////////

namespace PTAMM {

class ATANCamera;
class Map;
class MapMaker;
class Tracker;
//class ARDriver;
class MapViewer;
class MapSerializer;

class System
{
  public:
    System();
    ~System();
    void Run();
    void imageCallback(const sensor_msgs::ImageConstPtr & img, const sensor_msgs::ImageConstPtr & depth_msg);
    void add_tracker(const sensor_msgs::ImageConstPtr & img, const sensor_msgs::ImageConstPtr & depth_msg);
    message_filters::Subscriber<sensor_msgs::Image>* left_sub_p;
    message_filters::Subscriber<sensor_msgs::Image>* right_sub_p;
    message_filters::Subscriber<sensor_msgs::Image>* left_sub_p2;
    message_filters::Subscriber<sensor_msgs::Image>* right_sub_p2;


	  
/*
int DR;
int DC;
int STEP_R_MAX;
int STEP_R_MIN;
int STEP_C_MAX;
int STEP_C_MIN;
int Z_MAX;
int Z_MIN;
*/
  
  private:
    static void GUICommandCallBack(void* ptr, std::string sCommand, std::string sParams);  //process a console command
    bool GetSingleParam(int &nAnswer, std::string sCommand, std::string sParams);          //Extract an int param from a command param
    bool SwitchMap( int nMapNum, bool bForce = false );                                    // Switch to a particular map.
    void NewMap();                                  // Create a new map and move all elements to it
    bool DeleteMap( int nMapNum );                  // Delete a specified map
    void ResetAll();                                // Wipes out ALL maps, returning system to initial state
    void StartMapSerialization(std::string sCommand, std::string sParams);   //(de)serialize a map
    void DrawMapInfo();                             // draw a little info box about the maps
    void SaveFIFO();                                // save the video out to a FIFO (save to disk)

    ///////////////////////modified Loianno//////////////////////////////////
    image_transport::Subscriber mImageSub;
    ros::Subscriber depth_3D_points;
    ros::Subscriber points_depth_rotated;
    ros::Subscriber tot_map_points;
    ros::Subscriber tot_keyframes;

    ros::Publisher pub_pose;
    ros::Publisher pub_pose_world;       // camera in the world frame
    ros::Publisher pub_pose_overlap;
    ros::Publisher points;
    ros::Publisher points_depth;
    ros::Publisher num_map;
    ros::ServiceServer srvKF;
    ros::ServiceServer srvPC;
    ros::ServiceServer srvdensePC;
    ros::Subscriber sub_kb_input;
    ros::Subscriber subIMU;
    image_transport::Publisher pub_preview_image;


    std::vector<double> scale_factor_final_filtered;
    std::vector <double> scale_factor_final_scalar;
    std::vector<int> counter_scale_factor;

    std::vector<int>  size_old;
    bool tracker2_active;
    bool semaphore_dense_srv;

    sensor_msgs::ImageConstPtr globalimg;
    sensor_msgs::ImageConstPtr globaldepth_msg;
    TooN::SE3<double>* pose_global_img;

    //bool mbUserPressedSpacebar;
    bool mAutoreset;
    void PointCloud_convert(const sensor_msgs::ImageConstPtr depth_msg,
                                          const sensor_msgs::ImageConstPtr rgb_msg,
                                          pcl::PointCloud<pcl::PointXYZRGB>& cloud_msg,
                                           sensor_msgs::PointCloud2& msg_pub,
                                          int red_offset, int green_offset, int blue_offset, int color_step);
    void PointCloud_convert2(const sensor_msgs::ImageConstPtr depth_msg,
                                          const sensor_msgs::ImageConstPtr rgb_msg,
                                          pcl::PointCloud<pcl::PointXYZRGB>& cloud_msg,
                                           sensor_msgs::PointCloud2& msg_pub,
                                          int red_offset, int green_offset, int blue_offset, int color_step);
    void transformPoints(TooN::Matrix<3> RotCurr,
    		      TooN::Vector<3> TransCurr,
    		      pcl::PointCloud <pcl::PointXYZRGB>& depth_points);


        void extract_PointCloud(const sensor_msgs::ImageConstPtr depth_msg,
                                      const sensor_msgs::ImageConstPtr rgb_msg,
                                      pcl::PointCloud<pcl::PointXYZRGB>& cloud_msg,
                                       sensor_msgs::PointCloud2& msg_pub,
                                      int red_offset, int green_offset, int blue_offset, int color_step);

    //void srv_publishdepth(const sensor_msgs::PointCloud2& msg_ptr);
    void srv_publishdepth(pcl::PointCloud<pcl::PointXYZRGB> depth_points, sensor_msgs::PointCloud2 & resp);
    bool densepointcloudservice(PTAMM_RGBD_cooperative::PointCloudRequest & req, PTAMM_RGBD_cooperative::PointCloudResponse & resp);
    void scale_factor_computation(const sensor_msgs::ImageConstPtr & depth_msg, Tracker *curr_tracker);
    bool keyframesservice(PTAMM_RGBD_cooperative::KeyFrame_srvRequest & req, PTAMM_RGBD_cooperative::KeyFrame_srvResponse & resp);
    bool pointcloudservice(PTAMM_RGBD_cooperative::PointCloudRequest & req, PTAMM_RGBD_cooperative::PointCloudResponse & resp);
    void publishPreviewImage(CVD::Image<CVD::byte> & img, const std_msgs::Header & header);
    void keyboardCallback(const std_msgs::StringConstPtr& kb_input);
    void command(const std::string & cmd);
    void GUICommandHandler(string sCommand, string sParams);
    void DeleteTracker(int numtracker);
    void callback_new_tracker();
    void publishPoseAndInfo(const std_msgs::Header & header, Tracker* curr_tracker);
    bool drawfirsttracker;
    //void publish_depth(const sensor_msgs::ImageConstPtr& globaldepth_msg, const sensor_msgs::ImageConstPtr& globalimg);
    void readkeyframecallback(const PTAMM_RGBD_cooperative::KeyFrame_msg & resp);
    void readmapPointscallback(const PTAMM_RGBD_cooperative::PointCloud_msg & resp);
    TooN::SE3<double> Best_estimation;
    bool is_relocalized;
    bool scan_completed;
    bool semaphore_copy;
    int counter_frames;
    double best_score;
    ////////////////////////////////////////////////////////////////////////


    
  private:
    //VideoSource mVideoSource;                       // The video image source//modified Loianno
    GLWindow2* mGLWindow;                            // The OpenGL window
    CVD::Image<CVD::Rgb<CVD::byte> > mimFrameRGB;   // The RGB image used for AR
    CVD::Image<CVD::byte> mimFrameBW;               // The Black and white image for tracking/mapping

    std::vector<Map*> mvpMaps;                      // The set of maps
    Map *mpMap;                                     // The current map
    MapMaker *mpMapMaker;                           // The map maker
    std::vector<Tracker*> mvpTracker;               // The set of trackers modified Loianno
    Tracker *mpTracker;                             // The tracker
    ATANCamera *mpCamera;                           // The camera model
   // ARDriver *mpARDriver;                           // The AR Driver
    MapViewer *mpMapViewer;                         // The Map Viewer
    MapViewer *mpMapViewer2;                         // The Map Viewer

    MapSerializer *mpMapSerializer;                 // The map serializer for saving and loading maps
    
    bool mbDone;                                    // Kill?
    
    GVars3::gvar3<int> mgvnLockMap;                 // Stop a map being edited - i.e. keyframes added, points updated
    GVars3::gvar3<int> mgvnDrawMapInfo;             // Draw map info on the screen
    GVars3::gvar3<int> mgvnDrawNewTracker;             // Draw the second tracker

#ifdef _LINUX
    GVars3::gvar3<int> mgvnSaveFIFO;                // Output to a FIFO (make a video)
    GVars3::gvar3<int> mgvnBitrate;                 // Bitrate to encode at
#endif
};

}

#endif
