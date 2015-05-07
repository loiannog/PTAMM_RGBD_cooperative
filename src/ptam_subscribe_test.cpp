#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;

void imageCallback(const sensor_msgs::Image::ConstPtr &img1, const sensor_msgs::Image::ConstPtr &img2)
{
  ROS_INFO("imageCallback");
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ptamm_subscribe_test");
  ros::NodeHandle nh("~");

	message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nh, "rgb/image_raw", 10);
	message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, "depth_registered/image_raw", 10);

  message_filters::Synchronizer<MySyncPolicy> sync1(MySyncPolicy(4), rgb_sub, depth_sub);
	sync1.registerCallback(boost::bind(&imageCallback, _1, _2));

  ros::Rate r(100);
  while(nh.ok())
  {
    ros::spinOnce();
    r.sleep();
  }
  return 0;
}

