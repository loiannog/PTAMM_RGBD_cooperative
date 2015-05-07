// This is a very simple class to provide video input; this can be
// replaced with whatever form of video input that is needed.  It
// should open the video input on construction, and provide two
// function calls after construction: Size() must return the video
// format as an ImageRef, and GetAndFillFrameBWandRGB should wait for
// a new frame and then overwrite the passed-as-reference images with
// GreyScale and Colour versions of the new frame.
//

/*
author: Giuseppe Loianno
*/
#include <cvd/image.h>
#include <cvd/byte.h>
#include <cvd/rgb.h>
#include <sensor_msgs/Image.h>
#include "GLWindow2.h"
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
//#include <cvd/Linux/v4lbuffer.h>
#include <cvd/colourspace_convert.h>
#include <cvd/colourspaces.h>
#include <cvd/utility.h>
#include <gvars3/instances.h>
#include <time.h>

namespace PTAMM {

struct VideoSourceData;

class VideoSource
{
 public:
 CVD::Image<CVD::Rgb<CVD::byte> > mCurrentImage;

  VideoSource();
  //void imageCallback(const sensor_msgs::ImageConstPtr & img);
  void GetAndFillFrameBWandRGB(CVD::Image<CVD::byte> &imBW, CVD::Image<CVD::Rgb<CVD::byte> > &imRGB);
  CVD::ImageRef Size();
  bool mNewImage;
    CVD::ImageRef mirSize;
 private:
  void *mptr;

};


}
