// Copyright 2008 Isis Innovation Limited
//#include "VideoSource.h"
#include "VideoSource_ros.h"

namespace PTAMM {

using namespace CVD;
using namespace std;
using namespace GVars3;


VideoSource::VideoSource()
{
  cout << "  VideoSource_Linux: Opening video source..." << endl;
  cout << "  ... got video source." << endl;
};


ImageRef VideoSource::Size()
{ 
  return mirSize;
};




void VideoSource::GetAndFillFrameBWandRGB(Image<byte> &imBW, Image<Rgb<byte> > &imRGB)
{
  //while(mNewImage==false) usleep(50);
  //V4LBuffer<yuv422>* pvb = (V4LBuffer<yuv422>*) mptr;
  //VideoFrame<yuv422> *pVidFrame = pvb->get_frame();
  //cout<<"mCurrentImage:"<<endl;
  CVD::copy(mCurrentImage, imBW);
  CVD::copy(mCurrentImage, imRGB);
  //pvb->put_frame(pVidFrame);
  //mNewImage = false;

};

}
