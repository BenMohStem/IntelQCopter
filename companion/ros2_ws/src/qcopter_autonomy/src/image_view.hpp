#pragma once
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/imgproc.hpp>
#include <stdexcept>
#include <cstring>
#include <algorithm>

namespace qcopter {
// Own the result: a ROS message can leave scope after its callback. Validate the
// byte stride and storage before constructing any OpenCV view.
inline cv::Mat image_copy(const sensor_msgs::msg::Image& m) {
  int type;unsigned bytes;
  if(m.encoding=="rgb8"||m.encoding=="bgr8"){type=CV_8UC3;bytes=3;}
  else if(m.encoding=="32FC1"){type=CV_32FC1;bytes=4;}
  else if(m.encoding=="16UC1"){type=CV_16UC1;bytes=2;}
  else throw std::invalid_argument("Unsupported ROS image encoding");
  if(!m.width||!m.height||m.width>16384||m.height>16384
     ||m.step<size_t(m.width)*bytes||m.data.size()<size_t(m.step)*m.height)
    throw std::invalid_argument("Invalid ROS image size or stride");
  cv::Mat out(int(m.height),int(m.width),type);
  uint16_t one=1;const bool host_big=*reinterpret_cast<uint8_t*>(&one)==0;
  for(unsigned y=0;y<m.height;y++){
    const auto*src=m.data.data()+size_t(y)*m.step;auto*dst=out.ptr<uint8_t>(int(y));
    std::memcpy(dst,src,size_t(m.width)*bytes);
    if(bytes>1&&type!=CV_8UC3&&bool(m.is_bigendian)!=host_big)
      for(unsigned x=0;x<m.width;x++)std::reverse(dst+x*bytes,dst+(x+1)*bytes);
  }
  if(m.encoding=="rgb8")cv::cvtColor(out,out,cv::COLOR_RGB2BGR);
  return out;
}
}
