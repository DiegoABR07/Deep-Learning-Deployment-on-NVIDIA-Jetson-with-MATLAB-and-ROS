#ifndef ALEXNET_HPP_
#define ALEXNET_HPP_

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/UInt32.h>
#include <vector>
#include <cstdint>
#include "myAlexNetGPU.h"

class GpuNetPub
{
private:
  ros::Publisher data_pub_;
  ros::Publisher img_pub_;

public:
  GpuNetPub(const std::string& data_topic_name,
            const std::string& img_topic_name,
            int msg_limit,
            ros::NodeHandle* nh)
  {
      data_pub_ = nh->advertise<std_msgs::UInt32>(data_topic_name, msg_limit);
      img_pub_  = nh->advertise<sensor_msgs::Image>(img_topic_name, msg_limit);
  }

  void msgCallback(const sensor_msgs::Image::ConstPtr& inmsg)
  {
      constexpr int32_t SUB_SIZE  = 227;
      constexpr int32_t CHANNELS  = 3;

      if (inmsg->width < SUB_SIZE || inmsg->height < SUB_SIZE)
      {
          ROS_WARN_THROTTLE(5, "Incoming image too small (%ux%u)",
                            inmsg->width, inmsg->height);
          return;
      }

      uint8_t input[SUB_SIZE * SUB_SIZE * CHANNELS];
      std::vector<uint8_t> img_msg_data(SUB_SIZE * SUB_SIZE * CHANNELS);

      int32_t h_off = (inmsg->height - SUB_SIZE) / 2;
      int32_t w_off = (inmsg->width  - SUB_SIZE) / 2;
      int32_t out_idx = 0;

      // RGB row‑major  →  column‑major que espera GPU‑Coder
      for (int32_t i = 0; i < SUB_SIZE; ++i)
      {
          for (int32_t j = 0; j < SUB_SIZE; ++j)
          {
              int32_t base = (h_off + i) * inmsg->step
                           + (w_off + j) * CHANNELS;
              for (int32_t k = 0; k < CHANNELS; ++k)
              {
                  int32_t idx = (SUB_SIZE * SUB_SIZE * k)
                              + (SUB_SIZE * j) + i;
                  uint8_t v = inmsg->data[base + k];
                  input[idx]          = v;
                  img_msg_data[out_idx++] = v;
              }
          }
      }

      float class_idx = myAlexNetGPU(input);
      ROS_INFO_STREAM_THROTTLE(1, "Classification index: " << class_idx);

      std_msgs::UInt32 cls_msg;
      cls_msg.data = static_cast<uint32_t>(class_idx);
      data_pub_.publish(cls_msg);

      sensor_msgs::Image img_msg;
      img_msg.height   = SUB_SIZE;
      img_msg.width    = SUB_SIZE;
      img_msg.encoding = "rgb8";
      img_msg.step     = CHANNELS * SUB_SIZE;
      img_msg.data     = std::move(img_msg_data);
      img_pub_.publish(img_msg);
  }
};

#endif // ALEXNET_HPP_
