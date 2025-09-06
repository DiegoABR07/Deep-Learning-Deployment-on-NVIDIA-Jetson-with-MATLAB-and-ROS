#pragma once
#include <string>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Float32MultiArray.h>
#include <image_transport/image_transport.h>

// Interfaz C (emxArray) del código generado
#include "myDetectorGPU.h"
#include "myDetectorGPU_emxAPI.h"

class DetectorNode {
public:
  explicit DetectorNode(ros::NodeHandle& nh);
  ~DetectorNode();

private:
  // ROS
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  ros::Subscriber sub_;
  image_transport::Publisher pub_view_;
  ros::Publisher pub_dets_;

  // Parámetros
  std::string input_topic_;
  int sub_h_{416};
  int sub_w_{416};
  int channels_{3};        // rgb8
  double threshold_{0.5};

  // Mensaje reutilizable para /network_view
  sensor_msgs::Image view_msg_;

  // Callback
  void imageCb_(const sensor_msgs::ImageConstPtr& msg);

  // Lógica
  void runInference_(const uint8_t* crop_rgb8, int stride_bytes);
  void publishView_(const uint8_t* crop_rgb8);
  void publishDetections_(const emxArray_real_T* bboxes,
                          const emxArray_real_T* scores,
                          const emxArray_real_T* labels);
};
