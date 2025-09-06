#ifndef CLASSIFIER_HPP_
#define CLASSIFIER_HPP_

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/UInt32.h>
#include <string>
#include <vector>
#include <cstdint>

// Cabecera generada por GPU Coder para tu función
// (debe estar en tu include path; sustituye por el nombre exacto si difiere)
#include "myClassifierGPU.h"

class ClassifierNode
{
public:
  ClassifierNode(const std::string& out_topic,
                 const std::string& view_topic,
                 int queue_size,
                 ros::NodeHandle* nh,
                 int sub_h,
                 int sub_w,
                 int channels);

  void msgCallback(const sensor_msgs::Image::ConstPtr& inmsg);

private:
  ros::Publisher data_pub_;  // Publica índice de clase (UInt32)
  ros::Publisher img_pub_;   // Publica la imagen recortada/mostrada (Image)

  int32_t SUB_H;             // Alto de la subimagen (input a la red)
  int32_t SUB_W;             // Ancho de la subimagen (input a la red)
  int32_t CHANNELS;          // Canales que espera la red (normalmente 3)

  // (Opcional) guarda el último encoding visto para debug
  std::string last_encoding_;
};

#endif // CLASSIFIER_HPP_
