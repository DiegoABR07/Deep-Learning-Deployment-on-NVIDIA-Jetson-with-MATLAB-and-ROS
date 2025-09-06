#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/UInt32.h>
#include <string>
#include <vector>
#include <cstdint>

#include "classifier.hpp"  // Contiene la declaración de ClassifierNode y #include "myClassifierGPU.h"

ClassifierNode::ClassifierNode(const std::string& out_topic,
                               const std::string& view_topic,
                               int queue_size,
                               ros::NodeHandle* nh,
                               int sub_h,
                               int sub_w,
                               int channels)
: SUB_H(sub_h), SUB_W(sub_w), CHANNELS(channels)
{
  data_pub_ = nh->advertise<std_msgs::UInt32>(out_topic, queue_size);
  img_pub_  = nh->advertise<sensor_msgs::Image>(view_topic, queue_size);

  ROS_INFO("ClassifierNode: SUB_H=%d SUB_W=%d CHANNELS=%d",
           SUB_H, SUB_W, CHANNELS);
}

void ClassifierNode::msgCallback(const sensor_msgs::Image::ConstPtr& inmsg)
{
  // --- 0) Detectar encoding de entrada y número de canales fuente (SRC_CH) ---
  const std::string enc = inmsg->encoding; // "rgb8", "bgr8", "mono8", etc.
  if (enc != last_encoding_) {
    ROS_INFO("Incoming image encoding: %s (step=%u)", enc.c_str(), inmsg->step);
    last_encoding_ = enc;
  }

  const bool is_bgr  = (enc == "bgr8");
  const bool is_rgb  = (enc == "rgb8");
  const bool is_mono = (enc == "mono8");

  int SRC_CH = 0;
  if (is_rgb || is_bgr)      SRC_CH = 3;
  else if (is_mono)          SRC_CH = 1;
  else {
    ROS_WARN_THROTTLE(5.0, "Unsupported image encoding: %s", enc.c_str());
    return;
  }

  // --- 1) Verificar que exista tamaño suficiente para recorte centrado ---
  if (inmsg->width < static_cast<uint32_t>(SUB_W) ||
      inmsg->height < static_cast<uint32_t>(SUB_H))
  {
    ROS_WARN_THROTTLE(5.0,
      "Incoming image too small (%ux%u), need at least %dx%d",
      inmsg->width, inmsg->height, SUB_W, SUB_H);
    return;
  }

  // --- 2) Reservar buffers ---
  // buffer_raw: planar RGB para la función generada (R plane, G plane, B plane)
  const int totalSize = SUB_H * SUB_W * CHANNELS; // canales que espera la RED (normalmente 3)
  uint8_t* buffer_raw = new uint8_t[totalSize];

  // img_msg_data: intercalado RGB para visualizar correctamente en /network_view
  std::vector<uint8_t> img_msg_data;
  img_msg_data.resize(SUB_H * SUB_W * 3);

  // --- 3) Offsets para recorte centrado ---
  const int32_t h_off = (static_cast<int>(inmsg->height) - SUB_H) / 2;
  const int32_t w_off = (static_cast<int>(inmsg->width)  - SUB_W) / 2;

  // --- 4) Copia con mapeo de canales -> SIEMPRE construimos RGB para la RED ---
  // buffer_raw: planar (k*H*W + j*H + i) con k=0(R),1(G),2(B)
  // img_msg_data: intercalado RGB8 para visualización
  int32_t out_idx = 0;

  for (int32_t i = 0; i < SUB_H; ++i)
  {
    const uint8_t* row_ptr = &inmsg->data[(h_off + i) * inmsg->step];
    for (int32_t j = 0; j < SUB_W; ++j)
    {
      const uint8_t* px_ptr = row_ptr + (w_off + j) * SRC_CH;

      uint8_t R=0, G=0, B=0;
      if (SRC_CH == 3) {
        if (is_bgr) { B = px_ptr[0]; G = px_ptr[1]; R = px_ptr[2]; }  // BGR -> RGB
        else         { R = px_ptr[0]; G = px_ptr[1]; B = px_ptr[2]; }  // RGB -> RGB
      } else { // mono8
        R = G = B = px_ptr[0];
      }

      // --- planar RGB para la red ---
      const int32_t baseR = (0 * SUB_H * SUB_W) + (j * SUB_H) + i;
      const int32_t baseG = (1 * SUB_H * SUB_W) + (j * SUB_H) + i;
      const int32_t baseB = (2 * SUB_H * SUB_W) + (j * SUB_H) + i;
      buffer_raw[baseR] = R;
      buffer_raw[baseG] = G;
      buffer_raw[baseB] = B;
	//const int32_t pix = (i * SUB_W + j) * 3;
	//buffer_raw[pix + 0] = R;
	//buffer_raw[pix + 1] = G;
	//buffer_raw[pix + 2] = B;

      // --- intercalado RGB para visualización ---
      img_msg_data[out_idx++] = R;
      img_msg_data[out_idx++] = G;
      img_msg_data[out_idx++] = B;
    }
  }

  // --- [DEBUG] Estadísticas del buffer (debe tener rango amplio, no constante) ---
  {
    uint8_t minv = 255, maxv = 0;
    uint64_t sum = 0;
    for (int idx = 0; idx < totalSize; ++idx) {
      const uint8_t v = buffer_raw[idx];
      if (v < minv) minv = v;
      if (v > maxv) maxv = v;
      sum += v;
    }
    const double meanv = static_cast<double>(sum) / static_cast<double>(totalSize);
    ROS_INFO_THROTTLE(2.0, "buffer stats -> min=%u max=%u mean=%.1f", minv, maxv, meanv);
  }

  // --- 5) Llamar a la función generada por GPU Coder ---
  // IMPORTANTE: si tu función generada tiene otra firma/nombre, ajusta aquí.
  float class_idx_f = myClassifierGPU(buffer_raw);
  uint32_t class_idx = static_cast<uint32_t>(class_idx_f);

  // --- 6) Publicar índice de clase ---
  std_msgs::UInt32 cls_msg;
  cls_msg.data = class_idx;
  data_pub_.publish(cls_msg);

  // --- 7) Publicar la subimagen en RGB8 coherente con los datos que enviamos ---
  sensor_msgs::Image img_msg;
  img_msg.header    = inmsg->header;   // conserva timing/seq
  img_msg.height    = SUB_H;
  img_msg.width     = SUB_W;
  img_msg.encoding  = "rgb8";          // publicamos RGB real
  img_msg.step      = 3 * SUB_W;       // 3 canales (RGB) * width
  img_msg.data      = std::move(img_msg_data);
  img_pub_.publish(img_msg);

  delete[] buffer_raw;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "classifier_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  // --- Parámetros (ajusta defaults según tu red; ResNet-18 => 224x224x3) ---
  std::string input_topic = "/usb_cam/image_raw";
  int sub_h = 224;
  int sub_w = 224;
  int channels = 3;

  pnh.param<std::string>("input_topic", input_topic, input_topic);
  pnh.param("sub_h", sub_h, sub_h);
  pnh.param("sub_w", sub_w, sub_w);
  pnh.param("channels", channels, channels);

  ROS_INFO("Params: input_topic=%s sub_h=%d sub_w=%d channels=%d",
           input_topic.c_str(), sub_h, sub_w, channels);

  // 1) Instanciar publicadores
  ClassifierNode node("network_out", "network_view", 1, &nh, sub_h, sub_w, channels);

  // 2) Suscribirse al tópico de la cámara
  ros::Subscriber img_sub =
      nh.subscribe(input_topic, 1, &ClassifierNode::msgCallback, &node);

  ROS_INFO("Node Started Successfully");
  ros::spin();
  ROS_INFO("Node Shutting Down");
  return 0;
}
