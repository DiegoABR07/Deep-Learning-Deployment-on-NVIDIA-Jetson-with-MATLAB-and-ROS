#include "detector.hpp"
#include <algorithm>
#include <cstring>

// ---- ctor/dtor -----------------------------------------------------------

DetectorNode::DetectorNode(ros::NodeHandle& nh)
: nh_(nh), it_(nh_) {
  // Parámetros
  nh_.param<std::string>("input_topic", input_topic_, "/usb_cam/image_raw");
  nh_.param("sub_h", sub_h_, 416);
  nh_.param("sub_w", sub_w_, 416);
  nh_.param("channels", channels_, 3);
  nh_.param("threshold", threshold_, 0.5);

  if (channels_ != 3) {
    ROS_WARN("`channels` != 3; se forzará a 3 (rgb8).");
    channels_ = 3;
  }

  // Pubs/Subs
  pub_view_ = it_.advertise("/network_view", 1);
  pub_dets_ = nh_.advertise<std_msgs::Float32MultiArray>("/network_detections", 1);
  sub_ = nh_.subscribe(input_topic_, 1, &DetectorNode::imageCb_, this);

  // Preconfig vista
  view_msg_.encoding = sensor_msgs::image_encodings::RGB8;
  view_msg_.height = sub_h_;
  view_msg_.width  = sub_w_;
  view_msg_.step   = static_cast<uint32_t>(sub_w_ * channels_);
  view_msg_.data.resize(static_cast<size_t>(sub_h_ * view_msg_.step));

  // Inicialización del runtime generado (tu build exporta '..._init')
  myDetectorGPU_init();  // ver doc de initialize/housekeeping de Coder. :contentReference[oaicite:3]{index=3}

  ROS_INFO_STREAM("detector_node listo. ROI=" << sub_w_ << "x" << sub_h_
                  << " thr=" << threshold_ << " topic=" << input_topic_);
}

DetectorNode::~DetectorNode() {
  // Tu header no expone '..._terminate()'. Si en otro build aparece,
  // podrías llamarlo aquí para liberar housekeeping.
}

// ---- helpers -------------------------------------------------------------

void DetectorNode::publishView_(const uint8_t* crop_rgb8) {
  const int dst_step = static_cast<int>(view_msg_.step);
  const int src_step = sub_w_ * channels_;
  for (int i = 0; i < sub_h_; ++i) {
    const uint8_t* src_row = crop_rgb8 + i * src_step;
    uint8_t* dst_row = view_msg_.data.data() + i * dst_step;
    std::memcpy(dst_row, src_row, static_cast<size_t>(dst_step));
  }
  view_msg_.header.stamp = ros::Time::now();
  pub_view_.publish(view_msg_);
}

void DetectorNode::publishDetections_(const emxArray_real_T* bboxes,
                                      const emxArray_real_T* scores,
                                      const emxArray_real_T* labels) {
  std_msgs::Float32MultiArray out;
  out.layout.dim.clear();

  if (!bboxes || !scores || !labels) { pub_dets_.publish(out); return; }

  // emxArray (column-major). bboxes: [N x 4], scores/labels: [N x 1]
  const int Nb = (bboxes->numDimensions >= 1) ? bboxes->size[0] : 0;
  const int Cb = (bboxes->numDimensions >= 2) ? bboxes->size[1] : 0;
  const int Ns = (scores->numDimensions >= 1) ? scores->size[0] : 0;
  const int Nl = (labels->numDimensions >= 1) ? labels->size[0] : 0;
  const int N  = std::max(0, std::min({Nb, Ns, Nl}));

  if (Cb < 4 || N <= 0) {
    pub_dets_.publish(out);
    return;
  }

  out.data.reserve(static_cast<size_t>(N) * 6);
  const double* B = bboxes->data;
  const double* S = scores->data;
  const double* L = labels->data;
  const int stride = Nb;

  for (int i = 0; i < N; ++i) {
    const float x  = static_cast<float>(B[i + 0*stride]);
    const float y  = static_cast<float>(B[i + 1*stride]);
    const float w  = static_cast<float>(B[i + 2*stride]);
    const float h  = static_cast<float>(B[i + 3*stride]);
    const float sc = static_cast<float>(S[i]);
    const float lb = static_cast<float>(L[i]); // 1-based en double

    out.data.push_back(x);
    out.data.push_back(y);
    out.data.push_back(w);
    out.data.push_back(h);
    out.data.push_back(sc);
    out.data.push_back(lb);
  }
  pub_dets_.publish(out);
}

// ---- inferencia ----------------------------------------------------------

void DetectorNode::runInference_(const uint8_t* crop_rgb8, int stride_bytes) {
  // Convertir de row-major (ROS) a column-major (MATLAB) HxWxC (C=3)
  const int H = sub_h_, W = sub_w_, C = channels_;
  std::vector<uint8_t> im_colmajor(static_cast<size_t>(H*W*C));

  for (int k = 0; k < C; ++k) {
    for (int j = 0; j < W; ++j) {
      for (int i = 0; i < H; ++i) {
        const uint8_t* src = crop_rgb8 + i*stride_bytes + j*C + k; // row-major
        const int idx = i + j*H + k*H*W;                           // col-major
        im_colmajor[static_cast<size_t>(idx)] = *src;
      }
    }
  }

  // Crear contenedores de salida (emxArray<double>)
  emxArray_real_T *bboxes = nullptr, *scores = nullptr, *labels = nullptr;
  emxInitArray_real_T(&bboxes, 2);  // Nx4
  emxInitArray_real_T(&scores, 1);  // Nx1
  emxInitArray_real_T(&labels, 1);  // Nx1

  // Llamada al entry-point REAL (sin detectorFile en tu build)
  myDetectorGPU(im_colmajor.data(), threshold_, bboxes, scores, labels);

  // Publicar resultados
  publishDetections_(bboxes, scores, labels);

  // Liberar emxArray
  emxDestroyArray_real_T(bboxes);
  emxDestroyArray_real_T(scores);
  emxDestroyArray_real_T(labels);
}

// ---- callback ------------------------------------------------------------

void DetectorNode::imageCb_(const sensor_msgs::ImageConstPtr& msg) {
  const std::string& enc = msg->encoding;
  if (enc != sensor_msgs::image_encodings::RGB8 &&
      enc != sensor_msgs::image_encodings::BGR8 &&
      enc != sensor_msgs::image_encodings::MONO8) {
    ROS_WARN_THROTTLE(5.0, "Encoding '%s' no soportado. Usa rgb8/bgr8/mono8.", enc.c_str());
    return;
  }

  const int H0 = static_cast<int>(msg->height);
  const int W0 = static_cast<int>(msg->width);
  const int step = static_cast<int>(msg->step);
  if (H0 < sub_h_ || W0 < sub_w_) {
    ROS_WARN_THROTTLE(5.0, "Frame %dx%d menor que ROI %dx%d. Omitiendo.",
                      W0, H0, sub_w_, sub_h_);
    return;
  }

  // ROI centrado
  const int x0 = (W0 - sub_w_) / 2;
  const int y0 = (H0 - sub_h_) / 2;

  // Reusar buffer de view_msg_ como staging (rgb8)
  uint8_t* crop = view_msg_.data.data();
  const int dst_step = static_cast<int>(view_msg_.step);

  if (enc == sensor_msgs::image_encodings::RGB8) {
    for (int i = 0; i < sub_h_; ++i) {
      const uint8_t* src_row = msg->data.data() + (y0 + i) * step + x0 * 3;
      uint8_t* dst_row = crop + i * dst_step;
      std::memcpy(dst_row, src_row, static_cast<size_t>(dst_step));
    }
  } else if (enc == sensor_msgs::image_encodings::BGR8) {
    for (int i = 0; i < sub_h_; ++i) {
      const uint8_t* src_row = msg->data.data() + (y0 + i) * step + x0 * 3;
      uint8_t* dst_row = crop + i * dst_step;
      for (int j = 0; j < sub_w_; ++j) {
        const uint8_t b = src_row[3*j + 0];
        const uint8_t g = src_row[3*j + 1];
        const uint8_t r = src_row[3*j + 2];
        dst_row[3*j + 0] = r;
        dst_row[3*j + 1] = g;
        dst_row[3*j + 2] = b;
      }
    }
  } else { // mono8 -> rgb
    for (int i = 0; i < sub_h_; ++i) {
      const uint8_t* src_row = msg->data.data() + (y0 + i) * step + x0;
      uint8_t* dst_row = crop + i * dst_step;
      for (int j = 0; j < sub_w_; ++j) {
        const uint8_t v = src_row[j];
        dst_row[3*j + 0] = v;
        dst_row[3*j + 1] = v;
        dst_row[3*j + 2] = v;
      }
    }
  }

  publishView_(crop);
  runInference_(crop, dst_step);
}

// ---- main ----------------------------------------------------------------

int main(int argc, char** argv) {
  ros::init(argc, argv, "detector_node");
  ros::NodeHandle nh("~");
  DetectorNode node(nh);
  ros::spin();
  return 0;
}
