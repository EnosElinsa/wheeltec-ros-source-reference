// Copyright (c) 2024，D-Robotics.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <atomic>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <unordered_map>
#include <vector>

#ifdef CV_BRIDGE_CPP
#include <cv_bridge/cv_bridge.hpp>
#else
#include <cv_bridge/cv_bridge.h>
#endif
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

#ifdef SHARED_MEM_ENABLED
#include "hbm_img_msgs/msg/hbm_msg1080_p.hpp"
#endif

#include "ai_msgs/msg/capture_targets.hpp"
#include "ai_msgs/msg/perception_targets.hpp"
#include "dnn_node/dnn_node.h"
#include "dnn_node/util/output_parser/detection/fasterrcnn_output_parser.h"
#include "include/post_process/palm_det_output_parser.h"

#include "include/post_process/palm_det_parser.h"

#ifndef MONO2D_PALM_DET_NODE_H_
#define MONO2D_PALM_DET_NODE_H_

using rclcpp::NodeOptions;

using hobot::dnn_node::DNNInput;
using hobot::dnn_node::DnnNode;
using hobot::dnn_node::DnnNodeOutput;
using hobot::dnn_node::ModelTaskType;
using hobot::dnn_node::NV12PyramidInput;

using hobot::dnn_node::parser_fasterrcnn::FasterRcnnKpsParserPara;

using parser_palm::PalmDetResult;
using parser_palm::PalmRect;

// 使用output manage解决异步多线程情况下模型输出乱序的问题
class NodeOutputManage
{
public:
  void Feed(uint64_t ts_ms);
  std::vector<std::shared_ptr<DnnNodeOutput>> Feed(const std::shared_ptr<DnnNodeOutput>& node_output);
  void Erase(uint64_t ts_ms);

private:
  std::set<uint64_t> cache_frame_;
  std::map<uint64_t, std::shared_ptr<DnnNodeOutput>> cache_node_output_;
  // 如果图像采集频率是30fps，能够缓存10帧，对应要求端到端的推理耗时最长不能超过1000/30*10=750ms
  const uint8_t cache_size_limit_ = 10;
  std::mutex mtx_;
  std::condition_variable cv_;
  const uint64_t smart_output_timeout_ms_ = 1000;
};

class Mono2dPalmDetNode : public DnnNode
{
public:
  Mono2dPalmDetNode(const NodeOptions& options = NodeOptions());
  ~Mono2dPalmDetNode() override;
  int FeedFromLocal();

protected:
  int SetNodePara() override;
  int PostProcess(const std::shared_ptr<DnnNodeOutput>& outputs) override;

private:

  float min_score_ = 0.6;

#ifdef PLATFORM_X5
  std::string model_file_name_ = "config/palm_det_192_192.bin";
#else
  std::string model_file_name_ = "config/palm_det_192_192.hbm";
#endif
  std::string model_name_ = ""; // if empty, use the first model in the file
  ModelTaskType model_task_type_ = ModelTaskType::ModelInferType;

  int model_input_width_ = -1;
  int model_input_height_ = -1;

  std::shared_ptr<FasterRcnnKpsParserPara> parser_para_ = nullptr;

  int image_gap_ = 1;

  int is_sync_mode_ = 0;

  // 使用shared mem通信方式订阅图片
  int is_shared_mem_sub_ = 1;

  std::string ai_msg_pub_topic_name_ = "hobot_palm_detection";
  rclcpp::Publisher<ai_msgs::msg::PerceptionTargets>::SharedPtr msg_publisher_ = nullptr;

  int Predict(std::vector<std::shared_ptr<DNNInput>>& inputs, const std::shared_ptr<std::vector<hbDNNRoi>> rois,
              std::shared_ptr<DnnNodeOutput> dnn_output);

#ifdef SHARED_MEM_ENABLED
  rclcpp::Subscription<hbm_img_msgs::msg::HbmMsg1080P>::ConstSharedPtr sharedmem_img_subscription_ = nullptr;
  std::string sharedmem_img_topic_name_ = "/hbmem_img";
  void SharedMemImgProcess(const hbm_img_msgs::msg::HbmMsg1080P::ConstSharedPtr msg);
#endif

  rclcpp::Subscription<sensor_msgs::msg::Image>::ConstSharedPtr ros_img_subscription_ = nullptr;
  // 目前只支持订阅原图，可以使用压缩图"/image_raw/compressed" topic
  // 和sensor_msgs::msg::CompressedImage格式扩展订阅压缩图
  std::string ros_img_topic_name_ = "/image_raw";
  void RosImgProcess(const sensor_msgs::msg::Image::ConstSharedPtr msg);

  std::shared_ptr<NodeOutputManage> node_output_manage_ptr_ = std::make_shared<NodeOutputManage>();
};

#endif  // MONO2D_PALM_DET_NODE_H_
