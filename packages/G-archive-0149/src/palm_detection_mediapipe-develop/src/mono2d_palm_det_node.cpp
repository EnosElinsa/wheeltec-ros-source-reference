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

#include "include/mono2d_palm_det_node.h"

#include <fstream>
#include <memory>
#include <string>
#include <unistd.h>
#include <unordered_map>
#include <utility>
#include <vector>
#include <algorithm>

#include "dnn_node/dnn_node.h"
#include "dnn_node/util/image_proc.h"
#include "rclcpp/rclcpp.hpp"
#include "hobot_cv/hobotcv_imgproc.h"

#include "builtin_interfaces/msg/detail/time__struct.h"
#include "rcpputils/env.hpp"
#include "rcutils/env.h"
#include "opencv2/imgproc/types_c.h"

builtin_interfaces::msg::Time ConvertToRosTime(const struct timespec& time_spec)
{
  builtin_interfaces::msg::Time stamp;
  stamp.set__sec(time_spec.tv_sec);
  stamp.set__nanosec(time_spec.tv_nsec);
  return stamp;
}

int CalTimeMsDuration(const builtin_interfaces::msg::Time& start, const builtin_interfaces::msg::Time& end)
{
  return (end.sec - start.sec) * 1000 + end.nanosec / 1000 / 1000 - start.nanosec / 1000 / 1000;
}

void NodeOutputManage::Feed(uint64_t ts_ms)
{
  RCLCPP_DEBUG(rclcpp::get_logger("mono2d_palm_det"), "feed frame ts: %lu", ts_ms);

  std::unique_lock<std::mutex> lk(mtx_);
  cache_frame_.insert(ts_ms);
  if (cache_frame_.size() > cache_size_limit_)
  {
    cache_frame_.erase(cache_frame_.begin());
  }
  RCLCPP_DEBUG(rclcpp::get_logger("mono2d_palm_det"), "cache_frame_.size(): %ld", cache_frame_.size());
}

std::vector<std::shared_ptr<DnnNodeOutput>> NodeOutputManage::Feed(const std::shared_ptr<DnnNodeOutput>& in_node_output)
{
  std::vector<std::shared_ptr<DnnNodeOutput>> node_outputs{};
  auto palm_node_output = std::dynamic_pointer_cast<parser_palm::PalmNodeOutput>(in_node_output);
  if (!palm_node_output || !palm_node_output->image_msg_header)
  {
    return node_outputs;
  }

  uint64_t ts_ms = palm_node_output->image_msg_header->stamp.sec * 1000 +
                   palm_node_output->image_msg_header->stamp.nanosec / 1000 / 1000;
  RCLCPP_DEBUG(rclcpp::get_logger("mono2d_palm_det"), "feed ts: %lu", ts_ms);

  uint8_t loop_num = cache_size_limit_;
  {
    std::unique_lock<std::mutex> lk(mtx_);
    cache_node_output_[ts_ms] = in_node_output;
    if (cache_node_output_.size() > cache_size_limit_)
    {
      cache_node_output_.erase(cache_node_output_.begin());
    }
    if (cache_frame_.empty())
    {
      return node_outputs;
    }

    loop_num = cache_node_output_.size();
  }

  RCLCPP_DEBUG(rclcpp::get_logger("mono2d_palm_det"), "node_outputs.size(): %ld", node_outputs.size());

  // 按照时间戳顺序输出推理结果
  for (uint8_t idx = 0; idx < loop_num; idx++)
  {
    std::shared_ptr<DnnNodeOutput> node_output = nullptr;
    {
      std::unique_lock<std::mutex> lk(mtx_);
      if (cache_frame_.empty() || cache_node_output_.empty())
      {
        break;
      }

      auto first_frame = cache_frame_.begin();
      auto first_output = cache_node_output_.begin();
      if (*first_frame == first_output->first)
      {
        RCLCPP_DEBUG(rclcpp::get_logger("mono2d_palm_det"), "push ts: %lu", *first_frame);

        node_output = in_node_output;
        cache_frame_.erase(first_frame);
        cache_node_output_.erase(first_output);
      }
      else
      {
        if (first_output->first > *first_frame)
        {
          // 首个推理结果时间戳大于首帧图像数据的时间戳
          uint64_t time_ms_diff = first_output->first - *first_frame;
          if (time_ms_diff > smart_output_timeout_ms_)
          {
            // 首个推理结果和首帧图像数据的时间戳相差大于阈值，说明存在推理丢帧，删除首帧图像数据
            cache_frame_.erase(first_frame);
          }
        }
        else if (*first_frame > first_output->first)
        {
          // 首帧图像数据的时间戳大于首个推理结果时间戳
          uint64_t time_ms_diff = *first_frame - first_output->first;
          if (time_ms_diff > smart_output_timeout_ms_)
          {
            // 首帧图像数据和首个推理结果的时间戳相差大于阈值，删除首帧图像数据，理论上不应该出现这种case
            cache_node_output_.erase(first_output);
          }
        }
        else
        {
          // 时间戳相等的情况，不会走到这里
          break;
        }
      }
    }

    if (node_output)
    {
      node_outputs.emplace_back(node_output);
    }
  }

  return node_outputs;
}

void NodeOutputManage::Erase(uint64_t ts_ms)
{
  std::unique_lock<std::mutex> lk(mtx_);
  if (cache_frame_.find(ts_ms) != cache_frame_.end())
  {
    cache_frame_.erase(ts_ms);
  }
  if (cache_node_output_.find(ts_ms) != cache_node_output_.end())
  {
    cache_node_output_.erase(ts_ms);
  }
}

Mono2dPalmDetNode::Mono2dPalmDetNode(const NodeOptions& options) : DnnNode("mono2d_palm_det", options)
{
  // init Mode2dPalmDetNode
  this->declare_parameter<int>("is_sync_mode", is_sync_mode_);
  this->declare_parameter<std::string>("model_file_name", model_file_name_);
  this->declare_parameter<int>("is_shared_mem_sub", is_shared_mem_sub_);
  this->declare_parameter<std::string>("ai_msg_pub_topic_name", ai_msg_pub_topic_name_);
  this->declare_parameter<std::string>("ros_img_topic_name", ros_img_topic_name_);
  this->declare_parameter<std::string>("sharedmem_img_topic_name", sharedmem_img_topic_name_);
  this->declare_parameter<int>("image_gap", image_gap_);
  this->declare_parameter<float>("min_score", min_score_);

  this->get_parameter<int>("is_sync_mode", is_sync_mode_);
  this->get_parameter<std::string>("model_file_name", model_file_name_);
  this->get_parameter<int>("is_shared_mem_sub", is_shared_mem_sub_);
  this->get_parameter<std::string>("ai_msg_pub_topic_name", ai_msg_pub_topic_name_);
  this->get_parameter<std::string>("ros_img_topic_name", ros_img_topic_name_);
  this->get_parameter<std::string>("sharedmem_img_topic_name", sharedmem_img_topic_name_);
  this->get_parameter<int>("image_gap", image_gap_);
  this->get_parameter<float>("min_score", min_score_);
  {
    std::stringstream ss;
    ss << "Parameter:"
       << "\n is_sync_mode_: " << is_sync_mode_ 
       << "\n model_file_name_: " << model_file_name_
       << "\n is_shared_mem_sub: " << is_shared_mem_sub_ 
       << "\n ai_msg_pub_topic_name: " << ai_msg_pub_topic_name_
       << "\n ros_img_topic_name: " << ros_img_topic_name_ 
       << "\n image_gap: " << image_gap_
       << "\n min_score: " << min_score_;
    RCLCPP_WARN(rclcpp::get_logger("mono2d_palm_det"), "%s", ss.str().c_str());
  }

  if (Init() != 0)
  {
    RCLCPP_ERROR(rclcpp::get_logger("mono2d_palm_det"), "Init failed!");
    rclcpp::shutdown();
    return;
  }

  // Init()之后模型已经加载成功，查询kps解析参数
  auto model_manage = GetModel();
  if (!model_manage)
  {
    RCLCPP_ERROR(rclcpp::get_logger("mono2d_palm_det"), "Invalid model");
    rclcpp::shutdown();
    return;
  }

  // get model name if not set
  if (model_name_.empty())
  {
    if (!GetModel())
    {
      RCLCPP_ERROR(rclcpp::get_logger("mono2d_palm_det"), "Get model fail.");
    }
    else
    {
      model_name_ = GetModel()->GetName();
      RCLCPP_WARN(rclcpp::get_logger("mono2d_palm_det"), "Get model name: %s from load model.", model_name_.c_str());
    }
  }

  // create publisher to publish ai msgs
  msg_publisher_ = this->create_publisher<ai_msgs::msg::PerceptionTargets>(ai_msg_pub_topic_name_, 10);
  // get anchors for parse palm and key points
  for (const auto& anchor : anchors_)
  {
    anchors.emplace_back(anchor[0], anchor[1]);
  }

  if (GetModelInputSize(0, model_input_width_, model_input_height_) < 0)
  {
    RCLCPP_ERROR(rclcpp::get_logger("mono2d_palm_det"), "Get model input size fail!");
    rclcpp::shutdown();
  }
  else
  {
    RCLCPP_INFO(rclcpp::get_logger("mono2d_palm_det"), "The model input width is %d and height is %d",
                model_input_width_, model_input_height_);
  }
  if (is_shared_mem_sub_)
  {
    std::string ros_zerocopy_env = rcpputils::get_env_var("RMW_FASTRTPS_USE_QOS_FROM_XML");
    if (ros_zerocopy_env.empty())
    {
      RCLCPP_ERROR_STREAM(this->get_logger(),
                          "Launching with zero-copy, but env of `RMW_FASTRTPS_USE_QOS_FROM_XML` is not set. "
                              << "Transporting data without zero-copy!");
    }
    else
    {
      if ("1" == ros_zerocopy_env)
      {
        RCLCPP_WARN_STREAM(this->get_logger(), "Enabling zero-copy");
      }
      else
      {
        RCLCPP_ERROR_STREAM(this->get_logger(), "env of `RMW_FASTRTPS_USE_QOS_FROM_XML` is ["
                                                    << ros_zerocopy_env << "], which should be set to 1. "
                                                    << "Data transporting without zero-copy!");
      }
    }
#ifdef SHARED_MEM_ENABLED
    RCLCPP_WARN(rclcpp::get_logger("mono2d_palm_det"), "Create hbmem_subscription with topic_name: %s",
                sharedmem_img_topic_name_.c_str());
    sharedmem_img_subscription_ = this->create_subscription<hbm_img_msgs::msg::HbmMsg1080P>(
        sharedmem_img_topic_name_, rclcpp::SensorDataQoS(),
        std::bind(&Mono2dPalmDetNode::SharedMemImgProcess, this, std::placeholders::_1));
#else
    RCLCPP_ERROR(rclcpp::get_logger("mono2d_palm_det"), "Unsupport shared mem");
#endif
  }
  else
  {
    RCLCPP_WARN(rclcpp::get_logger("mono2d_palm_det"), "Create subscription with topic_name: %s",
                ros_img_topic_name_.c_str());
    ros_img_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
        ros_img_topic_name_, 10, std::bind(&Mono2dPalmDetNode::RosImgProcess, this, std::placeholders::_1));
  }
}

Mono2dPalmDetNode::~Mono2dPalmDetNode()
{
}

int Mono2dPalmDetNode::SetNodePara()
{
  RCLCPP_INFO(rclcpp::get_logger("mono2d_palm_det"), "Set node para.");
  if (!dnn_node_para_ptr_)
  {
    return -1;
  }
  dnn_node_para_ptr_->model_file = model_file_name_;
  dnn_node_para_ptr_->model_name = model_name_;
  dnn_node_para_ptr_->model_task_type = model_task_type_;
  dnn_node_para_ptr_->task_num = 2;
  return 0;
}

int Mono2dPalmDetNode::PostProcess(const std::shared_ptr<DnnNodeOutput>& outputs)
{
  // RCLCPP_INFO(rclcpp::get_logger("mono2d_palm_det"), "Pointer: %p, Value: %f", outputs);
  auto palmOutput = std::dynamic_pointer_cast<parser_palm::PalmNodeOutput>(outputs);

  if (!rclcpp::ok())
  {
    return 0;
  }

  if (!msg_publisher_)
  {
    RCLCPP_ERROR(rclcpp::get_logger("mono2d_palm_det"), "Invalid msg_publisher_");
    return -1;
  }
  std::vector<std::shared_ptr<DnnNodeOutput>> node_outputs{};
  auto palm_node_output = std::dynamic_pointer_cast<parser_palm::PalmNodeOutput>(outputs);
  if (node_output_manage_ptr_)
  {
    // 启用了输出排序功能
    node_outputs = node_output_manage_ptr_->Feed(outputs);
    if (node_outputs.empty())
    {
      if (!palm_node_output && !palm_node_output->image_msg_header)
      {
        // 由于没有消息头（时间戳）导致的排序后的输出为空
        // 直接使用当前帧输出
        node_outputs.push_back(outputs);
      }
    }
  }
  else
  {
    // 未启用输出排序功能，直接使用当前帧输出
    node_outputs.push_back(outputs);
  }

  for (const auto& node_output : node_outputs)
  {
    if (!node_output)
    {
      continue;
    }

    auto palmOutput = std::dynamic_pointer_cast<parser_palm::PalmNodeOutput>(node_output);
    {
      std::stringstream ss;
      ss << "Outputs from";
      ss << ", frame_id: " << palmOutput->image_msg_header->frame_id
         << ", stamp: " << palmOutput->image_msg_header->stamp.sec << "_" << palmOutput->image_msg_header->stamp.nanosec
         << ", infer time ms: " << node_output->rt_stat->infer_time_ms;
      RCLCPP_INFO(rclcpp::get_logger("mono2d_palm_det"), "%s", ss.str().c_str());
    }
    int scale = palmOutput->scale;
    std::shared_ptr<PalmDetResult> palm_det_res = nullptr;

    // 使用hobot dnn内置的Parse解析方法，解析算法输出的DNNTensor类型数据
    int ret = -1;
    ret = PalmDetParse(outputs->output_tensors, palm_det_res, scale, min_score_, 0.75);  // parse det results
    if (ret < 0)
    {
      RCLCPP_ERROR(rclcpp::get_logger("mono2d_palm_det"), "Parse node_output fail!");
      return -1;
    }
    // get det results
    auto boxes = palm_det_res->boxes;
    auto lmks = palm_det_res->lmks;
    auto indices = palm_det_res->indices;  // index of valid results

    struct timespec time_start = { 0, 0 };
    clock_gettime(CLOCK_REALTIME, &time_start);

    ai_msgs::msg::PerceptionTargets::UniquePtr pub_data(new ai_msgs::msg::PerceptionTargets());
    if (palm_node_output->image_msg_header)
    {
      pub_data->header.set__stamp(palm_node_output->image_msg_header->stamp);
      pub_data->header.set__frame_id(palm_node_output->image_msg_header->frame_id);
    }
    if (outputs->rt_stat)
    {
      pub_data->set__fps(round(outputs->rt_stat->output_fps));
    }

    Landmarks lmk_result;

    RCLCPP_INFO(rclcpp::get_logger("mono2d_palm_det"), "Outputs boxes size: %ld", boxes.size());
    for (const auto& idx : indices)
    {
      // if you want show palm detection results in web, you can use this code to filter low score detction results
      if (boxes[idx].conf < min_score_)
      {
        continue;
      }
      ai_msgs::msg::Target target;
      target.set__type("palm");

      if (static_cast<size_t>(idx) >= boxes.size())
      {
        RCLCPP_ERROR(rclcpp::get_logger("mono2d_palm_det"), "Outputs index: %d exceeds results size %ld", idx,
                     palm_det_res->indices.size());
        return -1;
      }
      auto rect = boxes[idx];
      if (rect.left < 0)
        rect.left = 0;
      if (rect.top < 0)
        rect.top = 0;
      {
        std::stringstream ss;
        ss << "rect: " << rect.left << " " << rect.top << " " << rect.right << " " << rect.bottom << ", " << rect.conf;
        RCLCPP_INFO(rclcpp::get_logger("mono2d_palm_det"), "%s", ss.str().c_str());
      }
      // for show rect in web
      ai_msgs::msg::Roi roi;
      roi.set__type("palm_ori");
      roi.rect.set__x_offset(rect.left);
      roi.rect.set__y_offset(rect.top);
      roi.rect.set__width((rect.right - rect.left));
      roi.rect.set__height((rect.bottom - rect.top));
      roi.set__confidence(rect.conf);
      target.rois.emplace_back(std::move(roi));

      // extend palm detction to hand, but this is deprecated
      auto zero_point = lmks[idx][0];
      auto x1 = zero_point.x - std::max(std::abs(rect.left - zero_point.x) * 3.5, 800.0 / 10.0);
      auto x2 = zero_point.x + std::max(std::abs(rect.right - zero_point.x) * 3.5, 800.0 / 10.0);
      auto y1 = zero_point.y - std::max(std::abs(rect.top - zero_point.y) * 3.5, 600 / 10.0);
      auto y2 = zero_point.y + std::max(std::abs(rect.bottom - zero_point.y) * 3.5, 600 / 10.0);

      if (x1 < 0)
        x1 = 0;
      if (y1 < 0)
        y1 = 0;
      if (y2 > palm_node_output->image_height_ori)
        y2 = palm_node_output->image_height_ori;
      if (x2 > palm_node_output->image_width_ori)
        x2 = palm_node_output->image_width_ori;

      // for show expand rect in web
      roi.set__type("palm_expand");
      roi.rect.set__x_offset(x1);
      roi.rect.set__y_offset(y1);
      roi.rect.set__width((x2 - x1));
      roi.rect.set__height((y2 - y1));
      target.rois.emplace_back(std::move(roi));

      ai_msgs::msg::Point target_point;
      target_point.set__type("palm_kps");
      std::vector<int> orders = { 0, 1, 2, 3, 4, 5, 6 };  // you can change order for your model
      for (const auto& ord : orders)
      {
        auto lmk = lmks[idx][ord];
        std::stringstream ss;
        ss << lmk.x << "," << lmk.y;
        geometry_msgs::msg::Point32 pt;
        pt.set__x(lmk.x);
        pt.set__y(lmk.y);
        target_point.point.emplace_back(pt);
        ss << "\n";
        RCLCPP_DEBUG(rclcpp::get_logger("mono2d_palm_det"), "Palm kps: %s", ss.str().c_str());
      }
      target.points.emplace_back(target_point);
      pub_data->targets.emplace_back(std::move(target));
    }

    struct timespec time_now = { 0, 0 };
    clock_gettime(CLOCK_REALTIME, &time_now);

    // preprocess
    ai_msgs::msg::Perf perf_preprocess;
    perf_preprocess.set__type(model_name_ + "_preprocess");
    perf_preprocess.set__stamp_start(ConvertToRosTime(palm_node_output->preprocess_timespec_start));
    perf_preprocess.set__stamp_end(ConvertToRosTime(palm_node_output->preprocess_timespec_end));
    perf_preprocess.set__time_ms_duration(CalTimeMsDuration(perf_preprocess.stamp_start, perf_preprocess.stamp_end));
    pub_data->perfs.emplace_back(perf_preprocess);

    // predict
    if (outputs->rt_stat)
    {
      ai_msgs::msg::Perf perf;
      perf.set__type(model_name_ + "_predict_infer");
      perf.set__stamp_start(ConvertToRosTime(outputs->rt_stat->infer_timespec_start));
      perf.set__stamp_end(ConvertToRosTime(outputs->rt_stat->infer_timespec_end));
      perf.set__time_ms_duration(outputs->rt_stat->infer_time_ms);
      pub_data->perfs.push_back(perf);

      perf.set__type(model_name_ + "_predict_parse");
      perf.set__stamp_start(ConvertToRosTime(outputs->rt_stat->parse_timespec_start));
      perf.set__stamp_end(ConvertToRosTime(outputs->rt_stat->parse_timespec_end));
      perf.set__time_ms_duration(outputs->rt_stat->parse_time_ms);
      pub_data->perfs.push_back(perf);
    }

    // postprocess
    ai_msgs::msg::Perf perf_postprocess;
    perf_postprocess.set__type(model_name_ + "_postprocess");
    perf_postprocess.set__stamp_start(ConvertToRosTime(time_start));
    clock_gettime(CLOCK_REALTIME, &time_now);
    perf_postprocess.set__stamp_end(ConvertToRosTime(time_now));
    perf_postprocess.set__time_ms_duration(CalTimeMsDuration(perf_postprocess.stamp_start, perf_postprocess.stamp_end));
    pub_data->perfs.emplace_back(perf_postprocess);

    // 从发布图像到发布AI结果的延迟
    ai_msgs::msg::Perf perf_pipeline;
    perf_pipeline.set__type(model_name_ + "_pipeline");
    perf_pipeline.set__stamp_start(pub_data->header.stamp);
    perf_pipeline.set__stamp_end(perf_postprocess.stamp_end);
    perf_pipeline.set__time_ms_duration(CalTimeMsDuration(perf_pipeline.stamp_start, perf_pipeline.stamp_end));
    pub_data->perfs.push_back(perf_pipeline);

    {
      std::stringstream ss;
      ss << "Publish frame_id: " << palm_node_output->image_msg_header->frame_id
         << ", time_stamp: " << std::to_string(pub_data->header.stamp.sec) << "_"
         << std::to_string(pub_data->header.stamp.nanosec) << "\n";
      RCLCPP_INFO(rclcpp::get_logger("mono2d_palm_det"), "%s", ss.str().c_str());
    }

    std::stringstream ss;
    ss << "Publish frame_id: " << palm_node_output->image_msg_header->frame_id
       << ", time_stamp: " << std::to_string(pub_data->header.stamp.sec) << "_"
       << std::to_string(pub_data->header.stamp.nanosec) << "\n";
    ss << "targets.size: " << pub_data->targets.size() << "\n";

    if (!pub_data->targets.empty())
    {
      for (const auto& target : pub_data->targets)
      {
        ss << "rois.size: " << target.rois.size();
        for (const auto& roi : target.rois)
        {
          ss << ", " << roi.type.c_str();
        }
        ss << ", points.size: " << target.points.size();
        // for (const auto& point : target.points)
        // {
        //   ss << ", " << point.type.c_str();
        // }
        ss << "\n";
      }
    }

    RCLCPP_INFO(rclcpp::get_logger("mono2d_palm_det"), "%s", ss.str().c_str());

    if (node_output->rt_stat->fps_updated)
    {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                           "input fps: %.2f, out fps: %.2f, infer time ms: %d, "
                           "post process time ms: %d",
                           node_output->rt_stat->input_fps, node_output->rt_stat->output_fps,
                           node_output->rt_stat->infer_time_ms, static_cast<int>(perf_postprocess.time_ms_duration));
    }
    msg_publisher_->publish(std::move(pub_data));
  }
  return 0;
}
int Mono2dPalmDetNode::FeedFromLocal()
{
  std::string image_file_ = "1.png";
  if (access(image_file_.c_str(), R_OK) == -1)
  {
    RCLCPP_ERROR(rclcpp::get_logger("mono2d_palm_det"), "Image: %s not exist!", image_file_.c_str());
    return -1;
  }

  // 1. 将图片处理成模型输入数据类型DNNInput
  // 使用图片生成pym，NV12PyramidInput为DNNInput的子类
  std::shared_ptr<hobot::dnn_node::NV12PyramidInput> pyramid = nullptr;
  // bgr img，支持将图片resize到模型输入size
  int img_h = 800;
  int img_w = 600;
  int resized_h;
  int resized_w;
  pyramid = hobot::dnn_node::ImageProc::GetNV12PyramidFromBGR(image_file_, img_h, img_w, resized_h, resized_w,
                                                              model_input_height_, model_input_width_);
  if (!pyramid)
  {
    RCLCPP_ERROR(this->get_logger(), "Get Nv12 pym fail with image: %s", image_file_.c_str());
    return -1;
  }
  // 2. 输入NV12 Input
  // inputs将会作为模型的输入通过InferTask接口传入
  auto inputs = std::vector<std::shared_ptr<DNNInput>>{ pyramid };
  auto dnn_output = std::make_shared<parser_palm::PalmNodeOutput>();
  // struct timespec time_now = { 0, 0 };
  // clock_gettime(CLOCK_REALTIME, &time_now);
  dnn_output->msg_header = std::make_shared<std_msgs::msg::Header>();
  dnn_output->msg_header->set__frame_id("feedback");
  // dnn_output->perf_preprocess.stamp_start.sec = time_now.tv_sec;
  // dnn_output->perf_preprocess.stamp_start.nanosec = time_now.tv_nsec;

  // 3. 开始预测
  if (Run(inputs, dnn_output, nullptr) != 0)
  {
    RCLCPP_ERROR(rclcpp::get_logger("hobot_dosod"), "Run predict failed!");
    return -1;
  }

  return 0;
}

void Mono2dPalmDetNode::RosImgProcess(const sensor_msgs::msg::Image::ConstSharedPtr img_msg)
{
  if (!img_msg || !rclcpp::ok())
  {
    return;
  }
  static int gap_cnt = 0;
  if (++gap_cnt < image_gap_)
  {
    return;
  }
  gap_cnt = 0;
  struct timespec time_start = { 0, 0 };
  clock_gettime(CLOCK_REALTIME, &time_start);
  std::stringstream ss;
  ss << "RosImgProcess Recved img encoding: " << img_msg->encoding << ", h: " << img_msg->height
     << ", w: " << img_msg->width << ", step: " << img_msg->step << ", frame_id: " << img_msg->header.frame_id
     << ", stamp: " << img_msg->header.stamp.sec << "_" << img_msg->header.stamp.nanosec
     << ", data size: " << img_msg->data.size();
  RCLCPP_INFO(rclcpp::get_logger("mono2d_palm_det"), "%s", ss.str().c_str());

  auto tp_start = std::chrono::system_clock::now();

  // 1. 将图片处理成模型输入数据类型DNNInput
  // 使用图片生成pym，NV12PyramidInput为DNNInput的子类

  std::shared_ptr<hobot::dnn_node::NV12PyramidInput> pyramid = nullptr;
  if ("rgb8" == img_msg->encoding || "bgr8" == img_msg->encoding)
  {
    auto cv_img = cv_bridge::cvtColorForDisplay(cv_bridge::toCvShare(img_msg), "bgr8");

    {
      auto tp_now = std::chrono::system_clock::now();
      auto interval = std::chrono::duration_cast<std::chrono::milliseconds>(tp_now - tp_start).count();
      RCLCPP_DEBUG(rclcpp::get_logger("mono2d_palm_det"), "after cvtColorForDisplay cost ms: %ld", interval);
    }
    pyramid =
        hobot::dnn_node::ImageProc::GetNV12PyramidFromBGRImg(cv_img->image, model_input_height_, model_input_width_);
  }
  if ("nv12" == std::string(reinterpret_cast<const char*>(img_msg->encoding.data())))
  {
    if (img_msg->height != static_cast<uint32_t>(model_input_height_) ||
        img_msg->width != static_cast<uint32_t>(model_input_width_))
    {
      // padding and resize image for model input
      float scale = std::min(static_cast<float>(model_input_width_) / img_msg->width,
                             static_cast<float>(model_input_height_) / img_msg->height);

      uint32_t scaled_width = static_cast<uint32_t>(img_msg->width * scale);
      uint32_t scaled_height = static_cast<uint32_t>(img_msg->height * scale);
      auto resized_img = hobot_cv::hobotcv_resize(reinterpret_cast<const char*>(img_msg->data.data()), img_msg->height,
                                                  img_msg->width, scaled_height, scaled_width);
      if (resized_img == nullptr)
      {
        return;
      }
      // create a new buffer to put padded image
      std::vector<uint8_t> padded_img(model_input_width_ * model_input_height_ * 3 / 2, 0);  // NV12 format

      uint32_t pad_left = 0;

      // copy Y component
      const uint8_t* src_y = reinterpret_cast<const uint8_t*>(resized_img->imageAddr);
      uint8_t* dst_y = padded_img.data();

      for (uint32_t y = 0; y < scaled_height; ++y)
      {
        std::copy(src_y + y * scaled_width, src_y + y * scaled_width + scaled_width,
                  dst_y + y * model_input_width_ + pad_left);
      }

      // copy UV component
      const uint8_t* src_uv = src_y + scaled_width * scaled_height;
      uint8_t* dst_uv = dst_y + model_input_width_ * model_input_height_;

      for (uint32_t y = 0; y < scaled_height / 2; ++y)
      {
        std::copy(src_uv + y * scaled_width, src_uv + y * scaled_width + scaled_width,
                  dst_uv + y * model_input_width_ + pad_left);
      }

      // get final pyramid
      pyramid = hobot::dnn_node::ImageProc::GetNV12PyramidFromNV12Img(reinterpret_cast<const char*>(padded_img.data()),
                                                                      model_input_height_, model_input_width_,
                                                                      model_input_height_, model_input_width_);
    }
    else
    {
      // if size is right, not padding and resize
      pyramid = hobot::dnn_node::ImageProc::GetNV12PyramidFromNV12Img(
          reinterpret_cast<const char*>(img_msg->data.data()), img_msg->height, img_msg->width, model_input_height_,
          model_input_width_);
    }
  }
  else
  {
    RCLCPP_INFO(rclcpp::get_logger("mono2d_palm_det"), "Unsupported img encoding: %s", img_msg->encoding.data());
  }

  if (!pyramid)
  {
    RCLCPP_ERROR(rclcpp::get_logger("mono2d_palm_det"), "Get Nv12 pym fail!");
    return;
  }

  {
    auto tp_now = std::chrono::system_clock::now();
    auto interval = std::chrono::duration_cast<std::chrono::milliseconds>(tp_now - tp_start).count();
    RCLCPP_DEBUG(rclcpp::get_logger("mono2d_palm_det"), "after GetNV12Pyramid cost ms: %ld", interval);
  }

  auto dnn_output = std::make_shared<parser_palm::PalmNodeOutput>();
  auto inputs = std::vector<std::shared_ptr<DNNInput>>{ pyramid };
  dnn_output->image_msg_header = std::make_shared<std_msgs::msg::Header>();
  dnn_output->image_msg_header->set__frame_id(img_msg->header.frame_id);
  dnn_output->image_msg_header->set__stamp(img_msg->header.stamp);
  dnn_output->pyramid = pyramid;

  if (node_output_manage_ptr_)
  {
    node_output_manage_ptr_->Feed(img_msg->header.stamp.sec * 1000 + img_msg->header.stamp.nanosec / 1000 / 1000);
  }

  dnn_output->preprocess_timespec_start = time_start;
  dnn_output->scale = std::max(img_msg->height, img_msg->width);
  dnn_output->image_width_ori = img_msg->width;
  dnn_output->image_height_ori = img_msg->height;

  uint32_t ret = 0;
  // run predict
  ret = Run(inputs, dnn_output, nullptr);

  {
    auto tp_now = std::chrono::system_clock::now();
    auto interval = std::chrono::duration_cast<std::chrono::milliseconds>(tp_now - tp_start).count();
    RCLCPP_DEBUG(rclcpp::get_logger("mono2d_palm_det"), "after Predict cost ms: %ld", interval);
  }
  if (ret != 0)
  {
    return;
  }
}

#ifdef SHARED_MEM_ENABLED
void Mono2dPalmDetNode::SharedMemImgProcess(const hbm_img_msgs::msg::HbmMsg1080P::ConstSharedPtr img_msg)
{
  if (!img_msg || !rclcpp::ok())
  {
    return;
  }
  static int gap_cnt = 0;
  if (++gap_cnt < image_gap_)
  {
    return;
  }
  gap_cnt = 0;
  struct timespec time_start = { 0, 0 };
  clock_gettime(CLOCK_REALTIME, &time_start);

  std::stringstream ss;
  ss << "SharedMemImgProcess Recved img encoding: "
     << std::string(reinterpret_cast<const char*>(img_msg->encoding.data())) << ", h: " << img_msg->height
     << ", w: " << img_msg->width << ", step: " << img_msg->step << ", index: " << img_msg->index
     << ", stamp: " << img_msg->time_stamp.sec << "_" << img_msg->time_stamp.nanosec
     << ", data size: " << img_msg->data_size;
  RCLCPP_INFO(rclcpp::get_logger("mono2d_palm_det"), "%s", ss.str().c_str());
  rclcpp::Time msg_ts = img_msg->time_stamp;
  rclcpp::Duration dura = this->now() - msg_ts;
  float duration_ms = dura.nanoseconds() / 1000.0 / 1000.0;
  RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000, "%s, comm delay [%.4f]ms", ss.str().c_str(),
                       duration_ms);

  auto tp_start = std::chrono::system_clock::now();

  std::shared_ptr<hobot::dnn_node::NV12PyramidInput> pyramid = nullptr;
  if ("nv12" == std::string(reinterpret_cast<const char*>(img_msg->encoding.data())))
  {
    if (img_msg->height != static_cast<uint32_t>(model_input_height_) ||
        img_msg->width != static_cast<uint32_t>(model_input_width_))
    {
      // padding and resize image for model input
      float scale = std::min(static_cast<float>(model_input_width_) / img_msg->width,
                             static_cast<float>(model_input_height_) / img_msg->height);

      uint32_t scaled_width = static_cast<uint32_t>(img_msg->width * scale);
      uint32_t scaled_height = static_cast<uint32_t>(img_msg->height * scale);
      auto resized_img = hobot_cv::hobotcv_resize(reinterpret_cast<const char*>(img_msg->data.data()), img_msg->height,
                                                  img_msg->width, scaled_height, scaled_width);
      if (resized_img == nullptr)
      {
        return;
      }
      // create a new buffer to put padded image
      std::vector<uint8_t> padded_img(model_input_width_ * model_input_height_ * 3 / 2, 0);  // NV12 format

      uint32_t pad_left = 0;

      // copy Y component
      const uint8_t* src_y = reinterpret_cast<const uint8_t*>(resized_img->imageAddr);
      uint8_t* dst_y = padded_img.data();

      for (uint32_t y = 0; y < scaled_height; ++y)
      {
        std::copy(src_y + y * scaled_width, src_y + y * scaled_width + scaled_width,
                  dst_y + y * model_input_width_ + pad_left);
      }

      // copy UV component
      const uint8_t* src_uv = src_y + scaled_width * scaled_height;
      uint8_t* dst_uv = dst_y + model_input_width_ * model_input_height_;

      for (uint32_t y = 0; y < scaled_height / 2; ++y)
      {
        std::copy(src_uv + y * scaled_width, src_uv + y * scaled_width + scaled_width,
                  dst_uv + y * model_input_width_ + pad_left);
      }

      // get final pyramid
      pyramid = hobot::dnn_node::ImageProc::GetNV12PyramidFromNV12Img(reinterpret_cast<const char*>(padded_img.data()),
                                                                      model_input_height_, model_input_width_,
                                                                      model_input_height_, model_input_width_);
    }
    else
    {
      // if size is right, not padding and resize
      pyramid = hobot::dnn_node::ImageProc::GetNV12PyramidFromNV12Img(
          reinterpret_cast<const char*>(img_msg->data.data()), img_msg->height, img_msg->width, model_input_height_,
          model_input_width_);
    }
  }
  else
  {
    RCLCPP_INFO(rclcpp::get_logger("mono2d_palm_det"), "Unsupported img encoding: %s", img_msg->encoding.data());
  }

  if (!pyramid)
  {
    RCLCPP_ERROR(rclcpp::get_logger("mono2d_palm_det"), "Get Nv12 pym fail!");
    return;
  }

  {
    auto tp_now = std::chrono::system_clock::now();
    auto interval = std::chrono::duration_cast<std::chrono::milliseconds>(tp_now - tp_start).count();
    RCLCPP_DEBUG(rclcpp::get_logger("mono2d_palm_det"), "after GetNV12Pyramid cost ms: %ld", interval);
  }

  auto dnn_output = std::make_shared<parser_palm::PalmNodeOutput>();
  auto inputs = std::vector<std::shared_ptr<DNNInput>>{ pyramid };
  dnn_output->image_msg_header = std::make_shared<std_msgs::msg::Header>();
  dnn_output->image_msg_header->set__frame_id(std::to_string(img_msg->index));
  dnn_output->image_msg_header->set__stamp(img_msg->time_stamp);
  dnn_output->pyramid = pyramid;

  if (node_output_manage_ptr_)
  {
    node_output_manage_ptr_->Feed(img_msg->time_stamp.sec * 1000 + img_msg->time_stamp.nanosec / 1000 / 1000);
  }

  dnn_output->preprocess_timespec_start = time_start;
  dnn_output->scale = std::max(img_msg->height, img_msg->width);
  dnn_output->image_width_ori = img_msg->width;
  dnn_output->image_height_ori = img_msg->height;

  uint32_t ret = 0;
  // run predict
  ret = Run(inputs, dnn_output, nullptr);

  {
    auto tp_now = std::chrono::system_clock::now();
    auto interval = std::chrono::duration_cast<std::chrono::milliseconds>(tp_now - tp_start).count();
    RCLCPP_DEBUG(rclcpp::get_logger("mono2d_palm_det"), "after Predict cost ms: %ld", interval);
  }
  if (ret != 0)
  {
    return;
  }
}
#endif  // SHARED_MEM_ENABLED

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(Mono2dPalmDetNode)
