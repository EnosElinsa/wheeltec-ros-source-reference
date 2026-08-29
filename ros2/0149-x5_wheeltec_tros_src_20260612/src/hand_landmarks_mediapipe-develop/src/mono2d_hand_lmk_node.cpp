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

#include "dnn_node/dnn_node.h"
#include "dnn_node/util/image_proc.h"
#include "rclcpp/rclcpp.hpp"
#include "hobot_cv/hobotcv_imgproc.h"

#include "builtin_interfaces/msg/detail/time__struct.h"
#include "rcpputils/env.hpp"
#include "rcutils/env.h"
#include "opencv2/imgproc/types_c.h"
#include "include/post_process/gesture.h"
#include "include/mono2d_hand_lmk_node.h"
#include "utils.h"

int Deduplication(std::vector<parser_hand_lmk::HandLmkResult> lmk_result,
                  std::vector<parser_hand_lmk::HandLmkResult>& filter_lmk_result)
{
  // sort by score from high to low
  std::sort(lmk_result.begin(), lmk_result.end(),
            [](const parser_hand_lmk::HandLmkResult& a, const parser_hand_lmk::HandLmkResult& b) {
              return a.scores > b.scores;
            });
  std::vector<bool> isDedup(lmk_result.size(), false);
  std::vector<cv::Rect> bbox_iou;
  for (auto& lmk : lmk_result)
  {
    auto rect = calculate_tight_roi(lmk.lmks);
    // from wrist to middle finger
    auto dir = Point(lmk.lmks[9].x - lmk.lmks[0].x, lmk.lmks[9].y - lmk.lmks[0].y);
    rect = MoveBox(rect, 2.0, 0.1, dir);
    bbox_iou.push_back(rect);
  }
  for (size_t i = 0; i < lmk_result.size(); i++)
  {
    // base_distance is the max distance of wrist and index finger and little finger
    auto base_distance = Distance(lmk_result[i].lmks[0], lmk_result[i].lmks[5]);
    base_distance = std::max(base_distance, Distance(lmk_result[i].lmks[5], lmk_result[i].lmks[17]));
    base_distance = std::max(base_distance, Distance(lmk_result[i].lmks[0], lmk_result[i].lmks[17]));
    base_distance *= 0.2;
    for (size_t j = i + 1; j < lmk_result.size(); j++)
    {
      if (CalculateIOU(bbox_iou[i], bbox_iou[j]) > 0.2)
      {
        int dequ_num = 0;
        for (int k = 0; k < 21; k++)
        {
          if (Distance(lmk_result[i].lmks[k], lmk_result[j].lmks[k]) < base_distance)
          {
            dequ_num++;
          }
        }
        // when 2 points' distance close than base distance, it is a same point.
        // if 10 and more point is the same, then dequ
        if (dequ_num >= 10)
        {
          isDedup[j] = true;
          continue;
        }
      }
      // filter contained bbox
      if ((bbox_iou[i] & bbox_iou[j]).area() > 0.7 * bbox_iou[j].area())
      {
        isDedup[j] = true;
        continue;
      }
    }
  }
  for (size_t i = 0; i < lmk_result.size(); i++)
  {
    if (!isDedup[i])
    {
      filter_lmk_result.emplace_back(lmk_result[i]);
    }
  }
  return 0;
}

void NodeOutputManage::Feed(uint64_t ts_ms)
{
  RCLCPP_DEBUG(rclcpp::get_logger("mono2d_hand_lmk"), "feed frame ts: %lu", ts_ms);

  std::unique_lock<std::mutex> lk(mtx_);
  cache_frame_.insert(ts_ms);
  if (cache_frame_.size() > cache_size_limit_)
  {
    cache_frame_.erase(cache_frame_.begin());
  }
  RCLCPP_DEBUG(rclcpp::get_logger("mono2d_hand_lmk"), "cache_frame_.size(): %ld", cache_frame_.size());
}

std::vector<std::shared_ptr<DnnNodeOutput>> NodeOutputManage::Feed(const std::shared_ptr<DnnNodeOutput>& in_node_output)
{
  std::vector<std::shared_ptr<DnnNodeOutput>> node_outputs{};
  auto hand_node_output = std::dynamic_pointer_cast<HandNodeOutput>(in_node_output);
  if (!hand_node_output || !hand_node_output->image_msg_header)
  {
    return node_outputs;
  }

  uint64_t ts_ms = hand_node_output->image_msg_header->stamp.sec * 1000 +
                   hand_node_output->image_msg_header->stamp.nanosec / 1000 / 1000;
  RCLCPP_DEBUG(rclcpp::get_logger("mono2d_hand_lmk"), "feed ts: %lu", ts_ms);

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

  RCLCPP_DEBUG(rclcpp::get_logger("mono2d_hand_lmk"), "node_outputs.size(): %ld", node_outputs.size());

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
        RCLCPP_DEBUG(rclcpp::get_logger("mono2d_hand_lmk"), "push ts: %lu", *first_frame);

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

Mono2dHandLmkNode::Mono2dHandLmkNode(const NodeOptions& options) : DnnNode("mono2d_hand_lmk", options)
{
  // init Mono2dHandLmkNode
  this->declare_parameter<int>("is_sync_mode", is_sync_mode_);
  this->declare_parameter<std::string>("model_file_name", model_file_name_);
  this->declare_parameter<int>("is_shared_mem_sub", is_shared_mem_sub_);
  this->declare_parameter<std::string>("ai_msg_pub_topic_name", ai_msg_pub_topic_name_);
  this->declare_parameter<std::string>("ros_img_topic_name", ros_img_topic_name_);
  this->declare_parameter<std::string>("sharedmem_img_topic_name", sharedmem_img_topic_name_);
  this->declare_parameter<int>("image_gap", image_gap_);
  this->declare_parameter<int>("dump_render_img", dump_render_img_);
  this->declare_parameter<std::string>("palm_topic_name", palm_topic_name_);
  this->declare_parameter<std::string>("image_file", image_file_);
  this->declare_parameter<float>("min_score", min_score_);
  this->declare_parameter<float>("nms_iou_thres", nms_iou_thres_);

  this->get_parameter<int>("is_sync_mode", is_sync_mode_);
  this->get_parameter<std::string>("model_file_name", model_file_name_);
  this->get_parameter<int>("is_shared_mem_sub", is_shared_mem_sub_);
  this->get_parameter<std::string>("ai_msg_pub_topic_name", ai_msg_pub_topic_name_);
  this->get_parameter<std::string>("ros_img_topic_name", ros_img_topic_name_);
  this->get_parameter<std::string>("sharedmem_img_topic_name", sharedmem_img_topic_name_);
  this->get_parameter<int>("image_gap", image_gap_);
  this->get_parameter<int>("dump_render_img", dump_render_img_);
  this->get_parameter<std::string>("palm_topic_name", palm_topic_name_);
  this->get_parameter<std::string>("image_file", image_file_);
  this->get_parameter<float>("min_score", min_score_);
  this->get_parameter<float>("nms_iou_thres", nms_iou_thres_);
  {
    std::stringstream ss;
    ss << "Parameter:"
       << "\n is_sync_mode_: " << is_sync_mode_ << "\n model_file_name_: " << model_file_name_
       << "\n is_shared_mem_sub: " << is_shared_mem_sub_ << "\n ai_msg_pub_topic_name: " << ai_msg_pub_topic_name_
       << "\n ros_img_topic_name: " << ros_img_topic_name_ << "\n image_gap: " << image_gap_
       << "\n dump_render_img: " << dump_render_img_  << "\n nms_iou_thres: " << nms_iou_thres_;
    RCLCPP_WARN(rclcpp::get_logger("mono2d_hand_lmk"), "%s", ss.str().c_str());
  }

  if (Init() != 0)
  {
    RCLCPP_ERROR(rclcpp::get_logger("mono2d_hand_lmk"), "Init failed!");
    rclcpp::shutdown();
    return;
  }

  // Init()之后模型已经加载成功，查询kps解析参数
  auto model_manage = GetModel();
  if (!model_manage)
  {
    RCLCPP_ERROR(rclcpp::get_logger("mono2d_hand_lmk"), "Invalid model");
    rclcpp::shutdown();
    return;
  }

  if (model_name_.empty())
  {
    if (!GetModel())
    {
      RCLCPP_ERROR(rclcpp::get_logger("mono2d_hand_lmk"), "Get model fail.");
    }
    else
    {
      model_name_ = GetModel()->GetName();
      RCLCPP_WARN(rclcpp::get_logger("mono2d_hand_lmk"), "Get model name: %s from load model.", model_name_.c_str());
    }
  }

  msg_publisher_ = this->create_publisher<ai_msgs::msg::PerceptionTargets>(ai_msg_pub_topic_name_, 10);

  palm_subscription_ = this->create_subscription<ai_msgs::msg::PerceptionTargets>(
      palm_topic_name_, 10, std::bind(&Mono2dHandLmkNode::PalmResProcess, this, std::placeholders::_1));

  if (GetModelInputSize(0, model_input_width_, model_input_height_) < 0)
  {
    RCLCPP_ERROR(rclcpp::get_logger("mono2d_hand_lmk"), "Get model input size fail!");
    rclcpp::shutdown();
  }
  else
  {
    RCLCPP_INFO(rclcpp::get_logger("mono2d_hand_lmk"), "The model input width is %d and height is %d",
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
    RCLCPP_WARN(rclcpp::get_logger("mono2d_hand_lmk"), "Create hbmem_subscription with topic_name: %s",
                sharedmem_img_topic_name_.c_str());
    sharedmem_img_subscription_ = this->create_subscription<hbm_img_msgs::msg::HbmMsg1080P>(
        sharedmem_img_topic_name_, rclcpp::SensorDataQoS(),
        std::bind(&Mono2dHandLmkNode::SharedMemImgProcess, this, std::placeholders::_1));
#else
    RCLCPP_ERROR(rclcpp::get_logger("mono2d_hand_lmk"), "Unsupport shared mem");
#endif
  }
  else
  {
    RCLCPP_WARN(rclcpp::get_logger("mono2d_hand_lmk"), "Create subscription with topic_name: %s",
                ros_img_topic_name_.c_str());
    ros_img_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
        ros_img_topic_name_, 10, std::bind(&Mono2dHandLmkNode::RosImgProcess, this, std::placeholders::_1));
  }
}

Mono2dHandLmkNode::~Mono2dHandLmkNode()
{
}

int Mono2dHandLmkNode::SetNodePara()
{
  RCLCPP_INFO(rclcpp::get_logger("mono2d_hand_lmk"), "Set node para.");
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

int Mono2dHandLmkNode::PostProcess(const std::shared_ptr<DnnNodeOutput>& outputs)
{
  // RCLCPP_INFO(rclcpp::get_logger("mono2d_hand_lmk"), "Pointer: %p, Value: %f", outputs);
  auto handOutput = std::dynamic_pointer_cast<HandNodeOutput>(outputs);

  if (!rclcpp::ok())
  {
    return 0;
  }

  if (!msg_publisher_)
  {
    RCLCPP_ERROR(rclcpp::get_logger("mono2d_hand_lmk"), "Invalid msg_publisher_");
    return -1;
  }

  auto hand_node_output = std::dynamic_pointer_cast<HandNodeOutput>(outputs);
  {
    std::stringstream ss;
    ss << "Outputs from";
    ss << ", frame_id: " << handOutput->image_msg_header->frame_id
       << ", stamp: " << handOutput->image_msg_header->stamp.sec << "_" << handOutput->image_msg_header->stamp.nanosec
       << ", infer time ms: " << handOutput->rt_stat->infer_time_ms;
    RCLCPP_DEBUG(rclcpp::get_logger("mono2d_hand_lmk"), "%s", ss.str().c_str());
  }

  // 使用hobot dnn内置的Parse解析方法，解析算法输出的DNNTensor类型数据
  int ret = -1;
  ret = HandLmkParse(outputs->output_tensors, hand_node_output);  // parse output tensor

  if (ret < 0)
  {
    RCLCPP_ERROR(rclcpp::get_logger("mono2d_hand_lmk"), "Parse node_output fail!");
    return -1;
  }

  struct timespec time_start = { 0, 0 };
  clock_gettime(CLOCK_REALTIME, &time_start);

  ai_msgs::msg::PerceptionTargets::UniquePtr pub_data(new ai_msgs::msg::PerceptionTargets());
  if (hand_node_output->image_msg_header)
  {
    pub_data->header.set__stamp(hand_node_output->image_msg_header->stamp);
    pub_data->header.set__frame_id(hand_node_output->image_msg_header->frame_id);
  }
  if (outputs->rt_stat)
  {
    pub_data->set__fps(round(outputs->rt_stat->output_fps));
  }

  std::vector<parser_hand_lmk::HandLmkResult> filter_lmk_result;
  Deduplication(hand_node_output->lmk_result, filter_lmk_result);
  track_hand_rects.clear();

  for (uint32_t b = 0; b < filter_lmk_result.size(); b++)
  {
    auto score = filter_lmk_result[b].scores;
    // ignore low score prediction
    if (score < min_score_)
    {
      continue;
    }
    ai_msgs::msg::Target target;  // a target is one hand

    target.set__type("hand_lmks");
    ai_msgs::msg::Roi roi;
    calculate_tight_roi(filter_lmk_result[b].lmks, roi, score);  // for show gesture in web
    target.rois.emplace_back(roi);

    // calc hand rect for track in next frame
    cv::Rect tight_rect = cv::Rect(roi.rect.x_offset, roi.rect.y_offset, roi.rect.width, roi.rect.height);
    Point dir = { filter_lmk_result[b].lmks[9].x - filter_lmk_result[b].lmks[0].x,
                  filter_lmk_result[b].lmks[9].y - filter_lmk_result[b].lmks[0].y };
    auto next_frame_rect = MoveBox(tight_rect, 2.0, 0.1, dir);
    track_hand_rects.emplace_back(next_frame_rect);

    // {  // for show roi range in web, not necessory
    //   roi.set__type("roi_range");
    //   roi.rect.set__x_offset((*hand_node_output->valid_rois)[b].left);
    //   roi.rect.set__y_offset((*hand_node_output->valid_rois)[b].top);
    //   roi.rect.set__width((*hand_node_output->valid_rois)[b].right - (*hand_node_output->valid_rois)[b].left);
    //   roi.rect.set__height((*hand_node_output->valid_rois)[b].bottom - (*hand_node_output->valid_rois)[b].top);
    //   target.rois.emplace_back(roi);
    // }
    ai_msgs::msg::Point target_point;
    target_point.set__type("hand_kps");
    // you can change order for your render order
    std::vector<int> orders = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20 };  // 21
                                                                                                             // points
    std::stringstream ss;
    for (const auto& ord : orders)
    {
      auto lmk = filter_lmk_result[b].lmks[ord];
      geometry_msgs::msg::Point32 pt;  // a hand landmark point
      pt.set__x(lmk.x);
      pt.set__y(lmk.y);
      target_point.point.emplace_back(pt);
      target_point.confidence.emplace_back(filter_lmk_result[b].scores);
      ss << "(" << lmk.x << " , " << lmk.y << ")  ";
    }
    RCLCPP_DEBUG(rclcpp::get_logger("mono2d_palm_det"), "Palm kps: %s", ss.str().c_str());
    target.points.emplace_back(target_point);
    ai_msgs::msg::Attribute attr;  // for show gesture in web
    attr.set__type("gesture");
    auto ges = recognise_gesture(filter_lmk_result[b].lmks);
    attr.set__value(static_cast<int>(ges));
    target.attributes.emplace_back(attr);
    pub_data->targets.emplace_back(std::move(target));
  }

  {
    std::stringstream ss;
    ss << "Publish frame_id: " << hand_node_output->image_msg_header->frame_id
       << ", time_stamp: " << std::to_string(pub_data->header.stamp.sec) << "_"
       << std::to_string(pub_data->header.stamp.nanosec) << "\n";
    RCLCPP_INFO(rclcpp::get_logger("mono2d_hand_lmk"), "%s", ss.str().c_str());
  }

  struct timespec time_now = { 0, 0 };
  clock_gettime(CLOCK_REALTIME, &time_now);

  // preprocess
  ai_msgs::msg::Perf perf_preprocess;
  perf_preprocess.set__type(model_name_ + "_preprocess");
  perf_preprocess.set__stamp_start(ConvertToRosTime(hand_node_output->preprocess_timespec_start));
  perf_preprocess.set__stamp_end(ConvertToRosTime(hand_node_output->preprocess_timespec_end));
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

  std::stringstream ss;
  ss << "Publish frame_id: " << hand_node_output->image_msg_header->frame_id
     << ", time_stamp: " << std::to_string(pub_data->header.stamp.sec) << "_"
     << std::to_string(pub_data->header.stamp.nanosec) << "\n";
  ss << "targets.size: " << pub_data->targets.size() << "\n";

  if (!pub_data->targets.empty())
  {
    for (const auto& target : pub_data->targets)
    {
      for (const auto& roi : target.rois)
      {
        ss << ", " << roi.type.c_str();
      }
      ss << ", points.size: " << target.points.size();
      for (const auto& point : target.points)
      {
        ss << ", " << point.type.c_str();
      }
      ss << "\n";
    }
  }

  RCLCPP_INFO(rclcpp::get_logger("mono2d_hand_lmk"), "%s", ss.str().c_str());

  if (hand_node_output->rt_stat->fps_updated)
  {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                         "input fps: %.2f, out fps: %.2f, infer time ms: %d, "
                         "post process time ms: %d",
                         hand_node_output->rt_stat->input_fps, hand_node_output->rt_stat->output_fps,
                         hand_node_output->rt_stat->infer_time_ms, static_cast<int>(perf_postprocess.time_ms_duration));
  }
  msg_publisher_->publish(std::move(pub_data));

  return 0;
}
int Mono2dHandLmkNode::FeedFromLocal()
{
  if (access(image_file_.c_str(), R_OK) == -1)
  {
    RCLCPP_ERROR(rclcpp::get_logger("mono2d_hand_lmk"), "Image: %s not exist!", image_file_.c_str());
    return -1;
  }
  cv::Mat image = cv::imread(image_file_);

  // 1. 将图片处理成模型输入数据类型DNNInput
  // 使用图片生成pym，NV12PyramidInput为DNNInput的子类
  std::shared_ptr<hobot::dnn_node::NV12PyramidInput> pyramid = nullptr;
  // bgr img，支持将图片resize到模型输入size
  int img_h = image.rows;
  int img_w = image.cols;
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
  auto dnn_output = std::make_shared<HandNodeOutput>();
  struct timespec time_now = { 0, 0 };
  clock_gettime(CLOCK_REALTIME, &time_now);
  dnn_output->msg_header = std::make_shared<std_msgs::msg::Header>();
  dnn_output->msg_header->set__frame_id("feedback");

  // 3. 开始预测
  if (Run(inputs, dnn_output, nullptr) != 0)
  {
    RCLCPP_ERROR(rclcpp::get_logger("hobot_dosod"), "Run predict failed!");
    return -1;
  }
  return 0;
}

void Mono2dHandLmkNode::RosImgProcess(const sensor_msgs::msg::Image::ConstSharedPtr img_msg)
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
  RCLCPP_INFO(rclcpp::get_logger("mono2d_hand_lmk"), "%s", ss.str().c_str());

  auto tp_start = std::chrono::system_clock::now();
  auto dnn_output = std::make_shared<HandNodeOutput>();
  auto rois = std::make_shared<std::vector<hbDNNRoi>>();  // input roi to model

  auto palms = std::move(palm_targets);
  for (auto& rect : track_hand_rects)
  {
    palms.emplace_back(rect);
  }
  uint result_size = palms.size();
  // if no palm is detected, put whole image as input. roi is all image
  if (result_size == 0)
  {
    auto roi_dst = std::make_shared<hbDNNRoi>();
    auto roi = hbDNNRoi();
    roi.left = 0;
    roi.top = 0;
    roi.right = img_msg->width;
    roi.bottom = img_msg->height;

    auto ret = NormalizeRoi(&roi, roi_dst.get(), 1.0, img_msg->width, img_msg->height);  // process roi and check valid
    RCLCPP_DEBUG(rclcpp::get_logger("mono2d_hand_lmk"), "ROI Range: x1:%d  y1:%d  x2:%d  y2:%d  ret:%d", roi_dst->left,
                 roi_dst->top, roi_dst->right, roi_dst->bottom, ret);
    if (ret < 0)
    {
      RCLCPP_ERROR(rclcpp::get_logger("mono2d_hand_lmk"), "ROI Range: x1:%d  y1:%d  x2:%d  y2:%d  ret:%d",
                   roi_dst->left, roi_dst->top, roi_dst->right, roi_dst->bottom, ret);
    }
    else
    {
      rois->emplace_back(*roi_dst);
    }
  }
  for (uint i = 0; i < result_size; i++)
  {
    auto roi_dst = std::make_shared<hbDNNRoi>();
    auto palm = palms[i];
    auto roi = hbDNNRoi();
    roi.left = palm.x;
    roi.top = palm.y;
    roi.right = palm.x + palm.width;
    roi.bottom = palm.y + palm.height;

    auto ret = NormalizeRoi(&roi, roi_dst.get(), 1.0, img_msg->width, img_msg->height);  // process roi and check valid
    RCLCPP_INFO(rclcpp::get_logger("mono2d_hand_lmk"), "ROI Range: x1:%d  y1:%d  x2:%d  y2:%d  ret:%d", roi_dst->left,
                roi_dst->top, roi_dst->right, roi_dst->bottom, ret);
    if (ret < 0)
    {
      RCLCPP_ERROR(rclcpp::get_logger("mono2d_hand_lmk"), "ROI Range: x1:%d  y1:%d  x2:%d  y2:%d  ret:%d",
                   roi_dst->left, roi_dst->top, roi_dst->right, roi_dst->bottom, ret);
    }
    else
    {
      rois->emplace_back(*roi_dst);
    }
  }
  auto pyramid = hobot::dnn_node::ImageProc::GetNV12PyramidFromNV12Img(
      reinterpret_cast<const char*>(img_msg->data.data()), img_msg->height, img_msg->width, img_msg->height,
      img_msg->width);

  if (!pyramid)
  {
    RCLCPP_ERROR(rclcpp::get_logger("mono2d_hand_lmk"), "Get Nv12 pym fail!");
    return;
  }
  // SaveNV12FromPyramid(*pyramid, "/root/img.nv12");

  {
    auto tp_now = std::chrono::system_clock::now();
    auto interval = std::chrono::duration_cast<std::chrono::milliseconds>(tp_now - tp_start).count();
    RCLCPP_INFO(rclcpp::get_logger("mono2d_hand_lmk"), "after GetNV12Pyramid cost ms: %ld", interval);
  }

  // 2. 使用pyramid创建DNNInput对象inputs
  // inputs将会作为模型的输入通过RunInferTask接口传入

  std::vector<std::shared_ptr<DNNInput>> inputs;
  // every roi has a pyramid
  for (size_t i = 0; i < rois->size(); i++)
  {
    inputs.push_back(pyramid);
  }
  dnn_output->valid_rois = rois;

  dnn_output->image_msg_header = std::make_shared<std_msgs::msg::Header>();
  dnn_output->image_msg_header->set__frame_id(img_msg->header.frame_id);
  dnn_output->image_msg_header->set__stamp(img_msg->header.stamp);

  if (node_output_manage_ptr_)
  {
    node_output_manage_ptr_->Feed(img_msg->header.stamp.sec * 1000 + img_msg->header.stamp.nanosec / 1000 / 1000);
  }

  dnn_output->preprocess_timespec_start = time_start;
  struct timespec time_now = { 0, 0 };
  clock_gettime(CLOCK_REALTIME, &time_now);
  dnn_output->preprocess_timespec_end = time_now;

  uint32_t ret = 0;
  // 3. 开始预测
  ret = Run(inputs, dnn_output, rois, is_sync_mode_ == 1 ? true : false);

  if (ret != 0)
  {
    return;
  }
}

void Mono2dHandLmkNode::PalmResProcess(const ai_msgs::msg::PerceptionTargets::ConstSharedPtr palm_res_msg)
{
  // get palm detection results
  palm_targets.clear();
  RCLCPP_DEBUG(rclcpp::get_logger("mono2d_hand_lmk"), "PalmResProcess Recved Palm detection results: bbox size: %ld",
               (*palm_res_msg.get()).targets.size());
  for (auto& target : (*palm_res_msg.get()).targets)
  {
    std::stringstream ss;
    auto dx = target.points[0].point[2].x - target.points[0].point[0].x;
    auto dy = target.points[0].point[2].y - target.points[0].point[0].y;
    cv::Rect ori_rect(target.rois[0].rect.x_offset, target.rois[0].rect.y_offset, target.rois[0].rect.width,
                      target.rois[0].rect.height);
    Point dir = Point(dx, dy);
    // get extend bbox from palm detction result
    auto new_rect = MoveBox(ori_rect, 2.6, 0.5, dir);

    palm_targets.emplace_back(new_rect);

    // palm_targets.emplace_back(cv::Rect(target.rois[1].rect.x_offset, target.rois[1].rect.y_offset,
    //                                    target.rois[1].rect.width, target.rois[1].rect.height));
    RCLCPP_DEBUG(rclcpp::get_logger("mono2d_hand_lmk"), "origin Bbox: %d  %d  %d  %d  %f, extend Bbox: %d  %d  %d  %d",
                 target.rois[0].rect.x_offset, target.rois[0].rect.y_offset, target.rois[0].rect.width,
                 target.rois[0].rect.height, target.rois[0].confidence, new_rect.x, new_rect.y, new_rect.width,
                 new_rect.height);
  }
  for (auto& rect : track_hand_rects)
  {
    RCLCPP_DEBUG(rclcpp::get_logger("mono2d_hand_lmk"), "last frame Bbox: %d  %d  %d  %d", rect.x, rect.y, rect.width,
                 rect.height);
  }
}

// save nv12 image before input to model for consistency check
void SaveNV12FromPyramid(const NV12PyramidInput& input, const std::string& filename)
{
  std::ofstream file(filename, std::ios::binary);
  if (!file.is_open())
  {
    std::cerr << "Failed to open file: " << filename << std::endl;
    return;
  }

  const uint8_t* y_plane = static_cast<const uint8_t*>(input.y_vir_addr);
  for (int i = 0; i < input.height; i++)
  {
    file.write(reinterpret_cast<const char*>(y_plane + i * input.y_stride), input.width);
  }

  const uint8_t* uv_plane = static_cast<const uint8_t*>(input.uv_vir_addr);
  for (int i = 0; i < input.height / 2; i++)
  {
    file.write(reinterpret_cast<const char*>(uv_plane + i * input.uv_stride), input.width);
  }

  file.close();
  RCLCPP_WARN(rclcpp::get_logger("mono2d_hand_lmk"), "NV12 image saved to: %s", filename.c_str());
}

#ifdef SHARED_MEM_ENABLED
void Mono2dHandLmkNode::SharedMemImgProcess(const hbm_img_msgs::msg::HbmMsg1080P::ConstSharedPtr img_msg)
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
  RCLCPP_DEBUG(rclcpp::get_logger("mono2d_hand_lmk"), "%s", ss.str().c_str());
  rclcpp::Time msg_ts = img_msg->time_stamp;
  rclcpp::Duration dura = this->now() - msg_ts;
  float duration_ms = dura.nanoseconds() / 1000.0 / 1000.0;
  RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000, "%s, comm delay [%.4f]ms", ss.str().c_str(),
                       duration_ms);

  auto tp_start = std::chrono::system_clock::now();
  auto dnn_output = std::make_shared<HandNodeOutput>();
  auto rois = std::make_shared<std::vector<hbDNNRoi>>();  // input roi to model

  auto palms = std::move(palm_targets);
  for (auto& rect : track_hand_rects)
  {
    palms.emplace_back(rect);
  }

  // NMS for palm boxes from track and detect
  std::vector<cv::Rect> filtered_palms;
  {
    std::vector<int> indices;
    std::vector<float> dummy_scores(palms.size(), 1.0); // ignore score, set all to 1.0
    cv::dnn::NMSBoxes(palms, dummy_scores, 0, nms_iou_thres_, indices, 1.f, 0);
    for (int i : indices)
    {
      filtered_palms.emplace_back(palms[i]);
    }
  }
  
  uint result_size = filtered_palms.size();
  // if no palm is detected, put whole image as input. roi is all image
  if (result_size == 0)
  {
    auto roi_dst = std::make_shared<hbDNNRoi>();
    auto roi = hbDNNRoi();
    roi.left = 0;
    roi.top = 0;
    roi.right = img_msg->width;
    roi.bottom = img_msg->height;

    auto ret = NormalizeRoi(&roi, roi_dst.get(), 1.0, img_msg->width, img_msg->height);  // process roi and check valid
    RCLCPP_DEBUG(rclcpp::get_logger("mono2d_hand_lmk"), "ROI Range: x1:%d  y1:%d  x2:%d  y2:%d  ret:%d", roi_dst->left,
                 roi_dst->top, roi_dst->right, roi_dst->bottom, ret);
    if (ret < 0)
    {
      RCLCPP_ERROR(rclcpp::get_logger("mono2d_hand_lmk"), "ROI Range: x1:%d  y1:%d  x2:%d  y2:%d  ret:%d",
                   roi_dst->left, roi_dst->top, roi_dst->right, roi_dst->bottom, ret);
    }
    else
    {
      rois->emplace_back(*roi_dst);
    }
  }
  for (uint i = 0; i < result_size; i++)
  {
    auto roi_dst = std::make_shared<hbDNNRoi>();
    auto palm = filtered_palms[i];
    auto roi = hbDNNRoi();
    roi.left = palm.x;
    roi.top = palm.y;
    roi.right = palm.x + palm.width;
    roi.bottom = palm.y + palm.height;

    auto ret = NormalizeRoi(&roi, roi_dst.get(), 1.0, img_msg->width, img_msg->height);  // process roi and check valid
    RCLCPP_INFO(rclcpp::get_logger("mono2d_hand_lmk"), "ROI Range: x1:%d  y1:%d  x2:%d  y2:%d  ret:%d", roi_dst->left,
                roi_dst->top, roi_dst->right, roi_dst->bottom, ret);
    if (ret < 0)
    {
      RCLCPP_ERROR(rclcpp::get_logger("mono2d_hand_lmk"), "ROI Range: x1:%d  y1:%d  x2:%d  y2:%d  ret:%d",
                   roi_dst->left, roi_dst->top, roi_dst->right, roi_dst->bottom, ret);
    }
    else
    {
      rois->emplace_back(*roi_dst);
    }
  }
  auto pyramid = hobot::dnn_node::ImageProc::GetNV12PyramidFromNV12Img(
      reinterpret_cast<const char*>(img_msg->data.data()), img_msg->height, img_msg->width, img_msg->height,
      img_msg->width);

  if (!pyramid)
  {
    RCLCPP_ERROR(rclcpp::get_logger("mono2d_hand_lmk"), "Get Nv12 pym fail!");
    return;
  }

  // if you want to save nv12 image, please uncomment below line
  // save all nv12 image may take much time, and extremely affect performance
  // {
  //   std::stringstream ss;
  //   ss << std::setw(4) << std::setfill('0') << img_msg->index;
  //   std::string save_path = "/mnt/disk1/deyu.li/workspace/video/" + ss.str() + ".nv12";
  //   SaveNV12FromPyramid(*pyramid, save_path);
  // }

  {
    auto tp_now = std::chrono::system_clock::now();
    auto interval = std::chrono::duration_cast<std::chrono::milliseconds>(tp_now - tp_start).count();
    RCLCPP_INFO(rclcpp::get_logger("mono2d_hand_lmk"), "after GetNV12Pyramid cost ms: %ld", interval);
  }

  // 2. 使用pyramid创建DNNInput对象inputs
  // inputs将会作为模型的输入通过RunInferTask接口传入

  std::vector<std::shared_ptr<DNNInput>> inputs;
  // every roi has a pyramid
  for (size_t i = 0; i < rois->size(); i++)
  {
    inputs.push_back(pyramid);
  }
  dnn_output->valid_rois = rois;

  dnn_output->image_msg_header = std::make_shared<std_msgs::msg::Header>();
  dnn_output->image_msg_header->set__frame_id(std::to_string(img_msg->index));
  dnn_output->image_msg_header->set__stamp(img_msg->time_stamp);

  if (node_output_manage_ptr_)
  {
    node_output_manage_ptr_->Feed(img_msg->time_stamp.sec * 1000 + img_msg->time_stamp.nanosec / 1000 / 1000);
  }

  dnn_output->preprocess_timespec_start = time_start;
  struct timespec time_now = { 0, 0 };
  clock_gettime(CLOCK_REALTIME, &time_now);
  dnn_output->preprocess_timespec_end = time_now;

  uint32_t ret = 0;
  // 3. 开始预测
  ret = Run(inputs, dnn_output, rois, is_sync_mode_ == 1 ? true : false);

  if (ret != 0)
  {
    return;
  }
}
#endif

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(Mono2dHandLmkNode)
