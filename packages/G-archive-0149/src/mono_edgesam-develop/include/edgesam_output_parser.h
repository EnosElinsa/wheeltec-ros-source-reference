// Copyright (c) 2025，D-Robotics.
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

#ifndef MIBILESAM_OUTPUT_PARSER_H_
#define MIBILESAM_OUTPUT_PARSER_H_

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "opencv2/core/mat.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"

#include "dnn_node/dnn_node_data.h"
#include "dnn_node/util/output_parser/perception_common.h"
#include "rclcpp/rclcpp.hpp"

using hobot::dnn_node::DNNTensor;
using hobot::dnn_node::Model;
using hobot::dnn_node::output_parser::DnnParserResult;
using hobot::dnn_node::output_parser::Parsing;
using hobot::dnn_node::output_parser::Perception;

int RenderSeg(cv::Mat &mat, Parsing &seg, std::string& saving_path);

class EdgeSamOutputParser {
 public:
  EdgeSamOutputParser(int model_h, int model_w, int output_height, int output_width, int num_classes, float mask_threshold) {
    model_h_ = model_h;
    model_w_ = model_w;
    output_height_ = output_height;
    output_width_ = output_width;
    num_classes_ = num_classes;
    mask_threshold_ = mask_threshold;
  }
  ~EdgeSamOutputParser() {}

  int32_t Parse(
      std::shared_ptr<DnnParserResult> &result,
      const int resized_img_h,
      const int resized_img_w,
      std::vector<std::shared_ptr<DNNTensor>>& output_tensors,
      std::vector<std::vector<float>>& boxes);

  int32_t GenMask(
    const float* mask,
    const int resized_img_h,
    const int resized_img_w,
    Perception& perception);

  int32_t GenMaskScale(
    const int16_t* scores,
    const int8_t* mask,
    const std::vector<std::vector<float>>& boxes,
    const int resized_img_h,
    const int resized_img_w,
    Perception& perception);

  int32_t GenMultiMask(
    std::vector<std::shared_ptr<DNNTensor>>& output_tensors,
    const std::vector<std::vector<float>>& boxes,
    const int resized_img_h,
    const int resized_img_w,
    Perception& perception);

  int32_t GenMultiMaskScale(
    std::vector<std::shared_ptr<DNNTensor>>& output_tensors,
    const std::vector<std::vector<float>>& boxes,
    const int resized_img_h,
    const int resized_img_w,
    Perception& perception);

 private:
  int num_classes_ = 1;
  int model_h_ = 1024;
  int model_w_ = 1024;
  int output_height_ = 256;
  int output_width_ = 256;
  float mask_threshold_ = 0.0;
};

#endif  // MIBILESAM_OUTPUT_PARSER_H_
