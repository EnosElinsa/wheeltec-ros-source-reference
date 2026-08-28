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

#ifndef LLAMA_OUTPUT_PARSER_H_
#define LLAMA_OUTPUT_PARSER_H_

#include <fstream>
#include <memory>
#include <string>
#include <unistd.h>
#include <unordered_map>
#include <utility>
#include <vector>
#include <algorithm>

#include <opencv2/core/types.hpp>

#include "rclcpp/rclcpp.hpp"
#include "dnn_node/dnn_node_data.h"
#include "dnn_node/util/output_parser/perception_common.h"
#include "dnn_node/util/output_parser/detection/nms.h"
#include "std_msgs/msg/string.hpp"

#include "hand_lmk_output_parser.h"
#include "include/mono2d_hand_lmk_node.h"

using hobot::dnn_node::DNNTensor;
using hobot::dnn_node::output_parser::Bbox;
using hobot::dnn_node::output_parser::Detection;
using hobot::dnn_node::output_parser::DnnParserResult;
using hobot::dnn_node::output_parser::Perception;
using hobot::dnn_node::parser_fasterrcnn::Landmarks;
using hobot::dnn_node::parser_fasterrcnn::Point;

struct ROI
{
  float x;
  float y;
  float width;
  float height;
};

// parse model output to hand landmarks result
int32_t HandLmkParse(const std::vector<std::shared_ptr<DNNTensor>>& output_tensors,
                     std::shared_ptr<HandNodeOutput>& output_hand_res);
// make sure input roi is valid
int NormalizeRoi(const hbDNNRoi* src, hbDNNRoi* dst, float norm_ratio, int32_t total_w, int32_t total_h);
// calculate tight roi for hand landmarks
cv::Rect calculate_tight_roi(const std::vector<Point>& landmarks);
// calculate tight roi for hand landmarks and write to roi msg
int calculate_tight_roi(const std::vector<Point>& landmarks, ai_msgs::msg::Roi& roi, float score);

#endif  // LLAMA_OUTPUT_PARSER_H_
