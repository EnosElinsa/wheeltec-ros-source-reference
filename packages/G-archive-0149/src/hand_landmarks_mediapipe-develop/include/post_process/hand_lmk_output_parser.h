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

#ifndef HAND_LANMARKS_OUTPUT_PARSER_H
#define HAND_LANMARKS_OUTPUT_PARSER_H

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <opencv2/opencv.hpp>

#include "dnn_node/dnn_node_data.h"
#include "dnn_node/util/output_parser/detection/fasterrcnn_output_parser.h"

using hobot::dnn_node::parser_fasterrcnn::Landmarks;
using hobot::dnn_node::DnnNodeOutput;

namespace parser_hand_lmk
{
class HandLmkResult
{
public:
  Landmarks lmks;    // landmarks results
  float scores;      // detection scores
  float left_right;  // left or right hand

  void Reset()
  {
    lmks.clear();
    scores = 0.0;
    left_right = 0.0;
  }
};

struct HandNodeOutput : public DnnNodeOutput
{
  std::shared_ptr<std_msgs::msg::Header> image_msg_header = nullptr;
  // use for render image in local host
  std::shared_ptr<hobot::dnn_node::NV12PyramidInput> pyramid = nullptr;

  std::shared_ptr<std::vector<hbDNNRoi>> valid_rois;

  struct timespec preprocess_timespec_start;
  struct timespec preprocess_timespec_end;
  HandNodeOutput()
  {
  }
  ~HandNodeOutput()
  {
  }
  std::vector<HandLmkResult> lmk_result;
};
}  // namespace parser_hand_lmk

#endif  // HAND_LANMARKS_OUTPUT_PARSER_H
