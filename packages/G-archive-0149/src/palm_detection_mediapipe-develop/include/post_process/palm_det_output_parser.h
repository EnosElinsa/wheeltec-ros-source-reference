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

#ifndef PALM_DETECTION_OUTPUT_PARSER_H
#define PALM_DETECTION_OUTPUT_PARSER_H

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <opencv2/opencv.hpp>

#include "anchors.h"

#include "dnn_node/dnn_node_data.h"
#include "dnn_node/util/output_parser/detection/fasterrcnn_output_parser.h"

using hobot::dnn_node::parser_fasterrcnn::Landmarks;
using hobot::dnn_node::DnnNodeOutput;

namespace parser_palm
{
// 解析后的检测框数据类型
class PalmRect
{
public:
  float left;    // x1
  float top;     // y1
  float right;   // x2
  float bottom;  // y2
  float conf;    // score
  PalmRect(float left_, float top_, float right_, float bottom_)
    : left(left_), top(top_), right(right_), bottom(bottom_), conf(0.0)
  {
  }
  void scale(int width, int heigh)
  {
    left *= width;
    right *= width;
    top *= heigh;
    bottom *= heigh;
  }
  friend bool operator>(const PalmRect& lhs, const PalmRect& rhs)
  {
    return (lhs.conf > rhs.conf);
  }
};

struct PalmNodeOutput : public DnnNodeOutput
{
  std::shared_ptr<std_msgs::msg::Header> image_msg_header = nullptr;
  // 算法推理使用的图像数据，用于本地渲染使用
  std::shared_ptr<hobot::dnn_node::NV12PyramidInput> pyramid = nullptr;

  struct timespec preprocess_timespec_start;
  struct timespec preprocess_timespec_end;
  PalmNodeOutput()
  {
  }
  ~PalmNodeOutput()
  {
  }
  int image_width_ori;
  int image_height_ori;
  int scale;
};

// parse palm detection output results
class PalmDetResult
{
public:
  std::vector<PalmRect> boxes;
  std::vector<Landmarks> lmks;
  std::vector<int> indices;

  void Reset()
  {
    boxes.clear();
    lmks.clear();
  }
};
}  // namespace parser_palm

#endif  // PALM_DETECTION_OUTPUT_PARSER_H
