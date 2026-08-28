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

#include <algorithm>
#include <cmath>

#include "include/post_process/hand_lmk_parser.h"

// resizer model input size limit
// roi, width & hight must be in range [16, 784]
uint32_t roi_h_size_max_ = 784;
uint32_t roi_h_size_min_ = 16;
uint32_t roi_w_size_max_ = 784;
uint32_t roi_w_size_min_ = 16;

int32_t HandLmkParse(const std::vector<std::shared_ptr<DNNTensor>>& output_tensors,
                     std::shared_ptr<HandNodeOutput>& output_hand_res)
{
  output_hand_res->lmk_result.clear();
  // // std::vector<Landmarks> landmarks;

  for (size_t i = 0; i < output_tensors.size(); i++)
  {
    output_tensors[i]->CACHE_INVALIDATE();
  }

  int batch_size = output_tensors.size() / 4;                                     // [1 ~ 10]
  int num_results = (*output_tensors[0]).properties.validShape.dimensionSize[1];  // 63
  int stride = (*output_tensors[0]).properties.stride[1] / sizeof(float);         // 1

  for (int b = 0; b < batch_size; b++)
  {
    Landmarks lmk;
    cv::Size img_size = cv::Size((*output_hand_res->valid_rois)[b].right - (*output_hand_res->valid_rois)[b].left,
                                 (*output_hand_res->valid_rois)[b].bottom - (*output_hand_res->valid_rois)[b].top);

    auto* point_output0 = output_tensors[b * 4 + 0]->GetTensorData<float>();
    auto* score_output1 = output_tensors[b * 4 + 1]->GetTensorData<float>();
    auto* left_right_output2 = output_tensors[b * 4 + 2]->GetTensorData<float>();
    // auto* output3 = output_tensors[b * 4 + 3]->GetTensorData<float>();
    for (int i = 0; i < num_results; i += 3)
    {
      // calc landmarks
      auto point_xy = Point(point_output0[i * stride + 0], point_output0[i * stride + 1]) / 224;
      point_xy = Point(point_xy.x * img_size.width, point_xy.y * img_size.height);  // scale to img size
      point_xy = point_xy + Point((*output_hand_res->valid_rois)[b].left,
                                  (*output_hand_res->valid_rois)[b].top);  // translate to roi
      lmk.emplace_back(point_xy);
    }
    parser_hand_lmk::HandLmkResult res;
    res.lmks = lmk;
    res.scores = score_output1[0];
    res.left_right = left_right_output2[0];
    output_hand_res->lmk_result.emplace_back(res);
  }

  return 0;
}

int NormalizeRoi(const hbDNNRoi* src, hbDNNRoi* dst, float norm_ratio, int32_t total_w, int32_t total_h)
{
  *dst = *src;
  // make sure dst left and top is not negative
  dst->left = dst->left <= 0 ? 0 : dst->left;
  dst->top = dst->top <= 0 ? 0 : dst->top;
  float box_w = dst->right - dst->left;
  float box_h = dst->bottom - dst->top;
  float center_x = (dst->left + dst->right) / 2.0f;
  float center_y = (dst->top + dst->bottom) / 2.0f;
  float w_new = box_w;
  float h_new = box_h;

  // {"norm_by_lside_ratio", NormMethod::BPU_MODEL_NORM_BY_LSIDE_RATIO},
  h_new = box_h * norm_ratio;
  w_new = box_w * norm_ratio;
  dst->left = center_x - w_new / 2;
  dst->right = center_x + w_new / 2;
  dst->top = center_y - h_new / 2;
  dst->bottom = center_y + h_new / 2;

  // dst->left = dst->left < 0 ? 0.0f : dst->left;
  // dst->top = dst->top < 0 ? 0.0f : dst->top;
  dst->right = dst->right > total_w ? total_w : dst->right;
  dst->bottom = dst->bottom > total_h ? total_h : dst->bottom;

  // roi's left and top must be even, right and bottom must be odd
  dst->left += (dst->left % 2 == 0 ? 0 : 1);
  dst->top += (dst->top % 2 == 0 ? 0 : 1);
  dst->right -= (dst->right % 2 == 1 ? 0 : 1);
  dst->bottom -= (dst->bottom % 2 == 1 ? 0 : 1);

  uint32_t roi_w = dst->right - dst->left;
  uint32_t roi_h = dst->bottom - dst->top;

  if (roi_w <= roi_w_size_max_ && roi_w >= roi_w_size_min_ && roi_h <= roi_h_size_max_ && roi_h >= roi_h_size_min_ &&
      dst->left >= 0 && dst->right <= total_w && dst->top >= 0 && dst->bottom <= total_h)
  {
    // check success
    return 0;
  }
  else
  {
    if (roi_w > roi_w_size_max_ || roi_h > roi_h_size_max_)
    {
      RCLCPP_ERROR(rclcpp::get_logger("mono2d_hand_lmk"), "Move far from sensor!");
    }
    else if (roi_w < roi_w_size_min_ || roi_h < roi_h_size_min_)
    {
      RCLCPP_ERROR(rclcpp::get_logger("mono2d_hand_lmk"), "Move close to sensor!");
    }

    return -1;
  }

  return 0;
}

cv::Rect calculate_tight_roi(const std::vector<Point>& landmarks)
{
  // if landmarks is empty, return a zero rect
  if (landmarks.empty())
  {
    return { 0, 0, 0, 0 };
  }

  // init a rect with the first landmark
  float x_min = landmarks[0].x;
  float x_max = landmarks[0].x;
  float y_min = landmarks[0].y;
  float y_max = landmarks[0].y;

  // find the min and max x and y values in the landmarks
  for (const auto& point : landmarks)
  {
    x_min = std::min(x_min, point.x);
    x_max = std::max(x_max, point.x);
    y_min = std::min(y_min, point.y);
    y_max = std::max(y_max, point.y);
  }

  int width = round(x_max - x_min);
  int height = round(y_max - y_min);

  return { static_cast<int>(x_min), static_cast<int>(y_min), width, height };
}

int calculate_tight_roi(const std::vector<Point>& landmarks, ai_msgs::msg::Roi& roi, float score)
{
  auto rect = calculate_tight_roi(landmarks);

  roi.set__type("tight_hand_lmks");
  roi.rect.set__x_offset(rect.x);
  roi.rect.set__y_offset(rect.y);
  roi.rect.set__width(rect.width);
  roi.rect.set__height(rect.height);
  roi.set__confidence(score);
  return 0;
}
