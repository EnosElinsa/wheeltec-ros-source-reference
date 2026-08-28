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

#include "include/post_process/palm_det_parser.h"

int32_t PalmDetParse(const std::vector<std::shared_ptr<DNNTensor>>& output_tensors,
                     std::shared_ptr<PalmDetResult>& output_palm_res, const int scale,
                     const int min_score, const int iou_thr)
{
  if (!output_palm_res)
  {
    output_palm_res = std::make_shared<PalmDetResult>();
    output_palm_res->Reset();
  }

  std::vector<cv::Rect> nms_bbox;
  std::vector<PalmRect> bboxes, output_bbox;
  std::vector<Landmarks> landmarks, output_landmarks;

  // get tensor from bpu
  output_tensors[0]->CACHE_INVALIDATE();
  output_tensors[1]->CACHE_INVALIDATE();
  // change tensor to float
  auto* output0 = output_tensors[0]->GetTensorData<float>();
  auto* output1 = output_tensors[1]->GetTensorData<float>();
  // get tensor shape
  int num_results = (*output_tensors[0]).properties.validShape.dimensionSize[1];
  int num_features = (*output_tensors[0]).properties.validShape.dimensionSize[2];
  int stride = (*output_tensors[0]).properties.stride[1] / (*output_tensors[0]).properties.stride[2];

  for (int i = 0; i < num_results; ++i)
  {
    // parse bboxs
    auto cxy_delta = Point(output0[i * stride + 0] / 192, output0[i * stride + 1] / 192);
    auto wh_delta = Point(output0[i * stride + 2] / 192, output0[i * stride + 3] / 192);
    auto xy1 = cxy_delta - wh_delta / 2.0 + anchors[i];
    auto xy2 = cxy_delta + wh_delta / 2.0 + anchors[i];
    auto bbox = PalmRect(xy1.x, xy1.y, xy2.x, xy2.y);
    bbox.scale(scale, scale);
    nms_bbox.push_back(cv::Rect(bbox.left, bbox.top, bbox.right - bbox.left, bbox.bottom - bbox.top));
    bboxes.push_back(bbox);

    // parse landmarks
    Landmarks lmk;
    for (int j = 4; j < num_features; j += 2)
    {
      auto point = Point(output0[i * stride + j], output0[i * stride + j + 1]);
      point = point / 192;
      point = point + anchors[i];
      lmk.push_back(point * scale);
    }
    landmarks.push_back(lmk);
  }

  num_results = (*output_tensors[1]).properties.validShape.dimensionSize[1];
  stride = (*output_tensors[1]).properties.stride[1] / (*output_tensors[1]).properties.stride[2];
  std::vector<float> scores;
  // parse scores
  for (int i = 0; i < num_results; ++i)
  {
    auto score = 1.0f / (1.0f + std::exp(-output1[i]));
    bboxes[i].conf = score;
    scores.push_back(score);
  }

  std::vector<int> indices;
  // do nms for palm detections
  cv::dnn::NMSBoxes(nms_bbox, scores, min_score, iou_thr, indices, 1.f, 10);
  output_palm_res->boxes = bboxes;
  output_palm_res->lmks = landmarks;
  output_palm_res->indices = indices;

  return 0;
}
