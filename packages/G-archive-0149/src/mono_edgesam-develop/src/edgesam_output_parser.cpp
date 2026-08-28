
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

#include "include/edgesam_output_parser.h"

int32_t EdgeSamOutputParser::Parse(
    std::shared_ptr<DnnParserResult> &result,
    const int resized_img_h,
    const int resized_img_w,
    std::vector<std::shared_ptr<DNNTensor>>& output_tensors,
    std::vector<std::vector<float>>& boxes) {

  if (!result) {
    result = std::make_shared<DnnParserResult>();
  }

  if (output_tensors.size() == 0) {
    return -1;
  }

  result->perception.type = Perception::SEG;
  int ret = -1;
  if (output_tensors.size() == 2) {
    output_tensors[0]->CACHE_INVALIDATE();
    output_tensors[1]->CACHE_INVALIDATE();
    if (output_tensors[1]->properties.quantiType == NONE) {
      float* masks = output_tensors[1]->GetTensorData<float>();
      ret = GenMask(masks, resized_img_h, resized_img_w, result->perception);
    } else if (output_tensors[0]->properties.quantiType == SCALE && 
                output_tensors[1]->properties.quantiType == SCALE) {
      int16_t* scores = output_tensors[0]->GetTensorData<int16_t>();
      int8_t* masks = output_tensors[1]->GetTensorData<int8_t>();
      ret = GenMaskScale(scores, masks, boxes, resized_img_h, resized_img_w, result->perception);
    }
  } else {
    if (output_tensors[1]->properties.quantiType == NONE) {
      ret = GenMultiMask(output_tensors, boxes, resized_img_h, resized_img_w, result->perception);
    } else if (output_tensors[0]->properties.quantiType == SCALE) {
      ret = GenMultiMaskScale(output_tensors, boxes, resized_img_h, resized_img_w, result->perception);
    }
  }

  if (ret != 0) {
    RCLCPP_ERROR(rclcpp::get_logger("sam ouput parser"),
                "postprocess return error, code = %d",
                ret);
  }

  return ret;
}

int32_t EdgeSamOutputParser::GenMask(const float* mask,
                                        const int resized_img_h,
                                        const int resized_img_w,
                                        Perception& perception) {
  int channel = 4;
  float valid_h_ratio = static_cast<float>(resized_img_h) / static_cast<float>(model_h_);
  float valid_w_ratio = static_cast<float>(resized_img_w) / static_cast<float>(model_w_);

  int valid_h = static_cast<int>(valid_h_ratio * output_height_);
  int valid_w = static_cast<int>(valid_w_ratio * output_width_);

  int stride = channel * output_height_ * output_width_;
  std::vector<cv::Mat> parsing_imgs;
  for (int n = 0; n < num_classes_; n++) {
    cv::Mat parsing_img(valid_h, valid_w, CV_32FC1);
    float *parsing_img_ptr = parsing_img.ptr<float>();
    for (int h = 0; h < valid_h; h++) {
      for (int w = 0; w < valid_w; w++) {
        int offect = h * output_width_ + w;
        const float* data = mask + n * stride + offect;
        *parsing_img_ptr++ = data[0];
      }
    }
    parsing_imgs.push_back(parsing_img);
  }

  valid_h = resized_img_h;
  valid_w = resized_img_w;
  cv::Size size(valid_w, valid_h);

  for (auto &parsing_img: parsing_imgs) {
    // resize parsing image
    cv::resize(parsing_img, parsing_img, size, 0, 0, cv::INTER_LINEAR);
  }

  perception.seg.data.resize(valid_h * valid_w);
  perception.seg.seg.resize(valid_h * valid_w);

  perception.seg.valid_h = valid_h;
  perception.seg.valid_w = valid_w;
  perception.seg.height = static_cast<int>(model_h_ * valid_h_ratio);
  perception.seg.width = static_cast<int>(model_w_ * valid_w_ratio);
  perception.seg.channel = channel;
  perception.seg.num_classes = num_classes_ + 1;

  for (int n = 0; n < num_classes_; n++) {
    auto &parsing_img = parsing_imgs[n];
    float *parsing_img_ptr = parsing_img.ptr<float>();
    for (int h = 0; h < valid_h; h++) {
      for (int w = 0; w < valid_w; w++) {
        int offect = h * valid_w + w;
        int top_index = -1;
        if (n == 0) {
          top_index = 0;
        }
        if (*parsing_img_ptr++ > mask_threshold_) {
          top_index = n + 1;
        }
        if (top_index != -1) {
          perception.seg.seg[h * valid_w + w] = top_index;
          perception.seg.data[h * valid_w + w] = static_cast<float>(top_index);
        }  
      }
    }
  }
  return 0;
}

int32_t EdgeSamOutputParser::GenMaskScale(const int16_t* scores,
                                        const int8_t* mask,
                                        const std::vector<std::vector<float>>& boxes,
                                        const int resized_img_h,
                                        const int resized_img_w,
                                        Perception& perception) {
  int channel = 4;

  float valid_h_ratio = static_cast<float>(resized_img_h) / static_cast<float>(model_h_);
  float valid_w_ratio = static_cast<float>(resized_img_w) / static_cast<float>(model_w_);

  int valid_h = static_cast<int>(valid_h_ratio * output_height_);
  int valid_w = static_cast<int>(valid_w_ratio * output_width_);

  int stride = output_height_ * output_width_;
  std::vector<cv::Mat> parsing_imgs;
  int num_classes = (num_classes_ < boxes.size()) ? num_classes_ : boxes.size();
  for (int n = 0; n < num_classes; n++) {

    int index = 0;
    int16_t score = 0;
    for (int i = 0; i < 4; i++) {
      if (scores[n * 8 + i] > score) {
        index = i;
        score = scores[n * 8 + i];
      }
    }
    cv::Mat parsing_img(valid_h, valid_w, CV_8UC1, cv::Scalar::all(0));
    int8_t *parsing_img_ptr = parsing_img.ptr<int8_t>();
    for (int h = 0; h < valid_h; h++) {
      for (int w = 0; w < valid_w; w++) {
        int offect = h * output_width_ + w;
        const int8_t* data = mask + (n * channel + index) * stride + offect;
        *parsing_img_ptr++ = data[0];
      }
    }
    parsing_imgs.push_back(parsing_img);
  }

  valid_h = resized_img_h;
  valid_w = resized_img_w;
  cv::Size size(valid_w, valid_h);

  for (auto &parsing_img: parsing_imgs) {
    // resize parsing image
    cv::resize(parsing_img, parsing_img, size, 0, 0, cv::INTER_LINEAR);
  }

  valid_h = valid_h;
  perception.seg.data.resize(valid_h * valid_w);
  perception.seg.seg.resize(valid_h * valid_w);

  perception.seg.valid_h = valid_h;
  perception.seg.valid_w = valid_w;
  perception.seg.height = static_cast<int>(model_h_ * valid_h_ratio);
  perception.seg.width = static_cast<int>(model_w_ * valid_w_ratio);
  perception.seg.channel = channel;
  perception.seg.num_classes = num_classes + 1;

  for (int n = 0; n < num_classes; n++) {
    auto &parsing_img = parsing_imgs[n];
    int8_t *parsing_img_ptr = parsing_img.ptr<int8_t>();
    for (int h = 0; h < valid_h; h++) {
      for (int w = 0; w < valid_w; w++) {
        int offect = h * valid_w + w;
        int top_index = -1;
        if (n == 0) {
          top_index = 0;
        }
        if (*parsing_img_ptr++ > 0) {
          top_index = n + 1;
        }
        if (top_index != -1) {
          perception.seg.seg[offect] = top_index;
          perception.seg.data[offect] = static_cast<float>(top_index);
        }  
      }
    }
  }

  return 0;
}

int32_t EdgeSamOutputParser::GenMultiMask(std::vector<std::shared_ptr<DNNTensor>>& output_tensors,
                                              const std::vector<std::vector<float>>& boxes,
                                              const int resized_img_h,
                                              const int resized_img_w,
                                              Perception& perception) {
  int channel = 4;

  float valid_h_ratio = static_cast<float>(resized_img_h) / static_cast<float>(model_h_);
  float valid_w_ratio = static_cast<float>(resized_img_w) / static_cast<float>(model_w_);

  int valid_h = static_cast<int>(valid_h_ratio * output_height_);
  int valid_w = static_cast<int>(valid_w_ratio * output_width_);

  int stride = output_height_ * output_width_;
  std::vector<cv::Mat> parsing_imgs;
  int num_classes = boxes.size();

  for (int i = 0; i < boxes.size(); i++) {
    output_tensors[i * 2]->CACHE_INVALIDATE();
    output_tensors[i * 2 + 1]->CACHE_INVALIDATE();
    float* scores = output_tensors[i * 2]->GetTensorData<float>();
    float* mask = output_tensors[i * 2 + 1]->GetTensorData<float>();

    int index = 0;
    float score = 0;
    for (int i = 0; i < 4; i++) {
      if (scores[i] > score) {
        index = i;
        score = scores[i];
      }
    }
    cv::Mat parsing_img(valid_h, valid_w, CV_32FC1, cv::Scalar::all(0));
    float *parsing_img_ptr = parsing_img.ptr<float>();
    for (int h = 0; h < valid_h; h++) {
      for (int w = 0; w < valid_w; w++) {
        int offect = h * output_width_ + w;
        const float* data = mask + index * stride + offect;
        *parsing_img_ptr++ = data[0];
      }
    }
    parsing_imgs.push_back(parsing_img);
  }

  valid_h = resized_img_h;
  valid_w = resized_img_w;
  cv::Size size(valid_w, valid_h);

  for (auto &parsing_img: parsing_imgs) {
    // resize parsing image
    cv::resize(parsing_img, parsing_img, size, 0, 0, cv::INTER_LINEAR);
  }

  valid_h = valid_h;
  perception.seg.data.resize(valid_h * valid_w);
  perception.seg.seg.resize(valid_h * valid_w);

  perception.seg.valid_h = valid_h;
  perception.seg.valid_w = valid_w;
  perception.seg.height = static_cast<int>(model_h_ * valid_h_ratio);
  perception.seg.width = static_cast<int>(model_w_ * valid_w_ratio);
  perception.seg.channel = channel;
  perception.seg.num_classes = num_classes + 1;

  for (int n = 0; n < num_classes; n++) {
    auto &parsing_img = parsing_imgs[n];
    float *parsing_img_ptr = parsing_img.ptr<float>();
    for (int h = 0; h < valid_h; h++) {
      for (int w = 0; w < valid_w; w++) {
        int offect = h * valid_w + w;
        int top_index = -1;
        if (n == 0) {
          top_index = 0;
        }
        if (*parsing_img_ptr++ > mask_threshold_) {
          top_index = n + 1;
        }
        if (top_index != -1) {
          perception.seg.seg[offect] = top_index;
          perception.seg.data[offect] = static_cast<float>(top_index);
        }  
      }
    }
  }

  return 0;
}

int32_t EdgeSamOutputParser::GenMultiMaskScale(std::vector<std::shared_ptr<DNNTensor>>& output_tensors,
                                              const std::vector<std::vector<float>>& boxes,
                                              const int resized_img_h,
                                              const int resized_img_w,
                                              Perception& perception) {
  int channel = 4;

  float valid_h_ratio = static_cast<float>(resized_img_h) / static_cast<float>(model_h_);
  float valid_w_ratio = static_cast<float>(resized_img_w) / static_cast<float>(model_w_);

  int valid_h = static_cast<int>(valid_h_ratio * output_height_);
  int valid_w = static_cast<int>(valid_w_ratio * output_width_);

  int stride = output_height_ * output_width_;
  std::vector<cv::Mat> parsing_imgs;
  int num_classes = boxes.size();

  for (int i = 0; i < boxes.size(); i++) {
    output_tensors[i * 2]->CACHE_INVALIDATE();
    output_tensors[i * 2 + 1]->CACHE_INVALIDATE();
    int16_t* scores = output_tensors[i * 2]->GetTensorData<int16_t>();
    int8_t* mask = output_tensors[i * 2 + 1]->GetTensorData<int8_t>();

    int index = 0;
    int16_t score = 0;
    for (int i = 0; i < 4; i++) {
      if (scores[i] > score) {
        index = i;
        score = scores[i];
      }
    }
    cv::Mat parsing_img(valid_h, valid_w, CV_8UC1, cv::Scalar::all(0));
    int8_t *parsing_img_ptr = parsing_img.ptr<int8_t>();
    for (int h = 0; h < valid_h; h++) {
      for (int w = 0; w < valid_w; w++) {
        int offect = h * output_width_ + w;
        const int8_t* data = mask + index * stride + offect;
        *parsing_img_ptr++ = data[0];
      }
    }
    parsing_imgs.push_back(parsing_img);
  }

  valid_h = resized_img_h;
  valid_w = resized_img_w;
  cv::Size size(valid_w, valid_h);

  for (auto &parsing_img: parsing_imgs) {
    // resize parsing image
    cv::resize(parsing_img, parsing_img, size, 0, 0, cv::INTER_LINEAR);
  }

  valid_h = valid_h;
  perception.seg.data.resize(valid_h * valid_w);
  perception.seg.seg.resize(valid_h * valid_w);

  perception.seg.valid_h = valid_h;
  perception.seg.valid_w = valid_w;
  perception.seg.height = static_cast<int>(model_h_ * valid_h_ratio);
  perception.seg.width = static_cast<int>(model_w_ * valid_w_ratio);
  perception.seg.channel = channel;
  perception.seg.num_classes = num_classes + 1;

  for (int n = 0; n < num_classes; n++) {
    auto &parsing_img = parsing_imgs[n];
    int8_t *parsing_img_ptr = parsing_img.ptr<int8_t>();
    for (int h = 0; h < valid_h; h++) {
      for (int w = 0; w < valid_w; w++) {
        int offect = h * valid_w + w;
        int top_index = -1;
        if (n == 0) {
          top_index = 0;
        }
        if (*parsing_img_ptr++ > 0) {
          top_index = n + 1;
        }
        if (top_index != -1) {
          perception.seg.seg[offect] = top_index;
          perception.seg.data[offect] = static_cast<float>(top_index);
        }  
      }
    }
  }

  return 0;
}

int RenderSeg(cv::Mat &mat, Parsing &seg, std::string& saving_path) {
  static uint8_t bgr_putpalette[] = {
      0, 0, 0, 128, 64,  128, 244, 35,  232, 70,  70,  70,  102, 102, 156, 190, 153, 153,
      153, 153, 153, 250, 170, 30,  220, 220, 0,   107, 142, 35,  152, 251, 152,
      0,   130, 180, 220, 20,  60,  255, 0,   0,   0,   0,   142, 0,   0,   70,
      0,   60,  100, 0,   80,  100, 0,   0,   230, 119, 11,  32};

  int parsing_width = seg.valid_w;
  int parsing_height = seg.valid_h;
  cv::Mat parsing_img(parsing_height, parsing_width, CV_8UC3);
  uint8_t *parsing_img_ptr = parsing_img.ptr<uint8_t>();

  for (int h = 0; h < parsing_height; ++h) {
    for (int w = 0; w < parsing_width; ++w) {
      auto id = seg.seg[h * parsing_width + w];
      *parsing_img_ptr++ = bgr_putpalette[id * 3];
      *parsing_img_ptr++ = bgr_putpalette[id * 3 + 1];
      *parsing_img_ptr++ = bgr_putpalette[id * 3 + 2];
    }
  }

  // resize parsing image
  cv::resize(parsing_img, parsing_img, mat.size(), 0, 0);

  // alpha blending
  float alpha_f = 0.5;
  cv::Mat dst;
  addWeighted(mat, alpha_f, parsing_img, 1 - alpha_f, 0.0, dst);
  mat = std::move(dst);

  RCLCPP_INFO(rclcpp::get_logger("sam ouput parser"),
              "Draw result to file: %s",
              saving_path.c_str());
  cv::imwrite(saving_path, mat);
  return 0;
}