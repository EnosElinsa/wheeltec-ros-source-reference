#include <vector>
#include "dnn_node/util/output_parser/detection/fasterrcnn_output_parser.h"

#ifndef PALM_DET_ANCHORS_H
#define PALM_DET_ANCHORS_H
using hobot::dnn_node::parser_fasterrcnn::Point;

extern std::vector<Point> anchors;

extern const std::vector<std::vector<float>> anchors_;

#endif
