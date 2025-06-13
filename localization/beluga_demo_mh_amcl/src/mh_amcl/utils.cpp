// Copyright 2025 Ekumen, Inc.
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

#include "beluga_demo_mh_amcl/utils.hpp"

namespace utils {

std_msgs::msg::ColorRGBA getColor(Color color_id, double alpha) {
  std_msgs::msg::ColorRGBA color;

  switch (color_id) {
  case RED:
    color.r = 0.8;
    color.g = 0.1;
    color.b = 0.1;
    color.a = alpha;
    break;
  case GREEN:
    color.r = 0.1;
    color.g = 0.8;
    color.b = 0.1;
    color.a = alpha;
    break;
  case BLUE:
    color.r = 0.1;
    color.g = 0.1;
    color.b = 0.8;
    color.a = alpha;
    break;
  case WHITE:
    color.r = 1.0;
    color.g = 1.0;
    color.b = 1.0;
    color.a = alpha;
    break;
  case GREY:
    color.r = 0.9;
    color.g = 0.9;
    color.b = 0.9;
    color.a = alpha;
    break;
  case DARK_GREY:
    color.r = 0.6;
    color.g = 0.6;
    color.b = 0.6;
    color.a = alpha;
    break;
  case BLACK:
    color.r = 0.0;
    color.g = 0.0;
    color.b = 0.0;
    color.a = alpha;
    break;
  case YELLOW:
    color.r = 1.0;
    color.g = 1.0;
    color.b = 0.0;
    color.a = alpha;
    break;
  case ORANGE:
    color.r = 1.0;
    color.g = 0.5;
    color.b = 0.0;
    color.a = alpha;
    break;
  case BROWN:
    color.r = 0.597;
    color.g = 0.296;
    color.b = 0.0;
    color.a = alpha;
    break;
  case PINK:
    color.r = 1.0;
    color.g = 0.4;
    color.b = 1;
    color.a = alpha;
    break;
  case LIME_GREEN:
    color.r = 0.6;
    color.g = 1.0;
    color.b = 0.2;
    color.a = alpha;
    break;
  case PURPLE:
    color.r = 0.597;
    color.g = 0.0;
    color.b = 0.597;
    color.a = alpha;
    break;
  case CYAN:
    color.r = 0.0;
    color.g = 1.0;
    color.b = 1.0;
    color.a = alpha;
    break;
  case MAGENTA:
    color.r = 1.0;
    color.g = 0.0;
    color.b = 1.0;
    color.a = alpha;
    break;
  }
  return color;
}

std::tuple<double, double>
mapToWorld(std::shared_ptr<beluga_ros::OccupancyGrid> costmap,
           int local_x,
           int local_y)
{
  // map origin in world frame
  double origin_x = costmap->origin().translation().x();
  double origin_y = costmap->origin().translation().y();
  double res     = costmap->resolution();

  // use the origin + the center of the cell (local + 0.5) times the resolution of each cell
  double world_x = origin_x + (local_x + 0.5) * res;
  double world_y = origin_y + (local_y + 0.5) * res;
  return std::make_tuple(world_x, world_y);
}

std::tuple<int, int>
worldToMapNoBounds(std::shared_ptr<beluga_ros::OccupancyGrid> costmap,
                   double world_x,
                   double world_y)
{
  double origin_x = costmap->origin().translation().x();
  double origin_y = costmap->origin().translation().y();
  double res     = costmap->resolution();

  // subtract origin, divide by resolution, floor to get cell index
  int local_x = static_cast<int>(std::floor((world_x - origin_x) / res));
  int local_y = static_cast<int>(std::floor((world_y - origin_y) / res));
  return std::make_tuple(local_x, local_y);
}

std::tuple<int, int>
worldToMapEnforceBounds(std::shared_ptr<beluga_ros::OccupancyGrid> costmap,
                        double world_x,
                        double world_y)
{
  double origin_x = costmap->origin().translation().x();
  double origin_y = costmap->origin().translation().y();
  double res     = costmap->resolution();
  int width      = costmap->width();
  int height     = costmap->height();

  // again subtract origin, divide by resolution, floor to get cell index
  int local_x = static_cast<int>(std::floor((world_x - origin_x) / res));
  int local_y = static_cast<int>(std::floor((world_y - origin_y) / res));

  // if completely before the map origin, snap to (0,0)
  if (world_x < origin_x || world_y < origin_y) {
    local_x = 0;
    local_y = 0;
  }

  // clamp into [0_width-1], [0_height-1]
  local_x = std::clamp(local_x, 0, width  - 1);
  local_y = std::clamp(local_y, 0, height - 1);

  return std::make_tuple(local_x, local_y);
}

std::tuple<double, double, double>
extractTranslationAndYaw(const Sophus::SE2d &se2_pose) {
  Eigen::Vector2d translation = se2_pose.translation();
  double yaw = se2_pose.so2().log();

  return std::make_tuple(translation.x(), translation.y(), yaw);
}

Sophus::SE2d tf2TransformToSE2d(const tf2::Transform &tf) {
  // Extract the translation (we ignore the z component for 2D)
  tf2::Vector3 origin = tf.getOrigin();
  double x = origin.x();
  double y = origin.y();

  // Extract the yaw angle from the rotation quaternion.
  double roll, pitch, yaw;
  tf.getBasis().getRPY(roll, pitch, yaw);

  // Construct and return the SE2 pose
  return Sophus::SE2d(yaw, Eigen::Vector2d(x, y));
}

tf2::Transform se2dToTf2Transform(const Sophus::SE2d &se2) {
  // Get translation and yaw from the SE2 pose
  Eigen::Vector2d t = se2.translation();
  double yaw = se2.so2().log();

  // Create a tf2 quaternion representing the rotation (only yaw is nonzero)
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw);

  // Construct the tf2::Transform
  tf2::Transform tf;
  tf.setOrigin(tf2::Vector3(t.x(), t.y(), 0.0));
  tf.setRotation(q);
  return tf;
}

double weighted_mean(const std::vector<double> &vector,
                     const std::vector<double> &weights) {
  if (vector.empty() || vector.size() != weights.size()) {
    return 0.0;
  }

  double weights_sum = 0.0;
  for (int i = 0; i < vector.size(); i++) {
    weights_sum = weights_sum + vector[i] * weights[i];
  }
  return weights_sum;
}

double angle_weighted_mean(const std::vector<double> &vector,
                           const std::vector<double> &weights) {
  if (vector.empty() || vector.size() != weights.size()) {
    return 0.0;
  }

  double x = 0.0;
  double y = 0.0;
  for (int i = 0; i < vector.size(); i++) {
    x += weights[i] * cos(vector[i]);
    y += weights[i] * sin(vector[i]);
  }

  return atan2(y, x);
}

double angle_mean(const std::vector<double> &vector) {
  if (vector.empty()) {
    return 0.0;
  }

  double x = 0.0;
  double y = 0.0;
  for (const auto &val : vector) {
    x += cos(val);
    y += sin(val);
  }

  return atan2(y, x);
}

double mean(const std::vector<double> &vector) {
  if (vector.empty()) {
    return 0.0;
  }
  return std::accumulate(vector.begin(), vector.end(), 0.0) / vector.size();
}

double covariance(const std::vector<double> &v1, const std::vector<double> &v2,
                  bool v1_is_angle, bool v2_is_angle) {
  assert(v1.size() == v2.size());

  if (v1.size() < 2) {
    return 0.0;
  }

  double mean_v1, mean_v2;
  if (v1_is_angle) {
    mean_v1 = angle_mean(v1);
  } else {
    mean_v1 = mean(v1);
  }

  if (v2_is_angle) {
    mean_v2 = angle_mean(v2);
  } else {
    mean_v2 = mean(v2);
  }

  double sum = 0.0;

  for (int i = 0; i < v1.size(); i++) {
    sum += (v1[i] - mean_v1) * (v2[i] - mean_v2);
  }

  return sum / static_cast<double>(v1.size() - 1.0);
}

double normalize_angle(double angle) {
  while (angle > M_PI) {
    angle = angle - 2.0 * M_PI;
  }
  while (angle < -M_PI) {
    angle = angle + 2.0 * M_PI;
  }
  return angle;
}

} // namespace utils
