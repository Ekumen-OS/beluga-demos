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

namespace utils
{

  std_msgs::msg::ColorRGBA getColor(Color color_id, double alpha)
  {
    std_msgs::msg::ColorRGBA color;

    switch (color_id)
    {
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
    double res = costmap->resolution();

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
    double res = costmap->resolution();

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
    double res = costmap->resolution();
    int width = costmap->width();
    int height = costmap->height();

    // again subtract origin, divide by resolution, floor to get cell index
    int local_x = static_cast<int>(std::floor((world_x - origin_x) / res));
    int local_y = static_cast<int>(std::floor((world_y - origin_y) / res));

    // if completely before the map origin, snap to (0,0)
    if (world_x < origin_x || world_y < origin_y)
    {
      local_x = 0;
      local_y = 0;
    }

    // clamp into [0_width-1], [0_height-1]
    local_x = std::clamp(local_x, 0, width - 1);
    local_y = std::clamp(local_y, 0, height - 1);

    return std::make_tuple(local_x, local_y);
  }

  std::tuple<double, double, double>
  extractTranslationAndYaw(const Sophus::SE2d &se2_pose)
  {
    Eigen::Vector2d translation = se2_pose.translation();
    double yaw = se2_pose.so2().log();

    return std::make_tuple(translation.x(), translation.y(), yaw);
  }

  Sophus::SE2d tf2TransformToSE2d(const tf2::Transform &tf)
  {
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

  tf2::Transform se2dToTf2Transform(const Sophus::SE2d &se2)
  {
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

  tf2::Transform get_transform_to_read(
      const sensor_msgs::msg::LaserScan &scan, int index)
  {
    double dist = scan.ranges[index];
    double angle =
        scan.angle_min + static_cast<double>(index) * scan.angle_increment;

    tf2::Transform transformation;

    double x = dist * cos(angle);
    double y = dist * sin(angle);

    transformation.setOrigin({x, y, 0.0});
    transformation.setRotation({0.0, 0.0, 0.0, 1.0});

    return transformation;
  }

  double get_error_distance_to_obstacle(
      const tf2::Transform &map2bf, const tf2::Transform &bf2laser,
      const tf2::Transform &laser2point, const sensor_msgs::msg::LaserScan &scan,
      std::shared_ptr<beluga_ros::OccupancyGrid> costmap,
      double distance_perception_error)
  {
    if (std::isinf(laser2point.getOrigin().x()) ||
        std::isnan(laser2point.getOrigin().x()))
    {
      return std::numeric_limits<double>::infinity();
    }

    tf2::Transform map2laser = map2bf * bf2laser;
    tf2::Transform map2point = map2laser * laser2point;
    tf2::Transform map2point_aux = map2point;
    tf2::Transform uvector;
    tf2::Vector3 unit =
        laser2point.getOrigin() / laser2point.getOrigin().length();

    if (get_cost(map2point, costmap) == utils::LETHAL_OBSTACLE)
    {
      return 0.0;
    }

    float dist = costmap->resolution();
    while (dist < (3.0 * distance_perception_error))
    {
      uvector.setOrigin(unit * dist);
      // For positive
      map2point = map2point_aux * uvector;
      auto cost = get_cost(map2point, costmap);

      if (cost == utils::LETHAL_OBSTACLE)
      {
        return dist;
      }

      // For negative
      uvector.setOrigin(uvector.getOrigin() * -1.0);
      map2point = map2point_aux * uvector;
      cost = get_cost(map2point, costmap);

      if (cost == utils::LETHAL_OBSTACLE)
      {
        return dist;
      }
      dist = dist + costmap->resolution();
    }

    return std::numeric_limits<double>::infinity();
  }

  signed char get_cost(
      const tf2::Transform &transform,
      std::shared_ptr<beluga_ros::OccupancyGrid> costmap)
  {
    auto [local_x, local_y] = utils::worldToMapNoBounds(
        costmap, transform.getOrigin().x(), transform.getOrigin().y());

    if (local_x >= 0 && local_y >= 0 && local_x < costmap->width() &&
        local_y < costmap->height() &&
        costmap->data_at(local_x, local_y).has_value())
    {
      return costmap->data_at(local_x, local_y).value();
    }
    else
    {
      return utils::NO_INFORMATION;
    }
  }

  signed char get_cost(
      const geometry_msgs::msg::Pose &pose,
      std::shared_ptr<beluga_ros::OccupancyGrid> costmap)
  {
    // Get the corresponding cost from the costmap for a specific pose
    auto [local_x, local_y] =
        utils::worldToMapNoBounds(costmap, pose.position.x, pose.position.y);

    std::optional<signed char> opt = costmap->data_at(local_x, local_y);
    if (opt.has_value())
    {
      return static_cast<unsigned char>(opt.value());
    }
    else
    {
      return utils::NO_INFORMATION;
    }
  }

  void get_distances(const geometry_msgs::msg::Pose &pose1,
                     const geometry_msgs::msg::Pose &pose2,
                     double &dist_xy, double &dist_theta)
  {
    double diff_x = pose1.position.x - pose2.position.x;
    double diff_y = pose1.position.y - pose2.position.y;
    dist_xy = sqrt(diff_x * diff_x + diff_y * diff_y);

    tf2::Quaternion q1(pose1.orientation.x, pose1.orientation.y,
                       pose1.orientation.z, pose1.orientation.w);
    tf2::Quaternion q2(pose2.orientation.x, pose2.orientation.y,
                       pose2.orientation.z, pose2.orientation.w);

    double roll1, pitch1, yaw1;
    double roll2, pitch2, yaw2;
    tf2::Matrix3x3(q1).getRPY(roll1, pitch1, yaw1);
    tf2::Matrix3x3(q2).getRPY(roll2, pitch2, yaw2);

    dist_theta = fabs(atan2(sin(yaw1 - yaw2), cos(yaw1 - yaw2)));
  }

  double normalize_angle(double angle)
  {
    while (angle > M_PI)
    {
      angle = angle - 2.0 * M_PI;
    }
    while (angle < -M_PI)
    {
      angle = angle + 2.0 * M_PI;
    }
    return angle;
  }

} // namespace utils
