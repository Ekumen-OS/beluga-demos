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

#ifndef MH_AMCL__MAPMATCHER_HPP_
#define MH_AMCL__MAPMATCHER_HPP_

#include <list>
#include <memory>
#include <vector>

#include <beluga_ros/occupancy_grid.hpp>

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2/transform_datatypes.h>

namespace mh_amcl
{

  /**
   * @brief Struct containing a transform and its associated weight
   */
  struct TransformWeighted
  {
    float weight;
    tf2::Transform transform;
  };

  /**
   * @brief Overload of the operator '<' to be able to compare TransformWeighted
   * structs
   */
  bool operator<(const TransformWeighted &tw1, const TransformWeighted &tw2);

  /**
   * MapMatcher class
   *
   * Performs the multi-resolution map matching process. It receives an occupancy
   * grid map and laser scan data to find transformations (poses) that align the
   * scan data with the map. It weights all potential transformations to select
   * the higher quality candidates.
   */
  class MapMatcher
  {
  public:
    explicit MapMatcher(std::shared_ptr<const nav_msgs::msg::OccupancyGrid> map);

    /**
     * @brief Get the candidate transformations from the laser information and the
     * map
     */
    std::vector<TransformWeighted>
    get_matches(const sensor_msgs::msg::LaserScan &scan);

  protected:
    static const int NUM_LEVEL_SCALE_COSTMAP = 4;

    /**
     * @brief Reduce the resolution of the map to half to perform the
     * multi-resolution map-matching
     */
    std::shared_ptr<beluga_ros::OccupancyGrid>
    half_scale(std::shared_ptr<beluga_ros::OccupancyGrid> costmap_in);

    /**
     * @brief Transform the LaserScan message in a vector of 3D points
     */
    std::vector<tf2::Vector3>
    laser2points(const sensor_msgs::msg::LaserScan &scan);

    // Auxiliary methods to get the matches between the laser data and the map
    std::vector<TransformWeighted>
    get_matches(int scale, const std::vector<tf2::Vector3> &scan, float min_x,
                float min_y, float max_y, float max_x);
    float match(int scale, std::shared_ptr<beluga_ros::OccupancyGrid> costmap,
                const std::vector<tf2::Vector3> &scan, const tf2::Transform &transform);

    // Current costmaps
    std::vector<std::shared_ptr<beluga_ros::OccupancyGrid>> costmaps_;
  };

  /**
   * @brief wrapper to transform the costmap to ROS Occupancy grid message
   */
  nav_msgs::msg::OccupancyGrid
  toMsg(std::shared_ptr<beluga_ros::OccupancyGrid> costmap);

} // namespace mh_amcl

#endif // MH_AMCL__MAPMATCHER_HPP_
