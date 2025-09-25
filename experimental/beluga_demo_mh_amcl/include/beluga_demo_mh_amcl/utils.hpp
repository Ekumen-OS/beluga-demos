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

#ifndef BELUGA_UTILS_HPP
#define BELUGA_UTILS_HPP

#include <sophus/se2.hpp>

#include <beluga_ros/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/color_rgba.hpp>

namespace utils
{

    // Define constants for occupancy values
    constexpr int8_t LETHAL_OBSTACLE =
        beluga_ros::OccupancyGrid::ValueTraits::kOccupiedValue;
    constexpr int8_t FREE_SPACE =
        beluga_ros::OccupancyGrid::ValueTraits::kFreeValue;
    constexpr int8_t NO_INFORMATION =
        beluga_ros::OccupancyGrid::ValueTraits::kUnknownValue;

    // Define auxiliary constant
    constexpr float INV_SQRT_2PI = 0.3989422804014327;

    /**
     * @brief Enum defined including the different colors for visualization of the
     * hypotheses
     */
    typedef enum TColor
    {
        RED,
        GREEN,
        BLUE,
        WHITE,
        GREY,
        DARK_GREY,
        BLACK,
        YELLOW,
        ORANGE,
        BROWN,
        PINK,
        LIME_GREEN,
        PURPLE,
        CYAN,
        MAGENTA,
        NUM_COLORS
    } Color;

    /**
     * @brief Function to get the ROS message for the chosen color
     */
    std_msgs::msg::ColorRGBA getColor(Color color_id, double alpha = 1.0);

    /**
     * @brief Function to transform the coordinates of a point from the local grid
     * frame to the global frame
     */
    std::tuple<double, double>
    mapToWorld(std::shared_ptr<beluga_ros::OccupancyGrid> costmap, int local_x,
               int local_y);

    /**
     * @brief Function to transform the coordinates of a point from global frame to
     * local grid frame, without checking if the computed indices are within the
     * grid boundaries.
     */
    std::tuple<int, int>
    worldToMapNoBounds(std::shared_ptr<beluga_ros::OccupancyGrid> costmap,
                       double world_x, double world_y);

    /**
     * @brief Function to transform the coordinates of a point from global frame to
     * local grid frame, checking if the computed indices are within the grid
     * boundaries. If the computed indices are out of bounds, they are clamped to
     * the closest valid index.
     */
    std::tuple<int, int>
    worldToMapEnforceBounds(std::shared_ptr<beluga_ros::OccupancyGrid> costmap,
                            double world_x, double world_y);

    /**
     * @brief extracts the X, Y coordinates and the yaw angle from a Sophus::SE2d
     * pose
     */
    std::tuple<double, double, double>
    extractTranslationAndYaw(const Sophus::SE2d &se2_pose);

    /**
     * @brief Convert a tf2::Transform to a Sophus::SE2d.
     *
     * Extracts the translation (x, y) and yaw from the tf2::Transform.
     */
    Sophus::SE2d tf2TransformToSE2d(const tf2::Transform &tf);

    /**
     * @brief Convert a Sophus::SE2d pose to a tf2::Transform.
     *
     * Creates a tf2::Transform from the given SE2 pose, using the yaw angle
     * and the translation (x, y). The z component is set to 0.
     */
    tf2::Transform se2dToTf2Transform(const Sophus::SE2d &se2);

    tf2::Transform get_transform_to_read(const sensor_msgs::msg::LaserScan &scan,
                                         int index);

    // Auxiliar functions to compute the steps of the algorithm
    double get_error_distance_to_obstacle(
        const tf2::Transform &map2bf, const tf2::Transform &bf2laser,
        const tf2::Transform &laser2point,
        const sensor_msgs::msg::LaserScan &scan,
        std::shared_ptr<beluga_ros::OccupancyGrid> costmap);

    signed char get_cost(const tf2::Transform &transform,
                         std::shared_ptr<beluga_ros::OccupancyGrid> costmap);

    signed char get_cost(const geometry_msgs::msg::Pose &pose,
                         std::shared_ptr<beluga_ros::OccupancyGrid> costmap);

    void get_distances(const geometry_msgs::msg::Pose &pose1,
                       const geometry_msgs::msg::Pose &pose2, double &dist_xy,
                       double &dist_theta);

    double normalize_angle(double angle);

} // namespace utils

#endif // BELUGA_UTILS_HPP
