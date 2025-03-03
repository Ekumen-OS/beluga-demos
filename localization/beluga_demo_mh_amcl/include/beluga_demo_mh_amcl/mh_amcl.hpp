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

#ifndef MH_AMCL__MH_AMCL_HPP_
#define MH_AMCL__MH_AMCL_HPP_

#include <list>
#include <memory>
#include <mutex>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/LU>

// The particle traits need to be included before the particle_cloud
#include "beluga_demo_mh_amcl/mh_amcl_particle_traits.hpp"
#include <beluga_ros/particle_cloud.hpp>

#include <bondcpp/bond.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <message_filters/subscriber.h>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2/transform_datatypes.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include "beluga_demo_mh_amcl/map_matcher.hpp"
#include "beluga_demo_mh_amcl/particles_distribution.hpp"

namespace mh_amcl {

// Alias for complex types
using CallbackReturnT =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
using LaserScanSubscriber =
    message_filters::Subscriber<sensor_msgs::msg::LaserScan, rclcpp_lifecycle::LifecycleNode>;
using LaserScanFilter = tf2_ros::MessageFilter<sensor_msgs::msg::LaserScan>;

/**
 * MH_AMCL_Node class
 *
 * Serves as the main lifecycle node, integrating all the components.
 * Instantiates a list of 'ParticlesDistribution', each element representing a
 * set of particles (hypothesis), and the map matcher.
 */
class MH_AMCL_Node : public rclcpp_lifecycle::LifecycleNode {
public:
  explicit MH_AMCL_Node(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

  // Initialization
  void init();

  // Lifecycle node methods
  CallbackReturnT on_configure(const rclcpp_lifecycle::State &state) override;
  CallbackReturnT on_activate(const rclcpp_lifecycle::State &state) override;
  CallbackReturnT on_deactivate(const rclcpp_lifecycle::State &state) override;
  CallbackReturnT on_cleanup(const rclcpp_lifecycle::State &state) override;
  CallbackReturnT on_shutdown(const rclcpp_lifecycle::State &state) override;
  CallbackReturnT on_error(const rclcpp_lifecycle::State &state) override;

protected:
  // Steps of the MH-AMCL algorithm
  void predict();
  void correct();
  void reseed();
  void manage_hypotheses();

  // Methods to publish the results of the algorithm (for localization and for
  // visualization)
  void publish_particles();
  void publish_position();

  // Auxiliar methods
  void get_distances(const geometry_msgs::msg::Pose &pose1,
                     const geometry_msgs::msg::Pose &pose2, double &dist_xy,
                     double &dist_theta);
  signed char get_cost(const geometry_msgs::msg::Pose &pose);
  geometry_msgs::msg::Pose toMsg(const tf2::Transform &tf);

private:
  // Publishers and subscribers
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
      init_pose_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
      pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr particles_pub_;

  // Message filter for the laser subscription
  std::unique_ptr<LaserScanSubscriber> laser_sub_;
  std::unique_ptr<LaserScanFilter> laser_scan_filter_;
  message_filters::Connection laser_scan_connection_;

  // Bond object to use with the lifecycle manager
  std::unique_ptr<bond::Bond> bond_;

  // Timers
  rclcpp::TimerBase::SharedPtr predict_timer_;
  rclcpp::TimerBase::SharedPtr correct_timer_;
  rclcpp::TimerBase::SharedPtr reseed_timer_;
  rclcpp::TimerBase::SharedPtr hypothesis_timer_;
  rclcpp::TimerBase::SharedPtr publish_particles_timer_;
  rclcpp::TimerBase::SharedPtr publish_position_timer_;

  // Configurable params
  int max_hypotheses_;
  bool multihypothesis_;
  float min_candidate_weight_;
  double min_candidate_distance_;
  double min_candidate_angle_;
  float low_q_hypo_threshold_;
  float very_low_q_hypo_threshold_;
  double hypo_merge_distance_;
  double hypo_merge_angle_;
  float good_hypo_threshold_;
  float min_hypo_diff_winner_;
  double bond_timeout_;

  // Time used as the timestamp for the published results
  rclcpp::Time last_time_;

  // Hypotheses
  std::list<std::shared_ptr<ParticlesDistribution>> particles_population_;
  std::shared_ptr<ParticlesDistribution> current_hypothesis_;
  float current_hypothesis_q_;

  // Transformations
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  tf2::Stamped<tf2::Transform> odom2prevbf_;
  bool valid_prev_odom2bf_{false};

  // Map-matching
  std::shared_ptr<beluga_ros::OccupancyGrid> costmap_;
  sensor_msgs::msg::LaserScan::ConstSharedPtr last_laser_;
  std::shared_ptr<mh_amcl::MapMatcher> map_matcher_;

  // Callbacks
  void map_callback(const nav_msgs::msg::OccupancyGrid::ConstSharedPtr &msg);
  void laser_callback(sensor_msgs::msg::LaserScan::ConstSharedPtr msg);
  void initpose_callback(
      const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr
          &pose_msg);
};

} // namespace mh_amcl

#endif // MH_AMCL__MH_AMCL_HPP_
