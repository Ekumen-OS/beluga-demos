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

#ifndef MH_AMCL__PARTICLESDISTRIBUTION_HPP_
#define MH_AMCL__PARTICLESDISTRIBUTION_HPP_

#include <random>
#include <vector>

#include <beluga_ros/occupancy_grid.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker_array.hpp>

#include "beluga_demo_mh_amcl/utils.hpp"

namespace mh_amcl {

/**
 * @brief Structure representing a Particle, containing the pose, probability
 * and hits (normalization of how well a particle aligns with the laser data).
 * This 'hits' value is used as the quality of the particle.
 */
typedef struct {
  Sophus::SE2d state;
  beluga::Weight weight;
  float hits;
} Particle;

/**
 * ParticlesDistribution class
 *
 * Defines a set of particles (hypothesis). the core logic of the algorithm is
 * performed in this class.
 */
class ParticlesDistribution {
public:
  explicit ParticlesDistribution(
      rclcpp_lifecycle::LifecycleNode::SharedPtr parent_node);

  // Initialization
  void init(const tf2::Transform &pose_init);

  /**
   * @brief The pose of every particle is updated according to the input
   * movement and some noise
   */
  void predict(const tf2::Transform &movement);

  /**
   * @brief The quality of the hypothesis is computed as the highest quality of
   * the particles. It's only done if laser data and costmap information are
   * present.
   */
  void correct_once(const sensor_msgs::msg::LaserScan &scan,
                    std::shared_ptr<beluga_ros::OccupancyGrid> costmap);

  /**
   * @brief Particles are ordered by probability, winners and losers are
   * computed, and new particles are introduced if there are too  bad or too
   * good particles.
   */
  void reseed();

  /**
   * @brief Get the vector of particles
   */
  const std::vector<mh_amcl::Particle> &get_particles() const {
    return particles_;
  }

  using CallbackReturnT =
      rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
  // Lifecycle node methods
  CallbackReturnT on_configure(const rclcpp_lifecycle::State &state);
  CallbackReturnT on_activate(const rclcpp_lifecycle::State &state);
  CallbackReturnT on_deactivate(const rclcpp_lifecycle::State &state);
  CallbackReturnT on_cleanup(const rclcpp_lifecycle::State &state);

  /**
   * @brief Publish the particles' markers for visualization in RViz
   */
  void publish_particles(int base_idx,
                         const std_msgs::msg::ColorRGBA &color) const;

  /**
   * @brief Get the weighted mean pose of the set of particles (hypothesis)
   */
  geometry_msgs::msg::PoseWithCovarianceStamped get_pose() const {
    return pose_;
  }

  /**
   * @brief Get the quality of the hypothesis
   */
  float get_quality() { return quality_; }

  /**
   * @brief Merge a hypothesis with this one when they are too similar
   */
  void merge(ParticlesDistribution &other);

protected:
  rclcpp_lifecycle::LifecycleNode::SharedPtr parent_node_;

  // Publisher for visualization in RViz
  rclcpp_lifecycle::LifecyclePublisher<
      visualization_msgs::msg::MarkerArray>::SharedPtr pub_particles_;

  // Auxiliary methods for the steps of the AMCL algorithm
  tf2::Transform add_noise(const tf2::Transform &dm);
  tf2::Transform get_tranform_to_read(const sensor_msgs::msg::LaserScan &scan,
                                      int index);
  double get_error_distance_to_obstacle(
      const tf2::Transform &map2bf, const tf2::Transform &bf2laser,
      const tf2::Transform &laser2point,
      const sensor_msgs::msg::LaserScan &scan,
      std::shared_ptr<beluga_ros::OccupancyGrid> costmap, double o);
  signed char get_cost(const tf2::Transform &transform,
                         std::shared_ptr<beluga_ros::OccupancyGrid> costmap);
  void normalize();
  void update_pose(geometry_msgs::msg::PoseWithCovarianceStamped &pose);
  void update_covariance(geometry_msgs::msg::PoseWithCovarianceStamped &pose);

  // Wighted mean pose of the set of particles
  geometry_msgs::msg::PoseWithCovarianceStamped pose_;

  // Mersenne Twister 19937 pseudo-random number generator
  std::random_device rd_;
  std::mt19937 generator_;

  // Particles used and quality of the hypothesis
  std::vector<Particle> particles_;
  float quality_;

  // Transformations
  tf2::BufferCore tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  tf2::Stamped<tf2::Transform> bf2laser_;
  bool bf2laser_init_{false};

  // MH-AMCL Parameters
  int max_particles_;
  int min_particles_;
  double init_pos_x_;
  double init_pos_y_;
  double init_pos_yaw_;
  double init_error_x_;
  double init_error_y_;
  double init_error_yaw_;
  double translation_noise_;
  double rotation_noise_;
  double distance_perception_error_;
  double reseed_percentage_losers_;
  double reseed_percentage_winners_;
  float good_hypo_threshold_;
  float low_q_hypo_threshold_;
  int particles_step_;
};

} // namespace mh_amcl

#endif // MH_AMCL__PARTICLESDISTRIBUTION_HPP_
