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

#include <algorithm>
#include <cmath>
#include <numeric>

#include <beluga/random/multivariate_normal_distribution.hpp>
#include <beluga/actions/propagate.hpp>
#include <beluga/actions/reweight.hpp>
#include <beluga/actions/normalize.hpp>
#include <beluga/actions/assign.hpp>
#include <beluga/algorithm/estimation.hpp>
#include <beluga/views/sample.hpp>

#include <beluga_ros/particle_cloud.hpp>
#include <tf2_ros/buffer_interface.h>
#include <visualization_msgs/msg/marker_array.hpp>

#include "beluga_demo_mh_amcl/particles_distribution.hpp"
#include "beluga_demo_mh_amcl/reweight_and_update_hits_action.hpp"

using namespace std::chrono_literals;

namespace mh_amcl
{

  ParticlesDistribution::ParticlesDistribution(
      rclcpp_lifecycle::LifecycleNode::SharedPtr parent_node)
      : parent_node_(parent_node), tf_buffer_(), tf_listener_(tf_buffer_), rd_(),
        generator_(rd_())
  {

    // Publisher for the markers of the particles
    pub_particles_ =
        parent_node->create_publisher<visualization_msgs::msg::MarkerArray>(
            "particle_markers", 1000);

    // Check if the parent node has all the required parameters. If not, declare
    // the missing ones
    if (!parent_node->has_parameter("max_particles"))
    {
      parent_node->declare_parameter<int>("max_particles", 200lu);
    }
    if (!parent_node->has_parameter("min_particles"))
    {
      parent_node->declare_parameter<int>("min_particles", 30lu);
    }
    if (!parent_node->has_parameter("init_error_x"))
    {
      parent_node->declare_parameter("init_error_x", 0.1);
    }
    if (!parent_node->has_parameter("init_error_y"))
    {
      parent_node->declare_parameter("init_error_y", 0.1);
    }
    if (!parent_node->has_parameter("init_error_yaw"))
    {
      parent_node->declare_parameter("init_error_yaw", 0.05);
    }
    if (!parent_node->has_parameter("translation_noise"))
    {
      parent_node->declare_parameter("translation_noise", 0.1);
    }
    if (!parent_node->has_parameter("rotation_noise"))
    {
      parent_node->declare_parameter("rotation_noise", 0.1);
    }
    if (!parent_node->has_parameter("distance_perception_error"))
    {
      parent_node->declare_parameter("distance_perception_error", 0.05);
    }
    if (!parent_node->has_parameter("reseed_percentage_losers"))
    {
      parent_node->declare_parameter("reseed_percentage_losers", 0.8);
    }
    if (!parent_node->has_parameter("reseed_percentage_winners"))
    {
      parent_node->declare_parameter("reseed_percentage_winners", 0.03);
    }
    if (!parent_node->has_parameter("good_hypo_threshold"))
    {
      parent_node->declare_parameter("good_hypo_threshold", 0.6);
    }
    if (!parent_node->has_parameter("low_q_hypo_threshold"))
    {
      parent_node->declare_parameter("low_q_hypo_threshold", 0.25);
    }
    if (!parent_node->has_parameter("particles_step"))
    {
      parent_node->declare_parameter<int>("particles_step", 30);
    }
  }

  using CallbackReturnT =
      rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  CallbackReturnT
  ParticlesDistribution::on_configure(const rclcpp_lifecycle::State &state)
  {
    // Get the parameters initialization from the parent node (mh_amcl)
    parent_node_->get_parameter("max_particles", max_particles_);
    parent_node_->get_parameter("min_particles", min_particles_);
    parent_node_->get_parameter("init_error_x", init_error_x_);
    parent_node_->get_parameter("init_error_y", init_error_y_);
    parent_node_->get_parameter("init_error_yaw", init_error_yaw_);
    parent_node_->get_parameter("translation_noise", translation_noise_);
    parent_node_->get_parameter("rotation_noise", rotation_noise_);
    parent_node_->get_parameter("distance_perception_error",
                                distance_perception_error_);
    parent_node_->get_parameter("reseed_percentage_losers",
                                reseed_percentage_losers_);
    parent_node_->get_parameter("reseed_percentage_winners",
                                reseed_percentage_winners_);
    parent_node_->get_parameter("low_q_hypo_threshold", low_q_hypo_threshold_);
    parent_node_->get_parameter("good_hypo_threshold", good_hypo_threshold_);
    parent_node_->get_parameter("particles_step", particles_step_);

    quality_ = kInitialHypothesisQuality;

    return CallbackReturnT::SUCCESS;
  }

  void ParticlesDistribution::update_pose(
      geometry_msgs::msg::PoseWithCovarianceStamped &pose)
  {
    // Create views over states and weights for Beluga
    auto state_view = beluga::views::states(particles_);
    auto weight_view = beluga::views::weights(particles_);

    // Use Beluga's estimate() to compute weighted mean (Sophus::SE2d) and 3x3 covariance
    auto [mean_se2, _] = beluga::estimate(state_view, weight_view);

    // Fill pose position and orientation from mean_se2
    auto [mean_x, mean_y, mean_yaw] = utils::extractTranslationAndYaw(mean_se2);
    pose.pose.pose.position.x = mean_x;
    pose.pose.pose.position.y = mean_y;
    pose.pose.pose.position.z = 0;

    tf2::Quaternion quat;
    quat.setRPY(0, 0, mean_yaw);

    pose.pose.pose.orientation.x = quat.x();
    pose.pose.pose.orientation.y = quat.y();
    pose.pose.pose.orientation.z = quat.z();
    pose.pose.pose.orientation.w = quat.w();
  }

  void ParticlesDistribution::update_covariance(
      geometry_msgs::msg::PoseWithCovarianceStamped &pose)
  {
    // Create views over states and weights for Beluga
    auto state_view = beluga::views::states(particles_);
    auto weight_view = beluga::views::weights(particles_);

    // Use Beluga's estimate() to compute weighted mean (Sophus::SE2d) and 3x3 covariance
    auto [_, cov3] = beluga::estimate(state_view, weight_view);

    // Prepare a 6x6 covariance (row-major) and zero it
    Eigen::Matrix<double, 6, 6> cov6 = Eigen::Matrix<double, 6, 6>::Zero();

    // Fill the 6x6 covariance matrix using the 3x3 covariance matrix obtained from the estimation
    constexpr std::array<int, 3> idx = {0, 1, 5};

    for (int i = 0; i < 3; ++i)
    {
      for (int j = 0; j < 3; ++j)
      {
        cov6(idx[i], idx[j]) = cov3(i, j);
      }
    }

    for (int r = 0; r < 6; ++r)
    {
      for (int c = 0; c < 6; ++c)
      {
        pose.pose.covariance[r * 6 + c] = cov6(r, c);
      }
    }
  }

  CallbackReturnT
  ParticlesDistribution::on_activate(const rclcpp_lifecycle::State &state)
  {
    pub_particles_->on_activate();

    return CallbackReturnT::SUCCESS;
  }

  CallbackReturnT
  ParticlesDistribution::on_deactivate(const rclcpp_lifecycle::State &state)
  {
    pub_particles_->on_deactivate();
    return CallbackReturnT::SUCCESS;
  }

  CallbackReturnT
  ParticlesDistribution::on_cleanup(const rclcpp_lifecycle::State &state)
  {
    return CallbackReturnT::SUCCESS;
  }

  void ParticlesDistribution::init(const tf2::Transform &pose_init,
                                   std::shared_ptr<beluga_ros::OccupancyGrid> costmap)
  {

    // Store the costmap as attribute
    costmap_ = costmap;

    // Transform the initial pose to SE2d and use it for the mean
    Sophus::SE2d init_pose_se2d = utils::tf2TransformToSE2d(pose_init);
    auto [mean_x, mean_y, mean_yaw] = utils::extractTranslationAndYaw(init_pose_se2d);

    // Build mean and covariance
    auto mean = Eigen::Vector3d{mean_x, mean_y, mean_yaw};

    Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
    cov(0, 0) = init_error_x_ * init_error_x_;
    cov(1, 1) = init_error_y_ * init_error_y_;
    cov(2, 2) = init_error_yaw_ * init_error_yaw_;

    // Create a Beluga multivariate normal distribution for (x,y,yaw)
    beluga::MultivariateNormalDistribution distribution(mean, cov);

    // Compute number of particles to use (same logic as before)
    const size_t num_particles = static_cast<size_t>((max_particles_ + min_particles_) / 2);

    // Use Beluga sample to get the sampled particles
    auto sampled = beluga::views::sample(std::move(distribution)) | ranges::views::take_exactly(num_particles);

    particles_.clear();
    particles_.reserve(num_particles);

    for (auto sample : sampled)
    {
      Particle particle{};
      particle.weight = 1.0 / static_cast<double>(num_particles);
      particle.state = Sophus::SE2d(static_cast<double>(sample[2]),
                                    Eigen::Vector2d(static_cast<double>(sample[0]),
                                                    static_cast<double>(sample[1])));
      particles_.push_back(particle);
    }

    particles_ |= beluga::actions::normalize();
    update_covariance(pose_);
    update_pose(pose_);
  }

  void ParticlesDistribution::predict(const tf2::Transform &movement)
  {
    // Convert movement to Sophus::SE2d
    const Sophus::SE2d se2_movement = utils::tf2TransformToSE2d(movement);

    // Define a motion model that takes a state and returns a new state.
    auto motion_model =
        [this, &se2_movement, &movement](const Sophus::SE2d &state) -> Sophus::SE2d
    {
      // Add noise in tf2 space
      const tf2::Transform noise_tf = add_noise(movement);

      // Convert noise to SE2
      const Sophus::SE2d se2_noise = utils::tf2TransformToSE2d(noise_tf);

      // Return the new state
      return state * se2_movement * se2_noise;
    };

    particles_ |= beluga::actions::propagate(motion_model);

    update_pose(pose_);
  }

  tf2::Transform ParticlesDistribution::add_noise(const tf2::Transform &dm)
  {
    tf2::Transform returned_noise;

    std::normal_distribution<double> translation_noise(0.0, translation_noise_);
    std::normal_distribution<double> rotation_noise(0.0, rotation_noise_);

    double noise_translation = translation_noise(generator_);
    double noise_rotation = rotation_noise(generator_);

    double x = dm.getOrigin().x() * noise_translation;
    double y = dm.getOrigin().y() * noise_translation;
    double z = 0.0;

    returned_noise.setOrigin(tf2::Vector3(x, y, z));

    double roll, pitch, yaw;
    tf2::Matrix3x3(dm.getRotation()).getRPY(roll, pitch, yaw);

    double new_yaw = yaw * noise_rotation;

    tf2::Quaternion quat;
    quat.setRPY(roll, pitch, new_yaw);
    returned_noise.setRotation(quat);

    return returned_noise;
  }

  void ParticlesDistribution::publish_particles(
      int base_idx, const std_msgs::msg::ColorRGBA &color) const
  {
    if (pub_particles_->get_subscription_count() == 0)
    {
      return;
    }

    visualization_msgs::msg::MarkerArray particles_markers_msg;
    beluga_ros::assign_particle_cloud(particles_, particles_markers_msg);
    beluga_ros::stamp_message("map", parent_node_->now(), particles_markers_msg);

    pub_particles_->publish(particles_markers_msg);
  }

  void ParticlesDistribution::correct_once(
      const sensor_msgs::msg::LaserScan &scan)
  {

    std::string error;
    if (tf_buffer_.canTransform("base_footprint", scan.header.frame_id,
                                tf2_ros::fromMsg(scan.header.stamp), &error))
    {
      auto base_to_laser_msg =
          tf_buffer_.lookupTransform("base_footprint", scan.header.frame_id,
                                     tf2_ros::fromMsg(scan.header.stamp));
      tf2::fromMsg(base_to_laser_msg, base_to_laser_tf_);
    }
    else
    {
      RCLCPP_WARN(parent_node_->get_logger(),
                  "Timeout while waiting TF %s -> base_footprint [%s]",
                  scan.header.frame_id.c_str(), error.c_str());
      return;
    }

    // A single pipeline updates weights and hits, then normalizes
    particles_ |= actions::reweight_and_update_hits(
                      scan, costmap_, base_to_laser_tf_, distance_perception_error_) |
                  beluga::actions::normalize();

    // After the update, we still need to find the best 'hits' value to update the hypothesis quality
    quality_ = 0.0;
    if (!particles_.empty())
    {
      auto max_it = std::max_element(
          particles_.begin(),
          particles_.end(),
          [](const Particle &a, const Particle &b)
          {
            return a.hits < b.hits;
          });
      quality_ = max_it->hits;
    }
  }

  void ParticlesDistribution::reseed()
  {
    // Sort particles by probability
    std::sort(particles_.begin(), particles_.end(),
              [](const Particle &a, const Particle &b) -> bool
              {
                return a.weight > b.weight;
              });

    int number_particles = static_cast<int>(particles_.size());
    // If current hypothesis quality is too bad or too good, modify the particles accordingly (adding them when it's too bad and removing them when it's too good)
    if (get_quality() < low_q_hypo_threshold_)
    {
      number_particles =
          std::clamp(static_cast<int>(number_particles + particles_step_),
                     min_particles_, max_particles_);
    }
    else if (get_quality() > good_hypo_threshold_)
    {
      number_particles =
          std::clamp(static_cast<int>(number_particles - particles_step_),
                     min_particles_, max_particles_);
    }

    const int number_losers = static_cast<int>(std::round(number_particles * reseed_percentage_losers_));
    const int number_no_losers = number_particles - number_losers;
    const int number_winners = static_cast<int>(std::round(number_particles * reseed_percentage_winners_));

    // Keep the best particles (survivors)
    std::vector<Particle> new_particles;
    new_particles.reserve(static_cast<size_t>(number_particles));
    new_particles.insert(new_particles.end(), particles_.begin(), particles_.begin() + number_no_losers);

    // Build winners container
    std::vector<Particle> winners;
    winners.insert(winners.end(), particles_.begin(), particles_.begin() + number_winners);

    // Build winner weight vector and a discrete_distribution so winners are sampled
    // proportionally to their weight (like it's done in the typical particle filter).
    std::vector<double> winner_weights;
    winner_weights.reserve(winners.size());
    for (const auto &w : winners)
    {
      winner_weights.push_back(w.weight);
    }
    std::discrete_distribution<size_t> pick_winner(winner_weights.begin(), winner_weights.end());

    // Build Beluga’s multivariate normal distribution for sampling
    Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
    cov(0, 0) = init_error_x_ * init_error_x_;
    cov(1, 1) = init_error_y_ * init_error_y_;
    cov(2, 2) = init_error_yaw_ * init_error_yaw_;

    beluga::MultivariateNormalDistribution<Eigen::Vector3d> normal_dist(
        Eigen::Vector3d::Zero(), cov);

    // Create a view that contains exactly "number_losers" noise vectors
    auto noises = beluga::views::sample(normal_dist) | ranges::views::take_exactly(static_cast<size_t>(number_losers));
    auto noise_it = noises.begin();

    // Sample losers: for each loser, pick a winner (weighted) and add one noise vector
    for (int i = 0; i < number_losers; ++i)
    {
      // pick a weighted winner index
      const size_t widx = pick_winner(generator_);
      const Particle &winner = winners[widx];

      auto [x, y, yaw] = utils::extractTranslationAndYaw(winner.state);

      // Get batch-sampled noise
      Eigen::Vector3d noise = *noise_it;
      ++noise_it; // advance the iterator

      double new_x = x + noise(0);
      double new_y = y + noise(1);
      double new_yaw = utils::normalize_angle(yaw + noise(2));

      Particle p;
      p.weight = winner.weight; // inherit weight from winner
      p.state = Sophus::SE2d(new_yaw, Eigen::Vector2d(new_x, new_y));
      p.hits = 0.0f;

      new_particles.emplace_back(std::move(p));
    }

    // Finally, ensure size and replace original
    if (static_cast<int>(new_particles.size()) > number_particles)
    {
      new_particles.resize(static_cast<size_t>(number_particles));
    }
    else
    {
      while (static_cast<int>(new_particles.size()) < number_particles)
      {
        new_particles.push_back(new_particles.front());
      }
    }
    particles_ = std::move(new_particles);
    particles_ |= beluga::actions::normalize();
    update_covariance(pose_);
  }

  void ParticlesDistribution::merge(ParticlesDistribution &other)
  {
    size_t size = particles_.size();
    particles_.insert(particles_.end(), other.particles_.begin(),
                      other.particles_.end());

    std::sort(particles_.begin(), particles_.end(),
              [](const Particle &a, const Particle &b) -> bool
              {
                return a.weight > b.weight;
              });
    particles_.erase(particles_.begin() + size, particles_.end());
  }

} // namespace mh_amcl
