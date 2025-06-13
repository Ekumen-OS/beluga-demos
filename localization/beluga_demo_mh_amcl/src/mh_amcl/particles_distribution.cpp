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

#include <beluga_ros/particle_cloud.hpp>

#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2_ros/buffer_interface.h>
#include <visualization_msgs/msg/marker_array.hpp>

#include "beluga_demo_mh_amcl/particles_distribution.hpp"

namespace mh_amcl {

using namespace std::chrono_literals;

ParticlesDistribution::ParticlesDistribution(
    rclcpp_lifecycle::LifecycleNode::SharedPtr parent_node)
    : parent_node_(parent_node), tf_buffer_(), tf_listener_(tf_buffer_), rd_(),
      generator_(rd_()) {

  // Publisher for the markers of the particles
  pub_particles_ =
      parent_node->create_publisher<visualization_msgs::msg::MarkerArray>(
          "particle_markers", 1000);

  // Check if the parent node has all the required parameters. If not, declare
  // the missing ones
  if (!parent_node->has_parameter("max_particles")) {
    parent_node->declare_parameter<int>("max_particles", 200lu);
  }
  if (!parent_node->has_parameter("min_particles")) {
    parent_node->declare_parameter<int>("min_particles", 30lu);
  }
  if (!parent_node->has_parameter("init_pos_x")) {
    parent_node->declare_parameter("init_pos_x", 0.0);
  }
  if (!parent_node->has_parameter("init_pos_y")) {
    parent_node->declare_parameter("init_pos_y", 0.0);
  }
  if (!parent_node->has_parameter("init_pos_yaw")) {
    parent_node->declare_parameter("init_pos_yaw", 0.0);
  }
  if (!parent_node->has_parameter("init_error_x")) {
    parent_node->declare_parameter("init_error_x", 0.1);
  }
  if (!parent_node->has_parameter("init_error_y")) {
    parent_node->declare_parameter("init_error_y", 0.1);
  }
  if (!parent_node->has_parameter("init_error_yaw")) {
    parent_node->declare_parameter("init_error_yaw", 0.05);
  }
  if (!parent_node->has_parameter("translation_noise")) {
    parent_node->declare_parameter("translation_noise", 0.01);
  }
  if (!parent_node->has_parameter("rotation_noise")) {
    parent_node->declare_parameter("rotation_noise", 0.01);
  }
  if (!parent_node->has_parameter("distance_perception_error")) {
    parent_node->declare_parameter("distance_perception_error", 0.05);
  }
  if (!parent_node->has_parameter("reseed_percentage_losers")) {
    parent_node->declare_parameter("reseed_percentage_losers", 0.8);
  }
  if (!parent_node->has_parameter("reseed_percentage_winners")) {
    parent_node->declare_parameter("reseed_percentage_winners", 0.03);
  }
  if (!parent_node->has_parameter("good_hypo_threshold")) {
    parent_node->declare_parameter("good_hypo_threshold", 0.6);
  }
  if (!parent_node->has_parameter("good_hypo_threshold")) {
    parent_node->declare_parameter("low_q_hypo_threshold", 0.25f);
  }
  if (!parent_node->has_parameter("particles_step")) {
    parent_node->declare_parameter<int>("particles_step", 30);
  }
}

using CallbackReturnT =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

CallbackReturnT
ParticlesDistribution::on_configure(const rclcpp_lifecycle::State &state) {
  // Get the parameters initialization from the parent node (mh_amcl)
  parent_node_->get_parameter("max_particles", max_particles_);
  parent_node_->get_parameter("min_particles", min_particles_);
  parent_node_->get_parameter("init_pos_x", init_pos_x_);
  parent_node_->get_parameter("init_pos_y", init_pos_y_);
  parent_node_->get_parameter("init_pos_yaw", init_pos_yaw_);
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

  // Initialize the initial pose
  tf2::Transform init_pose;
  init_pose.setOrigin(tf2::Vector3(init_pos_x_, init_pos_y_, 0.0));

  tf2::Quaternion quat;
  quat.setRPY(0.0, 0.0, init_pos_yaw_);
  init_pose.setRotation(quat);

  init(init_pose);
  quality_ = 0.25;

  return CallbackReturnT::SUCCESS;
}

void ParticlesDistribution::update_pose(
    geometry_msgs::msg::PoseWithCovarianceStamped &pose) {
  // How many particles we use to determine the pose. They are sorted by prob in
  // last reseed
  size_t particles_used = particles_.size();

  // Vectors to contain the coordinates of all the particles (a vector for every
  // coordinate is used to be able to do the weighted means later)
  std::vector<double> particles_x(particles_used, 0.0);
  std::vector<double> particles_y(particles_used, 0.0);
  std::vector<double> particles_z(particles_used, 0.0);
  std::vector<double> particles_roll(particles_used, 0.0);
  std::vector<double> particles_pitch(particles_used, 0.0);
  std::vector<double> particles_yaw(particles_used, 0.0);
  std::vector<double> weights(particles_used, 0.0);

  // Extract the weighted mean of the particles' poses to update the current
  // pose
  for (int i = 0; i < particles_used; i++) {
    auto [x, y, yaw] = utils::extractTranslationAndYaw(particles_[i].state);
    particles_x[i] = x;
    particles_y[i] = y;

    particles_yaw[i] = yaw;

    weights[i] = particles_[i].weight;
  }

  pose.pose.pose.position.x = utils::weighted_mean(particles_x, weights);
  pose.pose.pose.position.y = utils::weighted_mean(particles_y, weights);
  pose.pose.pose.position.z = 0;

  tf2::Quaternion quat;
  double mean_yaw = utils::angle_weighted_mean(particles_yaw, weights);

  quat.setRPY(0, 0, mean_yaw);

  pose.pose.pose.orientation.x = quat.x();
  pose.pose.pose.orientation.y = quat.y();
  pose.pose.pose.orientation.z = quat.z();
  pose.pose.pose.orientation.w = quat.w();
}

void ParticlesDistribution::update_covariance(
    geometry_msgs::msg::PoseWithCovarianceStamped &pose) {
  std::vector<double> particles_x(particles_.size(), 0.0);
  std::vector<double> particles_y(particles_.size(), 0.0);
  std::vector<double> particles_z(particles_.size(), 0.0);
  std::vector<double> particles_roll(particles_.size(), 0.0);
  std::vector<double> particles_pitch(particles_.size(), 0.0);
  std::vector<double> particles_yaw(particles_.size(), 0.0);

  for (int i = 0; i < particles_.size(); i++) {
    auto [x, y, yaw] = utils::extractTranslationAndYaw(particles_[i].state);
    particles_x[i] = x;
    particles_y[i] = y;
    particles_yaw[i] = yaw;
  }

  std::vector<std::vector<double>> particles_coordinates = {
      particles_x,    particles_y,     particles_z,
      particles_roll, particles_pitch, particles_yaw};

  for (int i = 0; i < 6; i++) {
    for (int j = 0; j < 6; j++) {
      bool is_i_angle = i >= 3;
      bool is_j_angle = j >= 3;
      pose.pose.covariance[i * 6 + j] =
          utils::covariance(particles_coordinates[i], particles_coordinates[j],
                            is_i_angle, is_j_angle);
    }
  }
}

CallbackReturnT
ParticlesDistribution::on_activate(const rclcpp_lifecycle::State &state) {
  pub_particles_->on_activate();

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
ParticlesDistribution::on_deactivate(const rclcpp_lifecycle::State &state) {
  pub_particles_->on_deactivate();
  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
ParticlesDistribution::on_cleanup(const rclcpp_lifecycle::State &state) {
  return CallbackReturnT::SUCCESS;
}

void ParticlesDistribution::init(const tf2::Transform &pose_init) {
  std::normal_distribution<double> noise_x(0, init_error_x_);
  std::normal_distribution<double> noise_y(0, init_error_y_);
  std::normal_distribution<double> noise_t(0, init_error_yaw_);

  // Transform the init pose to SE2d
  Sophus::SE2d init_pose_se2d = utils::tf2TransformToSE2d(pose_init);

  particles_.clear();
  particles_.resize((max_particles_ + min_particles_) / 2);

  for (auto &particle : particles_) {
    particle.weight = 1.0 / static_cast<double>(particles_.size());
    particle.state = init_pose_se2d;

    // Get the current translation (x,y) and rotation (yaw) from the particle's
    // pose
    auto [x, y, yaw] = utils::extractTranslationAndYaw(particle.state);
    // Add noise to the translation components
    x += noise_x(generator_);
    y += noise_y(generator_);

    double new_yaw = yaw + noise_t(generator_);

    particle.state = Sophus::SE2d(new_yaw, Eigen::Vector2d(x, y));
  }

  normalize();
  update_covariance(pose_);
  update_pose(pose_);
}

void ParticlesDistribution::predict(const tf2::Transform &movement) {
  // Convert movement to Sophus::SE2d
  const Sophus::SE2d se2_movement = utils::tf2TransformToSE2d(movement);

  for (auto &particle : particles_) {
    // Compute the noise for the particle
    const tf2::Transform noise_tf = add_noise(movement);

    // Convert the noise to Sophus::SE2d
    const Sophus::SE2d se2_noise = utils::tf2TransformToSE2d(noise_tf);

    // Update the pose of the particle
    particle.state = particle.state * se2_movement * se2_noise;
  }
  update_pose(pose_);
}

tf2::Transform ParticlesDistribution::add_noise(const tf2::Transform &dm) {
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
    int base_idx, const std_msgs::msg::ColorRGBA &color) const {
  if (pub_particles_->get_subscription_count() == 0) {
    return;
  }

  visualization_msgs::msg::MarkerArray particles_markers_msg;
  beluga_ros::assign_particle_cloud(particles_, particles_markers_msg);
  beluga_ros::stamp_message("map", parent_node_->now(), particles_markers_msg);

  pub_particles_->publish(particles_markers_msg);
}

void ParticlesDistribution::correct_once(
    const sensor_msgs::msg::LaserScan &scan,
    std::shared_ptr<beluga_ros::OccupancyGrid> costmap) {
  std::string error;
  if (tf_buffer_.canTransform(scan.header.frame_id, "base_footprint",
                              tf2_ros::fromMsg(scan.header.stamp), &error)) {
    auto bf2laser_msg =
        tf_buffer_.lookupTransform(scan.header.frame_id, "base_footprint",
                                   tf2_ros::fromMsg(scan.header.stamp));
    tf2::fromMsg(bf2laser_msg, bf2laser_);
  } 
  else {
    RCLCPP_WARN(parent_node_->get_logger(),
                "Timeout while waiting TF %s -> base_footprint [%s]",
                scan.header.frame_id.c_str(), error.c_str());
    return;
  }

  const double normal_comp_1 = utils::INV_SQRT_2PI / distance_perception_error_;

  for (auto &p : particles_) {
    p.hits = 0.0;
  }

  for (int j = 0; j < scan.ranges.size(); j++) {
    if (std::isnan(scan.ranges[j]) || std::isinf(scan.ranges[j])) {
      continue;
    }

    tf2::Transform laser2point = get_tranform_to_read(scan, j);

    for (int i = 0; i < particles_.size(); i++) {
      auto &p = particles_[i];

      tf2::Transform particle_pose_tf2 = utils::se2dToTf2Transform(p.state);

      double calculated_distance = get_error_distance_to_obstacle(
          particle_pose_tf2, bf2laser_, laser2point, scan, costmap,
          distance_perception_error_);
      
      if (!std::isinf(calculated_distance)) {
        const double a = calculated_distance / distance_perception_error_;
        const double normal_comp_2 = std::exp(-0.5 * a * a);

        double prob = std::clamp(normal_comp_1 * normal_comp_2, 0.0, 1.0);
        p.weight = std::max(p.weight + prob, 0.000001);

        p.hits += prob;
      }
    }
  }

  normalize();

  // Calculate quality
  quality_ = 0.0;
  for (auto &p : particles_) {
    p.hits = p.hits / static_cast<float>(scan.ranges.size());
    quality_ = std::max(quality_, p.hits);
  }
}

tf2::Transform ParticlesDistribution::get_tranform_to_read(
    const sensor_msgs::msg::LaserScan &scan, int index) {
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

signed char ParticlesDistribution::get_cost(
    const tf2::Transform &transform,
    std::shared_ptr<beluga_ros::OccupancyGrid> costmap) {
  auto [local_x, local_y] = utils::worldToMapNoBounds(
      costmap, transform.getOrigin().x(), transform.getOrigin().y());

  if (local_x > 0 && local_y > 0 && local_x < costmap->width() &&
      local_y < costmap->height() &&
      costmap->data_at(local_x, local_y).has_value()) {
    return costmap->data_at(local_x, local_y).value();
  } else {
    return utils::NO_INFORMATION;
  }
}

double ParticlesDistribution::get_error_distance_to_obstacle(
    const tf2::Transform &map2bf, const tf2::Transform &bf2laser,
    const tf2::Transform &laser2point, const sensor_msgs::msg::LaserScan &scan,
    std::shared_ptr<beluga_ros::OccupancyGrid> costmap,
    double distance_perception_error_) {
  if (std::isinf(laser2point.getOrigin().x()) ||
      std::isnan(laser2point.getOrigin().x())) {
    return std::numeric_limits<double>::infinity();
  }

  tf2::Transform map2laser = map2bf * bf2laser;
  tf2::Transform map2point = map2laser * laser2point;
  tf2::Transform map2point_aux = map2point;
  tf2::Transform uvector;
  tf2::Vector3 unit =
      laser2point.getOrigin() / laser2point.getOrigin().length();

  if (get_cost(map2point, costmap) == utils::LETHAL_OBSTACLE) {
    return 0.0;
  }

  float dist = costmap->resolution();
  while (dist < (3.0 * distance_perception_error_)) {
    uvector.setOrigin(unit * dist);
    // For positive
    map2point = map2point_aux * uvector;
    auto cost = get_cost(map2point, costmap);

    if (cost == utils::LETHAL_OBSTACLE) {
      return dist;
    }

    // For negative
    uvector.setOrigin(uvector.getOrigin() * -1.0);
    map2point = map2point_aux * uvector;
    cost = get_cost(map2point, costmap);

    if (cost == utils::LETHAL_OBSTACLE) {
      return dist;
    }
    dist = dist + costmap->resolution();
  }

  return std::numeric_limits<double>::infinity();
}

void ParticlesDistribution::reseed() {
  // Sort particles by probability
  std::sort(particles_.begin(), particles_.end(),
            [](const Particle &a, const Particle &b) -> bool {
              return a.weight > b.weight;
            });

  auto number_particles = particles_.size();
  // If current hypothesis quality is too bad or too good, modify the particles accordingly (adding them when it's too bad and removing them when it's too good)
  if (get_quality() < low_q_hypo_threshold_)
  {
    number_particles =
        std::clamp(static_cast<int>(number_particles + particles_step_),
                   min_particles_, max_particles_);
    for (int i = 0; i < number_particles - particles_.size(); i++)
    {
      Particle new_p = particles_.front();
      particles_.push_back(new_p);
    }
  }
  else if (get_quality() > good_hypo_threshold_)
  {
    number_particles =
        std::clamp(static_cast<int>(number_particles - particles_step_),
                   min_particles_, max_particles_);
    for (int i = 0; i < particles_.size() - number_particles; i++)
    {
      particles_.pop_back();
    }
  }

  int number_losers = number_particles * reseed_percentage_losers_;
  int number_no_losers = number_particles - number_losers;
  int number_winners = number_particles * reseed_percentage_winners_;

  // Drop the losers particles, and replace them with new particles
  std::vector<Particle> new_particles(particles_.begin(),
                                      particles_.begin() + number_no_losers);

  std::normal_distribution<double> selector(0, number_winners);
  std::normal_distribution<double> noise_x(0, init_error_x_ * init_error_x_);
  std::normal_distribution<double> noise_y(0, init_error_y_ * init_error_y_);
  std::normal_distribution<double> noise_t(0,
                                           init_error_yaw_ * init_error_yaw_);

  for (int i = 0; i < number_losers; i++) {
    Particle p;
    p.weight = new_particles.back().weight;

    // Extract the 2D pose of the particle
    auto [x, y, yaw] = utils::extractTranslationAndYaw(particles_[i].state);

    // Compute the noise
    double nx = noise_x(generator_);
    double ny = noise_y(generator_);

    // Get the new 2D pose of the particle, and assign it
    double new_x = x + nx;
    double new_y = y + ny;
    double new_yaw = yaw + noise_t(generator_);
    new_yaw = utils::normalize_angle(new_yaw);

    p.state = Sophus::SE2d(new_yaw, Eigen::Vector2d(x, y));

    new_particles.push_back(p);
  }

  // Update the whole group of particles after reseeding
  particles_ = new_particles;
  normalize();
  update_covariance(pose_);
}

void ParticlesDistribution::normalize() {
  double sum = 0.0;
  std::for_each(particles_.begin(), particles_.end(),
                [&sum](const Particle &p) { sum += p.weight; });

  if (sum != 0.0) {
    std::for_each(particles_.begin(), particles_.end(),
                  [&](Particle &p) { p.weight = p.weight / sum; });
  }
}

void ParticlesDistribution::merge(ParticlesDistribution &other) {
  size_t size = particles_.size();
  particles_.insert(particles_.end(), other.particles_.begin(),
                    other.particles_.end());

  std::sort(particles_.begin(), particles_.end(),
            [](const Particle &a, const Particle &b) -> bool {
              return a.weight > b.weight;
            });
  particles_.erase(particles_.begin() + size, particles_.end());
}

} // namespace mh_amcl
