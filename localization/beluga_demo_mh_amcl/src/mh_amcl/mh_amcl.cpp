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

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/create_timer_ros.h>

#include "beluga_demo_mh_amcl/mh_amcl.hpp"
#include "beluga_demo_mh_amcl/utils.hpp"

namespace mh_amcl {

using std::placeholders::_1;
using namespace std::chrono_literals;

MH_AMCL_Node::MH_AMCL_Node(const rclcpp::NodeOptions &options)
    : rclcpp_lifecycle::LifecycleNode("mh_amcl", "", options), tf_buffer_(this->get_clock()),
      tf_listener_(tf_buffer_) {

  // Declare all the node's parameters
  declare_parameter<int>("max_hypotheses", 5);
  declare_parameter<bool>("multihypothesis", true);
  declare_parameter<float>("min_candidate_weight", 0.5f);
  declare_parameter<double>("min_candidate_distance", 0.5);
  declare_parameter<double>("min_candidate_angle", M_PI_2);
  declare_parameter<float>("low_q_hypo_threshold", 0.25f);
  declare_parameter<float>("very_low_q_hypo_threshold", 0.1);
  declare_parameter<double>("hypo_merge_distance", 0.2);
  declare_parameter<double>("hypo_merge_angle", 0.3);
  declare_parameter<float>("good_hypo_threshold", 0.6);
  declare_parameter<float>("min_hypo_diff_winner", 0.2);
  declare_parameter<double>("bond_timeout", 4.0);
}

CallbackReturnT
MH_AMCL_Node::on_configure(const rclcpp_lifecycle::State &state) {
  RCLCPP_INFO(get_logger(), "Configuring...");

  current_hypothesis_ =
      std::make_shared<ParticlesDistribution>(shared_from_this());
  current_hypothesis_q_ = 1.0;
  particles_population_.push_back(current_hypothesis_);

  // Configure tf
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
    this->get_node_base_interface(),
    this->get_node_timers_interface()
  );
  tf_buffer_.setCreateTimerInterface(timer_interface);
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);

  std::list<CallbackReturnT> ret;
  for (auto &particles : particles_population_) {
    ret.push_back(particles->on_configure(state));
  }

  if (std::count(ret.begin(), ret.end(), CallbackReturnT::SUCCESS) ==
      ret.size()) {
    RCLCPP_INFO(get_logger(), "Configured");
    return CallbackReturnT::SUCCESS;
  }

  RCLCPP_ERROR(get_logger(), "Error configuring");
  return CallbackReturnT::FAILURE;
}

CallbackReturnT
MH_AMCL_Node::on_activate(const rclcpp_lifecycle::State &state) {
  RCLCPP_INFO(get_logger(), "Activating...");

  // Get the value of the parameters, or use the default
  get_parameter("use_sim_time", use_sim_time_);
  get_parameter("multihypothesis", multihypothesis_);
  get_parameter("max_hypotheses", max_hypotheses_);
  get_parameter("min_candidate_weight", min_candidate_weight_);
  get_parameter("min_candidate_distance", min_candidate_distance_);
  get_parameter("min_candidate_angle", min_candidate_angle_);
  get_parameter("low_q_hypo_threshold", low_q_hypo_threshold_);
  get_parameter("very_low_q_hypo_threshold", very_low_q_hypo_threshold_);
  get_parameter("hypo_merge_distance", hypo_merge_distance_);
  get_parameter("hypo_merge_angle", hypo_merge_angle_);
  get_parameter("good_hypo_threshold", good_hypo_threshold_);
  get_parameter("min_hypo_diff_winner", min_hypo_diff_winner_);
  get_parameter("bond_timeout", bond_timeout_);

  if (use_sim_time_) {
    RCLCPP_INFO(get_logger(), "use_sim_time = true");
  } else {
    RCLCPP_INFO(get_logger(), "use_sim_time = false");
  }

  // Subscribers
  map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
    "map", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
    std::bind(&MH_AMCL_Node::map_callback, this, _1));
  init_pose_sub_ =
      create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
          "initialpose", rclcpp::SystemDefaultsQoS(),
          std::bind(&MH_AMCL_Node::initpose_callback, this, _1));

  // For the laser subscription, implement a message filter
  laser_sub_ = std::make_unique<LaserScanSubscriber>(
      this, "scan", rmw_qos_profile_sensor_data
  );
  laser_scan_filter_ = std::make_unique<LaserScanFilter>(
    *laser_sub_,
    tf_buffer_,
    "base_footprint",
    100,
    get_node_logging_interface(),
    get_node_clock_interface(),
    tf2::durationFromSec(2)
  );
  laser_scan_connection_ = laser_scan_filter_->registerCallback(
    std::bind(&MH_AMCL_Node::laser_callback, this, std::placeholders::_1)
  );

  // Publishers
  pose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "pose", rclcpp::SystemDefaultsQoS());
  particles_pub_ =
      create_publisher<geometry_msgs::msg::PoseArray>("particle_cloud", rclcpp::SensorDataQoS());

  // Create the timers for the different stages of the algorithm
  predict_timer_ =
      create_wall_timer(10ms, std::bind(&MH_AMCL_Node::predict, this));
  correct_timer_ =
      create_wall_timer(100ms, std::bind(&MH_AMCL_Node::correct, this));
  reseed_timer_ = create_wall_timer(3s, std::bind(&MH_AMCL_Node::reseed, this));
  hypothesis_timer_ =
      create_wall_timer(3s, std::bind(&MH_AMCL_Node::manage_hypotheses, this));

  // Now, create the timers for the publishers
  publish_particles_timer_ = create_wall_timer(
      100ms, std::bind(&MH_AMCL_Node::publish_particles, this));
  publish_position_timer_ =
      create_wall_timer(30ms, std::bind(&MH_AMCL_Node::publish_position, this));

  // Create the bond
  auto on_bond_formed_callback = [this]() {
    RCLCPP_INFO(get_logger(), "The bond connection to the lifecycle manager is now fully formed");
  };
  auto on_bond_broken_callback = [this]() {
    RCLCPP_ERROR(get_logger(), "The bond connection to the lifecycle manager has been broken");
  };
  bond_ = std::make_unique<bond::Bond>(
      "bond", get_name(), shared_from_this(), on_bond_broken_callback, on_bond_formed_callback);
  bond_->setHeartbeatPeriod(0.10);
  const auto connect_timeout_value =
        std::max(bond_timeout_, static_cast<double>(bond::msg::Constants::DEFAULT_CONNECT_TIMEOUT));
  bond_->setConnectTimeout(connect_timeout_value);
  bond_->setHeartbeatTimeout(bond_timeout_);

  bond_->start();
  RCLCPP_INFO(
    get_logger(),
    "The bond (%s) connection to the lifecycle manager has been started (heartbeat timeout: %.2lf seconds)",
    get_name(), bond_timeout_);

  std::list<CallbackReturnT> ret;
  for (auto &particles : particles_population_) {
    ret.push_back(particles->on_activate(state));
  }

  if (std::count(ret.begin(), ret.end(), CallbackReturnT::SUCCESS) ==
      ret.size()) {
    RCLCPP_INFO(get_logger(), "Activated");
    return CallbackReturnT::SUCCESS;
  }

  RCLCPP_ERROR(get_logger(), "Error activating");
  return CallbackReturnT::FAILURE;
}

CallbackReturnT
MH_AMCL_Node::on_deactivate(const rclcpp_lifecycle::State &state) {
  RCLCPP_INFO(get_logger(), "Deactivating...");

  // Reset the timers
  predict_timer_.reset();
  correct_timer_.reset();
  reseed_timer_.reset();
  publish_particles_timer_.reset();
  publish_position_timer_.reset();
  hypothesis_timer_.reset();

  // Reset the subscribers
  map_sub_.reset();
  init_pose_sub_.reset();

  // Reset the message filter for laser messages
  laser_scan_connection_.disconnect();
  laser_scan_filter_.reset();
  laser_sub_.reset();

  // Reset the bond
  bond_.reset();

  std::list<CallbackReturnT> ret;
  for (auto &particles : particles_population_) {
    ret.push_back(particles->on_deactivate(state));
  }

  if (std::count(ret.begin(), ret.end(), CallbackReturnT::SUCCESS) ==
      ret.size()) {
    RCLCPP_INFO(get_logger(), "Deactivated");
    return CallbackReturnT::SUCCESS;
  }

  RCLCPP_ERROR(get_logger(), "Error deactivating");
  return CallbackReturnT::FAILURE;
}

CallbackReturnT MH_AMCL_Node::on_cleanup(const rclcpp_lifecycle::State &state) {
  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
MH_AMCL_Node::on_shutdown(const rclcpp_lifecycle::State &state) {
  return CallbackReturnT::SUCCESS;
}

CallbackReturnT MH_AMCL_Node::on_error(const rclcpp_lifecycle::State &state) {
  return CallbackReturnT::SUCCESS;
}

void MH_AMCL_Node::publish_particles() {
  auto start = now();

  // Publish the markers for the particles for each hypothesis, using a different color in each iteration
  utils::Color color = utils::RED;
  int i = 0;
  for (const auto &hypothesis : particles_population_) {
    color = static_cast<utils::Color>((color + 1) % utils::NUM_COLORS);
    hypothesis->publish_particles(i++, utils::getColor(color));
  }
  RCLCPP_DEBUG_STREAM(get_logger(),
                      "Publish [" << (now() - start).seconds() << " secs]");
}

void MH_AMCL_Node::predict() {
  auto start = now();

  // Get the transformation between the last pose and the actual pose to predict the movement of the particles
  geometry_msgs::msg::TransformStamped odom2bf_msg;
  std::string error;
  if (tf_buffer_.canTransform("odom", "base_footprint", tf2::TimePointZero,
                              &error)) {
    odom2bf_msg = tf_buffer_.lookupTransform("odom", "base_footprint",
                                             tf2::TimePointZero);

    last_time_ = odom2bf_msg.header.stamp;

    tf2::Stamped<tf2::Transform> odom2bf;
    tf2::fromMsg(odom2bf_msg, odom2bf);

    if (valid_prev_odom2bf_) {
      tf2::Transform bfprev2bf = odom2prevbf_.inverse() * odom2bf;

      for (auto &particles : particles_population_) {
        particles->predict(bfprev2bf);
      }
    }

    valid_prev_odom2bf_ = true;
    odom2prevbf_ = odom2bf;
  }

  RCLCPP_DEBUG_STREAM(get_logger(),
                      "Predict [" << (now() - start).seconds() << " secs]");
}

void MH_AMCL_Node::map_callback(
    nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg) {
  // Every time a new map is sent, update the costmap and the map matcher
  costmap_ = std::make_shared<beluga_ros::OccupancyGrid>(msg);
  map_matcher_ = std::make_shared<mh_amcl::MapMatcher>(msg);
}

void MH_AMCL_Node::laser_callback(sensor_msgs::msg::LaserScan::ConstSharedPtr msg) {
  // Store the last laser message received
  last_laser_ = msg;
}

void MH_AMCL_Node::correct() {
  auto start = now();

  if (last_laser_ == nullptr || last_laser_->ranges.empty() ||
      costmap_ == nullptr) {
    return;
  }

  for (auto &particles : particles_population_) {
    particles->correct_once(*last_laser_, costmap_);
  }

  last_time_ = last_laser_->header.stamp;
}

void MH_AMCL_Node::reseed() {
  auto start = now();

  for (auto &particles : particles_population_) {
    particles->reseed();
  }
}

void MH_AMCL_Node::initpose_callback(
    geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr
      pose_msg) {
  // Having a new initial pose retriggers the algorithm
  if (pose_msg->header.frame_id == "map") {
    RCLCPP_INFO(get_logger(), "Map initial pose received!");
    tf2::Transform pose;
    pose.setOrigin({pose_msg->pose.pose.position.x,
                    pose_msg->pose.pose.position.y,
                    pose_msg->pose.pose.position.z});

    pose.setRotation(
        {pose_msg->pose.pose.orientation.x, pose_msg->pose.pose.orientation.y,
         pose_msg->pose.pose.orientation.z, pose_msg->pose.pose.orientation.w});

    particles_population_.clear();
    current_hypothesis_q_ = 1.0;
    current_hypothesis_ =
        std::make_shared<ParticlesDistribution>(shared_from_this());

    particles_population_.push_back(current_hypothesis_);
    current_hypothesis_->on_configure(get_current_state());
    current_hypothesis_->init(pose);

    if (get_current_state().id() ==
        lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
      current_hypothesis_->on_activate(get_current_state());
    }
    RCLCPP_INFO(get_logger(), "Finished callback!");
  } else {
    RCLCPP_WARN(get_logger(), "Not possible to init particles in frame %s",
                pose_msg->header.frame_id.c_str());
  }
}

void MH_AMCL_Node::publish_position() {
  if (costmap_ == nullptr || last_laser_ == nullptr) {
    return;
  }

  geometry_msgs::msg::PoseWithCovarianceStamped pose =
      current_hypothesis_->get_pose();

  // Publish pose
  if (pose_pub_->get_subscription_count() > 0) {
    pose.header.frame_id = "map";
    pose.header.stamp = last_time_;
    pose_pub_->publish(pose);
  }

  // Publish particle cloud
  if (particles_pub_->get_subscription_count() > 0) {
    geometry_msgs::msg::PoseArray particles_msg;
    beluga_ros::assign_particle_cloud(current_hypothesis_->get_particles(),
                                      particles_msg);
    beluga_ros::stamp_message("map", last_time_, particles_msg);

    particles_pub_->publish(particles_msg);
  }

  // Publish tf map -> odom
  tf2::Transform map2robot;
  tf2::Stamped<tf2::Transform> robot2odom;
  const auto &tpos = pose.pose.pose.position;
  const auto &tor = pose.pose.pose.orientation;
  map2robot.setOrigin({tpos.x, tpos.y, tpos.z});
  map2robot.setRotation({tor.x, tor.y, tor.z, tor.w});

  std::string error;
  if (tf_buffer_.canTransform("base_footprint", "odom", tf2::TimePointZero,
                              &error)) {
    auto robot2odom_msg = tf_buffer_.lookupTransform("base_footprint", "odom",
                                                     tf2::TimePointZero);
    tf2::fromMsg(robot2odom_msg, robot2odom);

    auto map2odom = map2robot * robot2odom;

    geometry_msgs::msg::TransformStamped transform;
    transform.header.frame_id = "map";
    transform.header.stamp = last_time_;
    transform.child_frame_id = "odom";

    transform.transform = tf2::toMsg(map2odom);

    tf_broadcaster_->sendTransform(transform);
  } else {
    RCLCPP_WARN(get_logger(), "Timeout TFs [%s]", error.c_str());
  }
}

geometry_msgs::msg::Pose MH_AMCL_Node::toMsg(const tf2::Transform &tf) {
  geometry_msgs::msg::Pose ret;
  ret.position.x = tf.getOrigin().x();
  ret.position.y = tf.getOrigin().y();
  ret.position.z = tf.getOrigin().z();
  ret.orientation.x = tf.getRotation().x();
  ret.orientation.y = tf.getRotation().y();
  ret.orientation.z = tf.getRotation().z();
  ret.orientation.w = tf.getRotation().w();

  return ret;
}

void MH_AMCL_Node::manage_hypotheses() {
  if (!last_laser_ || !costmap_ || !map_matcher_ || !multihypothesis_) {
    RCLCPP_WARN(get_logger(), "Returning in manage hypotheses");
    return;
  }

  // Retrieve the candidate transforms
  const auto &tfs = map_matcher_->get_matches(*last_laser_);
  std::vector<std::shared_ptr<ParticlesDistribution>> new_hypotheses;

  // Create new Hypotheses for each tranform
  for (const auto &transform : tfs) {
    if (transform.weight <= min_candidate_weight_) {
      continue;
    }

    geometry_msgs::msg::Pose posetf = toMsg(transform.transform);
    bool covered = std::any_of(particles_population_.begin(), particles_population_.end(),
      [&](const auto &distr) {
        double dist_xy, theta_diff;
        get_distances(distr->get_pose().pose.pose, posetf, dist_xy, theta_diff);
        return (dist_xy < min_candidate_distance_ && theta_diff < min_candidate_angle_);
      });

    // Only add the hypothesis if it's representative and we haven't reached the maximum number of hypotheses
    if (!covered && particles_population_.size() < max_hypotheses_) {
      auto new_distr = std::make_shared<ParticlesDistribution>(shared_from_this());
      new_distr->on_configure(get_current_state());
      new_distr->init(transform.transform);
      new_distr->on_activate(get_current_state());
      new_hypotheses.push_back(new_distr);
    }
    if (particles_population_.size() + new_hypotheses.size() >= max_hypotheses_) {
      break;
    }
  }

  // Insert the new hypotheses
  particles_population_.insert(particles_population_.end(), new_hypotheses.begin(), new_hypotheses.end());

  // Remove low quality hypotheses
  particles_population_.erase(std::remove_if(particles_population_.begin(), particles_population_.end(),
    [&](const auto &distr) {
      double quality = distr->get_quality();
      bool low_quality = quality < low_q_hypo_threshold_;
      bool very_low_quality = quality < very_low_q_hypo_threshold_;
      bool in_free = get_cost(distr->get_pose().pose.pose) == utils::FREE_SPACE;
      return (particles_population_.size() > 1 && (!in_free || very_low_quality || (low_quality && particles_population_.size() == max_hypotheses_)));
    }), particles_population_.end());

  // Merge similar hypotheses
  for (auto it1 = particles_population_.begin(); it1 != particles_population_.end(); ++it1) {
    auto it2 = std::next(it1);
    while (it2 != particles_population_.end()) {
      double dist_xy, theta_diff;
      get_distances((*it1)->get_pose().pose.pose, (*it2)->get_pose().pose.pose, dist_xy, theta_diff);
      if (dist_xy < hypo_merge_distance_ && theta_diff < hypo_merge_angle_) {
        (*it1)->merge(**it2);
        it2 = particles_population_.erase(it2);
      } else {
        ++it2;
      }
    }
  }

  // Select the best hypothesis
  current_hypothesis_ = *std::max_element(particles_population_.begin(), particles_population_.end(),
    [](const auto &a, const auto &b) { return a->get_quality() < b->get_quality(); });
  current_hypothesis_q_ = current_hypothesis_->get_quality();

  if (current_hypothesis_q_ < good_hypo_threshold_) {
    current_hypothesis_ = particles_population_.front();
    current_hypothesis_q_ = current_hypothesis_->get_quality();
  }

  // Debug the output
  std::cout << "=====================================" << std::endl;
  for (const auto &amcl : particles_population_) {
    std::cout << (amcl == current_hypothesis_ ? "->\t" : "") << amcl->get_quality() << std::endl;
  }
  std::cout << "=====================================" << std::endl;
}

signed char MH_AMCL_Node::get_cost(const geometry_msgs::msg::Pose &pose) {
  // Get the corresponding cost from the costmap for a specific pose
  auto [local_x, local_y] =
      utils::worldToMapNoBounds(costmap_, pose.position.x, pose.position.y);

  std::optional<signed char> opt = costmap_->data_at(local_x, local_y);
  if (opt.has_value()) {
    return static_cast<unsigned char>(opt.value());
  } else {
    return utils::NO_INFORMATION;
  }
}

void MH_AMCL_Node::get_distances(const geometry_msgs::msg::Pose &pose1,
                                 const geometry_msgs::msg::Pose &pose2,
                                 double &dist_xy, double &dist_theta) {
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

} // namespace mh_amcl
