// Copyright 2023 Ekumen, Inc.
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

#include <chrono>
#include <execution>
#include <functional>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include <sophus/se2.hpp>
#include <sophus/se3.hpp>

#include <beluga/beluga.hpp>
#include <beluga/sensor.hpp>
#include <beluga/sensor/data/landmark_map.hpp>

#include <rclcpp/rclcpp.hpp>

#include <message_filters/subscriber.h>

#include <tf2/convert.h>
#include <tf2/utils.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/create_timer_ros.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav2_msgs/msg/particle_cloud.hpp>

#include <beluga_april_tag_adapter_msgs/msg/feature_detections.hpp>
#include <beluga_feature_map_server_msgs/msg/discrete_feature_map.hpp>

#include "beluga_demo_landmark_localization/tf2_sophus.hpp"

namespace beluga_demo_landmark_localization {

using BMCLInterface2d = beluga::mixin::compose_interfaces<
    beluga::BaseParticleFilterInterface,
    beluga::StorageInterface<Sophus::SE2d, beluga::Weight>,
    beluga::EstimationInterface2d, beluga::OdometryMotionModelInterface2d,
    beluga::BearingSensorModelInterface<beluga::LandmarkMap>>;

using BMCLFilter =
    ciabatta::mixin<beluga::BootstrapParticleFilter,
                    ciabatta::curry<beluga::StructureOfArrays, Sophus::SE2d,
                                    beluga::Weight, beluga::Cluster>::mixin,
                    beluga::WeightedStateEstimator2d,
                    beluga::RandomStateGenerator, beluga::AdaptiveSampler,
                    ciabatta::curry<beluga::KldLimiter, Sophus::SE2d>::mixin,
                    beluga::DifferentialDriveModel,
                    ciabatta::curry<beluga::BearingSensorModel,
                                    beluga::LandmarkMap, Sophus::SE2d>::mixin,
                    ciabatta::provides<BMCLInterface2d>::template mixin>;

void ExtractParticleCloud(const BMCLInterface2d &particle_filter,
                          nav2_msgs::msg::ParticleCloud *message) {
  // Particle weights from the filter may or may not be representative of the
  // true distribution. If we resampled, they are not, and there will be
  // multiple copies of the most likely candidates, all with unit weight. In
  // this case the number of copies is a proxy for the prob density at each
  // candidate. If we did not resample before updating the estimation and
  // publishing this message (which can happen if the resample interval is set
  // to something other than 1), then all particles are expected to be different
  // and their weights are proportional to the prob density at each candidate.
  //
  // Only the combination of both the state distribution and the candidate
  // weights together provide information about the probability density at each
  // candidate. To handle both cases, we group repeated candidates and compute
  // the accumulated weight

  struct RepresentativeData {
    Sophus::SE2d state;
    double weight{0.};
  };

  struct RepresentativeBinHash {
    std::size_t operator()(const Sophus::SE2d &s) const noexcept {
      std::size_t h1 = std::hash<double>{}(s.translation().x());
      std::size_t h2 = std::hash<double>{}(s.translation().y());
      std::size_t h3 = std::hash<double>{}(s.so2().log());
      return h1 ^ (h2 << 1) ^ (h3 << 2);
    }
  };

  struct RepresentativeBinEqual {
    bool operator()(const Sophus::SE2d &lhs,
                    const Sophus::SE2d &rhs) const noexcept {
      // good enough, since copies of the same candidate are expected to be
      // identical copies
      return lhs.translation().x() == rhs.translation().x() && //
             lhs.translation().y() == rhs.translation().y() && //
             lhs.so2().log() == rhs.so2().log();
    }
  };

  std::unordered_map<Sophus::SE2d, RepresentativeData, RepresentativeBinHash,
                     RepresentativeBinEqual>
      representatives_map;
  representatives_map.reserve(particle_filter.particle_count());
  double max_weight = 1e-5; // never risk dividing by zero

  for (const auto &[state, weight] : ranges::views::zip(
           particle_filter.states_view(), particle_filter.weights_view())) {
    auto &representative =
        representatives_map[state]; // if the element does not exist, create it
    representative.state = state;
    representative.weight += weight;
    if (representative.weight > max_weight) {
      max_weight = representative.weight;
    }
  }

  message->particles.reserve(particle_filter.particle_count());
  for (const auto &[key, representative] : representatives_map) {
    auto &particle = message->particles.emplace_back();
    tf2::toMsg(representative.state, particle.pose);
    particle.weight = representative.weight / max_weight;
  }
}

class BMCLNode : public rclcpp::Node {
public:
  explicit BMCLNode(const rclcpp::NodeOptions &options)
      : rclcpp::Node("lmcl_example_node", options) {
    constexpr bool kUseDedicatedThread = true;
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_buffer_->setCreateTimerInterface(
        std::make_shared<tf2_ros::CreateTimerROS>(get_node_base_interface(),
                                                  get_node_timers_interface()));
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(
        *tf_buffer_, this, !kUseDedicatedThread);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

    auto common_callback_group =
        create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    auto common_subscription_options = rclcpp::SubscriptionOptions{};
    common_subscription_options.callback_group = common_callback_group;

    {
      using namespace std::chrono_literals;
      auto callback = std::bind(&BMCLNode::timer_callback, this);
      timer_ =
          create_wall_timer(200ms, std::move(callback), common_callback_group);
    }

    {
      using namespace std::placeholders;
      const auto qos =
          rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
      auto callback = std::bind(&BMCLNode::map_callback, this, _1);
      feature_map_sub_ = create_subscription<
          beluga_feature_map_server_msgs::msg::DiscreteFeatureMap>(
          "landmarks_map", qos, std::move(callback),
          common_subscription_options);
      RCLCPP_INFO(get_logger(), "Subscribed to %s topic",
                  feature_map_sub_->get_topic_name());
    }

    {
      using namespace std::placeholders;
      auto callback = std::bind(&BMCLNode::initial_pose_callback, this, _1);
      initial_pose_sub_ =
          create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
              "initialpose", rclcpp::SystemDefaultsQoS(), std::move(callback),
              common_subscription_options);

      RCLCPP_INFO(get_logger(), "Subscribed to %s topic",
                  initial_pose_sub_->get_topic_name());
    }

    {
      feature_detections_sub_ = std::make_unique<message_filters::Subscriber<
          beluga_april_tag_adapter_msgs::msg::FeatureDetections>>(
          this, "landmark_detections", rmw_qos_profile_sensor_data,
          common_subscription_options);

      feature_detections_filter_ = std::make_unique<tf2_ros::MessageFilter<
          beluga_april_tag_adapter_msgs::msg::FeatureDetections>>(
          *feature_detections_sub_, *tf_buffer_, kOdomFrameID, 10,
          get_node_logging_interface(), get_node_clock_interface(),
          tf2::durationFromSec(kTransformTolerance));

      using namespace std::placeholders;
      feature_detections_connection_ =
          feature_detections_filter_->registerCallback(
              std::bind(&BMCLNode::feature_detections_callback, this, _1));
      RCLCPP_INFO(get_logger(), "Subscribed to %s topic",
                  feature_detections_sub_->getTopic().c_str());
    }

    particle_cloud_pub_ = create_publisher<nav2_msgs::msg::ParticleCloud>(
        "particle_cloud", rclcpp::SensorDataQoS());
    pose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "pose", rclcpp::SystemDefaultsQoS());
  }

private:
  void
  feature_detections_callback(const beluga_april_tag_adapter_msgs::msg::
                                  FeatureDetections::ConstSharedPtr &features) {
    if (!particle_filter_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "Ignoring detected features because the particle "
                           "filter has not been initialized");
      return;
    }

    const auto timestamp = tf2_ros::fromMsg(features->header.stamp);

    auto odom_to_base_transform = Sophus::SE2d{};
    try {
      // Use the lookupTransform overload with no timeout since we're not using
      // a dedicated tf thread. The message filter we are using avoids the need
      // for it.
      tf2::convert(
          tf_buffer_->lookupTransform(kOdomFrameID, kBaseFrameID, timestamp)
              .transform,
          odom_to_base_transform);
    } catch (const tf2::TransformException &error) {
      RCLCPP_ERROR(get_logger(), "Could not transform from odom to base: %s",
                   error.what());
      return;
    }

    auto base_to_sensor_transform = Sophus::SE3d{};
    try {
      const auto &sensor_frame_id = features->header.frame_id;
      tf2::convert(
          tf_buffer_->lookupTransform(kBaseFrameID, sensor_frame_id, timestamp)
              .transform,
          base_to_sensor_transform);
    } catch (const tf2::TransformException &error) {
      RCLCPP_ERROR(get_logger(),
                   "Could not transform from base to features: %s",
                   error.what());
      return;
    }

    const auto update_start_time = std::chrono::high_resolution_clock::now();

    const auto motion_delta =
        last_odom_to_base_transform_
            ? last_odom_to_base_transform_->inverse() * odom_to_base_transform
            : Sophus::SE2d{};

    const bool moved_enough = std::abs(motion_delta.translation().x()) > 0.25 ||
                              std::abs(motion_delta.translation().y()) > 0.25 ||
                              std::abs(motion_delta.so2().log()) > 0.2;

    const bool have_enough_features = features->positions.size() >= 1;

    bool did_update = false;
    if (force_update_ || (moved_enough && have_enough_features)) {
      particle_filter_->update_motion(odom_to_base_transform);
      particle_filter_->sample(std::execution::seq);

      BMCLFilter::measurement_type measurement;
      measurement.reserve(features->positions.size());
      for (size_t i = 0; i < features->positions.size(); ++i) {
        auto feature_position_in_sensor_frame = Eigen::Vector3d{};
        tf2::convert(features->positions[i], feature_position_in_sensor_frame);
        beluga::LandmarkBearingDetection detection_bearing_in_sensor;
        detection_bearing_in_sensor.detection_bearing_in_sensor =
            feature_position_in_sensor_frame;
        detection_bearing_in_sensor.category = features->categories[i];
        measurement.emplace_back(detection_bearing_in_sensor);
      }
      particle_filter_->update_sensor(std::move(measurement));

      particle_filter_->reweight(std::execution::seq);

      num_updates_ = (num_updates_ + 1) % kResampleIntervalCount;
      if (num_updates_ == 0) {
        particle_filter_->resample();
        did_update = true;
      }

      last_odom_to_base_transform_ = odom_to_base_transform;
    }
    force_update_ = false;

    if (did_update) {
      const auto update_stop_time = std::chrono::high_resolution_clock::now();
      const auto update_duration = update_stop_time - update_start_time;
      RCLCPP_INFO(
          get_logger(),
          "Particle filter update iteration stats: %ld particles %ld features "
          "- %.3fms",
          particle_filter_->particle_count(), features->positions.size(),
          std::chrono::duration<double, std::milli>(update_duration).count());
    }

    const auto publish_updated_estimation = !last_known_estimate_ || did_update;
    if (publish_updated_estimation) {
      last_known_estimate_ = particle_filter_->estimate();
    }

    const auto &[pose, covariance] = last_known_estimate_.value();

    // new pose messages are only published on updates to the filter
    if (publish_updated_estimation) {
      auto message = geometry_msgs::msg::PoseWithCovarianceStamped{};
      message.header.stamp = features->header.stamp;
      message.header.frame_id = kMapFrameID;
      tf2::toMsg(pose, message.pose.pose);
      tf2::covarianceEigenToRowMajor(covariance, message.pose.covariance);
      pose_pub_->publish(message);

      // Update the estimation for the transform between the global frame and
      // the odom frame
      last_map_to_odom_transform_ = pose * odom_to_base_transform.inverse();
    }

    // transforms are always published to keep them current
    if (last_map_to_odom_transform_) {
      auto message = geometry_msgs::msg::TransformStamped{};
      // Sending a transform that is valid into the future so that odom can be
      // used.
      const auto expiration_stamp = tf2_ros::fromMsg(features->header.stamp) +
                                    tf2::durationFromSec(kTransformTolerance);
      message.header.stamp = tf2_ros::toMsg(expiration_stamp);
      message.header.frame_id = kMapFrameID;
      message.child_frame_id = kOdomFrameID;
      message.transform = tf2::toMsg(last_map_to_odom_transform_.value());
      tf_broadcaster_->sendTransform(message);
    }
  }

  void map_callback(const beluga_feature_map_server_msgs::msg::
                        DiscreteFeatureMap::ConstSharedPtr &landmarks_map) {
    auto sampler_params = beluga::AdaptiveSamplerParam{};
    sampler_params.alpha_slow = 0.001;
    sampler_params.alpha_fast = 0.1;

    auto limiter_params = beluga::KldLimiterParam<Sophus::SE2d>{};
    limiter_params.min_samples = 500;
    limiter_params.max_samples = 2000;
    limiter_params.spatial_hasher = beluga::spatial_hash<Sophus::SE2d>{
        0.5, 0.5, 10. * Sophus::Constants<double>::pi() / 180.};
    limiter_params.kld_epsilon = 0.05;
    limiter_params.kld_z = 3.0;

    auto motion_model_params = beluga::DifferentialDriveModelParam{};
    motion_model_params.rotation_noise_from_rotation = 0.1;
    motion_model_params.rotation_noise_from_translation = 0.5;
    motion_model_params.translation_noise_from_translation = 0.1;
    motion_model_params.translation_noise_from_rotation = 0.5;

    auto sensor_model_params = beluga::BearingModelParam{};
    sensor_model_params.sigma_bearing = 0.025;
    sensor_model_params.sensor_pose_in_robot = Sophus::SE3d{
        Sophus::SO3d{}, // FIX THIS HERE AND IN SPOTLIGTH DETECTOR TODO
        Eigen::Vector3d{0.076, 0.000, 0.103} //
    };

    double xmin = std::numeric_limits<double>::infinity();
    double ymin = std::numeric_limits<double>::infinity();
    double zmin = std::numeric_limits<double>::infinity();
    double xmax = -std::numeric_limits<double>::infinity();
    double ymax = -std::numeric_limits<double>::infinity();
    double zmax = -std::numeric_limits<double>::infinity();
    beluga::LandmarkMap::landmarks_set_position_data landmarks;
    landmarks.reserve(landmarks_map->positions.size());
    for (size_t i = 0; i < landmarks_map->positions.size(); ++i) {
      xmin = std::min(landmarks_map->positions[i].x, xmin);
      ymin = std::min(landmarks_map->positions[i].y, ymin);
      zmin = std::min(landmarks_map->positions[i].z, zmin);
      xmax = std::max(landmarks_map->positions[i].x, xmax);
      ymax = std::max(landmarks_map->positions[i].y, ymax);
      zmax = std::max(landmarks_map->positions[i].z, zmax);
      beluga::LandmarkPositionDetection detection;
      detection.detection_position_in_robot = Eigen::Vector3d{
          landmarks_map->positions[i].x, landmarks_map->positions[i].y,
          landmarks_map->positions[i].z};
      detection.category =
          !landmarks_map->categories.empty() ? landmarks_map->categories[i] : 0;
      landmarks.emplace_back(detection);
    }

    beluga::LandmarkMapBoundaries extents;
    extents.x_min = xmin;
    extents.x_max = xmax;
    extents.y_min = ymin;
    extents.y_max = ymax;
    extents.z_min = zmin;
    extents.z_max = zmax;

    particle_filter_ = std::make_unique<BMCLFilter>(
        sampler_params, limiter_params, motion_model_params,
        sensor_model_params, beluga::LandmarkMap{extents, landmarks});

    if (last_known_estimate_) {
      const auto &[pose, covariance] = last_known_estimate_.value();
      initialize(pose, covariance);
    } else {
      auto initial_pose =
          Sophus::SE2d{Sophus::SO2d{}, Eigen::Vector2d{0., -2.}};
      initialize(initial_pose, Eigen::Matrix3d::Identity() * 0.25);
    }
  }

  void timer_callback() {
    if (!particle_filter_) {
      return;
    }

    if (particle_cloud_pub_->get_subscription_count() == 0) {
      return;
    }

    auto message = nav2_msgs::msg::ParticleCloud{};
    message.header.stamp = now();
    message.header.frame_id = kMapFrameID;
    ExtractParticleCloud(*particle_filter_, &message);
    particle_cloud_pub_->publish(std::move(message));
  }

  void initial_pose_callback(
      const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr
          &message) {
    if (!particle_filter_) {
      RCLCPP_WARN(get_logger(), "Ignoring initial pose request because the "
                                "particle filter has not been initialized");
      return;
    }

    auto pose = Sophus::SE2d{};
    tf2::convert(message->pose.pose, pose);

    auto covariance = Eigen::Matrix3d{};
    tf2::covarianceRowMajorToEigen(message->pose.covariance, covariance);

    initialize(pose, covariance);
  }

  void initialize(const Sophus::SE2d &pose, const Eigen::Matrix3d &covariance) {
    try {
      const auto mean = Eigen::Vector3d{
          pose.translation().x(), pose.translation().y(), pose.so2().log()};
      auto distribution =
          beluga::MultivariateNormalDistribution{mean, covariance};
      particle_filter_->initialize_states(
          ranges::views::generate([&distribution]() mutable {
            static auto generator = std::mt19937{std::random_device()()};
            const auto sample = distribution(generator);
            return Sophus::SE2d{Sophus::SO2d{sample.z()},
                                Eigen::Vector2d{sample.x(), sample.y()}};
          }));
      force_update_ = true;
    } catch (const std::runtime_error &error) {
      RCLCPP_ERROR(get_logger(), "Could not generate particles: %s",
                   error.what());
      return;
    }
    RCLCPP_INFO(get_logger(),
                "Particle filter initialized with %ld particles about initial "
                "pose x=%g, y=%g, yaw=%g",
                particle_filter_->particle_count(), pose.translation().x(),
                pose.translation().y(), pose.so2().log());
  }

  const std::string kMapFrameID = "map";
  const std::string kOdomFrameID = "odom";
  const std::string kBaseFrameID = "base_footprint";
  static constexpr double kTransformTolerance = 1.0;
  static constexpr int kResampleIntervalCount = 1;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::optional<Sophus::SE2d> last_map_to_odom_transform_;
  std::optional<Sophus::SE2d> last_odom_to_base_transform_;

  int num_updates_ = 0;
  bool force_update_ = false;
  std::unique_ptr<BMCLInterface2d> particle_filter_;
  std::optional<std::pair<Sophus::SE2d, Eigen::Matrix3d>> last_known_estimate_;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<nav2_msgs::msg::ParticleCloud>::SharedPtr
      particle_cloud_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
      pose_pub_;

  std::unique_ptr<message_filters::Subscriber<
      beluga_april_tag_adapter_msgs::msg::FeatureDetections>>
      feature_detections_sub_;
  std::unique_ptr<tf2_ros::MessageFilter<
      beluga_april_tag_adapter_msgs::msg::FeatureDetections>>
      feature_detections_filter_;
  message_filters::Connection feature_detections_connection_;

  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
      initial_pose_sub_;
  rclcpp::Subscription<
      beluga_feature_map_server_msgs::msg::DiscreteFeatureMap>::SharedPtr
      feature_map_sub_;
};

} // namespace beluga_demo_landmark_localization

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(beluga_demo_landmark_localization::BMCLNode)
