// Copyright 2023-2024 Ekumen, Inc.
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

#include <range/v3/view/concat.hpp>
#include <range/v3/view/repeat.hpp>

#include <torch/script.h>
#include <torch/torch.h>

#include <sophus/se2.hpp>
#include <sophus/se3.hpp>

#include <beluga/beluga.hpp>
#include <beluga/policies/every_n.hpp>
#include <beluga/policies/on_motion.hpp>
#include <beluga/policies/policy.hpp>
#include <beluga/sensor/data/landmark_map.hpp>

#include <beluga_ros/particle_cloud.hpp>
#include <beluga_ros/tf2_sophus.hpp>

// HEADER WITH BELUGA TORCH EXPERIMENTAL TYPES
#include "beluga_torch_utility.hpp"

#include <rclcpp/rclcpp.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <message_filters/subscriber.h>

#include <tf2/convert.h>
#include <tf2/utils.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/create_timer_ros.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <beluga_feature_map_server_msgs/msg/discrete_feature_map.hpp>

namespace beluga_demo_ml_localization {

template<typename Measurement, typename State, typename Weight>
class DenseGridSensorModel {
public:
  using pose_type = State;
  using weight_type = Weight;
  using image_type = Measurement;

  DenseGridSensorModel(const std::string& image_encoder_path, const std::string& dense_grid_path)
      {
      try {
        RCLCPP_INFO(rclcpp::get_logger("DGENode"), "Loading dense grid from %s", dense_grid_path.c_str());
        torch::jit::Module dense_grid_module = torch::jit::load(dense_grid_path);
        bool found = false;
        for (const auto& buffer : dense_grid_module.named_buffers()) {
          dense_grid_tensor_ = buffer.value;
          found = true;
          break;
        }

        if (!found) {
          RCLCPP_ERROR(rclcpp::get_logger("DGENode"), "Dense grid module at %s has no buffers.", dense_grid_path.c_str());
          throw std::runtime_error("Dense grid module has no buffers.");
        }

        RCLCPP_INFO(rclcpp::get_logger("DGENode"), "Loading image encoder from %s", image_encoder_path.c_str());
        image_encoder_ = torch::jit::load(image_encoder_path);

        // Move tensors to the correct device
        torch::Device device = torch::kCPU;
        bool device_found = false;
        for (const auto& param : image_encoder_.parameters()) {
          device = param.device();
          device_found = true;
          break;
        }
        if (!device_found) {
          for (const auto& buffer : image_encoder_.named_buffers()) {
            device = buffer.value.device();
            device_found = true;
            break;
          }
        }
        image_encoder_.to(device);
        dense_grid_tensor_ = dense_grid_tensor_.to(device);

        RCLCPP_INFO(rclcpp::get_logger("DGENode"), "Squeezing dense grid tensor");
        dense_grid_tensor_ = dense_grid_tensor_.squeeze();
      } catch (const c10::Error& e) {
        RCLCPP_ERROR(rclcpp::get_logger("DGENode"), "Error loading and squeezing dense grid tensor: %s", e.what());
        throw;
      }
      initialize_grid_parameters();
      RCLCPP_INFO(rclcpp::get_logger("DGENode"), "DenseGridSensorModel initialized");
  }

  DenseGridSensorModel(torch::jit::Module&& image_encoder, const torch::Tensor& dense_grid_tensor)
      : image_encoder_(std::move(image_encoder)), dense_grid_tensor_(dense_grid_tensor) {
      initialize_grid_parameters();
  }

  [[nodiscard]] auto operator()(image_type image) {
      // Convert image to torch::IValue and move it to the correct device
      auto image_tensor = beluga_torch::to_tensor(std::move(image));
      torch::Device device = torch::kCPU;
      bool device_found = false;
      for (const auto& param : image_encoder_.parameters()) {
        device = param.device();
        device_found = true;
        break;
      }
      if (!device_found) {
        for (const auto& buffer : image_encoder_.named_buffers()) {
          device = buffer.value.device();
          device_found = true;
          break;
        }
      }
      image_tensor = image_tensor.to(device);
      torch::Tensor image_embedding = image_encoder_.forward({image_tensor}).toTensor();

      return [this, image_embedding = std::move(image_embedding)](const Sophus::SE2d& pose) {
          // Search for the precomputed embedding from the dense grid used at initialization
          const torch::Tensor pose_embedding_tensor = get_embedding(pose);

          // Rotate the pose feature to match the orientation of the pose
          // double theta = pose.so2().log();
          // auto rotated_pose_feature = rotate_circular_feature(pose_embedding_tensor, theta);

          // Compute similarity
          double similarity = compute_similarity(image_embedding, pose_embedding_tensor);

          // Return the similarity as the weight
          return static_cast<weight_type>(similarity);
      };
  }

private:
    torch::jit::Module image_encoder_;
    torch::Tensor dense_grid_tensor_; // (X_size_, Y_size_, Features)

    // Grid parameters
    int X_size_;
    int Y_size_;
    int F_size_;

    double theta_max_ = 2 * M_PI;

    void initialize_grid_parameters() {
        X_size_ = dense_grid_tensor_.size(0);
        Y_size_ = dense_grid_tensor_.size(1);
        F_size_ = dense_grid_tensor_.size(2);
    }

    torch::Tensor get_embedding(const Sophus::SE2d& pose) const {
        // Extract x, y, theta from the pose
        double x = pose.translation().x();
        double y = pose.translation().y();
        double theta = pose.so2().log();

        // Normalize theta
        theta = std::fmod(theta, theta_max_);
        if (theta < 0) {
            theta += theta_max_;
        }

        // Compute indices
        int i = static_cast<int>((x));
        int j = static_cast<int>((y));
        int k = static_cast<int>((theta));

        // Clamp indices to valid range
        i = std::clamp(i, 0, X_size_ - 1);
        j = std::clamp(j, 0, Y_size_ - 1);
        // This is wrong, k should be interval instead of single number
        k = std::clamp(k, 0, F_size_ - 1);

        // Return the embedding from the tensor
        return dense_grid_tensor_.index({i, j, k});
    }


    torch::Tensor rotate_circular_feature(const torch::Tensor& feature, double theta) const {
      // This function is all weird, need to redo based on findings TODO
        int V = feature.size(0); // Segments in the circular feature
        double shift = V * theta / (2 * M_PI); // Shift in the index space

        // Create original and shifted index tensor
        torch::Tensor idx = torch::arange(V, torch::TensorOptions().dtype(torch::kDouble).device(feature.device())); // Original indices of the feature segments
        torch::Tensor shifted_idx = idx + shift; // Indices with the rotation shift applied

        // Wrap around indices
        shifted_idx = torch::remainder(shifted_idx, V);

        // Interpolation to map theta to the feature space
        torch::Tensor rotated_feature = torch::empty_like(feature);
        for (int i = 0; i < V; ++i) {
            double index = shifted_idx[i].item<double>();
            int lower_idx = static_cast<int>(std::floor(index)) % V;
            int upper_idx = (lower_idx + 1) % V;
            double frac = index - std::floor(index);

            rotated_feature[i] = (1.0 - frac) * feature[lower_idx] + frac * feature[upper_idx]; // Linear interpolation
        }

        return rotated_feature;
    }


    double compute_similarity(const torch::Tensor& feature1, const torch::Tensor& feature2) const {
        // Calculate cosine similarity between two tensors
        torch::Tensor dot_product = torch::sum(feature1 * feature2);
        torch::Tensor norm_feature1 = torch::norm(feature1);
        torch::Tensor norm_feature2 = torch::norm(feature2);

        double similarity = dot_product.item<double>() / (norm_feature1.item<double>() * norm_feature2.item<double>());
        return similarity;
    }
};

class MLMCLFilter {
public:
  using Particle = std::tuple<Sophus::SE2d, beluga::Weight>;

  explicit MLMCLFilter(const std::string& image_encoder_path, const std::string& dense_grid_path)
      : motion_model_(motion_model_params()),
        uniform_state_distribution_{
           // TODO: Calculate X, Y size from dense_grid_path
            Eigen::AlignedBox2d(1000,
                                2000)},
        sensor_model_(image_encoder_path, dense_grid_path) {
    auto motion_policy = beluga::policies::on_motion<Sophus::SE2d>(
        kMinTranslationForUpdate, kMinRotationForUpdate);
    update_policy_ = beluga::make_policy(
        [=](const Sophus::SE2d &odometry,
            const cv::Mat &image) mutable {
          return motion_policy(odometry) && !image.empty();
        });
    resample_policy_ = beluga::policies::every_n(kResampleInterval);
  }

  bool update(Sophus::SE2d odometry, cv::Mat image) {
    if (particles_.empty()) {
      return false;
    }

    bool did_update = false;
    if (update_policy_(odometry, image) || force_update_) {
      particles_ |=
          beluga::actions::propagate(
              motion_model_(odometry_window_ << odometry)) |                //
          beluga::actions::reweight(sensor_model_(std::move(image))) | //
          beluga::actions::normalize;

      const double random_state_probability =
          random_state_probability_estimator_(particles_);

      if (resample_policy_(particles_)) {
        auto random_state =
            ranges::compose(beluga::make_from_state<Particle>,
                            std::ref(uniform_state_distribution_));

        if (random_state_probability > 0.0) {
          random_state_probability_estimator_.reset();
        }

        particles_ |=
            beluga::views::sample |
            beluga::views::random_intersperse(std::move(random_state),
                                              random_state_probability) |
            beluga::views::take_while_kld(spatial_hasher_, //
                                          kMinParticles,   //
                                          kMaxParticles,   //
                                          kKldEpsilon,     //
                                          kKldZ) |
            beluga::actions::assign;
      }

      estimate_ = beluga::estimate(beluga::views::states(particles_),
                                   beluga::views::weights(particles_));

      force_update_ = false;
      did_update = true;
    }
    return did_update;
  }

  void initialize(const Sophus::SE2d &mean,
                  const Sophus::Matrix3d &covariance) {
    auto distribution =
        beluga::MultivariateNormalDistribution{mean, covariance};
    particles_ = beluga::views::sample(std::move(distribution)) |
                 ranges::views::transform(beluga::make_from_state<Particle>) |
                 ranges::views::take_exactly(kMaxParticles) |
                 ranges::to<beluga::TupleVector>;
    force_update_ = true;
  }

  const auto &particles() const { return particles_; }

  const auto &estimate() const { return estimate_; }

private:
  static constexpr beluga::DifferentialDriveModelParam motion_model_params() {
    auto params = beluga::DifferentialDriveModelParam{};
    params.rotation_noise_from_rotation = 0.1;
    params.rotation_noise_from_translation = 0.5;
    params.translation_noise_from_translation = 0.1;
    params.translation_noise_from_rotation = 0.5;
    return params;
  }

  static constexpr beluga::LandmarkModelParam sensor_model_params() {
    auto params = beluga::LandmarkModelParam{};
    params.sigma_bearing = 0.1;
    params.sigma_range = 0.2;
    return params;
  }

  static constexpr size_t kMinParticles = 500;
  static constexpr size_t kMaxParticles = 2000;

  static constexpr double kKldZ = 3.0;
  static constexpr double kKldEpsilon = 0.05;
  static constexpr double kLinearResolution = 0.5;
  const double kAngularResolution = Sophus::Constants<double>::pi() / 180.;
  beluga::spatial_hash<Sophus::SE2d> spatial_hasher_{kLinearResolution,
                                                     kAngularResolution};

  static constexpr double kMinTranslationForUpdate = 0.25;
  static constexpr double kMinRotationForUpdate = 0.2;
  beluga::any_policy<Sophus::SE2d, cv::Mat> update_policy_;

  static constexpr double kAlphaSlow = 0.001;
  static constexpr double kAlphaFast = 0.1;
  beluga::ThrunRecoveryProbabilityEstimator random_state_probability_estimator_{
      kAlphaSlow, kAlphaFast};
  static constexpr size_t kResampleInterval = 1;
  beluga::any_policy<beluga::TupleVector<Particle>> resample_policy_;

  beluga::TupleVector<Particle> particles_;
  std::optional<std::pair<Sophus::SE2d, Eigen::Matrix3d>> estimate_;

  beluga::RollingWindow<Sophus::SE2d, 2> odometry_window_;
  beluga::DifferentialDriveModel<Sophus::SE2d> motion_model_;
  DenseGridSensorModel<cv::Mat, Sophus::SE2d, double> sensor_model_;
  beluga::MultivariateUniformDistribution<Sophus::SE2d, Eigen::AlignedBox2d>
      uniform_state_distribution_;

  bool force_update_{false};
};

class DGENode : public rclcpp::Node {
public:
  explicit DGENode(const rclcpp::NodeOptions &options)
      : rclcpp::Node("dge_node", options) {
    {
      constexpr bool kUseDedicatedThread = true;
      tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
      tf_buffer_->setCreateTimerInterface(
          std::make_shared<tf2_ros::CreateTimerROS>(
              get_node_base_interface(), get_node_timers_interface()));
      tf_listener_ = std::make_unique<tf2_ros::TransformListener>(
          *tf_buffer_, this, !kUseDedicatedThread);
      tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
    }    

    auto common_callback_group =
        create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    auto common_subscription_options = rclcpp::SubscriptionOptions{};
    common_subscription_options.callback_group = common_callback_group;    

    {
      using namespace std::chrono_literals;
      auto callback = std::bind(&DGENode::timer_callback, this);
      timer_ =
          create_wall_timer(200ms, std::move(callback), common_callback_group);
    }

    {
      using namespace std::placeholders;
      auto callback = std::bind(&DGENode::initial_pose_callback, this, _1);
      initial_pose_sub_ = create_subscription<PoseWithCovarianceStamped>(
          "initialpose", rclcpp::SystemDefaultsQoS(), std::move(callback),
          common_subscription_options);

      RCLCPP_INFO(get_logger(), "Subscribed to %s topic",
                  initial_pose_sub_->get_topic_name());
    }

    {
      feature_detections_sub_ =
          std::make_unique<message_filters::Subscriber<FeatureDetections>>(
              this, "camera/image_raw", rmw_qos_profile_sensor_data,
              common_subscription_options);

      feature_detections_filter_ =
      std::make_unique<tf2_ros::MessageFilter<FeatureDetections>>(
              *feature_detections_sub_, *tf_buffer_, kOdomFrameID, 10,
              get_node_logging_interface(), get_node_clock_interface(),
              tf2::durationFromSec(kTransformTolerance));

      using namespace std::placeholders;
      feature_detections_connection_ =
          feature_detections_filter_->registerCallback(
              std::bind(&DGENode::feature_detections_callback, this, _1));
      RCLCPP_INFO(get_logger(), "Subscribed to %s topic",
                  feature_detections_sub_->getTopic().c_str());
    }

    particle_cloud_pub_ =
        create_publisher<PoseArray>("particle_cloud", rclcpp::SensorDataQoS());
    particle_markers_pub_ = create_publisher<MarkerArray>(
        "particle_markers", rclcpp::SensorDataQoS());
    pose_pub_ = create_publisher<PoseWithCovarianceStamped>(
        "pose", rclcpp::SystemDefaultsQoS());

    static auto initial_guess =
      std::make_pair(Sophus::SE2d{Sophus::SO2d{}, Eigen::Vector2d{0., -2.}},
                      Eigen::Matrix3d::Identity() * 0.25);

    auto last_known_estimate =
        particle_filter_ ? particle_filter_->estimate() : std::nullopt;
    const auto &[mean, covariance] =
        last_known_estimate.value_or(initial_guess);

    const auto package_share_directory =
        ament_index_cpp::get_package_share_directory("beluga_demo_ml_localization");
    const auto image_encoder_path =
        package_share_directory + "/models/fplocnet_s3d_pano_jit.pt";
    const auto dense_grid_path =
        package_share_directory + "/models/floorplan_tensors.pt";

    particle_filter_ =
        std::make_unique<MLMCLFilter>(image_encoder_path, dense_grid_path);
    particle_filter_->initialize(mean, covariance);
  }

private:
  using PoseArray = geometry_msgs::msg::PoseArray;
  using MarkerArray = visualization_msgs::msg::MarkerArray;
  using FeatureDetections =
      sensor_msgs::msg::Image;
  using PoseWithCovarianceStamped =
      geometry_msgs::msg::PoseWithCovarianceStamped;
  using DiscreteFeatureMap =
      beluga_feature_map_server_msgs::msg::DiscreteFeatureMap;

  void feature_detections_callback(
      const FeatureDetections::ConstSharedPtr &features) {
    if (!particle_filter_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "Ignoring detected features because the particle "
                           "filter has not been initialized");
      return;
    }
    const auto timestamp = tf2_ros::fromMsg(features->header.stamp);

    auto odom_to_base_transform = Sophus::SE2d{};
    try {
      const auto transform =
          tf_buffer_->lookupTransform(kOdomFrameID, kBaseFrameID, timestamp);
      tf2::convert(transform.transform, odom_to_base_transform);
    } catch (const tf2::TransformException &error) {
      RCLCPP_ERROR(get_logger(), "Could not transform from odom to base: %s",
                   error.what());
      return;
    }

    auto base_to_sensor_transform = Sophus::SE3d{};
    try {
      const auto &sensor_frame_id = features->header.frame_id;
      const auto transform =
          tf_buffer_->lookupTransform(kBaseFrameID, sensor_frame_id, timestamp);
      tf2::convert(transform.transform, base_to_sensor_transform);
    } catch (const tf2::TransformException &error) {
      RCLCPP_ERROR(get_logger(),
                   "Could not transform from base to features: %s",
                   error.what());
      return;
    }

    cv_bridge::CvImagePtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvCopy(features, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception& e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }

    const auto update_start_time = std::chrono::high_resolution_clock::now();
    const bool did_update =
        particle_filter_->update(odom_to_base_transform, cv_ptr->image);
    const auto update_stop_time = std::chrono::high_resolution_clock::now();

    if (did_update) {
      const auto update_duration = update_stop_time - update_start_time;
      RCLCPP_INFO(
          get_logger(),
          "Particle filter update iteration stats: %ld particles - %.3fms",
          particle_filter_->particles().size(),
          std::chrono::duration<double, std::milli>(update_duration).count());

      // new pose messages are only published on updates to the filter
      const auto &[mean, covariance] = particle_filter_->estimate().value();

      auto message = geometry_msgs::msg::PoseWithCovarianceStamped{};
      message.header.stamp = features->header.stamp;
      message.header.frame_id = kMapFrameID;
      tf2::toMsg(mean, message.pose.pose);
      tf2::covarianceEigenToRowMajor(covariance, message.pose.covariance);
      pose_pub_->publish(message);

      // Update the estimation for the transform between the global frame and
      // the odom frame
      last_map_to_odom_transform_ = mean * odom_to_base_transform.inverse();
    }

    // transforms are always published to keep them current
    if (last_map_to_odom_transform_.has_value()) {
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

  // void map_callback(const DiscreteFeatureMap::ConstSharedPtr &feature_map) {
  //   if (feature_map->positions.empty()) {
  //     RCLCPP_WARN(get_logger(), "Ignoring map update because it is empty");
  //     return;
  //   }

  //   std::vector<beluga::LandmarkPositionDetection> landmarks;
  //   landmarks.reserve(feature_map->positions.size());
  //   const auto categories = ranges::views::concat(feature_map->categories,
  //                                                 ranges::views::repeat(0));
  //   for (const auto &[position, category] :
  //        ranges::views::zip(feature_map->positions, categories)) {
  //     auto &landmark = landmarks.emplace_back();
  //     tf2::fromMsg(position, landmark.detection_position_in_robot);
  //     landmark.category = category;
  //   }

  //   static auto initial_guess =
  //       std::make_pair(Sophus::SE2d{Sophus::SO2d{}, Eigen::Vector2d{0., -2.}},
  //                      Eigen::Matrix3d::Identity() * 0.25);
  //   auto last_known_estimate =
  //       particle_filter_ ? particle_filter_->estimate() : std::nullopt;
  //   const auto &[mean, covariance] =
  //       last_known_estimate.value_or(initial_guess);

  //   particle_filter_ =
  //       std::make_unique<MLMCLFilter>("image_encoder_weights.npy", "dense_grid_tensor.npt");
  //   particle_filter_->initialize(mean, covariance);

  //   RCLCPP_INFO(get_logger(),
  //               "Particle filter initialized with %ld particles about mean "
  //               "pose x=%g, y=%g, yaw=%g",
  //               particle_filter_->particles().size(), mean.translation().x(),
  //               mean.translation().y(), mean.so2().log());
  // }

  void timer_callback() {
    if (!particle_filter_) {
      return;
    }

    if (particle_cloud_pub_->get_subscription_count() > 0) {
      auto message = geometry_msgs::msg::PoseArray{};
      beluga_ros::assign_particle_cloud(particle_filter_->particles(), message);
      beluga_ros::stamp_message(kMapFrameID, now(), message);
      particle_cloud_pub_->publish(message);
    }

    if (particle_markers_pub_->get_subscription_count() > 0) {
      auto message = visualization_msgs::msg::MarkerArray{};
      beluga_ros::assign_particle_cloud(particle_filter_->particles(), message);
      beluga_ros::stamp_message(kMapFrameID, now(), message);
      particle_markers_pub_->publish(message);
    }
  }

  void initial_pose_callback(
      const PoseWithCovarianceStamped::ConstSharedPtr &message) {
    if (!particle_filter_) {
      RCLCPP_WARN(get_logger(), "Ignoring initial pose request because the "
                                "particle filter has not been initialized");
      return;
    }

    auto mean = Sophus::SE2d{};
    tf2::convert(message->pose.pose, mean);

    auto covariance = Eigen::Matrix3d{};
    tf2::covarianceRowMajorToEigen(message->pose.covariance, covariance);

    try {
      particle_filter_->initialize(mean, covariance);
    } catch (const std::runtime_error &error) {
      RCLCPP_ERROR(get_logger(), "Could not reinitialize filter: %s",
                   error.what());
      return;
    }

    RCLCPP_INFO(get_logger(),
                "Particle filter reinitialized with %ld particles about mean "
                "pose x=%g, y=%g, yaw=%g",
                particle_filter_->particles().size(), mean.translation().x(),
                mean.translation().y(), mean.so2().log());
  }

  const std::string kMapFrameID = "map";
  const std::string kOdomFrameID = "odom";
  const std::string kBaseFrameID = "base_footprint";
  static constexpr double kTransformTolerance = 1.0;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::optional<Sophus::SE2d> last_map_to_odom_transform_;
  std::optional<Sophus::SE2d> last_odom_to_base_transform_;

  std::unique_ptr<MLMCLFilter> particle_filter_;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<PoseArray>::SharedPtr particle_cloud_pub_;
  rclcpp::Publisher<MarkerArray>::SharedPtr particle_markers_pub_;
  rclcpp::Publisher<PoseWithCovarianceStamped>::SharedPtr pose_pub_;

  std::unique_ptr<message_filters::Subscriber<FeatureDetections>>
      feature_detections_sub_;
  std::unique_ptr<tf2_ros::MessageFilter<FeatureDetections>>
      feature_detections_filter_;
  message_filters::Connection feature_detections_connection_;

  rclcpp::Subscription<PoseWithCovarianceStamped>::SharedPtr initial_pose_sub_;
  // rclcpp::Subscription<DiscreteFeatureMap>::SharedPtr feature_map_sub_;
};

} // namespace beluga_demo_ml_localization

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(beluga_demo_ml_localization::DGENode)
