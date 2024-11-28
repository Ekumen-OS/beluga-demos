// Copyright 2024 Ekumen, Inc.
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


#include <memory>
#include <utility>
#include <ratio>
#include <stdexcept>
#include <string>
#include <string_view>
#include <tuple>
#include <chrono>
#include <cstddef>
#include <execution>
#include <functional>
#include <limits>
#include <optional>

#include <range/v3/range/conversion.hpp>
#include <range/v3/view/take_exactly.hpp>
#include <range/v3/range/conversion.hpp>

#include <Eigen/Core>
#include <sophus/se2.hpp>
#include <sophus/se3.hpp>

#include <openvdb/openvdb.h>
#include <openvdb/Grid.h>
#include <openvdb/io/File.h>

#include <beluga/beluga.hpp>
#include <beluga_ros/beluga_ros.hpp>

#include <message_filters/subscriber.h>

#include <tf2/convert.h>
#include <tf2/exceptions.h>
#include <tf2/time.h>

#include <tf2_ros/buffer.h>
#include <tf2_ros/buffer_interface.h>
#include <tf2_ros/create_timer_ros.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <std_msgs/msg/color_rgba.hpp>


namespace beluga_demo_amcl3_localization {

// Weighted SE(2) state particle type.
using particle_type = std::tuple<Sophus::SE3d, beluga::Weight>;
// Motion model variant type for runtime selection support.
using motion_model = beluga::DifferentialDriveModel3d;
// Sensor model variant type for runtime selection support.
using sensor_model = beluga::LikelihoodFieldModel3<openvdb::FloatGrid, beluga_ros::SparsePointCloud3<float>>;
// Execution policy variant type for runtime selection support.
using execution_policy = std::execution::parallel_policy;

class Amcl3 {
 public:
    explicit Amcl3(motion_model &&motion, sensor_model &&sensor, const Sophus::SE3d& pose, const Sophus::Matrix6d& covariance, execution_policy policy = std::execution::par)
        : motion_model_{std::move(motion)},
          sensor_model_{std::move(sensor)},
          execution_policy_{std::move(policy)},
          spatial_hasher_{kSpatialResolutionLineal, kSpatialResolutionAngular},
          update_policy_{beluga::policies::on_motion<Sophus::SE3d>(kUpdateMinD, kUpdateMinA)},
          resample_policy_{beluga::policies::every_n(kResampleInterval)} {
        if (kRecoveryAlphaFast) {
            resample_policy_ = resample_policy_ && beluga::policies::on_effective_size_drop;
        }        
        // Initialize particles using a custom distribution.
        particles_ = beluga::views::sample(std::move(beluga::MultivariateNormalDistribution{pose, covariance})) |                    
                     ranges::views::transform(beluga::make_from_state<particle_type>) |  
                     ranges::views::take_exactly(kMaxParticles) |                
                     ranges::to<beluga::TupleVector>;
        force_update_ = true;
    }
    
    auto update(Sophus::SE3d base_pose_in_odom, beluga_ros::SparsePointCloud3<float> pointcloud)
        -> std::optional<std::pair<Sophus::SE3d, Sophus::Matrix6d>> {
        if (particles_.empty()) {
            return std::nullopt;
        }

        if (!update_policy_(base_pose_in_odom) && !force_update_) {
            return std::nullopt;
        }

        particles_ |=
            beluga::actions::propagate(execution_policy_, motion_model_(control_action_window_ << base_pose_in_odom)) | 
            beluga::actions::reweight(execution_policy_, sensor_model_(std::move(pointcloud))) |                        
            beluga::actions::normalize(execution_policy_);        

        if (resample_policy_(particles_)) {

            particles_ |= beluga::views::sample |
                          beluga::views::take_while_kld(
                            spatial_hasher_,        //
                            kMinParticles,  //
                            kMaxParticles,  //
                            kLdEpsilon,    //
                            kLdZ) |
                        beluga::actions::assign;
        }

        force_update_ = false;
        return beluga::estimate(beluga::views::states(particles_), beluga::views::weights(particles_));
    }

    // Returns a reference to the current set of particles.
    const auto& particles() const { return particles_; }

    private:
    beluga::TupleVector<particle_type> particles_;
    motion_model motion_model_;
    sensor_model sensor_model_;
    execution_policy execution_policy_;
    beluga::spatial_hash<Sophus::SE3d> spatial_hasher_;
    beluga::any_policy<Sophus::SE3d> update_policy_;
    beluga::any_policy<decltype(particles_)> resample_policy_;
    beluga::RollingWindow<Sophus::SE3d, 2> control_action_window_;
    bool force_update_{true};

    // Parameters
    // Particle filter
    static constexpr double kUpdateMinD = 0.25;
    static constexpr double kUpdateMinA = 0.2;
    static constexpr size_t kResampleInterval = 10;
    static constexpr int kMinParticles = 500;
    static constexpr int kMaxParticles = 5000;
    static constexpr double kLdEpsilon = 0.05;
    static constexpr double kLdZ = 3.0;
    static constexpr double kRecoveryAlphaSlow = 0.001;
    static constexpr double kRecoveryAlphaFast = 0.1;
    static constexpr double kSpatialResolutionLineal = 0.07;
    static constexpr double kSpatialResolutionAngular = 0.1;

};

class Amcl3Node : public rclcpp::Node {
 public:
    // Constructor.
    explicit Amcl3Node(const rclcpp::NodeOptions &options) 
        : rclcpp::Node("amcl3_example_node", options) {
        // Initialize OpenVDB
        openvdb::initialize();
        // Load OpenVDB map
        openvdb::io::File file(kMapFile);
        // Open the file.  This reads the file header, but not any grids.
        file.open();
        // Read the entire contents of the file and return a list of grid pointers.
        openvdb::GridPtrVecPtr grids = file.getGrids();
        // Close the file
        file.close();
        // Cast the generic grid pointer to a FloatGrid pointer.
        openvdb::FloatGrid::Ptr grid = openvdb::gridPtrCast<openvdb::FloatGrid>((*grids)[0]);

        // Create Likelihood 3D Field Sensor Model
        auto params_sm = beluga::LikelihoodFieldModel3Param{};
        params_sm.max_obstacle_distance = kLaserLikelihoodMaxDist;
        params_sm.max_laser_distance = kLaserMaxRange;
        params_sm.z_hit = kZHit;
        params_sm.z_random = kZRand;
        params_sm.sigma_hit = kSigmaHit;

        // Create Differential Motion Model
        auto params_mm = beluga::DifferentialDriveModelParam{};
        params_mm.rotation_noise_from_rotation = kAlpha1;
        params_mm.rotation_noise_from_translation = kAlpha2;
        params_mm.translation_noise_from_translation = kAlpha3;
        params_mm.translation_noise_from_rotation = kAlpha4;

        // Initial position
        const auto pose = Sophus::SE3d{
            Sophus::SO3d{},
            Eigen::Vector3d{
                kInitialPoseX,
                kInitialPoseY,
                kInitialPoseZ,
            },
        };

        Eigen::Matrix<double, 6, 6> covariance = Eigen::Matrix<double, 6, 6>::Zero();
        covariance.coeffRef(0, 0) = kInitialPoseCovarianceX;
        covariance.coeffRef(1, 1) = kInitialPoseCovarianceY;
        covariance.coeffRef(2, 2) = kInitialPoseCovarianceZ;
        covariance.coeffRef(3, 3) = kInitialPoseCovarianceYaw;
        covariance.coeffRef(4, 4) = kInitialPoseCovariancePitch;
        covariance.coeffRef(5, 5) = kInitialPoseCovarianceRoll;
        
        // Create filter
        particle_filter_ = std::make_unique<Amcl3>(
            motion_model{params_mm},
            sensor_model{params_sm, *grid},
            pose,
            covariance);

        // TFs
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
        tf_buffer_->setCreateTimerInterface(std::make_shared<tf2_ros::CreateTimerROS>(get_node_base_interface(), get_node_timers_interface()));

        tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_, this, !kUseDedicatedThread);
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

        // Callback options
        callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
        rclcpp::SubscriptionOptions common_subscription_options;
        common_subscription_options.callback_group = callback_group_;
        
        // Pointcloud subcription
        pointcloud_sub_ = std::make_unique<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>>(
            this, kPointcloudTopic, rmw_qos_profile_sensor_data,
            common_subscription_options);

        // Message filter that caches pointcloud readings until it is possible to transform
        // from lidar frame to odom frame and update the particle filter.
        pointcloud_filter_ = std::make_unique<tf2_ros::MessageFilter<sensor_msgs::msg::PointCloud2>>(
            *pointcloud_sub_, *tf_buffer_, kOdomFrameId, 10, get_node_logging_interface(),
            get_node_clock_interface(), tf2::durationFromSec(kTransformTolerance));

        pointcloud_connection_ =
            pointcloud_filter_->registerCallback(std::bind(&Amcl3Node::pointcloud_callback, this, std::placeholders::_1));
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Subscribed to pointcloud_topic: %s", pointcloud_sub_->getTopic().c_str());

        pose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("amcl3_poses", rclcpp::SystemDefaultsQoS());

        particle_markers_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("particle_markers", rclcpp::SensorDataQoS());

        timer_ = create_wall_timer(std::chrono_literals::operator""ms(200), std::bind(&Amcl3Node::timer_callback, this), callback_group_);

        if (kDisplayMap) {
            m_visualization_marker_pub_ = create_publisher<visualization_msgs::msg::Marker>("vdb_map_visualization", rclcpp::SystemDefaultsQoS());
            display_map(grid);
        } 
    }

 private:
    // Callback for laser scan updates.
    void pointcloud_callback(sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud) {
        if (!particle_filter_) {
            RCLCPP_WARN_THROTTLE(rclcpp::get_logger("rclcpp"), *get_clock(), 2000, "Ignoring pointcloud data because the particle filter has not been initialized");
            return;
        }

        auto base_pose_in_odom = Sophus::SE3d{};
        try {
            // Use the lookupTransform overload with no timeout since we're not using a dedicated
            // tf thread. The message filter we are using avoids the need for it.
            tf2::convert(
                tf_buffer_
                    ->lookupTransform(
                        kOdomFrameId, kBaseFrameId,
                        tf2_ros::fromMsg(cloud->header.stamp))
                    .transform,
                base_pose_in_odom);
        } catch (const tf2::TransformException& error) {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Could not transform from odom to base: %s", error.what());
            return;
        }

        auto lidar_pose_in_base = Sophus::SE3d{};
        try {
            tf2::convert(
                tf_buffer_
                    ->lookupTransform(
                        kBaseFrameId, cloud->header.frame_id,
                        tf2_ros::fromMsg(cloud->header.stamp))
                    .transform,
                lidar_pose_in_base);
        } catch (const tf2::TransformException& error) {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Could not transform from base to lidar: %s", error.what());
            return;
        }

        const auto update_start_time = std::chrono::high_resolution_clock::now();
        const auto new_estimate = particle_filter_->update(
            base_pose_in_odom,  //
            beluga_ros::SparsePointCloud3<float>{
                cloud,
                lidar_pose_in_base
            });
        const auto update_stop_time = std::chrono::high_resolution_clock::now();
        const auto update_duration = update_stop_time - update_start_time;

        if (new_estimate.has_value()) {
            const auto& [base_pose_in_map, _] = new_estimate.value();
            last_known_odom_transform_in_map_ = base_pose_in_map * base_pose_in_odom.inverse();
            last_known_estimate_ = new_estimate.value();

            RCLCPP_INFO(
                rclcpp::get_logger("rclcpp"), "Particle filter update iteration stats: %ld particles - %.3fms",
                particle_filter_->particles().size(), 
                std::chrono::duration<double, std::milli>(update_duration).count());
        }

        // Transforms are always published to keep them current.
        if (kTfBroadcast) {
            if (last_known_odom_transform_in_map_.has_value()) {
            auto message = geometry_msgs::msg::TransformStamped{};
            // Sending a transform that is valid into the future so that odom can be used.
            const auto expiration_stamp = tf2_ros::fromMsg(cloud->header.stamp) +
                                            tf2::durationFromSec(kTransformTolerance);
            message.header.stamp = tf2_ros::toMsg(expiration_stamp);
            message.header.frame_id = kGlobalFrameId;
            message.child_frame_id = kOdomFrameId;
            message.transform = tf2::toMsg(*last_known_odom_transform_in_map_);
            tf_broadcaster_->sendTransform(message);
            }
        }

        // New pose messages are only published on updates to the filter.
        if (new_estimate.has_value()) {
            auto message = geometry_msgs::msg::PoseWithCovarianceStamped{};
            message.header.stamp = cloud->header.stamp;
            message.header.frame_id = kGlobalFrameId;
            const auto& [base_pose_in_map, base_pose_covariance] = new_estimate.value();
            message.pose = tf2::toMsg(base_pose_in_map, base_pose_covariance);
            pose_pub_->publish(message);
        }
    }  

    void timer_callback() {
        if (!particle_filter_) {
            return;
        }

        if (particle_markers_pub_->get_subscription_count() > 0) {
            auto message = visualization_msgs::msg::MarkerArray{};
            //beluga_ros::assign_particle_cloud(particle_filter_->particles(), message);
            //beluga_ros::stamp_message(kGlobalFrameId, this->get_clock()->now(), message);
            //particle_markers_pub_->publish(message);
        }
    }
  
    // Map displayer
    // Adapted from:
    // https://github.com/fzi-forschungszentrum-informatik/vdb_mapping_ros
    void display_map(const openvdb::FloatGrid::Ptr grid) const {
        visualization_msgs::msg::Marker marker_msg;
        const auto point_color = []() { std_msgs::msg::ColorRGBA msg;
                                        msg.r = 1.0;
                                        msg.g = 1.0;
                                        msg.b = 1.0;
                                        msg.a = 1.0;
                                        return msg; 
        }();
        const openvdb::CoordBBox bbox = grid->evalActiveVoxelBoundingBox();
        const openvdb::Vec3d min_world_coord = grid->indexToWorld(bbox.getStart());
        const openvdb::Vec3d max_world_coord = grid->indexToWorld(bbox.getEnd());
        const double min_z = min_world_coord.z();
        const double max_z = max_world_coord.z();

        for (openvdb::FloatGrid::ValueOnCIter iter = grid->cbeginValueOn(); iter; ++iter) {
            openvdb::Vec3d world_coord = grid->indexToWorld(iter.getCoord());
            if (world_coord.z() < min_z || world_coord.z() > max_z) {
                continue;
            }
            
            geometry_msgs::msg::Point cube_center;
            cube_center.x = world_coord.x();
            cube_center.y = world_coord.y();
            cube_center.z = world_coord.z();
            marker_msg.points.push_back(cube_center);
            marker_msg.colors.push_back(point_color);
        }

        const double size             = grid->transform().voxelSize()[0];
        marker_msg.header.frame_id    = kGlobalFrameId;
        marker_msg.header.stamp       = this->get_clock()->now();
        marker_msg.id                 = 0;
        marker_msg.type               = visualization_msgs::msg::Marker::CUBE_LIST;
        marker_msg.scale.x            = size;
        marker_msg.scale.y            = size;
        marker_msg.scale.z            = size;
        marker_msg.color.a            = 1.0;
        marker_msg.pose.orientation.w = 1.0;
        marker_msg.frame_locked       = true;

        if (marker_msg.points.size() > 0) {
            marker_msg.action = visualization_msgs::msg::Marker::ADD;
        }
        else {
            marker_msg.action = visualization_msgs::msg::Marker::DELETE;
        }

        m_visualization_marker_pub_->publish(marker_msg);
    }

    // Callback group
    rclcpp::CallbackGroup::SharedPtr callback_group_;
    // Timer
    rclcpp::TimerBase::SharedPtr timer_;
    // Pointcloud updates subscription.
    std::unique_ptr<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>> pointcloud_sub_;
    // Transform synchronization filter for pointcloud updates.
    std::unique_ptr<tf2_ros::MessageFilter<sensor_msgs::msg::PointCloud2>> pointcloud_filter_;
    // Map publisher
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr m_visualization_marker_pub_;
    // Connection for pointcloud updates filter and callback.
    message_filters::Connection pointcloud_connection_;
    // Particle marker publisher
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr particle_markers_pub_;
    // Particle filter instance.
    std::unique_ptr<Amcl3> particle_filter_;
    // Last known pose estimate, if any.
    std::pair<Sophus::SE3d, Eigen::Matrix<double, 6, 6>> last_known_estimate_;
    // Estimated pose publisher.
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
    // Transforms buffer.
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    // Transforms broadcaster.
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
    // Last known map to odom correction estimate, if any.
    std::optional<Sophus::SE3d> last_known_odom_transform_in_map_;        

    // Parameters
    // Constructor
    static constexpr bool kUseDedicatedThread = true;
    static constexpr bool kDisplayMap = false;
    const std::string kPointcloudTopic = "/pointcloud";
    const std::string kOdomFrameId = "odom";
    const std::string kMapFile = "/home/developer/ws/src/beluga_demo/localization/beluga_demo_amcl3_localization/maps/map.vdb";

    // Get Initial Estimate
    static constexpr double kInitialPoseX = 0.0;
    static constexpr double kInitialPoseY = 0.0;
    static constexpr double kInitialPoseZ = 0.0;
    static constexpr double kInitialPoseYaw = 0.0;
    static constexpr double kInitialPosePitch = 0.0;
    static constexpr double kInitialPoseRoll = 0.0;

    static constexpr double kInitialPoseCovarianceX = 0.25;
    static constexpr double kInitialPoseCovarianceY = 0.25;
    static constexpr double kInitialPoseCovarianceZ = 0.25;
    static constexpr double kInitialPoseCovarianceYaw = 0.15;
    static constexpr double kInitialPoseCovariancePitch = 0.15;
    static constexpr double kInitialPoseCovarianceRoll = 0.15;

    // Get Motion Model
    static constexpr double kAlpha1 = 0.1;
    static constexpr double kAlpha2 = 0.05;
    static constexpr double kAlpha3 = 0.1;
    static constexpr double kAlpha4 = 0.05;

    // Get Sensor Model
    static constexpr double kLaserLikelihoodMaxDist = 100;
    static constexpr double kLaserMaxRange = 100;
    static constexpr double kZHit = 0.5;
    static constexpr double kZRand = 0.5;
    static constexpr double kSigmaHit = 0.2;

    // Pointcloud Callback
    const std::string kBaseFrameId = "base_link";
    const std::string kGlobalFrameId = "map";
    static constexpr bool kTfBroadcast = true;
    static constexpr double kTransformTolerance = 0.3;
};

}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(beluga_demo_amcl3_localization::Amcl3Node)

