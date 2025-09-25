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

#ifndef MH_AMCL__REWEIGHT_UPDATE_HITS_ACTION_HPP_
#define MH_AMCL__REWEIGHT_UPDATE_HITS_ACTION_HPP_

#include <range/v3/functional/bind_back.hpp>

#include <beluga_ros/occupancy_grid.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "beluga_demo_mh_amcl/utils.hpp"

namespace mh_amcl::actions
{

    // Define a struct for our custom action, following the Beluga pattern
    struct reweight_and_update_hits_fn
    {
        // Takes the particle range and does the work
        template <class Range>
        auto operator()(
            Range &range,
            const sensor_msgs::msg::LaserScan &scan,
            const std::shared_ptr<beluga_ros::OccupancyGrid> &costmap,
            const tf2::Transform &base_to_laser_tf,
            double distance_perception_error) const -> Range &
        {
            for (auto &particle : range)
            {
                // Update both weight and hits with a single loop
                tf2::Transform map2bf = utils::se2dToTf2Transform(particle.state);

                double total_prob = 0.0;
                double accum_log_likelihood = 0.0;
                int valid_beams = 0;

                for (size_t j = 0; j < scan.ranges.size(); ++j)
                {
                    if (!std::isfinite(scan.ranges[j]))
                    {
                        continue;
                    }

                    tf2::Transform laser2point = utils::get_transform_to_read(scan, static_cast<int>(j));
                    double err_m = utils::get_error_distance_to_obstacle(
                        map2bf, base_to_laser_tf, laser2point, scan, costmap);

                    if (!std::isinf(err_m))
                    {
                        // Update the accumulated probability
                        const double normalized_distance_error = err_m / distance_perception_error;
                        const double prob = (utils::INV_SQRT_2PI / distance_perception_error) * std::exp(-0.5 * normalized_distance_error * normalized_distance_error);
                        total_prob += std::clamp(prob, 0.0, 1.0);
                        valid_beams++;
                    }
                }

                // Update the particle's weight from the likelihood, the hits with the probability
                if (valid_beams > 0)
                {
                    particle.weight += total_prob;
                    particle.hits = total_prob / static_cast<float>(scan.ranges.size());
                }
                // TODO: Review if hits should be computed like this (not having good results so far)
                // // Update the particle's weight and hits using the average likelihood
                // if (valid_beams > 0)
                // {
                //     const float average_likelihood = total_prob / static_cast<float>(valid_beams);
                //     particle.weight *= average_likelihood;
                //     particle.hits = average_likelihood;
                // }
                else
                {
                    // Assign a small weight and 0 hits if no valid beams
                    particle.weight *= 1e-9;
                    particle.hits = 0.0;
                }
            }
            return range;
        }

        // Takes configuration, returns a pipeable action
        auto operator()(
            const sensor_msgs::msg::LaserScan &scan,
            const std::shared_ptr<beluga_ros::OccupancyGrid> &costmap,
            const tf2::Transform &base_to_laser_tf,
            double distance_perception_error) const
        {
            // Create a closure with all the arguments except the particles range
            return ranges::make_action_closure(ranges::bind_back(
                reweight_and_update_hits_fn{}, scan, costmap, base_to_laser_tf, distance_perception_error));
        }
    };

    // Create a single, global instance of our action object, just like Beluga does
    inline constexpr reweight_and_update_hits_fn reweight_and_update_hits;

} // namespace mh_amcl::actions

#endif // MH_AMCL__REWEIGHT_UPDATE_HITS_ACTION_HPP_
