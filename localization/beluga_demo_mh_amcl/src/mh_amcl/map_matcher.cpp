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

#include "beluga_demo_mh_amcl/map_matcher.hpp"
#include "beluga_demo_mh_amcl/utils.hpp"

namespace mh_amcl {

MapMatcher::MapMatcher(std::shared_ptr<const nav_msgs::msg::OccupancyGrid> map) {
  costmaps_.resize(NUM_LEVEL_SCALE_COSTMAP);
  // Construct the Beluga occupancy grid wrapper from the message pointer
  costmaps_[0] = std::make_shared<beluga_ros::OccupancyGrid>(map);

  for (int i = 1; i < NUM_LEVEL_SCALE_COSTMAP; i++) {
    costmaps_[i] = half_scale(costmaps_[i - 1]);
  }
}

std::shared_ptr<beluga_ros::OccupancyGrid>
MapMatcher::half_scale(std::shared_ptr<beluga_ros::OccupancyGrid> costmap_in) {
  // Compute new grid dimensions and resolution:
  unsigned int new_width  = costmap_in->width() / 2;
  unsigned int new_height = costmap_in->height() / 2;
  double new_resolution   = costmap_in->resolution() * 2.0;

  // Create a new ROS occupancy grid message and fill the metadata:
  nav_msgs::msg::OccupancyGrid new_grid;
  new_grid.info.width = new_width;
  new_grid.info.height = new_height;
  new_grid.info.resolution = new_resolution;

  // Set the new origin – for example, using the same origin as costmap_in.
  // (You might want to adjust the origin if the half-scale grid should be centered differently.)
  new_grid.info.origin.position.x = costmap_in->origin().translation().x();
  new_grid.info.origin.position.y = costmap_in->origin().translation().y();
  new_grid.info.origin.position.z = 0.0;
  // Use an identity quaternion for the orientation:
  new_grid.info.origin.orientation.w = 1.0;
  new_grid.info.origin.orientation.x = 0.0;
  new_grid.info.origin.orientation.y = 0.0;
  new_grid.info.origin.orientation.z = 0.0;

  // Resize the data vector and fill with a default value (e.g., FREE_SPACE)
  new_grid.data.resize(new_width * new_height, utils::FREE_SPACE);

  // Fill the new grid using a 2x2 window from costmap_in
  for (unsigned int i = 0; i < new_width; i++) {
    for (unsigned int j = 0; j < new_height; j++) {
      unsigned int ri = i * 2;
      unsigned int rj = j * 2;

      auto cost1_opt = costmap_in->data_at(ri, rj);
      auto cost2_opt = costmap_in->data_at(ri + 1, rj);
      auto cost3_opt = costmap_in->data_at(ri, rj + 1);
      auto cost4_opt = costmap_in->data_at(ri + 1, rj + 1);

      unsigned char cost1 = cost1_opt.has_value() ? static_cast<unsigned char>(cost1_opt.value()) : utils::NO_INFORMATION;
      unsigned char cost2 = cost2_opt.has_value() ? static_cast<unsigned char>(cost2_opt.value()) : utils::NO_INFORMATION;
      unsigned char cost3 = cost3_opt.has_value() ? static_cast<unsigned char>(cost3_opt.value()) : utils::NO_INFORMATION;
      unsigned char cost4 = cost4_opt.has_value() ? static_cast<unsigned char>(cost4_opt.value()) : utils::NO_INFORMATION;

      bool is_lethal = (cost1 == utils::LETHAL_OBSTACLE || cost2 == utils::LETHAL_OBSTACLE ||
                        cost3 == utils::LETHAL_OBSTACLE || cost4 == utils::LETHAL_OBSTACLE);
      bool is_free   = (cost1 == utils::FREE_SPACE || cost2 == utils::FREE_SPACE ||
                        cost3 == utils::FREE_SPACE || cost4 == utils::FREE_SPACE);
      bool all_unknown = (cost1 == utils::NO_INFORMATION && cost2 == utils::NO_INFORMATION &&
                          cost3 == utils::NO_INFORMATION && cost4 == utils::NO_INFORMATION);

      if (is_lethal) {
        new_grid.data[j * new_width + i] = utils::LETHAL_OBSTACLE;
      } else if (is_free) {
        new_grid.data[j * new_width + i] = utils::FREE_SPACE;
      } else if (all_unknown) {
        new_grid.data[j * new_width + i] = utils::NO_INFORMATION;
      } else {
        new_grid.data[j * new_width + i] = cost1;
      }
    }
  }

  // Wrap the new grid in a shared pointer and construct the Beluga occupancy grid
  auto new_grid_ptr = std::make_shared<nav_msgs::msg::OccupancyGrid>(new_grid);
  return std::make_shared<beluga_ros::OccupancyGrid>(new_grid_ptr);
}

std::list<TransformWeighted>
MapMatcher::get_matches(const sensor_msgs::msg::LaserScan &scan) {
  std::vector<tf2::Vector3> laser_poins = laser2points(scan);

  int start_level = NUM_LEVEL_SCALE_COSTMAP - 2;
  int min_level = 2;

  // costmaps_ is the vector of pointers to each level (resolution) individual
  // costmap
  auto [min_x, min_y] = utils::mapToWorld(costmaps_[start_level], 0, 0);
  auto [max_x, max_y] =
      utils::mapToWorld(costmaps_[start_level], costmaps_[start_level]->width(),
                        costmaps_[start_level]->height());

  std::list<TransformWeighted> candidates[NUM_LEVEL_SCALE_COSTMAP];
  candidates[start_level] =
      get_matches(start_level, laser_poins, min_x, min_y, max_x, max_y);
  candidates[start_level].sort();

  // Go from the top level to the minimum, getting the candidates of the next
  // level
  for (int level = start_level; level >= min_level; level--) {
    for (const auto &candidate : candidates[level]) {
      double min_x, max_x, min_y, max_y;
      min_x = candidate.transform.getOrigin().x() -
              costmaps_[level]->resolution() / 2.0;
      max_x = candidate.transform.getOrigin().x() +
              costmaps_[level]->resolution() / 2.0;
      min_y = candidate.transform.getOrigin().y() -
              costmaps_[level]->resolution() / 2.0;
      max_y = candidate.transform.getOrigin().y() +
              costmaps_[level]->resolution() / 2.0;

      auto new_candidates =
          get_matches(level - 1, laser_poins, min_x, min_y, max_x, max_y);

      candidates[level - 1].insert(candidates[level - 1].end(),
                                   new_candidates.begin(),
                                   new_candidates.end());
    }
  }

  candidates[min_level].sort();

  return candidates[min_level];
}

std::list<TransformWeighted>
MapMatcher::get_matches(int scale, const std::vector<tf2::Vector3> &scan,
                        float min_x, float min_y, float max_y, float max_x) {
  std::list<TransformWeighted> ret;
  const auto &costmap = costmaps_[scale];

  auto [init_i, init_j] = utils::worldToMapEnforceBounds(costmap, min_x, min_y);
  auto [end_i, end_j] = utils::worldToMapEnforceBounds(costmap, max_x, max_y);

  for (unsigned int i = init_i; i < end_i; i++) {
    for (unsigned int j = init_j; j < end_j; j++) {
      auto cost = costmap->data_at(i, j);
      if (cost == utils::FREE_SPACE) {
        double inc_theta = (M_PI / 4.0);
        for (double theta = 0; theta < 1.9 * M_PI; theta = theta + inc_theta) {
          auto [x, y] = utils::mapToWorld(costmap, i, j);
          TransformWeighted tw;

          tf2::Quaternion q;
          q.setRPY(0.0, 0.0, theta);
          tw.transform = tf2::Transform(q, {x, y, 0.0});

          tw.weight = match(scale, costmap, scan, tw.transform);

          if (tw.weight > 0.5) {
            ret.push_back(tw);
          }
        }
      }
    }
  }
  return ret;
}

float MapMatcher::match(
    int scale, const std::shared_ptr<beluga_ros::OccupancyGrid> costmap,
    const std::vector<tf2::Vector3> &scan, tf2::Transform &transform) {
  int hits = 0;
  int total = 0;

  for (int i = 0; i < scan.size(); i = i + scale) {
    tf2::Vector3 test_point = transform * scan[i];
    auto [local_x, local_y] =
        utils::worldToMapNoBounds(costmap, test_point.x(), test_point.y());

    if (local_x > 0 && local_y > 0 && local_x < costmap->width() && local_y < costmap->height() &&
        costmap->data_at(local_x, local_y) == utils::LETHAL_OBSTACLE) {
      hits++;
    }
    total++;
  }

  return static_cast<float>(hits) / static_cast<float>(total);
}

std::vector<tf2::Vector3>
MapMatcher::laser2points(const sensor_msgs::msg::LaserScan &scan) {
  std::list<tf2::Vector3> points;
  for (auto i = 0; i < scan.ranges.size(); i++) {
    if (std::isnan(scan.ranges[i]) || std::isinf(scan.ranges[i])) {
      continue;
    }

    tf2::Vector3 p;
    float dist = scan.ranges[i];
    float theta = scan.angle_min + i * scan.angle_increment;
    p.setX(dist * cos(theta));
    p.setY(dist * sin(theta));
    p.setZ(0.0);
    points.push_back(p);
  }
  return std::vector<tf2::Vector3>(points.begin(), points.end());
}

nav_msgs::msg::OccupancyGrid
toMsg(const std::shared_ptr<beluga_ros::OccupancyGrid> costmap) {
  nav_msgs::msg::OccupancyGrid grid;

  grid.info.resolution = costmap->resolution();
  grid.info.width = costmap->width();
  grid.info.height = costmap->height();

  auto [wx, wy] = utils::mapToWorld(costmap, 0, 0);
  grid.info.origin.position.x = wx - costmap->resolution() / 2;
  grid.info.origin.position.y = wy - costmap->resolution() / 2;
  grid.info.origin.position.z = 0.0;
  grid.info.origin.orientation.w = 1.0;

  grid.data.resize(grid.info.width * grid.info.height);
  // No cost translation is needed, they operate on same scale
  grid.data = costmap->data();

  return grid;
}

bool operator<(const TransformWeighted &tw1, const TransformWeighted &tw2) {
  // To sort incremental
  return tw1.weight > tw2.weight;
}

} // namespace mh_amcl
