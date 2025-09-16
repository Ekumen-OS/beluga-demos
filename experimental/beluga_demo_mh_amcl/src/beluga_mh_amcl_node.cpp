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

#include "lifecycle_msgs/msg/transition.hpp"
#include "rclcpp/rclcpp.hpp"

#include "beluga_demo_mh_amcl/mh_amcl.hpp"

int main(int argc, char **argv)
{
  // Just initiate rclcpp and create an instance of the node
  rclcpp::init(argc, argv);
  auto mh_amcl = std::make_shared<mh_amcl::MH_AMCL_Node>();
  auto executor = rclcpp::executors::MultiThreadedExecutor();
  executor.add_node(mh_amcl->get_node_base_interface());
  executor.spin();
  rclcpp::shutdown();

  return 0;
}
