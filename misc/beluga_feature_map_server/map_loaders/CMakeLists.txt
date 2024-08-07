# Copyright 2023 Ekumen, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

add_library(yaml_feature_map_loader STATIC)
target_include_directories(yaml_feature_map_loader PUBLIC include)
target_sources(yaml_feature_map_loader PUBLIC src/yaml_feature_map_loader.cpp)
target_compile_features(yaml_feature_map_loader PUBLIC cxx_std_17)
target_link_libraries(yaml_feature_map_loader PUBLIC)
set_target_properties(yaml_feature_map_loader
                      PROPERTIES POSITION_INDEPENDENT_CODE ON)
ament_target_dependencies(yaml_feature_map_loader
                          PUBLIC beluga_feature_map_server_msgs yaml_cpp_vendor)

add_library(map_loaders INTERFACE)
target_link_libraries(map_loaders INTERFACE yaml_feature_map_loader)

add_subdirectory(test)
