#!/usr/bin/python3

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


#
# This script generates a YAML file with a grid of features.
# corresponding to the apriltags gazebo world.
#

header = """frame_id: map
features:"""

item = """
  - position: [{}, {}, 0.6]
    category: {}"""

print(header, end="")

for x in range(-5, 6):
    for y in range(-5, 6):
        if x in [-4, 4] or y in [-4, 4] or not (x in [-5, 5] or y in [-5, 5]):
            continue

        if x in [-5, 5] and y in [-5, 5]:
            category = 2

        if x in [-2, 0, 2] or y in [-2, 0, 2]:
            category = 0

        if x in [-1, 1] or y in [-1, 1]:
            category = 1

        if x in [-3, 3] or y in [-3, 3]:
            category = 3

        print(item.format(x, y, category), end="")

print()
