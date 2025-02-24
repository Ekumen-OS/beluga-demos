# Copyright 2025 Ekumen, Inc.
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

import bpy
import random

obj = bpy.context.active_object

if obj.type == 'MESH':
    bpy.ops.object.mode_set(mode='OBJECT')

    mesh = obj.data
    faces = [face for face in mesh.polygons]

    num_vértices = 50000

    # Create random vertices
    for _ in range(num_vértices):
        face = random.choice(faces)
        v1, v2, v3 = [mesh.vertices[i] for i in face.vertices]

        u = random.random()
        v = random.random()

        if u + v > 1:
            u = 1 - u
            v = 1 - v

        point = v1.co * (1 - u - v) + v2.co * u + v3.co * v

        new_vert = mesh.vertices.add(count=1)
        mesh.vertices[-1].co = point

    mesh.update()

    bpy.ops.object.mode_set(mode='EDIT')

else:
    print("The selected object is not a mesh.")
