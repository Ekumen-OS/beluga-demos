# Additional scripts

These are the steps followed in the process of generating a `map.hdf5` file from the 3D model of the world, with this directory containing the scripts that had to be created to generate it.

1. **Get a `.ply` file from the model (`.stl` file).** Open the model in blender and use the script `generate_random_vertices.py` found in this directory to generate vertices along all the faces of the model, then export the file as `.ply`. The models can be found in `beluga-demos/common/beluga_demo_gazebo/models/<name_of_the_world>/meshes`.
2. **Get a `.hdf5` file from the `.ply` one.** Use the [script](https://github.com/Ekumen-OS/beluga/blob/main/beluga_tools/beluga_tools/ply_to_ndt.py) found in [Beluga repository](https://github.com/Ekumen-OS/beluga).
