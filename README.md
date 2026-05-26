# FUSION_TO_URDF

Fusion 360 script for exporting ROS 2 Jazzy-compatible robot description packages.

## Outputs

The exporter creates a ROS 2 package with:

- A pure URDF file in `urdf/<robot>.urdf`
- A Xacro file in `urdf/<robot>.xacro`
- STL meshes in `meshes/`
- ROS 2 launch files in `launch/`
- `ros2_control` configuration in `config/controller.yaml`

## ROS 2 Jazzy

After exporting a package, copy it into a ROS 2 workspace:

```bash
cp -r <robot>_description ~/ros2_ws/src/
cd ~/ros2_ws
colcon build
source install/setup.bash
ros2 launch <robot>_description display.launch.py
```

## Web Viewer

This repo also includes a local web viewer for generated URDF packages.
The viewer can load the exported meshes and move `revolute`, `continuous`, and `prismatic` joints with sliders.

On Windows:

```bat
viewer\start_viewer.bat
```

Then open:

```text
http://localhost:8000
```

Select the complete generated package folder, for example `<robot>_description`.
