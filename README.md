# FUSION_TO_URDF

Fusion 360 script for exporting ROS 2 Jazzy-compatible robot description packages.

## Outputs

The exporter creates a ROS 2 package with:

- A pure URDF file in `urdf/<robot>.urdf`
- A Xacro file in `urdf/<robot>.xacro`
- STL meshes in `meshes/`
- ROS 2 launch files in `launch/`
- `ros2_control` configuration in `config/controller.yaml`

## Fusion 360 Model Requirements

Prepare the Fusion 360 model before running the exporter:

- The model must have **zero Fusion 360 warnings**. Fix all timeline, joint, component, body, and feature warnings before exporting.
- The robot must have one component named exactly `base_link`.
- Every moving or fixed part that should become a URDF link must be a separate Fusion component.
- Components should have valid solid bodies and physical properties.
- Use joints to connect the robot as a tree starting from `base_link`.
- Avoid closed kinematic loops; URDF requires a tree structure.
- Supported joint types are rigid/fixed, revolute, and slider/prismatic.
- Revolute and prismatic joints should have limits configured in Fusion 360 unless the joint is intended to be continuous.
- Keep component and joint names simple: use letters, numbers, and underscores when possible.
- Do not export from a design that contains temporary `old_component` objects from a previous failed export. Undo that export or reopen a clean copy first.
- Save a clean copy of the Fusion design before exporting.

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
