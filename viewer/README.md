# URDF Viewer

Local web viewer for packages exported by this script.

1. On Windows, run `start_viewer.bat`.
2. Open `http://localhost:8000` in Chrome or Edge.
3. Press the folder button.
4. Select the complete generated package folder, for example `my_robot_description`.

The viewer reads `urdf/*.urdf` or `urdf/*.xacro`, resolves `package://.../meshes/*.stl` meshes inside the selected folder, and displays the robot in 3D.

Notes:
- The pure `.urdf` file is the recommended path.
- `revolute`, `continuous`, and `prismatic` joints appear as sliders for motion testing.
- Do not open `index.html` directly with `file://`; Chrome/Edge block local JavaScript modules because of CORS.
- The viewer uses Three.js and Lucide from CDNs, so it needs an internet connection when opened.
- It does not run ROS or Gazebo; it only visualizes geometry and applies URDF joint transforms.
