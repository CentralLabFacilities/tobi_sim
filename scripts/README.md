# Scripts

## simple_geometry_to_xacro.py

Script to generate mujoco geometries from ecwm simple_geometry.yaml file.

For more information see comment in file.

## generate_walls.py

Script to generate mujoco scene from map.

- **Left-Click** on blank: Add new node. If a node was selected previously, a wall is created
- **Left-Click** on existing node: Select/Deselect node
- **Ctrl+Left-Click** on existing node: Create wall from currently selected to clicked node
- **Enter**: Safe to file
- **Delete**: If node is selected, this node and all connections its involved in are deleted

1. Generate scene with script (try --help for more information)
2. Use `roslaunch tiago_mujoco tiago.launch end_effector:=schunk-wsg-cupro moveit:=false tuck_arm:=false use_moveit_camera:=false unpause:=true scene_xml:=<PathToYourScene> rviz:=false` to inspect results.
