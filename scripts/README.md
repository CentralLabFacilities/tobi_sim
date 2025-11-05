# Scripts

## simple_geometry_to_xacro.py

Script to generate mujoco geometries from ecwm simple_geometry.yaml file.

For more information see comment in file.

## generate_scene.py

Script to generate mujoco scene from map

1. Generate scene with script (try --help for more information)
2. Use `roslaunch tiago_mujoco tiago.launch end_effector:=schunk-wsg-cupro moveit:=false tuck_arm:=false use_moveit_camera:=false unpause:=true scene_xml:=<PathToYourScene> rviz:=false` to inspect results.
