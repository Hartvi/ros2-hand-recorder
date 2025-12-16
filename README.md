## kinect from wish


To launch rviz2:
`unset GTK_PATH`
`ros2 run rviz2 rviz2`


### TODOs
- get position and dir from hand points
- visualize robotics arm URDFs
- inverse kinematics on robot arm from pose or just position
- save data
- create data set scene

diagnose meshes:
```
ros2 topic pub --once /mesh_test visualization_msgs/msg/Marker "{
  header: {frame_id: base_link},
  ns: 'mesh_test',
  id: 0,
  type: 10,
  action: 0,
  pose: {orientation: {w: 1.0}},
  scale: {x: 1.0, y: 1.0, z: 1.0},
  color: {r: 0.8, g: 0.8, b: 0.8, a: 1.0},
  mesh_resource: 'package://hand_publisher/urdf/gen3_7dof/meshes/forearm_link.stl'
}"
```

moveit:
sudo apt install ros-${ROS_DISTRO}-moveit
ros2 interface show moveit_msgs/srv/GetPositionIK