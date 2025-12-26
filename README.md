## human hand => robotic hand
![Demo](media/hand_follow_viz.gif)

What's going on:
- `hand_image_node` - use opencv2 to capture webcam footage in 1280x720
- `hand_points_node` - subscribe to the image published in the above node and plug the feed into mediapipe hand landmark detection 
- `hand_publisher_node` - subscribe to aforementioned landmarks and publish 3D estimate of the points
- `hand_frame_node` - extract transform from the 3D estimate of the points and publish the transform w.r.t. the camera frame that is also defined in this node
- `controller_node` - take the hand transform and publish a `PoseStamped` for inverse kinematics
- `trac_ik_node` - use the `trac_ik` package to perform inverse kinematics every time the hand pose is published and publish state joints


To launch rviz2:
`unset GTK_PATH`
`ros2 run rviz2 rviz2`


### TODOs
- connect gripper urdf to robot urdf
- automatically detect finger & hand points to align with urdf gripper/ee pose
- save data
- create data set scene


### DONE
- smooth out movements
- fix hand transform to correspond to the correct rotation
- set camera transform
- get position and dir from hand points
- visualize robotics arm URDFs
- inverse kinematics on robot arm from pose or just position

Requirements:
`sudo apt install ros-${ROS_DISTRO}-trac-ik`


