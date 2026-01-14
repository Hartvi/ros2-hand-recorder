## ROS2 hand teleoperation from webcam
![Demo](media/hand_follow_viz.gif)


![Hand alignment](media/kinova_gripper.gif)

What's going on:
- `hand_image_node` - use opencv2 to capture webcam footage in 1280x720
- `hand_points_node` - subscribe to the image published in the above node and plug the feed into mediapipe hand landmark detection
- `hand_publisher_node` - subscribe to aforementioned landmarks and publish 3D estimate of the points
- `hand_frame_node` - extract transform from the 3D estimate of the points and publish the transform w.r.t. the camera frame that is also defined in this node
- `controller_node` - take the hand transform and publish a `PoseStamped` for inverse kinematics
- `trac_ik_node` - use the `trac_ik` package to perform inverse kinematics every time the hand pose is published and publish state joints
- `joint_state_merger` - joins the joint states - main joints for inverse kinematics, and the rest for the gripper joint
- `gripper_publisher` - takes in hand points and determines whether the gripper should be open or closed

## How to launch
- ./utils build - build the project
- ./utils run - run the project
- ./utils clean - remove the log & install & build directories (sometimes needed when build fails)
- ./utils clean build run - all of the above
- ./utils run robot:=panda - run with panda robot instead of kinova (no gripper atm)

To launch rviz2 independently (at least on my machine):
- `unset GTK_PATH`
- `ros2 run rviz2 rviz2`

Requirements:
- ROS2 Jazzy
- `sudo apt install ros-${ROS_DISTRO}-trac-ik`
- mediapipe

