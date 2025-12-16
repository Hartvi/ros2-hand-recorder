set -x
rm -r build/
rm -r install/
rm -r log/
unset GTK_PATH
colcon build --symlink-install
source ./install/setup.bash
ros2 launch hand_publisher hand_publisher_launch.py
