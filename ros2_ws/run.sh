# IMPORTANT NOTE: This does not keep the source outside of this file so if u run manually start by setting this source
source install/setup.bash
colcon build
ros2 run fizzbuzz number_publisher
