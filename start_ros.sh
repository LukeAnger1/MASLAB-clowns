
# Launch the camera nodes
ros2 run v4l2_camera v4l2_camera_node --ros-args -p video_device:="/dev/black" -p camera_info_url:="file:///home/team17/MASLAB-clownPen-15/camera_test/black/ost.yaml" --remap /image_raw/compressed:=/image_raw/compressed2 &
ros2 run v4l2_camera v4l2_camera_node --ros-args -p video_device:="/dev/grey" -p camera_info_url:="file:///home/team17/MASLAB-clownPen-15/camera_test/grey/ost.yaml" --remap /image_raw:=/image_raw2 &

# Use the grey camera
ros2 run image_processing cube_detect &
ros2 run image_processing cube_locate &

# Use the black camera
ros2 run image_processing2 cube_detect &
ros2 run image_processing2 cube_locate &

# Process the cameras, make decisions, run the motors
ros2 run motor_control map_generator &
ros2 run motor_control decision_node &
ros2 run motor_control drive_node &