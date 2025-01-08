# IMPORTANT NOTE: This does not keep the source outside of this file so if u run manually start by setting this source
source install/setup.bash
# NOTE: Add the & side at the end to run it in the background, but it is harder to stop
ros2 run fizzbuzz number_publisher &
ros2 run fizzbuzz fizzbuzz_subscriber &

# ros2 run fizzbuzz number_publisher
# ros2 run fizzbuzz fizzbuzz_subscriber