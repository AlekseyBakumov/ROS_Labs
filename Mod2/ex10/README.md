ros2 run ex10 text_to_cmd_vel</br>
ros2 topic pub /cmd_text std_msgs/msg/String "{data: turn_left}"</br>
ros2 run turtlesim turtlesim_node --ros-args --remap turtle1/cmd_vel:=/cmd_vel</br>
