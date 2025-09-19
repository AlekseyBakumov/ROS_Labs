ros2 run turtlesim turtlesim_node</br>
ros2 topic echo /turtle1/cmd_vel</br>
ros2 topic pub /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.8}}"</br>
