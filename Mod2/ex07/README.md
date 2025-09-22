ros2 run turtlesim turtlesim_node</br>
ros2 service call /spawn turtlesim/srv/Spawn "{x: 2, y: 1, theta: 0.2, name: ''}"</br>
ros2 param set /turtlesim background_g 124</br>
ros2 service list > rosservice_list.txt</br>
ros2 param dump /turtlesim > parameter_server.txt</br>
