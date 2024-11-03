NOTICE:
When starting Teleoperation mode, change the 25th line of code in the file spawn_robot_ros2.launch.py to position = [37.0, 42.5, 0.5], which will make it easier to position your footage to the robot's position.

Command to execute files:

Start the Gazebo environment and load the controller:
ros2 launch cars gazebo.launch.py

Start RVIZ 2 and the necessary settings for robot visualization:
ros2 launch cars display.launch.py

Launched Gazebo and RVIZ 2 at the same time:
ros2 launch cars debug.launch.py

Start Teleoperation mode:
open two terminals,
Enter this code in one of the terminals:
ros2 launch cars teleopp.launch.py
then enter this code in another terminals:
ros2 run keyboard_control keyboard_control_node


Start Autonomous mode:
open two terminals,
Enter this code in one of the terminals:
ros2 launch cars gazebo.launch.py
then enter this code in another terminals:
 ros2 run my_robot_control proportional_controller
