git clone -b jazzy https://github.com/ROBOTIS-GIT/DynamixelSDK.git src/DynamixelSDK
git clone -b jazzy https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git src/turtlebot3_msgs
git clone -b jazzy https://github.com/ROBOTIS-GIT/turtlebot3.git src/turtlebot3
git clone -b jazzy https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git src/turtlebot3_simulations

rm -rf build install log
ps -ef|grep ros2 |awk '{print $2}' | xargs kill -SIGINT
colcon build --symlink-install
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export TURTLEBOT3_MODEL=burger

ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py &
ros2 run turtle_autonomous_patrol monitor_node &
ros2 run turtle_autonomous_patrol max_speed_service &
ros2 run turtle_autonomous_patrol patrol_server &
ros2 topic echo /alert
ros2 topic echo /cmd_vel geometry_msgs/msg/TwistStamped

ros2 run turtle_autonomous_patrol patrol_client
ros2 service call /set_max_speed std_srvs/srv/SetBool "{data: true}"
ros2 param set /max_speed_service max_speed 0.7
ros2 topic pub /cmd_vel geometry_msgs/msg/TwistStamped "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: ''}, twist: {linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}}"拓展：

ros2 launch turtle_autonomous_patrol patrol_launch.py
GAZEBO_MODEL_PATH=:/ros2_ws/turtle_autonomous_patrol_ws/install/turtlebot3_description/share/turtlebot3_description ros2 launch turtle_autonomous_patrol patrol_launch.py
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/ros2_ws/turtle_autonomous_patrol_ws/install/turtlebot3_description/share/turtlebot3_description 

 export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/ros2_ws/turtle_autonomous_patrol_ws/install/turtlebot3_description/share/
 export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:/ros2_ws/turtle_autonomous_patrol_ws/install/turtlebot3_description/share/

ros2 launch turtle_autonomous_patrol patrol_launch.py