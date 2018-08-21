# Assignment 1 Notes

## Create package
`catkin_create_pkg ass1 rospy roscpp std_msgs sensor_msgs geometry_msgs nav_msgs image_transport rviz`

`rospack depends1 ass1`

- **comp3431_starter** depends
    - roscpp
    - std_msgs
    - sensor_msgs
    - geometry_msgs
    - image_transport
    - rviz
- **turtlebot3_follower** depends
    - rospy
    - std_msgs
    - sensor_msgs
    - geometry_msgs
    - nav_msgs
- **turtlebot3_follow_filter** depends
    - laser_filter
- **turtlebot3_teleop** depends
    - rospy
    - geometry_msgs