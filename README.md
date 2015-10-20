# turtlebot_tf
This node publish:

1- A tf from base_link to odom frame based on nav_msgs::Odometry,
2- A static tf from the laser to base_link frame.

Additional: To run the gmapping you must have a /scan topic for hokuyo laser you may refere to: http://wiki.ros.org/hokuyo_node

You can change the name of the Odometry topic and the pose of your laser

Enjoy!
