#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <turtlesim/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>

std::string turtle_name;



void poseCallback(const nav_msgs::Odometry::ConstPtr& msg){
  //odom tf
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin( tf::Vector3(msg->pose.pose.position.x, msg->pose.pose.position.y, 0.0) );
  tf::Quaternion q(0,0,msg->pose.pose.orientation.z,msg->pose.pose.orientation.w);
  
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/odom", "/base_link"));

  //laser tf
  static tf::TransformBroadcaster br2;
  tf::Transform transform2;
  transform2.setOrigin(tf::Vector3(0.0, 0.0, 0.0478) );
  tf::Quaternion q2;
  q2.setRPY(0.0, 0.0, 0.0);
  transform2.setRotation(q2);
  br2.sendTransform(tf::StampedTransform(transform2, ros::Time::now(), "/base_link", "/laser"));

}

int main(int argc, char** argv){
  ros::init(argc, argv, "my_tf_broadcaster");
  //if (argc != 2){ROS_ERROR("need turtle name as argument"); return -1;};
  //turtle_name = argv[1];

  ros::NodeHandle node;
  ros::Subscriber sub = node.subscribe<nav_msgs::Odometry>("/odom", 10, &poseCallback);

  ros::spin();
  return 0;
};
