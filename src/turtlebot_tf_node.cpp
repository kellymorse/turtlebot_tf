#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <turtlesim/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>


void OdometryCallback(const nav_msgs::Odometry::ConstPtr& msg){
  //tf from base_lonk to odom
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin( tf::Vector3(msg->pose.pose.position.x, msg->pose.pose.position.y, 0.0) );
  tf::Quaternion q(0,0,msg->pose.pose.orientation.z,msg->pose.pose.orientation.w);
  
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/odom", "/base_link"));

  //tf from laser to base_lonk
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

  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe<nav_msgs::Odometry>("/odom", 10, &OdometryCallback);

  ros::spin();
  return 0;
};
