#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

int main(int argc, char **argv){
  
  ros::init(argc, argv, "move_car");
  ros::NodeHandle n;
  ros::Rate loop_rate(0.1);
  
  ros::Publisher pub_movement = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1, true);
  ROS_INFO("Drinnen");
  
  geometry_msgs::Twist move_cmd;
  geometry_msgs::Twist move_cmd1;
  geometry_msgs::Twist move_cmd2;
  
  move_cmd.linear.x = 1;
  move_cmd.linear.y = 0;
  move_cmd.linear.z = 0;
  
  move_cmd.angular.x = 0;
  move_cmd.angular.y = 0;
  move_cmd.angular.z = 0;
 
 pub_movement.publish(move_cmd);
 ROS_INFO("Vorwärts");
 
 loop_rate.sleep();
 
  move_cmd1.linear.x = -1;
  move_cmd1.linear.y = 0;
  move_cmd1.linear.z = 0;
  
  move_cmd1.angular.x = 0;
  move_cmd1.angular.y = 0;
  move_cmd1.angular.z = 0;
 
 pub_movement.publish(move_cmd1);
 ROS_INFO("Rück");
 loop_rate.sleep();
 
  move_cmd2.linear.x = 1;
  move_cmd2.linear.y = 0;
  move_cmd2.linear.z = 0;
  
  move_cmd2.angular.x = 0;
  move_cmd2.angular.y = 0;
  move_cmd2.angular.z = 0;
 
 pub_movement.publish(move_cmd2);
 ROS_INFO("Vorwärts");
 ROS_INFO("Ende");

 
 
 
 ros::spinOnce();
    
 return 0;
 
}
