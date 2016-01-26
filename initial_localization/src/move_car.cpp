/*
 * Node to move car for Monte Carlo Localization
 * 
 * authors: Peter Leidl, Dominik Mücke
 * */

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"

//Callback function which publishes commands to move
void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_msg){

 ros::NodeHandle nh;
 ros::Publisher pub_movement = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1, true);
 
 //Display distance to obstacle in front of the car
 ROS_INFO("%f", scan_msg->ranges[300]);
 
 //Check if distance is big enough to move car forward
 if(scan_msg->ranges[300] >= 1){
  geometry_msgs::Twist move_cmd;
  geometry_msgs::Twist move_cmd1;
  geometry_msgs::Twist move_cmd2;
  geometry_msgs::Twist move_cmd3;

  ros::Rate loop_rate(0.10);
  
  //Velocity command
  move_cmd.linear.x = 1.8;
  move_cmd.linear.y = 0;
  move_cmd.linear.z = 0;
  
  //Correction of angular velocity
  move_cmd.angular.x = 0;
  move_cmd.angular.y = 0;
  move_cmd.angular.z = 0;
 
  pub_movement.publish(move_cmd);
 
  loop_rate.sleep();
 
  //Velocity command
  move_cmd1.linear.x = -0.5;
  move_cmd1.linear.y = 0;
  move_cmd1.linear.z = 0;

  //Correction of angular velocity
  move_cmd1.angular.x = 0;
  move_cmd1.angular.y = 0;
  move_cmd1.angular.z = 0;
 
  pub_movement.publish(move_cmd1);
 
  loop_rate.sleep();
 
  //Stop moving car
  move_cmd3.linear.x = 0;
  move_cmd3.linear.y = 0;
  move_cmd3.linear.z = 0;
  
  move_cmd3.angular.x = 0;
  move_cmd3.angular.y = 0;
  move_cmd3.angular.z = 0;
 
 pub_movement.publish(move_cmd3);

 ros::shutdown(); 
 }

}

//Main function initialzes node
int main(int argc, char **argv){
  
  ros::init(argc, argv, "move_car");
  ros::NodeHandle n;

  ros::Subscriber sub_scan = n.subscribe<sensor_msgs::LaserScan> ("/scan", 1, scanCallback);
  

 ros::spin();
    
 return 0;
 
}
