/*
File to control velocity depending on distance between the front of the car to obstacles.
Increase velocity if there much space and no obstacle in front.
Decrease velocity if there is an obstacle near the front of the car or we reach a curve.

authors: Peter Leidl, Dominik Mücke
*/

#include "control/control.h"
#include "sensor_msgs/LaserScan.h"

int check_dist;

/*Callback function
  Checks the scan which is in front of the car
  Assigns check_dist flag corresponding to distance
*/
void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_msg){

    if(scan_msg->ranges[300] > 2.0){
        check_dist = 0;
    }

    if(scan_msg->ranges[300] > 1.0 && scan_msg->ranges[300] < 2.0){
        check_dist = 1;
    }

    if(scan_msg->ranges[300] > 0.2 && scan_msg->ranges[300] < 1.0){
        check_dist = 2;
    }
    if(scan_msg->ranges[300] < 0.2){
        check_dist = 3;
    }
}

/*
Main function initialize node and subscribe Laser Scan
Based on the subscribed Laser Scan we choose the max velocity
*/

int main(int argc, char** argv)
{
    //Initialze node and Subscriber
    ros::init(argc, argv, "autonomous_control");
    control autonomous_control;
    ros::NodeHandle n;
    ros::Subscriber sub_scan = n.subscribe<sensor_msgs::LaserScan> ("/scan", 1, scanCallback);
    ros::Rate loop_rate(50);

    //Loop to assign velocity
    while(ros::ok())
    {
	//Check if car is navigated with remote control
        if(autonomous_control.control_Mode.data==0)
        {
            ROS_INFO("Manually Control!");
        }

        else
        {
	  //Autonomous button on remote control is pressed
	  ROS_INFO("Automatic Control!");
	    
            //control_servo == 1500 --> car is not moving
            if(autonomous_control.control_Brake.data==1)
            {
                autonomous_control.control_servo.x=1500;
                autonomous_control.control_servo.y=1500;
            }
		
	    
            else
            {
                ROS_INFO("%f", autonomous_control.cmd_linearVelocity);
                if(check_dist == 0)
                {
                    ROS_INFO("Distance more than 2 meters in front of the car!");
                    autonomous_control.control_servo.x = 1550;
                }
                else if(check_dist == 1)
                {
                    ROS_INFO("Distance between 1 and 2 meters in front of the car!");
                    autonomous_control.control_servo.x = 1550;
                }
                else if(check_dist == 2)
                {
                    ROS_INFO("Dsiatnce between 0.2 and 1 meters in front of the car!");
                    autonomous_control.control_servo.x = 1530;
                }
                else if(check_dist == 3)
                {
            	    ROS_INFO("STOP CAR!");
                    autonomous_control.control_servo.x = 1500;
                }

                if(autonomous_control.cmd_linearVelocity < 0)
		{
		    ROS_INFO("Driving backwards");
                    autonomous_control.control_servo.x = 1300;
                }

                else if(autonomous_control.cmd_linearVelocity == 0)
                {
            	    ROS_INFO("Car not moving!");
                    autonomous_control.control_servo.x = 1500;
                }



                autonomous_control.control_servo.y = autonomous_control.cmd_steeringAngle;
            }

            autonomous_control.control_servo_pub_.publish(autonomous_control.control_servo);

        }

        ros::spinOnce();
        loop_rate.sleep();

    }

    return 0;

}
