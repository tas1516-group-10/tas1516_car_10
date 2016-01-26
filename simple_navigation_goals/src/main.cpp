/**
 * File to load waypoints for round on LSR 3. floor.
 * 
 * authors: Fabian Colapietro, Christian Pfaffenzeller, Peter Leidl, Dominik Mücke
 */

#include <iostream>
#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <move_base_msgs/MoveBaseActionResult.h>
#include <actionlib/client/simple_action_client.h>

using namespace std;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

/**
 * Callback function
 */
void doneCb(const actionlib::SimpleClientGoalState& state, const move_base_msgs::MoveBaseResultConstPtr& result) {
    ROS_INFO("Finished in state [%s]", state.toString().c_str());
}

/**
 * Callback function, called once when the goal becomes active
 */
void activeCb() {
    ROS_INFO("Goal just went active");
}

/**
 * Callback function, called every time feedback is received for the goal
 */
void feedbackCb(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback) {
    ROS_INFO("[X]:%f [Y]:%f [W]: %f [Z]: %f", feedback->base_position.pose.position.x,feedback->base_position.pose.position.y,feedback->base_position.pose.orientation.w, feedback->base_position.pose.orientation.z);
}

/**
 * Main function
 */
int main(int argc, char** argv){
    ros::init(argc, argv, "simple_navigation_goals"); // init and set name
    ros::NodeHandle n;
    ros::Rate loop_rate(0.05);

    std::vector<geometry_msgs::Pose> waypoints; // vector of goals, with position and orientation

    
 geometry_msgs::Pose waypoint2;
 waypoint2.position.x = 10.1456050873;
 waypoint2.position.y = -0.263573169708;
 waypoint2.position.z = 0.000;
 waypoint2.orientation.x = 0.000;
 waypoint2.orientation.y = 0.000;
 waypoint2.orientation.z = -0.000334157606055;
 waypoint2.orientation.w = 0.999999944169;
 waypoints.push_back(waypoint2);


 geometry_msgs::Pose waypoint3;
 waypoint3.position.x = 12.9451246262;
 waypoint3.position.y = 9.76045322418;
 waypoint3.position.z = 0.000;
 waypoint3.orientation.x = 0.000;
 waypoint3.orientation.y = 0.000;
 waypoint3.orientation.z = 0.71426517663;
 waypoint3.orientation.w = 0.699875172765;
 waypoints.push_back(waypoint3);




// geometry_msgs::Pose waypoint5;
// waypoint5.position.x = 9.98787784576;
// waypoint5.position.y = 13.0794792175;
// waypoint5.position.z = 0.000;
// waypoint5.orientation.x = 0.000;
// waypoint5.orientation.y = 0.000;
// waypoint5.orientation.z = 0.999968792519;
// waypoint5.orientation.w = 0.00790025239718;
// waypoints.push_back(waypoint5);

 geometry_msgs::Pose waypoint6;
 waypoint6.position.x = 3.4828555584;
 waypoint6.position.y = 13.0072402954;
 waypoint6.position.z = 0.000;
 waypoint6.orientation.x = 0.000;
 waypoint6.orientation.y = 0.000;
 waypoint6.orientation.z = 0.999888155536;
 waypoint6.orientation.w = 0.0149558155386;
 waypoints.push_back(waypoint6);

// geometry_msgs::Pose waypoint7;
// waypoint7.position.x = -0.411442756653;
// waypoint7.position.y = 9.79710483551;
// waypoint7.position.z = 0.000;
// waypoint7.orientation.x = 0.000;
// waypoint7.orientation.y = 0.000;
// waypoint7.orientation.z = -0.693034263807;
// waypoint7.orientation.w = 0.720904646392;
// waypoints.push_back(waypoint7);

geometry_msgs::Pose waypoint7;
waypoint7.position.x = -0.684961080551;
waypoint7.position.y = 9.46679496765;
waypoint7.position.z = 0.000;
waypoint7.orientation.x = 0.000;
waypoint7.orientation.y = 0.000;
waypoint7.orientation.z = -0.698141406242;
waypoint7.orientation.w = 0.71595989894;
waypoints.push_back(waypoint7);

 //Startposition, schaut aber zum Drucker hinter (Sackgasse)
 geometry_msgs::Pose waypoint8;
 waypoint8.position.x = -0.0261330604553;
 waypoint8.position.y = 0.0563100576401;
 waypoint8.position.z = 0.000;
 waypoint8.orientation.x = 0.000;
 waypoint8.orientation.y = 0.000;
 waypoint8.orientation.z = -0.696654571148;
 waypoint8.orientation.w = 0.717406724598;
 waypoints.push_back(waypoint8);




	//Alternativ, schaut in nächsten Raum (könnte weiterfahren)
   // geometry_msgs::Pose waypoint8;
   // waypoint8.position.x = 2.07394552231;
   // waypoint8.position.y = -0.355675816536;
   // waypoint8.position.z = 0.000;
   // waypoint8.orientation.x = 0.000;
   // waypoint8.orientation.y = 0.000;
   // waypoint8.orientation.z = 0.00469572933264;
   // waypoint8.orientation.w = 0.999988975002;
   // waypoints.push_back(waypoint8);


	//Alternativ, nicht ganz bis zum Ziel
   // geometry_msgs::Pose waypoint8;
   // waypoint8.position.x = -0.0496511459351;
   // waypoint8.position.y = 1.67589426041;
   // waypoint8.position.z = 0.000;
   // waypoint8.orientation.x = 0.000;
   // waypoint8.orientation.y = 0.000;
   // waypoint8.orientation.z = -0.700857910806;
   // waypoint8.orientation.w = 0.71330091x: 0.0578737258911

   // waypoints.push_back(waypoint8);
    
    MoveBaseClient ac("move_base", true); // action client to spin a thread by default

    while (!ac.waitForServer(ros::Duration(5.0))) { // wait for the action server to come up
        ROS_INFO("Waiting for the move_base action server to come up");
    }

    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "map"; // set target pose frame of coordinates

    for(int i = 0; i < waypoints.size(); ++i) { // loop over all goal points, point by point
        goal.target_pose.header.stamp = ros::Time::now(); // set current time
        goal.target_pose.pose = waypoints.at(i);
        ROS_INFO("Sending goal");
        ac.sendGoal(goal, &doneCb, &activeCb, &feedbackCb); // send goal and register callback handler

        //loop_rate.sleep();

        ac.waitForResult(); // wait for goal result

        if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
            ROS_INFO("The base moved to %d goal", i);
        } else {
            ROS_INFO("The base failed to move to %d goal for some reason", i);
        }
    }
    return 0;
}
