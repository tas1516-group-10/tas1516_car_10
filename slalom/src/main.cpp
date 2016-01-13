/**
 * This node sends fixed goals to move base via ROS Action API and receives feedback via callback functions.
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
    ros::init(argc, argv, "slalom"); // init and set name
    std::vector<geometry_msgs::Pose> waypoints; // vector of goals, with position and orientation

    geometry_msgs::Pose waypoint1;
    waypoint1.position.x = -0.0680644512177;
    waypoint1.position.y = 12.4799041748;
    waypoint1.position.z = 0.000;
    waypoint1.orientation.x = 0.000;
    waypoint1.orientation.y = 0.000;
    waypoint1.orientation.z = -0.0402670559068;
    waypoint1.orientation.w = 0.999188953206;
    waypoints.push_back(waypoint1);

    geometry_msgs::Pose waypoint2;
    waypoint2.position.x = 1.38289213181;
    waypoint2.position.y = 12.9106273651;
    waypoint2.position.z = 0.000;
    waypoint2.orientation.x = 0.000;
    waypoint2.orientation.y = 0.000;
    waypoint2.orientation.z = -0.0460160601099;
    waypoint2.orientation.w = 0.998940700048;
    waypoints.push_back(waypoint2);

    geometry_msgs::Pose waypoint3;
    waypoint3.position.x = 2.2465877533;
    waypoint3.position.y = 12.533905983;
    waypoint3.position.z = 0.000;
    waypoint3.orientation.x = 0.000;
    waypoint3.orientation.y = 0.000;
    waypoint3.orientation.z = -0.375389644785;
    waypoint3.orientation.w = 0.926867096507;
    waypoints.push_back(waypoint3);

    geometry_msgs::Pose waypoint4;
    waypoint4.position.x = 3.23049879074;
    waypoint4.position.y = 11.932179451;
    waypoint4.position.z = 0.000;
    waypoint4.orientation.x = 0.000;
    waypoint4.orientation.y = 0.000;
    waypoint4.orientation.z = -0.0188119854049;
    waypoint4.orientation.w = 0.999823038945;
    waypoints.push_back(waypoint4);

    geometry_msgs::Pose waypoint5;
    waypoint5.position.x = 4.11123037338;
    waypoint5.position.y = 12.0440406799;
    waypoint5.position.z = 0.000;
    waypoint5.orientation.x = 0.000;
    waypoint5.orientation.y = 0.000;
    waypoint5.orientation.z = 0.258324229408;
    waypoint5.orientation.w = 0.966058275934;
    waypoints.push_back(waypoint5);

    geometry_msgs::Pose waypoint6;
    waypoint6.position.x = 4.9279961586;
    waypoint6.position.y = 12.4426183701;
    waypoint6.position.z = 0.000;
    waypoint6.orientation.x = 0.000;
    waypoint6.orientation.y = 0.000;
    waypoint6.orientation.z = -0.0992695562941;
    waypoint6.orientation.w = 0.995060578655;
    waypoints.push_back(waypoint6);

    geometry_msgs::Pose waypoint7;
    waypoint7.position.x = 5.73929691315;
    waypoint7.position.y = 12.1841840744;
    waypoint7.position.z = 0.000;
    waypoint7.orientation.x = 0.000;
    waypoint7.orientation.y = 0.000;
    waypoint7.orientation.z = -0.350541523744;
    waypoint7.orientation.w = 0.936547190552;
    waypoints.push_back(waypoint7);

    geometry_msgs::Pose waypoint8;
    waypoint8.position.x = 6.51041889191;
    waypoint8.position.y = 11.612030983;
    waypoint8.position.z = 0.000;
    waypoint8.orientation.x = 0.000;
    waypoint8.orientation.y = 0.000;
    waypoint8.orientation.z = 0.0117987942079;
    waypoint8.orientation.w = 0.999930391805;
    waypoints.push_back(waypoint8);

    geometry_msgs::Pose waypoint9;
    waypoint9.position.x = 7.59879779816;
    waypoint9.position.y = 11.9347524643;
    waypoint9.position.z = 0.000;
    waypoint9.orientation.x = 0.000;
    waypoint9.orientation.y = 0.000;
    waypoint9.orientation.z = -0.0249440965683;
    waypoint9.orientation.w = 0.999688847615;
    waypoints.push_back(waypoint9);




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
        ac.waitForResult(); // wait for goal result

        if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
            ROS_INFO("The base moved to %d goal", i);
        } else {
            ROS_INFO("The base failed to move to %d goal for some reason", i);
        }
    }
    return 0;
}
