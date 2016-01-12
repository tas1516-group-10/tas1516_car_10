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
    ros::init(argc, argv, "simple_navigation_goals"); // init and set name
    std::vector<geometry_msgs::Pose> waypoints; // vector of goals, with position and orientation
 geometry_msgs::Pose waypoint1;
    
    waypoint1.position.x = 3.53484582901;
    waypoint1.position.y = -0.603849411011;
    waypoint1.position.z = 0.000;
    waypoint1.orientation.x = 0.000;
    waypoint1.orientation.y = 0.000;
    waypoint1.orientation.z = -0.011815247304;
    waypoint1.orientation.w = 0.999930197529;
    waypoints.push_back(waypoint1);

    geometry_msgs::Pose waypoint2;
    waypoint2.position.x = 8.41067409515;
    waypoint2.position.y = -1.04025697708;
    waypoint2.position.z = 0.000;
    waypoint2.orientation.x = 0.000;
    waypoint2.orientation.y = 0.000;
    waypoint2.orientation.z = -0.00477235886277;
    waypoint2.orientation.w = 0.999988612231;
    waypoints.push_back(waypoint2);

    geometry_msgs::Pose waypoint3;
    waypoint3.position.x = 12.6150474548;
    waypoint3.position.y = 0.933893322945;
    waypoint3.position.z = 0.000;
    waypoint3.orientation.x = 0.000;
    waypoint3.orientation.y = 0.000;
    waypoint3.orientation.z = 0.699253915277;
    waypoint3.orientation.w = 0.714873388769;
    waypoints.push_back(waypoint3);

    geometry_msgs::Pose waypoint4;
    waypoint4.position.x = 12.6171894073;
    waypoint4.position.y = 5.83105897903;
    waypoint4.position.z = 0.000;
    waypoint4.orientation.x = 0.000;
    waypoint4.orientation.y = 0.000;
    waypoint4.orientation.z = 0.695218406906;
    waypoint4.orientation.w = 0.718798557802;
    waypoints.push_back(waypoint4);

    geometry_msgs::Pose waypoint5;
    waypoint5.position.x = 13.2246952057;
    waypoint5.position.y = 9.92380142212;
    waypoint5.position.z = 0.000;
    waypoint5.orientation.x = 0.000;
    waypoint5.orientation.y = 0.000;
    waypoint5.orientation.z = 0.692546916414;
    waypoint5.orientation.w = 0.721372836032;
    waypoints.push_back(waypoint5);

    geometry_msgs::Pose waypoint6;
    waypoint6.position.x = 11.5204734802;
    waypoint6.position.y = 12.3808498383;
    waypoint6.position.z = 0.000;
    waypoint6.orientation.x = 0.000;
    waypoint6.orientation.y = 0.000;
    waypoint6.orientation.z = 0.999937274092;
    waypoint6.orientation.w = 0.0112003518103;
    waypoints.push_back(waypoint6);

    geometry_msgs::Pose waypoint7;
    waypoint7.position.x = 8.93709182739;
    waypoint7.position.y = 12.1817502975;
    waypoint7.position.z = 0.000;
    waypoint7.orientation.x = 0.000;
    waypoint7.orientation.y = 0.000;
    waypoint7.orientation.z = 0.998994335431;
    waypoint7.orientation.w = -0.0448365674044;
    waypoints.push_back(waypoint7);

    geometry_msgs::Pose waypoint8;
    waypoint8.position.x = 1.76086378098;
    waypoint8.position.y = 12.3767881393;
    waypoint8.position.z = 0.000;
    waypoint8.orientation.x = 0.000;
    waypoint8.orientation.y = 0.000;
    waypoint8.orientation.z = 0.9995762209;
    waypoint8.orientation.w = 0.0291097683285;
    waypoints.push_back(waypoint8);

    geometry_msgs::Pose waypoint9;
    waypoint9.position.x = 1.86955142021;
    waypoint9.position.y = 12.3542404175;
    waypoint9.position.z = 0.000;
    waypoint9.orientation.x = 0.000;
    waypoint9.orientation.y = 0.000;
    waypoint9.orientation.z = 0.999580002915;
    waypoint9.orientation.w = 0.0289796095918;
    waypoints.push_back(waypoint9);

    geometry_msgs::Pose waypoint10;
    waypoint10.position.x = -0.635229587555;
    waypoint10.position.y = 9.92689323425;
    waypoint10.position.z = 0.000;
    waypoint10.orientation.x = 0.000;
    waypoint10.orientation.y = 0.000;
    waypoint10.orientation.z = -0.722377381096;
    waypoint10.orientation.w = 0.691499037802;
    waypoints.push_back(waypoint10);

    geometry_msgs::Pose waypoint11;
    waypoint11.position.x = -0.341611802578;
    waypoint11.position.y = 5.183157444;
    waypoint11.position.z = 0.000;
    waypoint11.orientation.x = 0.000;
    waypoint11.orientation.y = 0.000;
    waypoint11.orientation.z = -0.721713988958;
    waypoint11.orientation.w = 0.692191388376;
    waypoints.push_back(waypoint11);

    geometry_msgs::Pose waypoint12;
    waypoint12.position.x = -1.29383611679;
    waypoint12.position.y = 1.65584826469;
    waypoint12.position.z = 0.000;
    waypoint12.orientation.x = 0.000;
    waypoint12.orientation.y = 0.000;
    waypoint12.orientation.z = -0.687164424422;
    waypoint12.orientation.w = 0.726501929666;
    waypoints.push_back(waypoint12);

    geometry_msgs::Pose waypoint13;
    waypoint13.position.x = 0.399018168449;
    waypoint13.position.y = -0.20387506485;
    waypoint13.position.z = 0.000;
    waypoint13.orientation.x = 0.000;
    waypoint13.orientation.y = 0.000;
    waypoint13.orientation.z = -0.019947427425;
    waypoint13.orientation.w = 0.999801030275;
    waypoints.push_back(waypoint13);
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
