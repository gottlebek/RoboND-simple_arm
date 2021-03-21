#include "ros/ros.h"
#include "simple_arm/GoToPosition.h"
#include <std_msgs/Float64.h>

// joint publishers
ros::Publisher joint1_pub, joint2_pub;

// checks and clamps the joint angles to a safe zone
std::vector<float> clamp_at_boundaries(float requested_j1, float requested_j2)
{
    // define clamped joint angles
    float clamped_j1 = requested_j1;
    float clamped_j2 = requested_j2;

    // min, max parameters for every joint
    float min_j1, max_j1, min_j2, max_j2;

    // new node
    ros::NodeHandle n2;

    // get the node name
    std::string node_name = ros::this_node::getName();

    // get joints parameters
    n2.getParam(node_name + "/min_joint_1_angle", min_j1);
    n2.getParam(node_name + "/max_joint_1_angle", max_j1);
    n2.getParam(node_name + "/min_joint_2_angle", min_j2);
    n2.getParam(node_name + "/max_joint_2_angle", max_j2);

    // check for that joint 1 is inside the safe zone
    if (clamped_j1 < min_j1 || clamped_j1 > max_j1)
    {
        clamped_j1 = std::min(std::max(clamped_j1, min_j1), max_j1);
        ROS_WARN("j1 is out of bounds, the valid range is (%1.2f, %1.2f), clamping to: %1.2f", min_j1, max_j1, clamped_j1);
    }

    // check for that joint 2 is inside the safe zone
    if (clamped_j2 < min_j2 || clamped_j2 > max_j2)
    {
        clamped_j2 = std::min(std::max(clamped_j2, min_j2), max_j2);
        ROS_WARN("j2 is out of bounds, the valid range is (%1.2f, %1.2f), clamping to: %1.2f", min_j2, max_j2, clamped_j2);
    }

    // store clamped joint angles in a vector
    std::vector<float> clamped_data = { clamped_j1, clamped_j2 };

    return clamped_data;
}

// this callback function executes whenever a safe_move service is requested
bool handle_safe_move_request(simple_arm::GoToPosition::Request& req, simple_arm::GoToPosition::Response& res)
{
    ROS_INFO("GoToPositionREquest received -j1:%1.2f, j2:%1.2f", (float)req.joint_1, (float)req.joint_2);

    // check requested joint angles are in the safe zone
    std::vector<float> joints_angles = clamp_at_boundaries(req.joint_1, req.joint_2);

    // publish clamped jount angles
    std_msgs::Float64 joint1_angle, joint2_angle;

    joint1_angle.data = joints_angles[0];
    joint2_angle.data = joints_angles[1];

    joint1_pub.publish(joint1_angle);
    joint2_pub.publish(joint2_angle);

    // give the arm some seconds to settle
    ros::Duration(3).sleep();

    // return response message
    res.msg_feedback = "Joint angles set -j1: " + std::to_string(joints_angles[0]) + " , j2: " + std::to_string(joints_angles[1]);
    ROS_INFO_STREAM(res.msg_feedback);

    return true;

}

// main
int main(int argc, char** argv)
{
    // init arm_mover ros node
    ros::init(argc, argv, "arm_mover");
    ros::NodeHandle n;

    //define two publishers
    joint1_pub = n.advertise<std_msgs::Float64>("/simple_arm/joint_1_position_controller/command", 10);
    joint2_pub = n.advertise<std_msgs::Float64>("/simple_arm/joint_2_position_controller/command", 10);

    // define a save_move service with a callback function
    ros::ServiceServer service = n.advertiseService("/arm_mover/save_move", handle_safe_move_request);

    ROS_INFO("Ready to send joint commands");

    // handle ros communication events
    ros::spin();

    return 0;
}

