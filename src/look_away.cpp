#include "ros/ros.h"
#include "simple_arm/GoToPosition.h"
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Image.h>

// global vector for the last joint position
std::vector<double> joints_last_position {0, 0};

// global variable for the moving state of the arm
bool moving_state = false;

// global service client
ros::ServiceClient client;

// calls the save_move service to move the arm to the center position
void move_arm_center()
{
    ROS_INFO_STREAM("Moving the arm to the center");

    // request centered joint angles
    simple_arm::GoToPosition srv;
    srv.request.joint_1 = 1.57;
    srv.request.joint_2 = 1.57;

    // call the save_move service
    if(!client.call(srv))
    {
        ROS_ERROR("Failed to call service safe_move");
    }
}

// callback function for continuously executing and reading the arm joint angles
void joint_state_callback(const sensor_msgs::JointState js)
{
    // get joints current position
    std::vector<double> joints_current_position = js.position;

    // define tolerance thresold
    double tolerance = 0.0005;

    // check if the arm is moving
    if (fabs(joints_current_position[0] - joints_last_position[0]) < tolerance && fabs(joints_last_position[1] - joints_last_position[1]) < tolerance)
    {
        moving_state = false;
    }
    else
    {
        moving_state = true;
        joints_last_position = joints_current_position;
    }
}

// callback function for continuously executing and reading the image data
void look_away_callback(const sensor_msgs::Image img)
{
    bool uniform_image = true;

    // loop for each pixel to compare it with the first pixel
    for (int i = 0; i < img.height * img.step; i++)
    {
        if(img.data[i] - img.data[0] != 0)
        {
            uniform_image = false;
            break;
        }
    }

    // if the image is uniform and the arm is not moving, move the arm to the center
    if(uniform_image == true && moving_state == false)
    {
        move_arm_center();
    }
}

// main
int main(int argc, char** argv)
{
    // init node
    ros::init(argc, argv, "look_away");
    ros::NodeHandle n;

    // define client service
    client = n.serviceClient<simple_arm::GoToPosition>("/arm_mover/save_move");

    // subscribe to joint_states
    ros::Subscriber sub1 = n.subscribe("/simple_arm/joint_states", 10, joint_state_callback);

    // subcribe to camera image
    ros::Subscriber sub2 = n.subscribe("/rgb_camera/image_raw", 10, look_away_callback);

    // Handle ROS events
    ros::spin();

    return 0;
}




