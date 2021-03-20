#include "ros/ros.h"
#include "std_msgs/Float64.h"

int main(int argc, char** argv)
{
    // init node
    ros::init(argc, argv, "arm_mover");

    // create a node handle
    ros::NodeHandle n;

    // create publisher for joint1
    ros::Publisher joint1_pub = n.advertise<std_msgs::Float64>("/simple_arm/joint_1_position_controller/command", 10);

    // create publisher for joint2
    ros::Publisher joint2_pub = n.advertise<std_msgs::Float64>("/simple_arm/joint_2_position_controller/command", 10);

    // set loop frequency [Hz]
    ros::Rate loop_rate(10);

    int start_time, elapsed;
    
    // get start time of ros
    // like waiting for a message from /clock topic
    while(not start_time)
    {
        start_time = ros::Time::now().toSec();
    }

    while(ros::ok())
    {
        // get elapsed time
        elapsed = ros::Time::now().toSec() - start_time;

        // set arm joint angles
        std_msgs::Float64 joint1_angle, joint2_angle;
        joint1_angle.data = sin(2 * M_PI * 0.1 * elapsed) * (M_PI / 2);
        joint2_angle.data = sin(2 * M_PI * 0.1 * elapsed) * (M_PI / 2);

        // publish joints
        joint1_pub.publish(joint2_angle);
        joint2_pub.publish(joint2_angle);

        // sleep loop until the loop rate
        loop_rate.sleep();

    }

    return 0;
}