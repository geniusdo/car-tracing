#include <iostream> //used for testing
#include "ros/ros.h"
#include "mavros_msgs/CommandSetMode.h"
#include "mavros_msgs/CommandBool.h"
#include "mavros_msgs/CommandTakeoffLocal.h"
#include "mavros_msgs/SetTFListen.h"
#include <mavros_msgs/State.h>
#include <geometry_msgs/TwistStamped.h>
#include <unistd.h>
#include <tf/transform_broadcaster.h>

mavros_msgs::State current_state;
geometry_msgs::TwistStamped vs;
geometry_msgs::TwistStamped vs_body_axis;

void state_cb(const mavros_msgs::State::ConstPtr &msg)
{
    current_state = *msg;
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "simple_fly");
    ros::NodeHandle n;
    ros::Publisher vel_sp_pub = n.advertise<geometry_msgs::TwistStamped>("/mavros/setpoint_velocity/cmd_vel", 10);
    ros::Subscriber state_sub = n.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
    ros::ServiceClient client1 = n.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    ros::ServiceClient client2 = n.serviceClient<mavros_msgs::CommandSetMode>("/mavros/cmd/set_mode");
    ros::ServiceClient client3 = n.serviceClient<mavros_msgs::SetTFListen>("/mavros/setpoint_position/set_tf_listen");
    tf::TransformBroadcaster broadcaster;
    ros::Rate rate(20.0);

    mavros_msgs::CommandSetMode offb_set_mode;
    offb_set_mode.request.base_mode = 0;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    mavros_msgs::SetTFListen tf_listen;
    tf_listen.request.value = true;

    while (!current_state.connected)
    {
        ROS_INFO("%d", current_state.connected);
        ros::spinOnce();
        rate.sleep();
    }
    client2.call(offb_set_mode);
    sleep(1);
    client1.call(arm_cmd);
    sleep(1);
    client3.call(tf_listen);
    sleep(1);
    auto time1 = ros::Time::now();

    while (1)
    {
        if (ros::Time::now() - time1 < ros::Duration(10))
        {
            broadcaster.sendTransform(
                tf::StampedTransform(
                    tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0, 0, 1)),
                    ros::Time::now(), "map", "target_position"));
        }

        if (ros::Time::now() - time1 >= ros::Duration(10) && ros::Time::now() - time1 < ros::Duration(20))
        {
            broadcaster.sendTransform(
                tf::StampedTransform(
                    tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(1, 0, 1)),
                    ros::Time::now(), "map", "target_position"));
        }
        if(ros::Time::now() - time1 >= ros::Duration(20) && ros::Time::now() - time1 < ros::Duration(25))
        {
            ROS_INFO("shut");
            //test here
            // broadcaster.sendTransform(
            //     tf::StampedTransform(
            //         tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(1, 0, 1)),
            //         ros::Time::now(), "map", "target_position"));
        }
        if (ros::Time::now() - time1 >= ros::Duration(25) && ros::Time::now() - time1 < ros::Duration(30))
        {
            broadcaster.sendTransform(
                tf::StampedTransform(
                    tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0, 0, -0.2)),
                    ros::Time::now(), "map", "target_position"));
        }
        if (ros::Time::now() - time1 >= ros::Duration(30))
        {
            break;
        }
        ros::spinOnce();
        rate.sleep();
    }
    tf_listen.request.value = false;
    arm_cmd.request.value = false;
    client3.call(tf_listen);
    client1.call(arm_cmd);
    return 0;
}
