#include <boost/sml.hpp>
#include <cassert>
#include <iostream> //used for testing
#include "ros/ros.h"
#include "mavros_msgs/CommandSetMode.h"
#include "mavros_msgs/CommandBool.h"
#include "mavros_msgs/CommandTakeoffLocal.h"
#include "mavros_msgs/SetTFListen.h"
#include <mavros_msgs/State.h>
#include <geometry_msgs/TwistStamped.h>
#include <unistd.h>
#include "../include/color.hpp"
#include "../include/PID.hpp"
#include <yaml-cpp/yaml.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

namespace sml = boost::sml;

namespace
{

    struct velocity
    {
        double linear_x = 0;
        double linear_y = 0;
        double linear_z = 0;
        double angular_x = 0;
        double angular_y = 0;
        double angular_z = 0;
    };
    struct position
    {
        double x = 0;
        double y = 0;
        double z = 0;
        void set(double a, double b, double c)
        {
            this->x = a;
            this->y = b;
            this->z = c;
        }
    };
    struct release
    {
    };
    struct takeoff
    {
    };
    struct set_speed
    {
    };
    struct lock_
    {
    };
    struct track
    {
    };
    struct stop
    {
    };
    struct sleep
    {
    };
    struct gettime
    {
    };
    struct set_position
    {
    };

    struct fly_test
    {
        auto operator()()
        {
            using namespace sml;

            // 创建guard

            // guard1 判断是否ros是否连接

            auto is_init = [](mavros_msgs::State &current_state)
            {
                if (ros::ok() && current_state.connected)
                    // if (ros::ok())
                    return true;
                else
                    ROS_INFO("not connected");
                // abort();
                return false;
            };

            // guard2 判断是否能够起飞
            auto is_armed = [](mavros_msgs::State &current_state)
            {
                if (ros::ok() && current_state.armed && current_state.mode == "OFFBOARD")
                    // if (ros::ok() && current_state.mode == "OFFBOARD")

                    return true;
                else
                {
                    if (!ros::ok())
                    {
                        ROS_INFO("ros not ok");
                    }
                    if (!current_state.connected)
                    {
                        ROS_INFO("not connected");
                    }
                    if (!current_state.armed)
                    {
                        ROS_INFO("nor armed");
                    }
                    if (current_state.mode != "OFFBOARD")
                    {
                        ROS_INFO("not offboard");
                    }
                    ROS_INFO("mode guard faild");
                    return false;
                }
            };

            //创建action

            // action1 初始化
            auto init = []()
            {
                std::cout << "init done" << std::endl;
            };

            // action2  设置模式
            auto takeoffset = [](ros::NodeHandle n, ros::ServiceClient &client)
            {
                //设置offboard模式
                mavros_msgs::CommandSetMode offb_set_mode;
                offb_set_mode.request.base_mode = 0;
                offb_set_mode.request.custom_mode = "OFFBOARD";
                client = n.serviceClient<mavros_msgs::CommandSetMode>("/mavros/cmd/set_mode");
                client.call(offb_set_mode);

                mavros_msgs::CommandBool arm_cmd;
                arm_cmd.request.value = true;
                client = n.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
                client.call(arm_cmd);

                mavros_msgs::SetTFListen tf_listen;
                tf_listen.request.value = true;
                client = n.serviceClient<mavros_msgs::SetTFListen>("/mavros/setpoint_position/set_tf_listen");
                client.call(tf_listen);
            };

            // action3 设置速度
            auto setspeed = [](velocity &v1, geometry_msgs::TwistStamped vs, ros::Publisher vel_sp_pub)
            {
                vs.twist.linear.x = v1.linear_x;
                vs.twist.linear.y = v1.linear_y;
                vs.twist.linear.z = v1.linear_z;
                vs.twist.angular.x = v1.angular_x;
                vs.twist.angular.y = v1.angular_y;
                vs.twist.angular.z = v1.angular_z;
                vs.header.stamp = ros::Time::now();
                vel_sp_pub.publish(vs);
                std::cout << "setspeed done" << std::endl;
                ROS_INFO("setspeed done");
            };

            // action4 锁桨
            auto lock = [](ros::NodeHandle n, ros::ServiceClient client)
            {
                mavros_msgs::CommandBool arm_cmd;
                client = n.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
                arm_cmd.request.value = false;
                client.call(arm_cmd);
                std::cout << "loked!" << std::endl;
                ROS_INFO("loked!");
            };

            auto setposition = [](position &target_pos, tf::TransformBroadcaster broadcaster)
            {
                broadcaster.sendTransform(
                    tf::StampedTransform(
                        tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(target_pos.x, target_pos.y, target_pos.z)),
                        ros::Time::now(), "map", "target_position"));
                ROS_INFO("fly to (%f, %f, %f)", target_pos.x, target_pos.y, target_pos.z);
            };

            //退出发布tf
            auto quit_tf = [](ros::NodeHandle n, ros::ServiceClient client)
            {
                mavros_msgs::SetTFListen tf_listen;
                tf_listen.request.value = false;
                client = n.serviceClient<mavros_msgs::SetTFListen>("/mavros/setpoint_position/set_tf_listen");
                client.call(tf_listen);
            };

            auto open_tf = [](ros::NodeHandle n, ros::ServiceClient client)
            {
                mavros_msgs::SetTFListen tf_listen;
                tf_listen.request.value = true;
                client = n.serviceClient<mavros_msgs::SetTFListen>("/mavros/setpoint_position/set_tf_listen");
                client.call(tf_listen);
            };

            // auto set_tf = [](ros::NodeHandle n, tf::TransformBroadcaster broadcaster, position &tf_position)
            // {

            //     // broadcaster.sendTransform(
            //     //     tf::StampedTransform(
            //     //         tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(tf_position.x, tf_position.y, tf_position.z)),
            //     //         ros::Time::now(), "map", "target_position"));
            // };

            return make_transition_table(
                *"idle"_s + event<release> / init = "ready"_s,
                //在takeoffset中开启tf监听
                "ready"_s + event<takeoff>[is_init] / takeoffset = "normal"_s,
                "normal"_s + event<set_speed>[is_armed] / setspeed = "normal"_s,
                "normal"_s + event<set_position>[is_armed] / setposition = "normal"_s,
                //进入track模式退出tf监听
                "normal"_s + event<track> / quit_tf = "tracking"_s,
                "normal"_s + event<stop> / [] {} = "landing"_s,
                "tracking"_s + event<set_speed>[is_armed] / setspeed = "tracking"_s,
                "tracking"_s + event<set_position>[is_armed] / setposition = "tracking"_s,
                "tracking"_s + event<stop>[is_armed] / open_tf = "landing"_s,
                "landing"_s + event<set_speed>[is_armed] / setspeed = "landing"_s,
                "landing"_s + event<lock_>[is_armed] / (quit_tf, lock) = X
                //"normal"_s / lock = X
                //"s1"_s + event<e2>/[]{} = X
                //"s2"_s+event<e2>[is_armed]/setspeed()
            );
        };
    };
};
// namespace
mavros_msgs::State current_state;
geometry_msgs::TwistStamped vs;
geometry_msgs::TwistStamped vs_body_axis;

void state_cb(const mavros_msgs::State::ConstPtr &msg)
{
    current_state = *msg;
}

void read_PID(const YAML::Node &pid_yaml, PIDController &pid, int i)
{
    if (i = 0) // x,y
    {
        pid.Kd = pid_yaml["Kd"].as<double>();
        pid.Ki = pid_yaml["Ki"].as<double>();
        pid.Kp = pid_yaml["Kp"].as<double>();
        pid.limMax = pid_yaml["limMax"].as<double>();
        pid.limMin = pid_yaml["limMin"].as<double>();
        pid.limMaxInt = pid_yaml["limMaxInt"].as<double>();
        pid.limMinInt = pid_yaml["limMinInt"].as<double>();
        pid.T = pid_yaml["T"].as<double>();
    }
    if (i = 1) // z yaw
    {
        pid.Kd = pid_yaml["Kd_"].as<double>();
        pid.Ki = pid_yaml["Ki_"].as<double>();
        pid.Kp = pid_yaml["Kp_"].as<double>();
        pid.limMax = pid_yaml["limMax_"].as<double>();
        pid.limMin = pid_yaml["limMin_"].as<double>();
        pid.limMaxInt = pid_yaml["limMaxInt_"].as<double>();
        pid.limMinInt = pid_yaml["limMinInt_"].as<double>();
        pid.T = pid_yaml["T"].as<double>();
    }
}

int main(int argc, char **argv)
{
    using namespace sml;

    ros::init(argc, argv, "simple_fly");
    ros::NodeHandle n;
    ros::Publisher vel_sp_pub = n.advertise<geometry_msgs::TwistStamped>("/mavros/setpoint_velocity/cmd_vel", 10);
    ros::Subscriber state_sub = n.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
    ros::Rate rate(20.0);
    PIDController mypid_x;
    PIDController mypid_y;
    PIDController mypid_z;
    PIDController mypid_z_angular;
    YAML::Node message = YAML::LoadFile("../config/msg.yaml");
    read_PID(message, mypid_x, 0);
    read_PID(message, mypid_y, 0);
    read_PID(message, mypid_z, 1);
    read_PID(message, mypid_z_angular, 1);
    double coff = message["coff"].as<double>();
    print_PID(mypid_x);
    print_PID(mypid_y);
    ros::Time time2;

    while (!current_state.connected)
    {
        ROS_INFO("%d", current_state.connected);
        ros::spinOnce();
        rate.sleep();
    }
    // ros::ServiceClient set_mode_client = n.serviceClient<mavros_msgs::CommandSetMode>("/mavros/cmd/set_mode");
    // ros::ServiceClient arming_client = n.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    velocity my_velocity;
    position target;
    ros::ServiceClient client;
    tf::TransformListener listener;
    cv::VideoCapture cap;
    cap.open(0, cv::CAP_V4L2);
    cv::Mat img;
    cap >> img;
    color::color_Range(img, img, color::red);
    cv::Point2f my_point[2];
    sml::sm<fly_test> sm{n, client, my_velocity, vel_sp_pub, current_state, target, listener};
    auto time1 = ros::Time::now();

    while (1)
    {
        if (sm.is("idle"_s))
        {
            sm.process_event(release{});
        }
        else if (sm.is("ready"_s))
        {
            sm.process_event(takeoff{});
        }

        // normal 用于进入追踪前的模式
        //进入normal模式前 开启tf监听
        //起飞->寻车->退出
        else if (sm.is("normal"_s))
        {
            if (ros::Time::now() - time1 < ros::Duration(2.0))
            {
                // wait, do nothing
            }
            if (ros::Time::now() - time1 >= ros::Duration(2.0) && ros::Time::now() - time1 < ros::Duration(3.0))
            {
                target.set(0, 0, 1);
                sm.process_event(set_position{});
            }

            if (ros::Time::now() - time1 >= ros::Duration(2.0))
            {
                cap >> img;
                color::color_Range(img, img, color::red);
                if (color::color_center(img, my_point[1]))
                {
                    ROS_INFO("found car");
                    PIDController_Init(mypid_x);
                    PIDController_Init(mypid_y);
                    sm.process_event(track{});
                }
            }
            if (ros::Time::now() - time1 >= ros::Duration(10.0))
            {
                ROS_INFO("no");
                time2 = ros::Time::now();
                sm.process_event(stop{});
            }
            target.set(5, 0, 1);
            sm.process_event(set_position{});
        }
        else if (sm.is("tracking"_s))
        {
            ROS_INFO("tracking!");
            //监听tf
            tf::TransformListener listener;
            tf::StampedTransform transform;
            try
            {
                listener.waitForTransform("map", "base_link", ros::Time(0), ros::Duration(0.05));
                listener.lookupTransform("map", "base_link", ros::Time(0), transform);
            }
            catch (tf::TransformException &ex)
            {
                ROS_ERROR("%s", ex.what());
                continue;
            }
            //角度信息 待加

            my_point[0] = my_point[1];
            cap >> img;
            color::color_Range(img, img, color::red);
            if (color::isstopped(0, 1, my_point[0], my_point[1], coff))
            {
                ROS_INFO("car is static");
                time2 = ros::Time::now();
                sm.process_event(stop{});
            }
            if (!color::color_center(img, my_point[1]))
            {
                PIDController_Init(mypid_x);
                PIDController_Init(mypid_y);
                time2 = ros::Time::now();
                sm.process_event(stop{});
            }
            my_velocity.linear_x = PIDController_Update(mypid_x, my_point[1].x, 240, coff);
            my_velocity.linear_y = PIDController_Update(mypid_y, my_point[1].y, 320, coff);
            my_velocity.linear_z = PIDController_Update(mypid_z, transform.getOrigin().z(), 1.0, 1);
            ROS_INFO("car cord is (%f,%f)", my_point[1].x, my_point[1].y);
            ROS_INFO("giving speed is (%f,%f,%f)", my_velocity.linear_x, my_velocity.linear_y, my_velocity.linear_z);
            sm.process_event(set_speed{});
        }
        else if (sm.is("landing"_s))
        {

            ROS_INFO("enter landing");
            if (ros::Time::now() - time2 <= ros::Duration(5.0))
            {
                target.set(0, 0, -0.2);
                sm.process_event(set_position{});
            }

            if (ros::Time::now() - time2 > ros::Duration(10.0))
            {
                ROS_INFO("gonna lock");
                sm.process_event(lock_{});
                abort();
            }
        }
        ros::spinOnce();
        rate.sleep();
    }
}
