#include <boost/sml.hpp>
#include <cassert>
#include <iostream> //used for testing
#include "ros/ros.h"
#include "mavros_msgs/CommandSetMode.h"
#include "mavros_msgs/CommandBool.h"
#include "mavros_msgs/CommandTakeoffLocal.h"
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

            // auto is_target = [](cv::VideoCapture cap, cv::point2f *my_point[2]) // my_point[0] 为上一次数据
            // {
            //     if (!cap.isOpened())
            //     {
            //         ROS_INFO("cap init fail");
            //         abort();
            //     }
            //     cv::Mat img;
            //     cap >> img;
            //     color::color_Range(img, img, color::red);
            //     my_point[0] = my_point[1];
            //     return color::color_center(img, my_point[1]);
            // };

            //创建action

            // action1 初始化
            auto init = []()
            {
                std::cout << "init done" << std::endl;
            };

            // action2  设置模式
            auto takeoffset = [](ros::NodeHandle n, ros::ServiceClient client)
            {
                mavros_msgs::CommandSetMode offb_set_mode;
                offb_set_mode.request.base_mode = 0;
                offb_set_mode.request.custom_mode = "OFFBOARD";
                client = n.serviceClient<mavros_msgs::CommandSetMode>("/mavros/cmd/set_mode");
                client.call(offb_set_mode);

                mavros_msgs::CommandBool arm_cmd;
                arm_cmd.request.value = true;
                client = n.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
                client.call(arm_cmd);
                std::cout << "setmode done" << std::endl;
                ROS_INFO("setmode done");
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

            // auto PID_init[](PIDController * my_pid){

            // };

            // auto ret_speed=[](velocity  v1, cv::point2f & my_point[2], PIDController & my_pid)
            // {
            //     float setpoint;
            //     *v1.linear_x = PIDController_Update(my_pid, setpoint, my_point[1].x - my_point[0].x);
            //     *v1.linear_y = PIDController_Update(my_pid, setpoint, my_point[1].y - my_point[0].y);
            // };

            return make_transition_table(
                *"idle"_s + event<release> / init = "ready"_s,
                "ready"_s + event<takeoff>[is_init] / takeoffset = "normal"_s,
                "normal"_s + event<set_speed>[is_armed] / setspeed = "normal"_s,
                "normal"_s + event<track> / [] {} = "tracking"_s,
                "normal"_s + event<stop> / [] {} = "landing"_s,
                "tracking"_s + event<set_speed>[is_armed] / setspeed = "tracking"_s,
                "tracking"_s + event<stop>[is_armed] / [] {} = "landing"_s,
                "landing"_s + event<set_speed>[is_armed] / setspeed = "landing"_s,
                "landing"_s + event<lock_>[is_armed] / lock = X
                //"normal"_s / lock = X
                //"s1"_s + event<e2>/[]{} = X
                //"s2"_s+event<e2>[is_armed]/setspeed()
            );
        };
    };
} // namespace
mavros_msgs::State current_state;
geometry_msgs::TwistStamped vs;
geometry_msgs::TwistStamped vs_body_axis;

void state_cb(const mavros_msgs::State::ConstPtr &msg)
{
    current_state = *msg;
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
    YAML::Node message = YAML::LoadFile("/home/liuchuangye/catkin_ws/src/sml_test/config/msg.yaml");
    mypid_x.Ki = message["Ki"].as<double>();
    mypid_x.Kd = message["Kd"].as<double>();
    mypid_x.Kp = message["Kp"].as<double>();
    mypid_x.tau = message["tau"].as<double>();
    mypid_x.limMax = message["limMax"].as<double>();
    mypid_x.limMin = message["limMin"].as<double>();
    mypid_x.limMinInt = message["limMinInt"].as<double>();
    mypid_x.limMaxInt = message["limMaxInt"].as<double>();
    mypid_x.T = message["T"].as<double>();
    double coff = message["coff"].as<double>();
    mypid_x = mypid_y;
    print_PID(mypid_x);
    print_PID(mypid_y);

    while (!current_state.connected)
    {
        ROS_INFO("%d", current_state.connected);
        ros::spinOnce();
        rate.sleep();
    }
    // ros::ServiceClient set_mode_client = n.serviceClient<mavros_msgs::CommandSetMode>("/mavros/cmd/set_mode");
    // ros::ServiceClient arming_client = n.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    velocity my_velocity;
    ros::ServiceClient client;
    cv::VideoCapture cap;
    cap.open(0, cv::CAP_V4L2);
    cv::Mat img;
    cap >> img;
    color::color_Range(img, img, color::red);
    cv::Point2f my_point[2];
    sml::sm<fly_test> sm{n, client, my_velocity, vel_sp_pub, current_state};
    auto time1 = ros::Time::now();
    ros::Time time2;

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
        else if (sm.is("normal"_s))
        {
            if (ros::Time::now() - time1 < ros::Duration(5.0))
            {
                my_velocity.linear_z = 0;
                sm.process_event(set_speed{});
            }
            if (ros::Time::now() - time1 >= ros::Duration(5.0) && ros::Time::now() - time1 < ros::Duration(10.0))
            {
                my_velocity.linear_z = message["takeof_vel"].as<double>();
                sm.process_event(set_speed{});
            }
            if (ros::Time::now() - time1 >= ros::Duration(10.0))
            {
                my_velocity.linear_z = 0;
                my_velocity.linear_x = 0.2;

                cap >> img;
                color::color_Range(img, img, color::red);
                if (color::color_center(img, my_point[1]))
                {
                    my_velocity.linear_z = 0;
                    my_velocity.linear_x = 0;
                    sm.process_event(set_speed{});
                    ROS_INFO("found car");
                    sleep(2);
                    PIDController_Init(mypid_x);
                    PIDController_Init(mypid_y);
                    sm.process_event(track{});
                }
                sm.process_event(set_speed{});
            }
            if (ros::Time::now() - time1 >= ros::Duration(20.0))
            {
                ROS_INFO("LL");
                time2 = ros::Time::now();
                sm.process_event(stop{});
            }
        }
        else if (sm.is("tracking"_s))
        {
            ROS_INFO("tracking!");
            my_point[0] = my_point[1];
            cap >> img;
            color::color_Range(img, img, color::red);
            if (color::isstopped(0.05, 1, my_point[0], my_point[1], coff))
            {
                ROS_INFO("car is static");
                time2 = ros::Time::now();
                sm.process_event(stop{});
            }
            if (!color::color_center(img, my_point[1]))
            {
                PIDController_Init(mypid_x);
                PIDController_Init(mypid_y);
                sm.process_event(stop{});
            }
            my_velocity.linear_x = PIDController_Update(mypid_x, my_point[1].x, 320, coff);
            my_velocity.linear_y = PIDController_Update(mypid_y, my_point[1].y, 240, coff);
            ROS_INFO("car cord is (%f,%f)", my_point[1].x, my_point[1].y);
            ROS_INFO("giving speed is (%f,%f,%f)", my_velocity.linear_x, my_velocity.linear_y, my_velocity.linear_z);
            sm.process_event(set_speed{});
        }
        else if (sm.is("landing"_s))
        {
            ROS_INFO("enter landing");
            if (ros::Time::now() - time2 <= ros::Duration(5.0))
            {
                my_velocity.linear_x = 0;
                my_velocity.linear_y = 0;
                my_velocity.linear_z = 0;
                sm.process_event(set_speed{});
            }

            if (ros::Time::now() - time2 >= ros::Duration(5.0) && ros::Time::now() - time2 <= ros::Duration(7.0))
            {
                my_velocity.linear_x = -0.2;
                sm.process_event(set_speed{});
            }
            if (ros::Time::now() - time2 > ros::Duration(7.0) && ros::Time::now() - time2 < ros::Duration(12.0))
            {
                my_velocity.linear_x = 0;
                my_velocity.linear_z = -0.2;
                sm.process_event(set_speed{});
            }
            if (ros::Time::now() - time2 > ros::Duration(12.0))
            {
                ROS_INFO("gonna lock");
                sleep(5);
                sm.process_event(lock_{});
                abort();
            }
        }
        ros::spinOnce();
        rate.sleep();
    }
}
