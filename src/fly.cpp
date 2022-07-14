#include <boost/sml.hpp>
#include <cassert>
#include <iostream> //used for testing
#include "ros/ros.h"
#include "mavros_msgs/CommandSetMode.h"
#include "mavros_msgs/CommandBool.h"
#include "mavros_msgs/CommandTakeoffLocal.h"
#include "mavros_msgs/SetTFListen.h"
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/State.h>
#include <geometry_msgs/TwistStamped.h>
#include <image_transport/image_transport.h>
#include <unistd.h>
#include "../include/color.hpp"
#include "../include/PID.hpp"
#include <yaml-cpp/yaml.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
//#include <cv_bridge/cv_bridge.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#define MOVE_POSITION 3576
#define SET_SPEED 3527
#define SET_SPEED_FIX_Z 3555
#define TAKEOFF_FIX_SPEED 3551


namespace sml = boost::sml;

namespace
{

    struct target
    {
        // FLU-> NED 转换
        double linear_x = 0;
        double linear_y = 0;
        double linear_z = 0;
        double position_x = 0;
        double position_y = 0;
        double position_z = 0;
        uint type_mask = 0;
        void set(double a, double b, double c, double d, double e, double f, uint mask)
        {
            this->linear_x = a;
            this->linear_y = b;
            this->linear_z = c;
            this->position_x = d;
            this->position_y = e;
            this->position_z = f;
            this->type_mask = mask;
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
    struct lost_target
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

            // guard2 判断是否能够起飞 若不能则尝试修复
            auto is_armed = [](ros::NodeHandle n, mavros_msgs::State &current_state, ros::ServiceClient &client)
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
                        mavros_msgs::CommandSetMode offb_set_mode;
                        offb_set_mode.request.base_mode = 0;
                        offb_set_mode.request.custom_mode = "OFFBOARD";
                        client = n.serviceClient<mavros_msgs::CommandSetMode>("/mavros/cmd/set_mode");
                        client.call(offb_set_mode);
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
            };

            // action3 设置速度
            auto SetAction = [](target &v1, ros::Publisher vel_sp_pub)
            {
                mavros_msgs::PositionTarget target_speed;
                geometry_msgs::Vector3 myvelocity;
                geometry_msgs::Point myposition;
                myvelocity.x = v1.linear_x;
                myvelocity.y = v1.linear_y;
                myvelocity.z = v1.linear_z;
                myposition.x = v1.position_x;
                myposition.y = v1.position_y;
                myposition.z = v1.position_z;
                target_speed.header.stamp = ros::Time::now();
                target_speed.coordinate_frame = 8;
                target_speed.type_mask = v1.type_mask;
                target_speed.velocity = myvelocity;
                target_speed.position = myposition;
                vel_sp_pub.publish(target_speed);
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

            return make_transition_table(
                *"idle"_s + event<release> / init = "ready"_s,
                "ready"_s + event<takeoff>[is_init] / takeoffset = "normal"_s,
                "normal"_s + event<set_speed>[is_armed] / SetAction = "normal"_s,
                "normal"_s + event<track>[is_armed] / [] {} = "tracking"_s,
                "normal"_s + event<stop> / [] {} = "landing"_s,
                "tracking"_s + event<set_speed>[is_armed] / SetAction = "tracking"_s,
                "tracking"_s + event<stop>[is_armed] / [] {} = "landing"_s,
                "landing"_s + event<set_speed>[is_armed] / SetAction = "landing"_s,
                "landing"_s + event<lock_>[is_armed] / lock = X);
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
    if (i == 0) // x,y
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
    if (i == 1) // z yaw
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
    int j = 0;
    ros::init(argc, argv, "simple_fly");
    ros::NodeHandle n;
    ros::Publisher vel_sp_pub = n.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 10);
    ros::Subscriber state_sub = n.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
    ros::Rate rate(10.0);
    PIDController mypid_x;
    PIDController mypid_y;
    PIDController mypid_z;
    PIDController mypid_z_angular;
    YAML::Node message = YAML::LoadFile("/home/drone/drones/whu_simple_fly/src/car-tracing/config/msg.yaml");
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
    target my_target;
    ros::ServiceClient client;
    // image_transport::ImageTransport it(n);
    // image_transport::Publisher pub = it.advertise("camera/image", 1);
    cv::VideoCapture cap;
    cap.open(0, cv::CAP_V4L2);
    cv::Mat img;
    cap >> img;
    color::color_Range(img, img, color::red);
    cv::Point2f my_point[2];
    sml::sm<fly_test> sm{n, client, my_target, vel_sp_pub, current_state};
    auto time1 = ros::Time::now();

    while (1)
    {

        // sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
        // pub.publish(msg);
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
            if (ros::Time::now() - time1 < ros::Duration(5.0))
            {
                // wait, do nothing
            }
            if (ros::Time::now() - time1 >= ros::Duration(5.0) && ros::Time::now() - time1 < ros::Duration(8.6))
            {
                my_target.set(0, 0, 0, 0, 0, 0.8, MOVE_POSITION);
                sm.process_event(set_speed{});
            }

            if (ros::Time::now() - time1 >= ros::Duration(8.61))
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
                my_target.set(0.3, 0, 0, 0, 0, 0.8, SET_SPEED_FIX_Z);
                sm.process_event(set_speed{});
            }
            if (ros::Time::now() - time1 >= ros::Duration(18.6))
            {
                ROS_INFO("no");
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
            if (color::isstopped(5, 1, my_point[0], my_point[1], coff))
            {
                ROS_INFO("car is static");
                time2 = ros::Time::now();
                sm.process_event(stop{});
            }
            if (!color::color_center(img, my_point[1]))
            {
                if (j == 3)
                {
                    PIDController_Init(mypid_x);
                    PIDController_Init(mypid_y);
                    time2 = ros::Time::now();
                    sm.process_event(stop{});
                }
                j++;
            }
            my_target.set(-PIDController_Update(mypid_y, my_point[1].y, 240, coff),
                          -PIDController_Update(mypid_x, my_point[1].x, 320, coff),
                          0, 0, 0, 0.8, SET_SPEED_FIX_Z);
            ROS_INFO("car cord is (%f,%f)", my_point[1].x, my_point[1].y);
            ROS_INFO("giving speed is (%f,%f,%f)", my_target.linear_x, my_target.linear_y, my_target.linear_z);

            sm.process_event(set_speed{});
        }
        else if (sm.is("landing"_s))
        {

            ROS_INFO("enter landing");
            if (ros::Time::now() - time2 <= ros::Duration(6.0))
            {
                ROS_INFO("TEST");
                my_target.set(0, 0, -0.2, 0, 0, 0, SET_SPEED);
                sm.process_event(set_speed{});
            }

            if (ros::Time::now() - time2 > ros::Duration(6.0))
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
