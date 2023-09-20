//
// Created by xyl on 23-9-19.
//
#include "offboard/main.h"
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>


mavros_msgs::State current_state;
geometry_msgs::PoseStamped current_pose;
geometry_msgs::PoseStamped target_pose;
geometry_msgs::Twist  target_velocity;
ros::Time time_start;

void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}
void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    current_pose = *msg;
}

enum TakeoffState{
    SetMode,
    Arm,
    TakeOff,
    Wait,
    Flight,
};

int main(int argc, char **argv)
{
    ROS_INFO("HELLO");
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 1, state_cb);
    ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("mavros/local_position/pose",1,pose_cb);
    ros::Publisher local_velocity_pub = nh.advertise<geometry_msgs::Twist>
            ("mavros/setpoint_velocity/cmd_vel_unstamped",1);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 1);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");
    ros::ServiceClient takeoff_client = nh.serviceClient<mavros_msgs::CommandTOL>
            ("/mavros/cmd/takeoff");

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(50.0);

    // wait for FCU connection
    while(ros::ok() && current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }


    mavros_msgs::SetMode offb_set_mode_srv;
    offb_set_mode_srv.request.custom_mode = "GUIDED";

    mavros_msgs::CommandBool arm_cmd_srv;
    arm_cmd_srv.request.value = true;

    mavros_msgs::CommandTOL takeoff_srv;
    takeoff_srv.request.altitude = 2;
    takeoff_srv.request.latitude = 0;
    takeoff_srv.request.longitude = 0;
    takeoff_srv.request.min_pitch = 0;
    takeoff_srv.request.yaw = 0;

    ros::Time last_request = ros::Time::now();
    TakeoffState takeoff_flag;

    while(ros::ok()){
        switch (takeoff_flag) {
            case SetMode:
            {
                if(set_mode_client.call(offb_set_mode_srv) &&
                   offb_set_mode_srv.response.mode_sent){
                    ROS_INFO("GUIDED enabled");
                } else{
                    ROS_INFO("GUIDED error");
                }
                if (current_state.mode == "GUIDED"){
                    takeoff_flag = Arm;
                    ROS_INFO("GUIDED enabled successfully");
                }
                break;
            }
            case Arm:
            {
                if( arming_client.call(arm_cmd_srv) &&
                    arm_cmd_srv.response.success){
                    ROS_INFO("Vehicle armed");
                }else{
                    ROS_INFO("Vehicle disarmed");
                }
                if(current_state.armed){
                    takeoff_flag = TakeOff;
                    ROS_INFO("Vehicle armed successsfully");
                }
                break;
            }
            case TakeOff:
            {
                if( takeoff_client.call(takeoff_srv) &&
                    takeoff_srv.response.success){
                    ROS_INFO("takeoff");
                    takeoff_flag = Wait;
                    time_start = ros::Time::now();
                }else{
                    ROS_INFO("can not takeoff");
                    return -1;
                }
                break;
            }
            case Wait:
            {
                if((ros::Time::now() - time_start).toSec() > 10) {
                    if (current_pose.pose.position.z > 1.5) {
                        ROS_INFO("takeoff successfuly");
                        takeoff_flag = Flight;
                        time_start = ros::Time::now();
                    } else {
                        ROS_INFO("takeoff unsuccessfuly");
                        return -1;
                    }
                }
                break;
            }
            case Flight:
            {
                target_pose.pose.position.z = 2;
                target_pose.pose.position.x = RADIUS * sin(2.0 * M_PI * 2.0 * (ros::Time::now() - time_start).toSec());
                target_pose.pose.position.y = RADIUS * cos(2.0 * M_PI * 2.0 * (ros::Time::now() - time_start).toSec());

                target_velocity.linear.x = RADIUS * 2.0 * M_PI * 2.0 * cos(2.0 * M_PI * 0.1 * (ros::Time::now() - time_start).toSec());
                target_velocity.linear.y = -RADIUS * 2.0 * M_PI * 2.0 * sin(2.0 * M_PI * 0.1 * (ros::Time::now() - time_start).toSec());
                local_pos_pub.publish(target_pose);
                local_velocity_pub.publish(target_velocity);
                break;
            }
        }
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}