#pragma once
#include "ros/ros.h"
#include<iostream>
#include<string>
#include<vector>
#include "yolo_system/Pid.hpp"
#include <darknet_ros_msgs/BoundingBox.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <darknet_ros_msgs/ObjectCount.h>
#include<geometry_msgs/Twist.h>

class YoloSystem
{
    private:
        ros::NodeHandle n_;

        ros::Subscriber count_subscriber_;
        ros::Subscriber boundingbox_subscriber_;

        ros::Publisher vel_publisher_;

        float kp_{0.001};
        float ki_{0};
        float kd_{0};
        
        unsigned class_count_{0};
        bool isfree_{true};
        double end_range_{0.0004};

        std::string class_now_;
        std::string class_expect_;

        geometry_msgs::Twist static_vel_;
        geometry_msgs::Twist control_vel_;

        Pid pid_controller_;

        void PublisherInit();
        void SubscriberInit();
        void VelocityInit();
        void CaculateAndPublishVel(const darknet_ros_msgs::BoundingBoxes::ConstPtr& msg,int expect_index);

        void CountCallback(const darknet_ros_msgs::ObjectCount::ConstPtr& msg);
        void BoundingBoxCallback(const darknet_ros_msgs::BoundingBoxes::ConstPtr& msg);

    public:
        YoloSystem(ros::NodeHandle *n);
        ~YoloSystem();
        void SetGoal(std::string class_expect);
        void Reset();
        bool IsFree();
};
