#pragma once
#include "ros/ros.h"
#include <string>
#include <unistd.h>
#include "geometry_msgs/PoseStamped.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Bool.h"

class ControlSystem
{
    private:
            ros::NodeHandle n_;

            ros::Subscriber current_height_subscriber_;

            ros::Publisher takeoff_publisher_;
            ros::Publisher land_publisher_;
            ros::Publisher height_control_publisher_;

            std_msgs::Float64 takeoff_height_;
            std_msgs::Bool islanding_;
            std_msgs::Float64 relative_height_;

            bool iscomplete_{true};
            bool setflag_{false};

            double height_{0.0};
            double expect_height_{0.0};
            double accept_range_{0.1};

            bool IsInRange();
            void PublisherInit();
            void SubscriberInit();
            void CurrentHeightCb(const geometry_msgs::PoseStamped::ConstPtr& msg);

        public:
            ControlSystem(ros::NodeHandle *n);
            ~ControlSystem();
            void TakeOff(double takeoff_height_data);
            void Land(bool land_data);
            void HeightControl(double height_control_data);
            void Reset();
            bool GetStatus();
};