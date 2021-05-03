#pragma once
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "whud_stateOS/DataStructure.hpp"

namespace whud_stateOS
{
    class VelocityDistributor
    {
        private:
            ros::NodeHandle n_;

            ros::Subscriber waypoint_subscriber_;
            ros::Subscriber object_track_subscriber_;

            ros::Publisher cmd_vel_publisher_;

            bool waypoint_data_ready{false};
            bool object_track_data_ready{false};

            geometry_msgs::Twist waypoint_velocity_;
            geometry_msgs::Twist object_track_velocity_;

            geometry_msgs::Twist cmd_vel_;

            void PublisherInit();
            void SubscriberInit();
            void VelocityInit();
            void WaypointCb(const geometry_msgs::Twist::ConstPtr& msg);
            void ObjectTrackCb(const geometry_msgs::Twist::ConstPtr& msg);

        public:
            VelocityDistributor(ros::NodeHandle *n);
            ~VelocityDistributor();
            void Spin(MovementTaskSet task);

    };
}
