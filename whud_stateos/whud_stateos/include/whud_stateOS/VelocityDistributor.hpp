#pragma once
#include "ros/ros.h"
#include <string>
#include "geometry_msgs/Twist.h"
#include "whud_stateOS/DataStructure.hpp"

namespace whud_stateOS
{
    class VelocityDistributor
    {
        private:
            ros::NodeHandle n_;
            ros::NodeHandle ns_;

            ros::Subscriber waypoint_subscriber_;
            ros::Subscriber object_track_subscriber_;

            ros::Publisher cmd_vel_publisher_;

            std::string movement_plugin_name_;

            void PublisherInit();
            void SubscriberInit();
            void WaypointCb(const geometry_msgs::Twist::ConstPtr& msg);
            void ObjectTrackCb(const geometry_msgs::Twist::ConstPtr& msg);

        public:
            VelocityDistributor(ros::NodeHandle *n,ros::NodeHandle *ns);
            ~VelocityDistributor();
            void GetPluginName(std::string movement_plugin_name);

    };
}
