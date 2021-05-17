#include "whud_stateOS/VelocityDistributor.hpp"

namespace whud_stateOS
{
    VelocityDistributor::VelocityDistributor(ros::NodeHandle *n,
    ros::NodeHandle *ns):n_(*n),ns_(*ns)
    {
        PublisherInit();
        SubscriberInit();
    }
    VelocityDistributor::~VelocityDistributor()
    {
        n_.~NodeHandle();
        ns_.~NodeHandle();
    }
    void VelocityDistributor::PublisherInit()
    {
            cmd_vel_publisher_= n_.advertise<geometry_msgs::Twist>("/mavros/whud/cmd_vel", 5);
    }
    void VelocityDistributor::SubscriberInit()
    {
            waypoint_subscriber_ = n_.subscribe("/cmd_vel", 5, &VelocityDistributor::WaypointCb, this);
            object_track_subscriber_= n_.subscribe("/yolo/control_vel", 5, &VelocityDistributor::ObjectTrackCb, this);
    }
    void VelocityDistributor::WaypointCb(const geometry_msgs::Twist::ConstPtr& msg)
    {
        if(movement_plugin_name_ == "whud_stateOS::MovebaseClient")
        {
            cmd_vel_publisher_.publish(*msg);
        }
    }
    void VelocityDistributor::ObjectTrackCb(const geometry_msgs::Twist::ConstPtr& msg)
    {
        if(movement_plugin_name_ == "whud_stateOS::YoloSystem")
        {
            cmd_vel_publisher_.publish(*msg);
        }
    }
    void VelocityDistributor::GetPluginName(std::string movement_plugin_name)
    {
        movement_plugin_name_ = movement_plugin_name;
    }
}