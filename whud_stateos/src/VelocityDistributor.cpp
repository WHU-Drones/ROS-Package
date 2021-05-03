#include "whud_stateOS/VelocityDistributor.hpp"

namespace whud_stateOS
{
    VelocityDistributor::VelocityDistributor(ros::NodeHandle *n):n_(*n)
    {
        PublisherInit();
        SubscriberInit();
    }
    VelocityDistributor::~VelocityDistributor()
    {
        n_.~NodeHandle();
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
    void VelocityDistributor::VelocityInit()
    {
            geometry_msgs::Twist vel;
            vel.linear.x = 0.0;
            vel.linear.y = 0.0;
            vel.linear.z = 0.0;
            vel.angular.x = 0.0;
            vel.angular.y = 0.0;
            vel.angular.z = 0.0;
            waypoint_velocity_ = vel;
            object_track_velocity_ = vel;
            cmd_vel_ = vel;
    }
    void VelocityDistributor::WaypointCb(const geometry_msgs::Twist::ConstPtr& msg)
    {
        waypoint_velocity_ = *msg;
        waypoint_data_ready = true;
    }
    void VelocityDistributor::ObjectTrackCb(const geometry_msgs::Twist::ConstPtr& msg)
    {
        object_track_velocity_ = *msg;
        object_track_data_ready = true;
    }
    void VelocityDistributor::Spin(MovementTaskSet task)
    {
        switch (task)
        {
        case MovementTaskSet::WAYPOINT:
            if(waypoint_data_ready)
            {
                cmd_vel_ = waypoint_velocity_;
                cmd_vel_publisher_.publish(cmd_vel_);
                waypoint_data_ready = false;
            }
            break;
        case MovementTaskSet::OBJECT_TRACK:
            if(object_track_data_ready)
            {
                cmd_vel_ = object_track_velocity_;
                cmd_vel_publisher_.publish(cmd_vel_);
                object_track_data_ready = false;
            }
            break;

        default:
            break;
        }

    }
}