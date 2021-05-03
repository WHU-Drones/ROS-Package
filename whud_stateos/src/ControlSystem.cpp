#include "whud_stateOS/ControlSystem.hpp"

ControlSystem::ControlSystem(ros::NodeHandle *n):n_(*n)
{
    PublisherInit();
    SubscriberInit();
}
ControlSystem::~ControlSystem()
{
    n_.~NodeHandle();
}
void ControlSystem::PublisherInit()
{
    takeoff_publisher_ = n_.advertise<std_msgs::Float64>("/takeoff_height", 5);
    land_publisher_ = n_.advertise<std_msgs::Bool>("/land", 5);
    height_control_publisher_ =  n_.advertise<std_msgs::Float64>("/height_control", 5);
}
void ControlSystem::SubscriberInit()
{
    current_height_subscriber_ = n_.subscribe("/mavros/local_position/pose", 5, &ControlSystem::CurrentHeightCb, this);
}
void ControlSystem::CurrentHeightCb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    height_ = msg->pose.position.z;
}            
bool ControlSystem::IsInRange()
{
    double error = (height_-expect_height_)*(height_-expect_height_);
    double range =  accept_range_ *accept_range_;
    return error<range?true:false;
}
void ControlSystem::Reset()
{
    iscomplete_ = true;
    setflag_ = false;
}
void ControlSystem::TakeOff(double takeoff_height_data)
{
    if(!setflag_)
    {
        iscomplete_ = false;
        expect_height_ += takeoff_height_data;
        setflag_ = true;
    }
    takeoff_height_.data = takeoff_height_data;
    ROS_INFO("current_height: %.2f\n, expect_height: %.2f.",height_,expect_height_);
    if(!IsInRange())
    {
        takeoff_publisher_.publish(takeoff_height_);
        usleep(500000);
    }
    else
    {
        iscomplete_ = true;
        setflag_ = false;
    }
}
void ControlSystem::Land(bool land_data)
{
    if(!setflag_)
    {
        iscomplete_ = false;
        expect_height_ = 0.0;
        setflag_ = true;
    }
    islanding_.data =land_data;
    ROS_INFO("current_height: %.2f\n, expect_height: %.2f.",height_,expect_height_);
    if(!IsInRange())
    {
        land_publisher_.publish(islanding_);
        usleep(500000);
    }
    else
    {
        iscomplete_ = true;
        setflag_ = false;
    }

}
void ControlSystem::HeightControl(double height_control_data)
{   
    if(!setflag_)
    {
        iscomplete_ = false;
        expect_height_ += height_control_data;
        setflag_ = true;
    }
    relative_height_.data = height_control_data;
    ROS_INFO("current_height: %.2f\n, expect_height: %.2f.",height_,expect_height_);
    if(!IsInRange())
    {
        height_control_publisher_.publish(relative_height_);
        usleep(500000);
    }
    else
    {
        iscomplete_ = true;
        setflag_ = false;
    }

}
bool ControlSystem::GetStatus()
{
    return iscomplete_;
}