#include "yolo_system/YoloSystem.hpp"

YoloSystem::YoloSystem(ros::NodeHandle* n):n_(*n),pid_controller_(kp_,ki_,kd_)
{
    PublisherInit();
    SubscriberInit();
}
YoloSystem::~YoloSystem()
{
    n_.~NodeHandle();
}
void YoloSystem::PublisherInit()
{
    vel_publisher_ = n_.advertise<geometry_msgs::Twist>("/yolo/control_vel",5);
}
void YoloSystem::SubscriberInit()
{
    count_subscriber_ = n_.subscribe<darknet_ros_msgs::ObjectCount>
    ("/darknet_ros/found_object", 5, &YoloSystem::CountCallback,this);
    boundingbox_subscriber_ = n_.subscribe<darknet_ros_msgs::BoundingBoxes>
    ("/darknet_ros/bounding_boxes", 5, &YoloSystem::BoundingBoxCallback,this);
}
void YoloSystem::VelocityInit()
{
    geometry_msgs::Twist vel;
    vel.linear.x = 0.0;
    vel.linear.y = 0.0;
    vel.linear.z = 0.0;
    vel.angular.x = 0.0;
    vel.angular.y = 0.0;
    vel.angular.z = 0.0;
    static_vel_ = vel;
    control_vel_ = vel;
}
void YoloSystem::CountCallback(const darknet_ros_msgs::ObjectCount::ConstPtr& msg)
{
    class_count_ = msg->count;
    if(class_count_ == 0 && isfree_ == false)
    {
        vel_publisher_.publish(static_vel_);
    }
}

void YoloSystem::BoundingBoxCallback(const darknet_ros_msgs::BoundingBoxes::ConstPtr& msg)
{
    if(isfree_==false)
    {
            int expect_index = -1;
            bool isexpect = false;

            for(int i=0;i<class_count_;i++)
            {
                class_now_ = msg->bounding_boxes[i].Class;
                if(class_now_ == class_expect_)
                {
                    expect_index = i;
                    isexpect =true;
                }
            }

            if(isexpect)
            {
                CaculateAndPublishVel(msg,expect_index);
            }
            else
            {
                vel_publisher_.publish(static_vel_);
            }
    }
}
void YoloSystem::CaculateAndPublishVel(const darknet_ros_msgs::BoundingBoxes::ConstPtr& msg,int expect_index)
{
    unsigned center_x = 320;
    unsigned center_y = 240;
    unsigned current_x,current_y= 0;
    double vel_x,vel_y = 0.0;
    current_x = (msg->bounding_boxes[expect_index].xmin + msg->bounding_boxes[expect_index].xmax)/2;
    current_y = (msg->bounding_boxes[expect_index].ymin + msg->bounding_boxes[expect_index].ymax)/2;
    vel_x = pid_controller_.pid_core(center_x,current_x);
    vel_y = pid_controller_.pid_core(center_y,current_y);
    control_vel_.linear.x = vel_y;
    control_vel_.linear.y = vel_x;
    vel_publisher_.publish(control_vel_);
    if(vel_x*vel_x <end_range_ && vel_y*vel_y<end_range_)
    {
        isfree_ = true;
        control_vel_ = static_vel_;
    }
}
 void YoloSystem::SetGoal(std::string class_expect)
 {
    class_expect_ = class_expect;
    isfree_ = false;
 }
void YoloSystem::Reset()
{
    isfree_ = true;
}
bool YoloSystem::IsFree()
{
    return isfree_;
}