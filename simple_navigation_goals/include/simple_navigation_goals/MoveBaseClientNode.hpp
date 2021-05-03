#pragma once

#include <iostream>
#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_listener.h>


using MoveBaseClient=actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>;

class MoveBaseClientNode
{
    private:
        ros::NodeHandle nh_;
        ros::NodeHandle ns_{"~"};

        MoveBaseClient ac_{"move_base", true};
        move_base_msgs::MoveBaseGoal goal_;

        tf::TransformListener tf_listener_;
        tf::StampedTransform start_point_transform_;

        std::string movebase_nodename_;

        double dist_{0.1},dist_last_{0.2};
        double begin_{ros::Time::now().toSec()};
        double last_position_[2]{0.0,0.0};
        double relative_dis_{0};
        double journey_{0};
        double end_range_{0.01};

        bool final_state_{false};
        bool isfree_{true};

        void doneCb(const actionlib::SimpleClientGoalState &state, const move_base_msgs::MoveBaseResultConstPtr &result_ptr);
        void activeCb();
        void feedbackCb(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback_ptr);
        void setGoal(double x,double y);
        void tfDetector(std::string target_frame,std::string source_frame);
        void startInfo(double x,double y);
        void processInfo(const move_base_msgs::MoveBaseFeedbackConstPtr &feedback_ptr);
        void distanceJudge(const move_base_msgs::MoveBaseFeedbackConstPtr &feedback_ptr);

    public:
        MoveBaseClientNode(std::string node_name=std::string("move_base"),
        std::string action_name=std::string("move_base"));
        virtual ~MoveBaseClientNode(){}
        void sendGoal(double x,double y);
        void Reset();
        bool IsFree();

};