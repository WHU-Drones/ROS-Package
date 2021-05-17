#pragma once
#include "ros/ros.h"
#include <vector>
#include <pluginlib/class_loader.h>
#include "whud_stateOS/DataStructure.hpp"

namespace whud_stateOS
{
    class MovementPluginBase
    {  
        protected:
            ros::NodeHandle n_;
            ros::NodeHandle ns_;

            MovementParam param_;
            TaskStatus task_status_;

        public:
            MovementPluginBase():task_status_(TaskStatus::TASKFREE){}
            ~MovementPluginBase(){}
            TaskStatus GetTaskStatus();
            void SetTaskStatus(TaskStatus task_status);
            virtual void SetTask(MovementParam param);
            virtual void OnInit(ros::NodeHandle *n,ros::NodeHandle *ns){}
            virtual void TaskSpin(){}
    };
    TaskStatus MovementPluginBase::GetTaskStatus()
    {
        return task_status_;
    }
    void MovementPluginBase::SetTaskStatus(TaskStatus task_status)
    {
        task_status_ = task_status;
    }
    void MovementPluginBase::SetTask(MovementParam param)
    {
        param_ = param;
    }
}