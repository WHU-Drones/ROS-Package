#pragma once
#include "ros/ros.h"
#include "whud_stateOS/DataStructure.hpp"

namespace whud_stateOS
{
        class AttachPluginBase
        {
            protected:
                ros::NodeHandle n_;
                ros::NodeHandle ns_;

                TaskStatus task_status_;
                AttachParam param_;

            public:
                AttachPluginBase():task_status_(TaskStatus::TASKFREE){}
                ~AttachPluginBase(){}
                void SetTask(AttachParam param);
                TaskStatus GetTaskStatus();
                void SetTaskStatus(TaskStatus task_status);
                virtual void OnInit(ros::NodeHandle *n,ros::NodeHandle *ns){}
                virtual void TaskSpin(){}
        };
        TaskStatus AttachPluginBase::GetTaskStatus()
        {
            return task_status_;
        }
        void AttachPluginBase::SetTaskStatus(TaskStatus task_status)
        {
            task_status_ = task_status;
        }
        void AttachPluginBase::SetTask(AttachParam param)
        {
            param_ = param;
        }
}