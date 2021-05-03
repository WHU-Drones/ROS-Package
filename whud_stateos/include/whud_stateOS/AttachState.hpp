#pragma once
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "whud_stateOS/DataStructure.hpp"
#include "whud_stateOS/DropCargo.hpp"

namespace whud_stateOS
{
    class AttachState
    {
        private:
            ros::NodeHandle n_;

            AttachStateParam param_;
            TaskStatus task_status_;

            DropCargo drop_cargo_;


        public:
            AttachState(ros::NodeHandle *n);
            ~AttachState();
            void SetTask(AttachStateParam param);
            TaskStatus GetTaskStatus();
            void TaskSpin();
    };
}