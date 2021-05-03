#pragma once
#include "ros/ros.h"
#include <queue>
#include <string>

#include "whud_stateOS/AttachState.hpp"
#include "whud_stateOS/MovementState.hpp"
#include "whud_stateOS/DataStructure.hpp"


namespace whud_stateOS
{
    class StateOS
    {
        private:
            ros::NodeHandle n_;

            Task current_task_;
            OSStatus os_status_;
            std::queue<Task> task_queue_;

            int task_count_;
            double begin_time_{0.0};
            double now_time_{0.0};

            MovementState movement_state_;
            AttachState attach_state_;

        public:
            StateOS(ros::NodeHandle *n,int task_count);
            ~StateOS();
            void AddTask(Task task);
            void SetTask();
            void TaskSpin();
            void CheckStatus();
    };
}