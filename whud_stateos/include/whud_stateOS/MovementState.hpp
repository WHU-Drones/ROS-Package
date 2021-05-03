#pragma once
#include "ros/ros.h"
#include <iostream>
#include "whud_stateOS/VelocityDistributor.hpp"
#include "whud_stateOS/ControlSystem.hpp"
#include "whud_stateOS/DataStructure.hpp"

#include "yolo_system/YoloSystem.hpp"
#include "simple_navigation_goals/MoveBaseClientNode.hpp"

namespace whud_stateOS
{
    class MovementState
    {
        private:
            ros::NodeHandle n_;

            MovementStateParam param_;
            TaskStatus task_status_;

            bool setflag_{false};

            VelocityDistributor velocity_distributor_;
            
            ControlSystem control_system_;
            std::unique_ptr<MoveBaseClientNode> movebase_client_;
            YoloSystem yolo_system_;

        public:
            MovementState(ros::NodeHandle *n);
            ~MovementState();
            void SetTask(MovementStateParam param);
            TaskStatus GetTaskStatus();
            void TaskSpin();
            void Reset();
    };
}
