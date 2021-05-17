#pragma once
#include <string>
#include <iostream>

namespace whud_stateOS
{
    enum TaskStatus{TASKFREE,TASKWORK};
    enum OSStatus{OSFREE,OSWORK,OSTIMEOUT,END};
    struct MovementParam
    {
        double takeoff_height_data_;
        double height_control_data_;
        bool land_data_;
        std::string track_object_name_;
        double target_x_;
        double target_y_;
    };
    struct AttachParam
    {
        std::string cargo_name_;
    };
    struct Task
    {
        std::string movement_plugin_;
        std::string attach_plugin_;
        double task_timeout_seconds_;
        MovementParam movement_param_;
        AttachParam attach_param_;
    };
}