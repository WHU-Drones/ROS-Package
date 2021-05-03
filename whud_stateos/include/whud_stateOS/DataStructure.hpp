#pragma once
#include <set>
#include <string>
#include <iostream>

namespace whud_stateOS
{
    enum MovementTaskSet {MOVEMENT_FREE,TAKEOFF,LAND,HEIGHT_CONTROL,WAYPOINT,OBJECT_TRACK};
    enum TaskStatus{TASKFREE,TASKWORK};
    enum OSStatus{OSFREE,OSWORK,OSTIMEOUT,END};
    struct MovementStateParam
    {
        MovementTaskSet task_;
        double takeoff_height_data_;
        double height_control_data_;
        bool land_data_;
        std::string track_object_name_;
        double target_x_;
        double target_y_;
    };
    struct AttachStateParam
    {
        bool drop_cargo_;
        std::string cargo_name_;
    };
    struct Task
    {
        bool is_movement_task_working_;
        bool is_attach_task_working_;
        double task_timeout_seconds_;
        MovementStateParam movement_state_param_;
        AttachStateParam attach_state_param_;
    };
}