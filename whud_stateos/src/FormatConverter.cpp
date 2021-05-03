#include "whud_stateOS/FormatConverter.hpp"

namespace whud_stateOS
{
    FormatConverter::FormatConverter(){}
    void FormatConverter::ParamFromYamlToCpp(std::string filename)
    {
        YAML::Node yamlConfig = YAML::LoadFile(filename);

        movement_Task_on_off_ = yamlConfig["movement_Task_on_off"].as<bool>();
        attach_Task_on_off_ = yamlConfig["attach_Task_on_off"].as<bool>();

        task_timeout_seconds_ =  yamlConfig["task_timeout_seconds"].as<double>();

        movement_task_ =  yamlConfig["movement_task"].as<int>();

        takeoff_height_data_ = yamlConfig["takeoff_height_data"].as<double>();
        height_control_data_ = yamlConfig["height_control_data"].as<double>();
        land_data_ = yamlConfig["land_data"].as<bool>();
        track_object_name_ = yamlConfig["track_object_name"].as<std::string>();
        target_x_ = yamlConfig["target_x"].as<double>();
        target_y_ = yamlConfig["target_y"].as<double>();

        drop_cargo_ = yamlConfig["drop_cargo"].as<bool>();
        cargo_name_ = yamlConfig["cargo_name"].as<std::string>();

    }
    Task FormatConverter::CppParamToTaskStruct()
    {
        result_task_.is_movement_task_working_ = movement_Task_on_off_;
        result_task_.is_attach_task_working_ = attach_Task_on_off_;
        result_task_.task_timeout_seconds_ = task_timeout_seconds_;

        TaskIntToEnum(movement_task_);

        result_task_.movement_state_param_.takeoff_height_data_ = takeoff_height_data_;
        result_task_.movement_state_param_.height_control_data_ = height_control_data_;
        result_task_.movement_state_param_.land_data_ = land_data_;
        result_task_.movement_state_param_.track_object_name_ = track_object_name_;
        result_task_.movement_state_param_.target_x_ = target_x_;
        result_task_.movement_state_param_.target_y_ = target_y_;

        result_task_.attach_state_param_.drop_cargo_ = drop_cargo_;
        result_task_.attach_state_param_.cargo_name_ = cargo_name_;

        return result_task_;
    }
    void FormatConverter::TaskIntToEnum(int task_int)
    {
        switch (task_int)
        {
        case 0:
            result_task_.movement_state_param_.task_ = MovementTaskSet::MOVEMENT_FREE;
            break;
        case 1:
           result_task_.movement_state_param_.task_ = MovementTaskSet::TAKEOFF;
            break;
        case 2:
            result_task_.movement_state_param_.task_ = MovementTaskSet::LAND;
            break;
        case 3:
            result_task_.movement_state_param_.task_ = MovementTaskSet::HEIGHT_CONTROL;
            break;
        case 4:
            result_task_.movement_state_param_.task_ = MovementTaskSet::WAYPOINT;
            break;
        case 5:
            result_task_.movement_state_param_.task_ = MovementTaskSet::OBJECT_TRACK;
            break;
        default:
            break;
        }
    }
} 
