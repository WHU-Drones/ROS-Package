#include "whud_stateOS/FormatConverter.hpp"

namespace whud_stateOS
{
    FormatConverter::FormatConverter(){}
    void FormatConverter::ParamFromYamlToCpp(std::string filename)
    {
        YAML::Node yamlConfig = YAML::LoadFile(filename);

        task_timeout_seconds_ =  yamlConfig["task_timeout_seconds"].as<double>();

        movement_plugin_ =  yamlConfig["movement_plugin"].as<std::string>();

        takeoff_height_data_ = yamlConfig["takeoff_height_data"].as<double>();
        height_control_data_ = yamlConfig["height_control_data"].as<double>();
        land_data_ = yamlConfig["land_data"].as<bool>();
        track_object_name_ = yamlConfig["track_object_name"].as<std::string>();
        target_x_ = yamlConfig["target_x"].as<double>();
        target_y_ = yamlConfig["target_y"].as<double>();

        attach_plugin_ = yamlConfig["attach_plugin"].as<std::string>();

        cargo_name_ = yamlConfig["cargo_name"].as<std::string>();
    }
    Task FormatConverter::CppParamToTaskStruct()
    {
        result_task_.task_timeout_seconds_ = task_timeout_seconds_;

        result_task_.movement_plugin_ = movement_plugin_;

        result_task_.movement_param_.takeoff_height_data_ = takeoff_height_data_;
        result_task_.movement_param_.height_control_data_ = height_control_data_;
        result_task_.movement_param_.land_data_ = land_data_;
        result_task_.movement_param_.track_object_name_ = track_object_name_;
        result_task_.movement_param_.target_x_ = target_x_;
        result_task_.movement_param_.target_y_ = target_y_;

        result_task_.attach_plugin_ = attach_plugin_;

        result_task_.attach_param_.cargo_name_ = cargo_name_;

        return result_task_;
    }
} 
