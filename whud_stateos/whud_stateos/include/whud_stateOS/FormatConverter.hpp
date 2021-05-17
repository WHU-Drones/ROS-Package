#pragma once
#include "yaml-cpp/yaml.h"
#include <string>
#include "whud_stateOS/DataStructure.hpp"

namespace whud_stateOS
{
    class FormatConverter
    {
        private:
            Task result_task_;

            double task_timeout_seconds_;

            std::string movement_plugin_;

            double takeoff_height_data_;
            double height_control_data_;
            bool land_data_;
            std::string track_object_name_;
            double target_x_;
            double target_y_;

            std::string attach_plugin_;

            std::string cargo_name_;

        public:
            FormatConverter();
            void ParamFromYamlToCpp(std::string filename);
            Task CppParamToTaskStruct();
    };

}