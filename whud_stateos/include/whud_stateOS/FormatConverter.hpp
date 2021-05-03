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

            bool movement_Task_on_off_;
            bool attach_Task_on_off_;

            double task_timeout_seconds_;

            //movement_task paramlist
            int movement_task_;

            double takeoff_height_data_;
            double height_control_data_;
            bool land_data_;
            std::string track_object_name_;
            double target_x_;
            double target_y_;

            //attach_task paramlist
            bool drop_cargo_;
            std::string cargo_name_;

            void TaskIntToEnum(int task_int);

        public:
            FormatConverter();
            void ParamFromYamlToCpp(std::string filename);
            Task CppParamToTaskStruct();


    };

}