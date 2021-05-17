#include "ros/ros.h"
#include "whud_stateOS/FormatConverter.hpp"
#include "whud_stateOS/StateOS.hpp"
#include <sstream>
#include <string>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "StateOS");
    ros::NodeHandle n;
    ros::NodeHandle ns("~"); 
    ros::Rate loop_rate(10);
    ros::AsyncSpinner spinner(8);

    //TaskConfig
    std::string file_suffix = ".yaml";
    std::string file_path;
    int task_count;
    ns.param<std::string>("file_path",  file_path, "");
    ns.param("task_count", task_count, 0);
    
    whud_stateOS::StateOS stateOS(&n,&ns,task_count);
    whud_stateOS::FormatConverter format_converter;
    stateOS.PluginInit();
    //ParseYaml and AddTaskIntoTaskQueue
    for(int i = 0;i<task_count;i++)
    {
        std::stringstream ss;
        ss<<i+1;
        std::string filename = file_path + ss.str() + file_suffix;
        format_converter.ParamFromYamlToCpp(filename);
        stateOS.AddTask(format_converter.CppParamToTaskStruct());
    }
    spinner.start();
    while (ros::ok())
    {
        stateOS.SetTask();
        stateOS.TaskSpin();
        stateOS.CheckStatus();
        loop_rate.sleep();
    }
    ros::waitForShutdown();
    return 0;
}