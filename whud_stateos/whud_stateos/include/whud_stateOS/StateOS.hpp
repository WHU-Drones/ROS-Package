#pragma once
#include "ros/ros.h"
#include <queue>
#include <string>
#include <vector>
#include <map>

#include <pluginlib/class_loader.h>
#include "movement_plugin/MovementPluginBase.hpp"
#include "attach_plugin/AttachPluginBase.hpp"
#include "whud_stateOS/DataStructure.hpp"
#include "whud_stateOS/VelocityDistributor.hpp"


namespace whud_stateOS
{
    using MovementPluginPtr = boost::shared_ptr<whud_stateOS::MovementPluginBase>;
    using AttachPluginPtr = boost::shared_ptr<whud_stateOS::AttachPluginBase>;
    class StateOS
    {
        private:
            ros::NodeHandle n_;
            ros::NodeHandle ns_;

            Task current_task_;
            OSStatus os_status_;
            std::queue<Task> task_queue_;

            int task_count_;
            double begin_time_{0.0};
            double now_time_{0.0};

            VelocityDistributor velocity_distributor_;

            MovementPluginPtr movement_plugin_;
            AttachPluginPtr  attach_plugin_;

            std::vector<std::string> movement_plugin_list_;
            std::vector<std::string> attach_plugin_list_;

            pluginlib::ClassLoader<whud_stateOS::MovementPluginBase> movement_loader_;
            pluginlib::ClassLoader<whud_stateOS::AttachPluginBase> attach_loader_;

            std::map<std::string, MovementPluginPtr> movement_plugin_map_;
            std::map<std::string, AttachPluginPtr> attach_plugin_map_;

            void MovementPluginInit();
            void AttachPluginInit();
            void MovementInit();
            void AttachInit();

        public:
            StateOS(ros::NodeHandle *n,ros::NodeHandle *nh,int task_count);
            ~StateOS();
            void PluginInit();
            void AddTask(Task task);
            void SetTask();
            void TaskSpin();
            void CheckStatus();
    };
}