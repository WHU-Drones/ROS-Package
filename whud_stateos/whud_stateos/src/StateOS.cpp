#include "whud_stateOS/StateOS.hpp"

namespace whud_stateOS
{
    StateOS::StateOS(ros::NodeHandle *n,ros::NodeHandle *ns,int task_count)
    :n_(*n), ns_(*ns),task_count_(task_count),velocity_distributor_(&n_,&ns_),
    movement_loader_("movement_plugin","whud_stateOS::MovementPluginBase"),
    attach_loader_("attach_plugin","whud_stateOS::AttachPluginBase")
    {
        os_status_ = OSStatus::OSFREE;
        ROS_INFO("%d task(s) in all.",task_count_);
    }
    StateOS::~StateOS()
    {
        n_.~NodeHandle();
    }
    void StateOS::MovementInit()
    {
        movement_plugin_ = movement_plugin_map_[current_task_.movement_plugin_];
        movement_plugin_->SetTask(current_task_.movement_param_);
        movement_plugin_->SetTaskStatus(TaskStatus::TASKWORK);
    }
    void StateOS::AttachInit()
    {
        attach_plugin_ = attach_plugin_map_[current_task_.attach_plugin_];
        attach_plugin_->SetTask(current_task_.attach_param_);
        attach_plugin_->SetTaskStatus(TaskStatus::TASKWORK);
    }
    void StateOS::AddTask(Task task)
    {
        task_queue_.push(task);
        ROS_INFO("1 task has been pushed in TaskQueue.");
    }
    void StateOS::SetTask()
    {
        if (os_status_ == OSStatus::OSFREE)
        {
            current_task_ = task_queue_.front();
            MovementInit();
            AttachInit();
            velocity_distributor_.GetPluginName(current_task_.movement_plugin_);
            task_queue_.pop();
            ROS_INFO_STREAM("Task "<<task_count_-task_queue_.size()<<" is working.");
            begin_time_ = ros::Time::now().toSec();
            os_status_ = OSStatus::OSWORK;
        }
    }
    void StateOS::TaskSpin()
    {
        if (os_status_ == OSStatus::OSWORK)
        {
            movement_plugin_->TaskSpin();
            attach_plugin_->TaskSpin();
            now_time_ = ros::Time::now().toSec();
            ROS_INFO("The mission has been on for %.2f seconds",now_time_ - begin_time_);
            if(now_time_ - begin_time_ >= current_task_.task_timeout_seconds_)
            {
                os_status_ = OSStatus::OSTIMEOUT;
            }
        }
    }
    void StateOS::CheckStatus()
    {
        if (os_status_ == OSStatus::OSWORK)
        {
            ROS_INFO_STREAM(task_queue_.size()<<" task(s) remaining");
            if (movement_plugin_->GetTaskStatus() == TaskStatus::TASKFREE
                && attach_plugin_->GetTaskStatus() == TaskStatus::TASKFREE)
                {
                        os_status_ = OSStatus::OSFREE;
                        ROS_INFO("Task Complete.");
                        if(task_queue_.empty())
                        {
                            os_status_ = OSStatus::END;
                            ROS_INFO("Mission Complete.");
                        }
                }
        }
        if(os_status_ == OSStatus::OSTIMEOUT)
        {
            ROS_WARN("Task Timeout.Skipped current task.");
            os_status_ = OSStatus::OSFREE;
            if(task_queue_.empty())
            {
                os_status_ = OSStatus::END;
                ROS_INFO("Mission Complete.");
            }
        }
    }
    void StateOS::PluginInit()
    {
        MovementPluginInit();
        AttachPluginInit();
    }
    void StateOS::MovementPluginInit()
    {
        if(ns_.getParam("movement_plugin_list",movement_plugin_list_))
        {
            ROS_INFO("Load Movement_plugins Successfully.");
        }
        else
        {
            ROS_WARN("Fail to Load Movement_plugins.");
        }       
        for(auto it=movement_plugin_list_.begin();it!=movement_plugin_list_.end();++it)
        {
            movement_plugin_map_[*it] = movement_loader_.createInstance(*it);
            movement_plugin_map_[*it]->OnInit(&n_,&ns_);
        }
    }
    void StateOS::AttachPluginInit()
    {
        if(ns_.getParam("attach_plugin_list",attach_plugin_list_))
        {
             ROS_INFO("Load Attach_plugins Successfully.");
        }
        else
        {
            ROS_WARN("Fail to Load Attach_plugins.");
        }
        for(auto it=attach_plugin_list_.begin();it!=attach_plugin_list_.end();++it)
        {
            attach_plugin_map_[*it] = attach_loader_.createInstance(*it);
            attach_plugin_map_[*it]->OnInit(&n_,&ns_);
        }
    }
}