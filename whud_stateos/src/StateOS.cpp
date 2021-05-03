#include "whud_stateOS/StateOS.hpp"

namespace whud_stateOS
{
    StateOS::StateOS(ros::NodeHandle *n,int task_count) : n_(*n), 
    task_count_(task_count),movement_state_(&n_), attach_state_(&n_)
    {
        os_status_ = OSStatus::OSFREE;
        ROS_INFO("%d task(s) in all.",task_count_);
    }
    StateOS::~StateOS()
    {
        n_.~NodeHandle();
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
            movement_state_.SetTask(current_task_.movement_state_param_);
            attach_state_.SetTask(current_task_.attach_state_param_);
            task_queue_.pop();

            ROS_INFO("Task %d is working.",(int)(task_count_-task_queue_.size()));
            ros::param::set("/state",(int)(task_count_-task_queue_.size()));
            begin_time_ = ros::Time::now().toSec();

            os_status_ = OSStatus::OSWORK;
        }
    }
    void StateOS::TaskSpin()
    {
        if (os_status_ == OSStatus::OSWORK)
        {
            if (current_task_.is_movement_task_working_)
            {
                movement_state_.TaskSpin();
            }
            if (current_task_.is_attach_task_working_)
            {
                attach_state_.TaskSpin();
            }
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
            ROS_INFO("%d task(s) remaining",(int)task_queue_.size());
            if (movement_state_.GetTaskStatus() == TaskStatus::TASKFREE && 
                attach_state_.GetTaskStatus() == TaskStatus::TASKFREE)
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
            movement_state_.Reset();
            os_status_ = OSStatus::OSFREE;
            if(task_queue_.empty())
            {
                os_status_ = OSStatus::END;
                ROS_INFO("Mission Complete.");
            }
        }
    }

}