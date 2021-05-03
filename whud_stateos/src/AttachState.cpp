#include "whud_stateOS/AttachState.hpp"

namespace whud_stateOS
{
    AttachState::AttachState(ros::NodeHandle *n):n_(*n),drop_cargo_(&n_)
    {
        task_status_ = TaskStatus::TASKFREE;
    }
    AttachState::~AttachState()
    {
        n_.~NodeHandle();
    }
    void AttachState::SetTask(AttachStateParam param)
    {
        param_.drop_cargo_ = param.drop_cargo_;

        param_.cargo_name_ = param.cargo_name_;
    }
    TaskStatus AttachState::GetTaskStatus()
    {
        return task_status_;
    }
    void AttachState::TaskSpin()
    {
        if(param_.drop_cargo_)
        {
            task_status_ = TaskStatus::TASKWORK;
            ROS_INFO("DROP_CARGO");
            drop_cargo_.Drop(param_.cargo_name_);
            if(drop_cargo_.GetStatus())
                task_status_ = TaskStatus::TASKFREE;
        }
    }
}