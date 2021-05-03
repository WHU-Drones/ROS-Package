#include "whud_stateOS/MovementState.hpp"

namespace whud_stateOS
{
    MovementState::MovementState(ros::NodeHandle *n):n_(*n),
    control_system_(&n_),velocity_distributor_(&n_),yolo_system_(&n_)
    {
        task_status_ = TaskStatus::TASKFREE;
        movebase_client_.reset(new MoveBaseClientNode());
    }
    MovementState::~MovementState()
    {
        n_.~NodeHandle();
    }
    void MovementState::SetTask(MovementStateParam param)
    { 
        param_.task_ = param.task_;
        param_.land_data_ = param.land_data_;
        param_.takeoff_height_data_ = param.takeoff_height_data_; 
        param_.height_control_data_ = param.height_control_data_;
        param_.track_object_name_ = param.track_object_name_;
        param_.target_x_ = param.target_x_;
        param_.target_y_ = param.target_y_;
    }
    TaskStatus MovementState::GetTaskStatus()
    {
        return task_status_;
    }
    void MovementState::Reset()
    {
        control_system_.Reset(); 
        movebase_client_->Reset();
        yolo_system_ .Reset();             
        setflag_ = false;
         task_status_ = TaskStatus::TASKFREE;
        ROS_WARN("MovementState Reset.");
    }
    void MovementState::TaskSpin()
    {
        switch (param_.task_)
        {
            case MovementTaskSet::TAKEOFF:
                task_status_ = TaskStatus::TASKWORK;
                ROS_INFO("TAKEOFF");
                
                control_system_.TakeOff(param_.takeoff_height_data_);
                if(control_system_.GetStatus())
                    task_status_ = TaskStatus::TASKFREE;

                break;


            case MovementTaskSet::LAND:
                task_status_ = TaskStatus::TASKWORK;
                ROS_INFO("LAND");

                control_system_.Land(param_.land_data_);
                if(control_system_.GetStatus())
                    task_status_ = TaskStatus::TASKFREE;

                break;


            case MovementTaskSet::HEIGHT_CONTROL:
                task_status_ = TaskStatus::TASKWORK;
                ROS_INFO("HEIGHT CONTROL");

                control_system_.HeightControl(param_.height_control_data_);
                if(control_system_.GetStatus())
                    task_status_ = TaskStatus::TASKFREE;

                break;

    
             case MovementTaskSet::WAYPOINT:
                task_status_ = TaskStatus::TASKWORK;
                ROS_INFO("WAYPOINT");

                if(movebase_client_->IsFree())
                {
                    if(!setflag_)
                    {
                        movebase_client_->sendGoal(param_.target_x_,param_.target_y_);
                        setflag_ = true;
                    }
                    else
                    {
                        task_status_ = TaskStatus::TASKFREE;
                        setflag_ = false;
                    }
                }

                break;

             case MovementTaskSet::OBJECT_TRACK:
                task_status_ = TaskStatus::TASKWORK;
                ROS_INFO("OBJECT_TRACK");

                if(yolo_system_.IsFree())
                {
                    if(!setflag_)
                    {
                        yolo_system_.SetGoal(param_.track_object_name_);
                        setflag_ = true;
                    }
                    else
                    {
                        task_status_ = TaskStatus::TASKFREE;
                        setflag_ = false;
                    }
                }
                
                break;

            default:
                task_status_ = TaskStatus::TASKFREE;
                break;
            }

            velocity_distributor_.Spin(param_.task_); 
    }
}
