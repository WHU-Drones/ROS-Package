#include "whud_stateOS/DropCargo.hpp"

 DropCargo::DropCargo(ros::NodeHandle *n):n_(*n)
 {
     drop_cargo_publisher_ = n_.advertise<std_msgs::String>("/cargo_control", 5);
 }
DropCargo::~DropCargo()
{
    n_.~NodeHandle();
}
void DropCargo::Drop(std::string cargo_name_data)
{
    iscomplete_ = false;
    
    std_msgs::String cargo_name;
    cargo_name.data = cargo_name_data;
    if(index_ < sendtimes_)
    {
        drop_cargo_publisher_.publish(cargo_name);
        index_ ++;
        ROS_INFO("sendtimes: %d",index_);
        usleep(500000);
    }
    else
    {
        iscomplete_ = true;
        index_ = 0;
    }
}
bool DropCargo::GetStatus()
{
    return iscomplete_;
}