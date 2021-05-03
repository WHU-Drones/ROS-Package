#include "simple_navigation_goals/MoveBaseClientNode.hpp"

MoveBaseClientNode::MoveBaseClientNode(std::string node_name,std::string action_name):ac_(action_name, true)
{
  std::cout << "MoveBaseClientNode constructor with the name of "<< action_name<< std::endl;
  ROS_INFO("Waiting for %s action to start...", action_name.c_str());

  if (ac_.waitForServer(ros::Duration(20.0))) 
  {
    ROS_INFO("The robot is ready to work.");
  }
  else
  {
    ROS_ERROR("Timeout.");
  }
}

void MoveBaseClientNode::sendGoal(double x = 0.0, double y = 0.0)
{
  tfDetector("map", "base_link");
  setGoal(x,y);
  startInfo(x,y);
  
  ROS_INFO("Sending goal...");
  isfree_ = false;

  ac_.sendGoal(
      goal_, boost::bind(&MoveBaseClientNode::doneCb, this, _1, _2),
      boost::bind(&MoveBaseClientNode::activeCb, this),
      boost::bind(&MoveBaseClientNode::feedbackCb, this, _1));
}

void MoveBaseClientNode::activeCb()
{
  begin_ = ros::Time::now().toSec(); 
  ROS_INFO("The robot starts working.");
}

void MoveBaseClientNode::feedbackCb(
    const move_base_msgs::MoveBaseFeedbackConstPtr &feedback_ptr)
{
  processInfo(feedback_ptr);
  distanceJudge(feedback_ptr);
}

void MoveBaseClientNode::doneCb(
    const actionlib::SimpleClientGoalState &state,
    const move_base_msgs::MoveBaseResultConstPtr &result_ptr)
{
    if (state == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
      final_state_ = true; 
      ROS_INFO("task complete.");
    }
    else
    {
      ROS_INFO("Enter end range,task complete.");
    }

    if (final_state_ == true || relative_dis_ < end_range_)
    {
        isfree_ = true;
    }
}

void MoveBaseClientNode::setGoal(double x = 0.0, double y = 0.0)
{
    goal_.target_pose.header.frame_id = "map";
    goal_.target_pose.header.stamp = ros::Time::now();
    goal_.target_pose.pose.position.x = x;
    goal_.target_pose.pose.position.y = y;

    tf::Quaternion tfq;
    geometry_msgs::Quaternion msgq;
    tfq.setRPY(0.0, 0.0, 0); 
    tf::quaternionTFToMsg(tfq, msgq); 

    goal_.target_pose.pose.orientation = msgq;
}

void MoveBaseClientNode::tfDetector(std::string target_frame,std::string source_frame)
{
    tf_listener_.waitForTransform(target_frame, source_frame, ros::Time(0),ros::Duration(10.0)); 
    tf_listener_.lookupTransform(target_frame, source_frame, ros::Time(0),start_point_transform_); 
}

void MoveBaseClientNode::startInfo(double x = 0.0, double y = 0.0)
{
     journey_ = sqrt(
      pow(start_point_transform_.getOrigin().x() - x, 2) +
      pow(start_point_transform_.getOrigin().y() - y,2)); 
    
  ROS_INFO("The starting point is (%.2f,%.2f).The target point is(%.2f,%.2f). "
           "The journey between them is %.2fm",
           start_point_transform_.getOrigin().x(),
           start_point_transform_.getOrigin().y(), x, y, journey_);
  
  last_position_[0] = start_point_transform_.getOrigin().x(); 
  last_position_[1] = start_point_transform_.getOrigin().y();
}

void MoveBaseClientNode::processInfo(const move_base_msgs::MoveBaseFeedbackConstPtr &feedback_ptr)
{
  dist_last_ = dist_;
  dist_ = sqrt(pow(feedback_ptr->base_position.pose.position.x -goal_.target_pose.pose.position.x,2) 
                      +  pow(feedback_ptr->base_position.pose.position.y -goal_.target_pose.pose.position.y,2));
  ROS_INFO("coordinate:(%.2f,%.2f).%.2fm to the goal. completed %.1f%% already",
           feedback_ptr->base_position.pose.position.x,
           feedback_ptr->base_position.pose.position.y, dist_,
           100.0 - dist_ / journey_ * 100);

  last_position_[0] = feedback_ptr->base_position.pose.position.x; 
  last_position_[1] = feedback_ptr->base_position.pose.position.y;
}

void MoveBaseClientNode::distanceJudge(const move_base_msgs::MoveBaseFeedbackConstPtr &feedback_ptr)
{
  relative_dis_ = pow(feedback_ptr->base_position.pose.position.x - goal_.target_pose.pose.position.x, 2) 
                              + pow(feedback_ptr->base_position.pose.position.y - goal_.target_pose.pose.position.y, 2);
  if (relative_dis_ < end_range_)
  {
    ac_.cancelGoal();
  }
}
void MoveBaseClientNode::Reset()
{
  isfree_ = true;
}
bool MoveBaseClientNode::IsFree ()
{
    return isfree_;
}

