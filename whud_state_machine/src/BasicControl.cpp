#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <stdlib.h>

#include "DataStructure.hpp"

using namespace std;

namespace whud_state_machine {
class WhudBasicControl : public PluginBase {
 public:
  WhudBasicControl() : PluginBase(), sm_nh_(~basic_control);

  void OnInit(MavRosPublisher &mavros_pub) { mavros_pub_ = &mavros_pub; }

  bool SetTask(ros::V_string param) {
    PluginBase::SetTask(param);
    switch (param[0]) {
      case "takeoff":
        // set z axis speed
        takeoff_params[0] = atof(param[1].c_str());
        // set height
        takeoff_params[1] = atof(param[2].c_str());
        takeoff->data = takeoff_params;
        command_ = Command::TAKEOFF;
        break;
      case "land":
        // set z axis speed
        land->data = atof(param[1].c_str());
        command_ = Command::LAND;
        break;
      case "height_control":
        // set z axis speed
        height_control_params[0] = atof(param[1].c_str());
        // set target height(relatively)
        height_control_params[1] = atof(param[2].c_str());
        height_control->data = height_control_params;
        command_ = Command::HEIGHT_CONTROL;
        break;
      case "yaw_control":
        // set target angle
        yaw_control_params[0] = atof(param[1].c_str());
        // 0: absolute, 1: relative
        yaw_control_params[1] = atof(param[2].c_str());
        yaw_control->data = yaw_control_params;
        command_ = Command::YAW_CONTROL;
        break;
      default:
        command_ = Command::NONE;
        break;
    }
  }

  void TaskSpin() {
    sm_nh_.getparam("/mavros/whud_basic/ack_cmd_index", mavros_command_);
    sm_nh_.getparam("/mavros/whud_basic/ack_result_", mavros_result_);

    switch (command_) {
      case Command::TAKEOFF:
        if (mavros_command_ == 24 && mavros_result_ == 0) {
          task_status_ = TaskStatus::DONE;
        } else if (mavros_command_ == 24 && mavros_result_ != 5) {
          if (mavros_pub_ != nullptr && control_flag_ == true) {
            mavros_pub_->takeoff_pub.publish(takeoff);
          }
        }
        break;
      case Command::LAND:
        if (mavros_command_ == 23 && mavros_result_ == 0) {
          task_status_ = TaskStatus::DONE;
        } else if (mavros_command_ == 23 && mavros_result_ != 5) {
          if (mavros_pub_ != nullptr && control_flag_ == true) {
            mavros_pub_->land_pub.publish(land);
          }
        }
        break;
      case Command::HEIGHT_CONTROL:
        if (mavros_command_ == 113 && mavros_result_ == 0) {
          task_status_ = TaskStatus::DONE;
        } else if (mavros_command_ == 113 && mavros_result_ != 5) {
          if (mavros_pub_ != nullptr && control_flag_ == true) {
            mavros_pub_->height_pub.publish(height_control);
          }
        }
        break;
      case Command::YAW_CONTROL:
        if (mavros_command_ == 115 && mavros_result_ == 0) {
          task_status_ = TaskStatus::DONE;
        } else if (mavros_command_ == 115 && mavros_result_ != 5) {
          if (mavros_pub_ != nullptr && control_flag_ == true) {
            mavros_pub_->yaw_pub.publish(yaw_control);
          }
        }
        break;
      default:
        break;
    }
  }
  void StopTask() {}

 private:
  ros::NodeHandle sm_nh_;
  std_msgs::Float64MultiArray takeoff;
  float takeoff_params[2];
  std_msgs::Float64 land;
  std_msgs::Float64MultiArray height_control;
  float height_control_params[2];
  std_msgs::Float64MultiArray yaw_control;
  float yaw_control_params[2];
  int command_;

  int mavros_command_;
  int mavros_result_;
};

enum Command { NONE = 0, TAKEOFF, LAND, HEIGHT_CONTROL, YAW_CONTROL };

}  // namespace whud_state_machine
