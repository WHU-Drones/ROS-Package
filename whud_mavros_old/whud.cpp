#include <geometry_msgs/Twist.h>
#include <mavros/mavros_plugin.h>
#include <tf/tfMessage.h>
#include <tf/transform_listener.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Int32.h>
#include <nav_msgs/Odometry.h>

namespace mavros {
namespace std_plugins {
/**
 * @brief Mav Control plugin
 */
class WhudPlugin : public plugin::PluginBase {
 public:
  WhudPlugin() : PluginBase(), mav_control_nh("~whud") {}

  void initialize(UAS &uas_) override {
    PluginBase::initialize(uas_);

    cmd_vel_sub = mav_control_nh.subscribe("/mavros/whud/cmd_vel", 1, &WhudPlugin::cmd_vel_cb, this);
    tf_sub = mav_control_nh.subscribe("/tf", 1, &WhudPlugin::tf_cb, this);
    takeoff_sub = mav_control_nh.subscribe("/takeoff_height", 1, &WhudPlugin::takeoff_cb, this);
    land_sub = mav_control_nh.subscribe("/land", 1, &WhudPlugin::land_cb, this);
    height_control_sub = mav_control_nh.subscribe("/height_control", 1, &WhudPlugin::height_control_cb, this);
    yaw_sub = mav_control_nh.subscribe("/yaw", 1, &WhudPlugin::yaw_cb, this);
    vision_pose_sub = mav_control_nh.subscribe("/vision", 1, &WhudPlugin::vision_pose_cb, this);
    vision_speed_sub = mav_control_nh.subscribe("/vision", 1, &WhudPlugin::vision_speed_cb, this);
    conversion_sub = mav_control_nh.subscribe("/conversion", 1, &WhudPlugin::conversion_cb, this);
    
    progress_pub = mav_control_nh.advertise<std_msgs::Int32>("/progress", 1);
  }

  	Subscriptions get_subscriptions() override
	{
		return {
              make_handler(&WhudPlugin::handle_progress),
    };
	}

 private:
  ros::NodeHandle mav_control_nh;

  ros::Subscriber cmd_vel_sub;
  ros::Subscriber tf_sub;
  ros::Subscriber takeoff_sub;
  ros::Subscriber land_sub;
  ros::Subscriber height_control_sub;
  ros::Subscriber yaw_sub;
  ros::Subscriber vision_pose_sub;
  ros::Subscriber vision_speed_sub;
  ros::Subscriber conversion_sub;

  ros::Publisher progress_pub;

  tf::TransformListener tf_listener_;
  int conversion_ = 0;
  int vision_pose_fre_div_ = 0;
  int vision_speed_fre_div_ = 0;

  /* -*- callbacks -*- */

  void cmd_vel_cb(const geometry_msgs::Twist::ConstPtr &req)
  {
    mavlink::common::msg::COMMAND_LONG msg;
    // MAV_CMD_USER_1
    msg.command = 31010;
    msg.param1 = req->linear.x;
    msg.param2 = req->linear.y;
    msg.param3 = req->linear.z;
    msg.param4 = req->angular.z;
    // maxRoll
    msg.param5 = 0.5;
    // maxPitch
    msg.param6 = 0.5;
    msg.param7 = conversion_;

    UAS_FCU(m_uas)->send_message_ignore_drop(msg);
  }

  void conversion_cb(const std_msgs::Int32::ConstPtr &conversion)
  {
    conversion_ = conversion->data;
    mavlink::common::msg::COMMAND_LONG msg;
    // MAV_CMD_USER_1
    msg.command = 31010;
    msg.param1 = 0;
    msg.param2 = 0;
    msg.param3 = 0;
    msg.param4 = 0;
    // maxRoll
    msg.param5 = 0.5;
    // maxPitch
    msg.param6 = 0.5;
    msg.param7 = conversion_;

    UAS_FCU(m_uas)->send_message_ignore_drop(msg);
  }

  void tf_cb(const tf::tfMessage::ConstPtr &tf)
  {
    mavlink::common::msg::ATT_POS_MOCAP msg;
    tf::StampedTransform tf_transform;

    try
    {
        tf_listener_.lookupTransform("/map", "/track_link", ros::Time(0), tf_transform);
    }
    catch(tf::TransformException &ex)
    {
        ROS_WARN("%s", ex.what());
    }

    msg.q[0] = tf_transform.getRotation().getW();
    msg.q[1] = tf_transform.getRotation().getAxis().getX();
    msg.q[2] = tf_transform.getRotation().getAxis().getY();
    msg.q[3] = tf_transform.getRotation().getAxis().getZ();
    msg.x = tf_transform.getOrigin().getX();
    msg.y = tf_transform.getOrigin().getY();
    msg.z = tf_transform.getOrigin().getZ();

    UAS_FCU(m_uas)->send_message_ignore_drop(msg);
  }

  void takeoff_cb(const std_msgs::Float64MultiArray::ConstPtr &takeoff_msg)
  {
    mavlink::common::msg::COMMAND_LONG msg;
    // MAV_CMD_NAV_TAKEOFF_LOCAL
    msg.command = 24;
    // set z axis speed
    msg.param3 = takeoff_msg->data[0];
    // set height
    msg.param7 = takeoff_msg->data[1];

    UAS_FCU(m_uas)->send_message_ignore_drop(msg);
  }

  void land_cb(const std_msgs::Float64::ConstPtr &land_msg)
  {
    mavlink::common::msg::COMMAND_LONG msg;
    // MAV_CMD_NAV_LAND_LOCAL
    msg.command = 23;
    // set z axis speed
    msg.param3 = land_msg->data;
    UAS_FCU(m_uas)->send_message_ignore_drop(msg);
  }

  void height_control_cb(const std_msgs::Float64MultiArray::ConstPtr &height_msg)
  {
    mavlink::common::msg::COMMAND_LONG msg;
    // MAV_CMD_CONDITION_CHANGE_ALT
    msg.command = 113;
    // set descent/ascend rate
    msg.param1 = height_msg->data[0];
    // set target altitude
    msg.param7 = height_msg->data[1];

    UAS_FCU(m_uas)->send_message_ignore_drop(msg);
  }

  void yaw_cb(const std_msgs::Float64MultiArray::ConstPtr &yaw_msg)
  {
    mavlink::common::msg::COMMAND_LONG msg;
    // MAV_CMD_CONDITION_YAW
    msg.command = 115;
    // set target angle
    msg.param1 = yaw_msg->data[0];
    // 0: absolute angle, 1: relative offset
    msg.param4 = yaw_msg->data[1];

    UAS_FCU(m_uas)->send_message_ignore_drop(msg);
  }

  void vision_pose_cb(const nav_msgs::Odometry::ConstPtr &odom)
  {
    mavlink::common::msg::VISION_POSITION_ESTIMATE msg;
    msg.x = odom->pose.pose.position.x;
    msg.y = odom->pose.pose.position.y;
    msg.z = odom->pose.pose.position.z;
    // Divide the frequency of vision pose by 10 
    if(vision_pose_fre_div_ == 9)
      UAS_FCU(m_uas)->send_message_ignore_drop(msg);
    vision_pose_fre_div_ = vision_pose_fre_div_ == 9? 0: vision_pose_fre_div_ + 1;
  }

  void vision_speed_cb(const nav_msgs::Odometry::ConstPtr &odom)
  {
    mavlink::common::msg::VISION_SPEED_ESTIMATE msg;
    msg.x = odom->twist.twist.linear.x;
    msg.y = odom->twist.twist.linear.y;
    msg.z = odom->twist.twist.linear.z;
    // Divide the frequency of vision speed by 4
    if(vision_speed_fre_div_ == 3)
      UAS_FCU(m_uas)->send_message_ignore_drop(msg);
    vision_speed_fre_div_ = vision_speed_fre_div_ == 3? 0: vision_speed_fre_div_ + 1;
  }

  void handle_progress(const mavlink::mavlink_message_t *msg, mavlink::common::msg::COMMAND_ACK &progress_msg)
  {
    auto progress = boost::make_shared<std_msgs::Int32>();
    progress->data = progress_msg.result;
    progress_pub.publish(progress);
    // Modify the following conditions to adapt your use
    if(progress_msg.command != 31010 && progress->data == 0 && conversion_ != 0)
      conversion_ = 0;
  }
};
}  // namespace std_plugins
}  // namespace mavros

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mavros::std_plugins::WhudPlugin,
                       mavros::plugin::PluginBase)
