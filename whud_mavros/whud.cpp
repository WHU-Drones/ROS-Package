#include <geometry_msgs/Twist.h>
#include <mavros/mavros_plugin.h>
#include <tf/tfMessage.h>
#include <tf/transform_listener.h>
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

    cmd_vel_sub = mav_control_nh.subscribe("cmd_vel", 1, &WhudPlugin::cmd_vel_cb, this);
    tf_sub = mav_control_nh.subscribe("/tf", 1, &WhudPlugin::tf_cb, this);
  }

  	Subscriptions get_subscriptions() override
	{
		return { /* Rx disabled */ };
	}

 private:
  ros::NodeHandle mav_control_nh;

  ros::Subscriber cmd_vel_sub;
  ros::Subscriber tf_sub;

  tf::TransformListener tf_listener_;

  /* -*- callbacks -*- */

  void cmd_vel_cb(const geometry_msgs::Twist::ConstPtr req)
  {
    mavlink::common::msg::COMMAND_LONG msg;
    
    msg.command = 31010;//MAV_CMD_USER_1
    msg.param1 = req->linear.x;
    msg.param2 = req->linear.y;
    msg.param3 = req->linear.z;
    msg.param4 = req->angular.z;

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
        ROS_ERROR("%s", ex.what());
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
};
}  // namespace std_plugins
}  // namespace mavros

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mavros::std_plugins::WhudPlugin,
                       mavros::plugin::PluginBase)