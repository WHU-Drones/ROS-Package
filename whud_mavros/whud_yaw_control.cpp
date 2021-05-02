#include <mavros/mavros_plugin.h>

#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Int32.h>

namespace mavros{
    namespace std_plugins{
        class WhudYawControlPlugin : public plugin::PluginBase{
            public:
                WhudYawControlPlugin() : PluginBase(), mav_control_nh("~whud_yaw_control"){}
                void initialize(UAS &uas_) override {
                    PluginBase::initialize(uas_);

                    yaw_control_sub = mav_control_nh.subscribe("yaw_control", 1, &WhudYawControlPlugin::yaw_control_cb, this);

                    yaw_control_progress_pub = mav_control_nh.advertise<std_msgs::Int32>("yaw_control_progress", 1);
                }
                Subscriptions get_subscriptions() override
                {
                    return {
                            make_handler(&WhudYawControlPlugin::handle_progress),
                    };
                }
            private:
                ros::NodeHandle mav_control_nh;
                ros::Subscriber yaw_control_sub;

                ros::Publisher yaw_control_progress_pub;

                void yaw_control_cb(const std_msgs::Float64MultiArray::ConstPtr &yaw_msg)
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

                void handle_progress(const mavlink::mavlink_message_t *msg, mavlink::common::msg::COMMAND_ACK &progress_msg)
                {
                    auto progress = boost::make_shared<std_msgs::Int32>();
                    progress->data = progress_msg.result;

                    if(progress_msg.command == 115)
                        yaw_control_progress_pub.publish(progress);
                }
        };
    }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mavros::std_plugins::WhudYawControlPlugin,
                       mavros::plugin::PluginBase)