#include <mavros/mavros_plugin.h>

#include <geometry_msgs/Twist.h>
#include <std_msgs/Int32.h>

namespace mavros{
    namespace std_plugins{
        class WhudNavigationPlugin : public plugin::PluginBase{
            public:
                WhudNavigationPlugin() : PluginBase(), mav_control_nh("~whud_navigation"){}
                void initialize(UAS &uas_) override {
                    PluginBase::initialize(uas_);

                    navigation_sub = mav_control_nh.subscribe("cmd_vel", 1, &WhudNavigationPlugin::navigation_cb, this);
                    conversion_sub = mav_control_nh.subscribe("conversion", 1, &WhudNavigationPlugin::conversion_cb, this);

                    navigation_progress_pub = mav_control_nh.advertise<std_msgs::Int32>("navigation_progress", 1);
                }
                Subscriptions get_subscriptions() override
                {
                    return {
                            make_handler(&WhudNavigationPlugin::handle_progress),
                    };
                }
            private:
                ros::NodeHandle mav_control_nh;
                ros::Subscriber navigation_sub;
                ros::Subscriber conversion_sub;

                ros::Publisher navigation_progress_pub;

                int conversion_ = 0;

                void navigation_cb(const geometry_msgs::Twist::ConstPtr &nav_msg)
                {
                    mavlink::common::msg::COMMAND_LONG msg;
                    // MAV_CMD_USER_1
                    msg.command = 31010;
                    msg.param1 = nav_msg->linear.x;
                    msg.param2 = nav_msg->linear.y;
                    msg.param3 = nav_msg->linear.z;
                    msg.param4 = nav_msg->angular.z;
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

                void handle_progress(const mavlink::mavlink_message_t *msg, mavlink::common::msg::COMMAND_ACK &progress_msg)
                {
                    auto progress = boost::make_shared<std_msgs::Int32>();
                    progress->data = progress_msg.result;

                    if(progress_msg.command == 31010)
                        navigation_progress_pub.publish(progress);
                    // Modify the following conditions to adapt your use
                    if(progress_msg.command != 31010 && progress->data == 0 && conversion_ != 0)
                        conversion_ = 0;
                }
        };
    }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mavros::std_plugins::WhudNavigationPlugin,
                       mavros::plugin::PluginBase)
