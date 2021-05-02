#include <mavros/mavros_plugin.h>

#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Int32.h>

namespace mavros{
    namespace std_plugins{
        class WhudHeightControlPlugin : public plugin::PluginBase{
            public:
                WhudHeightControlPlugin() : PluginBase(), mav_control_nh("~whud_height_control"){}
                void initialize(UAS &uas_) override {
                    PluginBase::initialize(uas_);

                    height_control_sub = mav_control_nh.subscribe("height_control", 1, &WhudHeightControlPlugin::height_control_cb, this);

                    height_control_progress_pub = mav_control_nh.advertise<std_msgs::Int32>("height_control_progress", 1);
                }
                Subscriptions get_subscriptions() override
                {
                    return {
                            make_handler(&WhudHeightControlPlugin::handle_progress),
                    };
                }
            private:
                ros::NodeHandle mav_control_nh;
                ros::Subscriber height_control_sub;

                ros::Publisher height_control_progress_pub;

                void height_control_cb(const std_msgs::Float64MultiArray::ConstPtr &height_msg)
                {
                    mavlink::common::msg::COMMAND_LONG msg;
                    // MAV_CMD_CONDITION_CHANGE_ALT
                    msg.command = 113;
                    // set descent/ascend rate, default unit m/s
                    msg.param1 = height_msg->data[0];
                    // set target altitude, default unit m
                    msg.param7 = height_msg->data[1];

                    UAS_FCU(m_uas)->send_message_ignore_drop(msg);
                }

                void handle_progress(const mavlink::mavlink_message_t *msg, mavlink::common::msg::COMMAND_ACK &progress_msg)
                {
                    auto progress = boost::make_shared<std_msgs::Int32>();
                    progress->data = progress_msg.result;

                    if(progress_msg.command == 113)
                        height_control_progress_pub.publish(progress);
                }
        };
    }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mavros::std_plugins::WhudHeightControlPlugin,
                       mavros::plugin::PluginBase)