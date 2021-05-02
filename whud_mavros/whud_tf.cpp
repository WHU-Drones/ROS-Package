#include <mavros/mavros_plugin.h>

#include <tf/tfMessage.h>
#include <tf/transform_listener.h>

namespace mavros{
    namespace std_plugins{
        class WhudTfPlugin : public plugin::PluginBase{
            public:
                WhudTfPlugin() : PluginBase(), mav_control_nh("~whud_tf"){}
                void initialize(UAS &uas_) override {
                    PluginBase::initialize(uas_);

                    tf_sub = mav_control_nh.subscribe("/tf", 1, &WhudTfPlugin::tf_cb, this);
                }
                Subscriptions get_subscriptions() override
                {
                    return{};
                }
            private:
                ros::NodeHandle mav_control_nh;
                ros::Subscriber tf_sub;

                tf::TransformListener tf_listener_;

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
        };
    }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mavros::std_plugins::WhudTfPlugin,
                       mavros::plugin::PluginBase)
