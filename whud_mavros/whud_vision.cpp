#include <mavros/mavros_plugin.h>

#include <nav_msgs/Odometry.h>

namespace mavros{
    namespace std_plugins{
        class WhudVisionPlugin : public plugin::PluginBase{
            public:
                WhudVisionPlugin() : PluginBase(), mav_control_nh("~whud_vision"){}
                void initialize(UAS &uas_) override {
                    PluginBase::initialize(uas_);

                    vision_pose_sub = mav_control_nh.subscribe("vision", 1, &WhudVisionPlugin::vision_pose_cb, this);
                    vision_speed_sub = mav_control_nh.subscribe("vision", 1, &WhudVisionPlugin::vision_speed_cb, this);

                }
                Subscriptions get_subscriptions() override
                {
                    return{};
                }
            private:
                ros::NodeHandle mav_control_nh;
                ros::Subscriber vision_pose_sub;
                ros::Subscriber vision_speed_sub;

                int vision_pose_fre_div_ = 0;
                int vision_speed_fre_div_ = 0;

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
        };
    }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mavros::std_plugins::WhudVisionPlugin,
                       mavros::plugin::PluginBase)
