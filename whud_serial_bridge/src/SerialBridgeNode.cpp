/**
 * @file SerialBridgeNode.cpp
 * @author LauZanMo (LauZanMo@whu.edu.cn)
 * @brief 
 * @version 1.0
 * @date 2021-01-05
 * 
 * MIT License
 * 
 * @copyright Copyright (c) 2021 WHU-Drones
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 * 
 */
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Odometry.h>

#include "SerialBridgeNode.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "serial_bridge_node");
    ros::start();

    ros::NodeHandle node_handle;
    SerialBridgeNode node(node_handle);

    ros::spin();

    return 0;
}

/**
 * @brief Construct a new Serial Bridge Node:: Serial Bridge Node object
 * 
 * @param nh node handle
 */
SerialBridgeNode::SerialBridgeNode(ros::NodeHandle &nh):nh_(nh) {
    node_name_ = ros::this_node::getName();

    nh_.param<std::string>(node_name_ + "/serial_port_path", serial_port_path_, "/dev/ttyUSB0");
    nh_.param<int>(node_name_ + "/serial_baundrate", serial_baundrate_, 115200);

    nh_.param<std::string>(node_name_ + "/lidar_slam_point_topic", lidar_slam_point_topic_, "lidar_slam_point");
    nh_.param<std::string>(node_name_ + "/vslam_point_topic", vslam_point_topic_, "vslam_point");

    SerialInit();

    PublisherInit();
    SubscriberInit();
    ServiceInit();
    ClientInit();
    ActionInit();
    DynamicParamInit();
}

/**
 * @brief Destroy the Serial Bridge Node:: Serial Bridge Node object
 * 
 */
SerialBridgeNode::~SerialBridgeNode() {
    if (serial_bridge_ != NULL) {
        delete serial_bridge_;
    }
}

/**
 * @brief class interface runs in while(ros::ok())
 * 
 */
void SerialBridgeNode::Run() { }

/**
 * @brief init serial
 * 
 */
void SerialBridgeNode::SerialInit() {
    serial_bridge_ = new TransmitExtend(serial_port_path_, serial_baundrate_);
}

/**
 * @brief dynamic parameter callback
 * 
 */
//void SerialBridgeNode::DynamicParamServer_callback(serial_bridge::SerialBridgeNodeConfig &config, uint32_t level) {}

/**
 * @brief init publisher
 * 
 */
void SerialBridgeNode::PublisherInit() { }

/**
 * @brief init subscriber
 * 
 */
void SerialBridgeNode::SubscriberInit() {
    lidar_slam_point_subscriber_ = nh_.subscribe<nav_msgs::Odometry>(lidar_slam_point_topic_, 1,
                            &TransmitExtend::LidarSlam_Point_callback, this->serial_bridge_);
    vslam_point_subscriber_ = nh_.subscribe<nav_msgs::Odometry>(vslam_point_topic_, 1,
                            &TransmitExtend::VSlam_Point_callback, this->serial_bridge_);
}

/**
 * @brief init service
 * 
 */
void SerialBridgeNode::ServiceInit() {}

/**
 * @brief init client
 * 
 */
void SerialBridgeNode::ClientInit() {}

/**
 * @brief init action
 * 
 */
void SerialBridgeNode::ActionInit() {}

/**
 * @brief init dynamic parameter server
 * 
 */
void SerialBridgeNode::DynamicParamInit() {
    //f_ = boost::bind(&SerialBridgeNode::DynamicParamServer_callback, this, _1, _2);
    //dynamic_param_server_.setCallback(f_);
}
