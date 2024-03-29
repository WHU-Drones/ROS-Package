/**
 * @file SerialBridgeNode.hpp
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
#pragma once

#include <iostream>
#include <ros/ros.h>
//#include <dynamic_reconfigure/server.h>

//#include "SerialBridgeNodeConfig.h"
#include "TransmitExtend.hpp"

class SerialBridgeNode
{
public:
    SerialBridgeNode(ros::NodeHandle &nh);
    ~SerialBridgeNode();
    void Run();

private:
    ros::NodeHandle nh_;
    std::string node_name_;

    TransmitExtend* serial_bridge_ = NULL;
    std::string serial_port_path_;
    int serial_baundrate_;

    ros::Subscriber lidar_slam_point_subscriber_;
    std::string lidar_slam_point_topic_;
    ros::Subscriber vslam_point_subscriber_;
    std::string vslam_point_topic_;
    
    //dynamic_reconfigure::Server<serial_bridge::SerialBridgeNodeConfig> dynamic_param_server_;
    //dynamic_reconfigure::Server<serial_bridge::SerialBridgeNodeConfig>::CallbackType f_;
    
    void SerialInit();
    
    void PublisherInit();
    void SubscriberInit();
    void ServiceInit();
    void ClientInit();
    void ActionInit();
    void DynamicParamInit();
    //void DynamicParamServer_callback(serial_bridge::SerialBridgeNodeConfig &config, uint32_t level);
};
