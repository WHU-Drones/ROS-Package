#pragma once

#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <vector>
#include <queue>
#include <thread>
#include <mutex>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Imu.h>
#include <image_transport/image_transport.h>

#include <opencv2/core/core.hpp>

#include "System.h"
#include "ImuTypes.h"
#include "CommonNode.h"

using namespace std;

class StereoInertialNode : public CommonNode
{
public:
    StereoInertialNode(const ORB_SLAM3::System::eSensor sensor, ros::NodeHandle &node_handle, image_transport::ImageTransport &image_transport);
    ~StereoInertialNode();
    void SyncWithImu();
    cv::Mat GetImage(const sensor_msgs::ImageConstPtr &img_msg);
    void GrabImu(const sensor_msgs::ImuConstPtr &imu_msg);
    void GrabImageLeft(const sensor_msgs::ImageConstPtr& img_msg);
    void GrabImageRight(const sensor_msgs::ImageConstPtr& img_msg);
private:
    ros::Subscriber left_sub_;
    ros::Subscriber right_sub_;
    ros::Subscriber imu_sub_;

    std::string name_of_node_;

    mutex imu_mutex_, left_mutex_, right_mutex_;
    queue<sensor_msgs::ImuConstPtr> imu_buf_;
    queue<sensor_msgs::ImageConstPtr> left_buf_, right_buf_;

    bool clahe_param_;
    cv::Ptr<cv::CLAHE> clahe_ = cv::createCLAHE(3.0, cv::Size(8, 8));
};
