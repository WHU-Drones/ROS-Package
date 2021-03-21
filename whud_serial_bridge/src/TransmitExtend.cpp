/**
 * @file TransmitExtend.cpp
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
#include "TransmitExtend.hpp"

using namespace std;

/**
 * @brief Construct a new Transmit Extend:: Transmit Extend object
 * 
 * @param port_path 
 * @param baundrate 
 */
TransmitExtend::TransmitExtend(std::string port_path, int baundrate):
    TransmitBasic(port_path, baundrate){ }

/**
 * @brief Destroy the Transmit Extend:: Transmit Extend object
 * 
 */
TransmitExtend::~TransmitExtend(){ }

/**
 * @brief receive Lidar-SLAM Point(ros msg) and send it to stm32
 * 
 * @param msg odometry given by other nodes
 */
void TransmitExtend::LidarSlam_Point_callback(const tf::tfMessage::ConstPtr& msg)
{
    pFrame frame;
    p_LidarSlam_Point lidar_slam_point;
    frame.func = F_LidarSlam_Point;
    frame.length = L_LidarSlam_Point;

    tf::StampedTransform tf_transform;

    try
    {
        tf_listener_.lookupTransform("/map", "/track_link", ros::Time(0), tf_transform);
    }
    catch(tf::TransformException &ex)
    {
        ROS_ERROR("%s", ex.what());
    }

    lidar_slam_point.point.x = (int16_t)(tf_transform.getOrigin().getX() * 100);
    lidar_slam_point.point.y = (int16_t)(tf_transform.getOrigin().getY() * 100);

    // Show the tf data
    // ROS_INFO("x:%d, y:%d", lidar_slam_point.point.x, lidar_slam_point.point.y);

    SendData<p_LidarSlam_Point>(&frame, &lidar_slam_point);
}

/**
 * @brief receive V-Slam Point(ros msg) and send it to stm32
 * 
 * @param msg odometry given by other nodes
 */
void TransmitExtend::VSlam_Point_callback(const nav_msgs::Odometry::ConstPtr& msg)
{
    pFrame frame;
    p_VSlam_Point vslam_point;
    frame.func = F_VSlam_Point;
    frame.length = L_VSlam_Point;
    vslam_point.point.x = (int16_t)(msg->pose.pose.position.x * 100);
    vslam_point.point.y = (int16_t)(msg->pose.pose.position.y * 100);
    vslam_point.point.z = (int16_t)(msg->pose.pose.position.z * 100);

    SendData<p_VSlam_Point>(&frame, &vslam_point);
}