/**
 * @file TransmitExtend.hpp
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

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <tf/tfMessage.h>
#include <tf/transform_listener.h>

#include "ProtocolExtend.hpp"
#include "TransmitBasic.hpp"

class TransmitExtend: public TransmitBasic
{ 
public:
    TransmitExtend(std::string port_path, int baundrate);
    ~TransmitExtend();

    void LidarSlam_Point_callback(const tf::tfMessage::ConstPtr& msg);
    void VSlam_Point_callback(const nav_msgs::Odometry::ConstPtr& msg);
private:
    tf::TransformListener tf_listener_;
};