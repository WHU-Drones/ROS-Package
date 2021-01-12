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
 * @brief receive point(ros msg) and send it to stm32
 * 
 * @param msg pose given by other nodes
 */
void TransmitExtend::PointXY_callback(const nav_msgs::Odometry::ConstPtr& msg)
{
    pFrame frame;
    pPointXY pointxy;
    frame.func = FU_PointXY;
    frame.length = LU_PointXY;
    pointxy.point.x = (float)msg->pose.pose.position.x * 100;
    pointxy.point.y = (float)msg->pose.pose.position.y * 100;

    SendData<pPointXY>(&frame, &pointxy);
}

/**
 * @brief receive velocity(ros msg) and send it to stm32
 * 
 * @param msg twist given by other nodes
 */
void TransmitExtend::ExpectVel_callback(const geometry_msgs::Twist::ConstPtr& msg)
{
    pFrame frame;
    pExpectVel expect_vel;
    frame.func = FU_ExpectVel;
    frame.length = LU_ExpectVel;
    expect_vel.linear_vel.x = msg->linear.x * 100;
    expect_vel.linear_vel.y = msg->linear.y * 100;
    expect_vel.linear_vel.z = msg->linear.z * 100;
    expect_vel.z_angular_vel = msg->angular.z;

    SendData<pExpectVel>(&frame, &expect_vel);
}