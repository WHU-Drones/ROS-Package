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
 * @param serial_port serial object
 * @param port_path serial port path
 * @param baundrate serial baundrate
 */
TransmitExtend::TransmitExtend(serial::Serial* serial_port, std::string port_path, int baundrate):
    mSerial(serial_port), TransmitBasic(serial_port, port_path, baundrate){ }

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

void TransmitExtend::ClearFlag(ParseFlag *flag)
{
    flag->IMU_Flag = false;
    flag->T_Flag = false;
    flag->PointXYZ_Flag = false;
}

/**
 * @brief read data from serial buffer and parse it
 * 
 * @return int data parse state
 */
ParseFlag TransmitExtend::DataParse()
{
    ParseFlag flag;
    ClearFlag (&flag);
    
    size_t size = mSerial->available();
    if(size)
    {
        vector<uint8_t> buffer;
        mSerial->read(buffer, size);

        for (int i = 0; i < buffer.size(); i++)
        {
            mRecData = buffer[i];
            if (mRecCount == 0)
                mSum = 0; //reset
            if (mRecCount < 2)
            {
                HeaderCheck();
            }
            else if (mRecCount < 4)
            {
                FrameParse();
            }
            else if (mRecCount < 4 + mFrame.length)
            {
                PayloadParse();
            }
            else
            {
                if (CheckSum())
                {
                    if (mFrame.func == FL_IMU)
                    {
                        mIMU = mTempIMU;
                        flag.IMU_Flag = true;
                    }
                    else if (mFrame.func == FL_T)
                    {
                        mT = mTempT;
                        flag.T_Flag = true;
                    }
                    else if (mFrame.func == FL_PointXYZ)
                    {
                        mPointXYZ = mTempPointXYZ;
                        flag.PointXYZ_Flag = true;
                    }
                }
                mRecCount = 0;
            }
        }
    }
    return flag;
}

/**
 * @brief check header
 * 
 */
void TransmitExtend::HeaderCheck()
{
    if (mRecData != packet_ID[mRecCount])
        mRecCount = 0;
    else
    {
        mSum += mRecData;
        ++mRecCount;
    }
}

/**
 * @brief parse frame and save it in mFrame menber
 * 
 */
void TransmitExtend::FrameParse()
{
    ((uint8_t*)&mFrame)[mRecCount - 2] = mRecData;
    
    mSum += (uint8_t)mRecData;
    ++mRecCount;
}

/**
 * @brief parse payload and save it in appropriate member
 * 
 */
void TransmitExtend::PayloadParse()
{
    if(mFrame.func == FL_IMU)
        ((uint8_t*)&mTempIMU)[mRecCount - 4] = mRecData;
    else if(mFrame.func == FL_T)
        ((uint8_t*)&mTempT)[mRecCount - 4] = mRecData;
    else if(mFrame.func == FL_PointXYZ)
        ((uint8_t*)&mTempPointXYZ)[mRecCount - 4] = mRecData;
    
    mSum += (uint8_t)mRecData;
    ++mRecCount;
}

/**
 * @brief check sum
 * 
 * @return true frame transmitted correctly
 * @return false frame transmitted incorrectly
 */
bool TransmitExtend::CheckSum()
{
    return (mRecData == mSum);
}