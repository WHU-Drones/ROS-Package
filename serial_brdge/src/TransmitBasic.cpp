/**
 * @file TransmitBasic.cpp
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
#include <ros/ros.h>

#include "TransmitBasic.hpp"

/**
 * @brief Construct a new Transmit Basic:: Transmit Basic object
 * 
 * @param serial_port serial object
 * @param port_path serial port path
 * @param baundrate serial baundrate
 */
TransmitBasic::TransmitBasic(serial::Serial* serial_port, std::string port_path, int baundrate):
    mSerial(serial_port), mPortPath(port_path), mBaundrate(baundrate)
{
    SerialInit();
}

/**
 * @brief Destroy the Transmit Basic:: Transmit Basic object
 * 
 */
TransmitBasic::~TransmitBasic()
{
    mSerial->close();
}

/**
 * @brief init serial with given parameters
 * 
 */
void TransmitBasic::SerialInit()
{
    mSerial->setPort(mPortPath);
    mSerial->setBaudrate(mBaundrate);
    serial::Timeout timeout = serial::Timeout::simpleTimeout(1000);
    mSerial->setTimeout(timeout);

    try
    {
        mSerial->open();
    }
    catch (serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open serial port.");
    }

    if (mSerial->isOpen())
    {
        ROS_INFO_STREAM("Serial start.");
    }
}