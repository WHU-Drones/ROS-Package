/**
 * @file TransmitBasic.hpp
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

#include <serial/serial.h>

#include "ProtocolExtend.hpp"

class TransmitBasic
{
private:
    serial::Serial* mSerial;
    std::string mPortPath;
    int mBaundrate;

    void SerialInit();
    template <typename T> void add_sum(uint8_t* sum, T* sum_data, uint8_t length);
    
public:
    TransmitBasic(serial::Serial* serial_port, std::string port_path, int baundrate);
    ~TransmitBasic();
    template <typename T> void SendData(pFrame* frame, T* data);
};

template <typename T>
void TransmitBasic::add_sum(uint8_t* sum, T* sum_data, uint8_t length)
{
    for(uint8_t i = 0;  i< length; i++)
    {
        *sum += ((uint8_t*)sum_data)[i]; 
    }
}

template <typename T>
void TransmitBasic::SendData(pFrame* frame, T* data)
{
    if (mSerial->isOpen())
    {
        uint8_t sum = 0;
        add_sum<const uint8_t>(&sum, packet_ID, 2);
        add_sum<pFrame>(&sum, frame, 2);
        add_sum<T>(&sum, data, frame->length);
        
        mSerial->write(packet_ID, 2);
        mSerial->write((uint8_t*)frame, 2);
        mSerial->write((uint8_t*)data, frame->length);
        mSerial->write(&sum, 1);
    }
}