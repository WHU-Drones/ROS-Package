/**
 * @file ProtocolExtend.hpp
 * @author LauZanMo (LauZanMo@whu.edu.cn)
 * @brief 
 * @version 1.0
 * @date 2021-01-03
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

#include "ProtocolBasic.hpp"

/**
 * @brief enum the function words with order number
 * 
 * @note 
 * if the messages are transmitted from stm32 to Xavier, the first order number
 * should be less than 5(recommended from 0 to 4), in the opposite, the first
 * order number should not be less than 5(recommended from 5 to 9).
 * 
 * first character:
 * F means function word
 * 
 * second character:
 * L means from lower machine
 * U means from upper machine
 */
enum
{
    FL_IMU = 0x01,
    FL_T = 0x02,            //use ano optical flow
    FL_PointXYZ = 0x03,
    FU_PointXY = 0x51,
    FU_ExpectVel = 0x52,
    FU_ExpectPos = 0x53,
};

/**
 * @brief enum the data struct length of payload
 * 
 * @note 
 * first character:
 * L means data length
 * 
 * second character:
 * L means from lower machine
 * U means from upper machine
 */
enum
{
    LL_IMU = 24,
    LL_T = 44,
    LL_PointXYZ = 12,
    LU_PointXY = 8,
    LU_ExpectVel = 8,
    LU_ExpectPos = 8,
};

#pragma pack(1)

/**
 * @brief IMU struct in protocol
 * 
 * @note 
 * first character:
 * p means data protocol
 * 
 * unit:
 * linear_acc: cm/s^2
 * angular_vel: rad/s
 */
struct pIMU
{
    pVector3<float> linear_acc;
    pVector3<float> angular_vel;
};

/**
 * @brief T struct in protocol
 * 
 * @note 
 * first character:
 * p means data protocol
 * 
 * unit:
 * point: cm
 */
struct pT
{
    pQuat orientation;
    pVector3<float> point;
};

/**
 * @brief PointXYZ struct in protocol
 *  
 * @note 
 * first character:
 * p means data protocol
 * 
 * unit:
 * point: cm
 */
struct pPointXYZ
{
    pVector3<float> point;
};


/**
 * @brief PointXY struct in protocol
 *  
 * @note 
 * first character:
 * p means data protocol
 * 
 * unit:
 * point: cm
 */
struct pPointXY
{
    pVector2<float> point;
};

/**
 * @brief ExpectVel struct in protocol
 *  
 * @note 
 * first character:
 * p means data protocol
 * 
 * unit:
 * linear_vel: cm/s
 * z_angular_vel: rad/s
 */
struct pExpectVel
{
    pVector3<int16_t> linear_vel;
    int16_t z_angular_vel;
};

/**
 * @brief ExpectPos struct in protocol
 *  
 * @note 
 * first character:
 * p means data protocol
 * 
 * unit:
 * linear_pos: cm
 * z_angular_pos: rad
 */
struct pExpectPos
{
    pVector3<int16_t> linear_pos;
    int16_t z_angular_pos;
};

#pragma pack(0)