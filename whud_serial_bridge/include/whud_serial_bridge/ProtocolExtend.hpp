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
 * first character:
 * F means function word
 */
enum
{
    F_LidarSlam_Point = 0x01,
    F_VSlam_Point = 0x02,
};

/**
 * @brief enum the data struct length of payload
 * 
 * @note 
 * first character:
 * L means data length
 */
enum
{
    L_LidarSlam_Point = 4,
    L_VSlam_Point = 6,
};

#pragma pack(1)

/**
 * @brief Lidar-SLAM Point struct in protocol
 * 
 * @note 
 * first character:
 * p means data protocol
 * 
 * unit:
 * point: cm
 */
struct p_LidarSlam_Point
{
    pVector2<int16_t> point;
};

/**
 * @brief V-Slam Point struct in protocol
 * 
 * @note 
 * first character:
 * p means data protocol
 * 
 * unit:
 * point: cm
 */
struct p_VSlam_Point
{
    pVector3<int16_t> point;
};

#pragma pack(0)