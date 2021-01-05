/**
 * @file ProtocolBasic.hpp
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

#pragma pack(1)

/**
 * @brief header of frame
 * 
 */
static const uint8_t packet_ID[2] = {0xAA, 0xAA};

/**
 * @brief frame struct in protocol
 * 
 * @note 
 * first character:
 * p means data protocol
 */
struct pFrame
{
    uint8_t func;
    uint8_t length;
};

/**
 * @brief vector2d struct in protocol
 * 
 * @tparam T 
 * 
 * @note 
 * first character:
 * p means data protocol
 */
template <typename T>
struct pVector2
{
    T x;
    T y;
};

/**
 * @brief vector3d struct in protocol
 * 
 * @tparam T 
 * 
 * @note 
 * first character:
 * p means data protocol
 */
template <typename T>
struct pVector3
{
    T x;
    T y;
    T z;
};

/**
 * @brief quaternion struct in protocol
 * 
 * @note 
 * first character:
 * p means data protocol
 */
struct pQuat
{
    double qx;
    double qy;
    double qz;
    double qw;
};

#pragma pack(0)