/*******************************************************************************
* Copyright 2017 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/* Author: Ryu Woon Jung (Leon) */

#ifndef DYNAMIXEL_SDK_INCLUDE_DYNAMIXEL_SDK_ROBOTISDEF_C_H_
#define DYNAMIXEL_SDK_INCLUDE_DYNAMIXEL_SDK_ROBOTISDEF_C_H_

#if defined(_WIN32) || defined(_WIN64)
typedef signed char         int8_t;
typedef signed short int    int16_t;
typedef signed int          int32_t;
#endif

typedef unsigned char       uint8_t;
typedef unsigned short int  uint16_t;
typedef unsigned int        uint32_t;

#define True                1
#define False               0

#define NOT_USED_ID         255

#endif /* DYNAMIXEL_SDK_INCLUDE_DYNAMIXEL_SDK_ROBOTISDEF_C_H_ */
