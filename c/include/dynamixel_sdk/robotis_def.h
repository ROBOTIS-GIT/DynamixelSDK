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

// Common definitions for Group Bulk/Sync
#ifndef DXL_MAX_GROUPS
#define DXL_MAX_GROUPS              16
#endif

#ifndef DXL_MAX_NODES
#define DXL_MAX_NODES               253
#endif

#ifndef DXL_MAX_NODE_BUFFER_SIZE
#define DXL_MAX_NODE_BUFFER_SIZE    128
#endif

#ifndef DXL_MAX_PORTS
#define DXL_MAX_PORTS               16
#endif

#define NOT_USED_ID         255

#ifndef DXL_MAX_BUFFER_LEN
#define DXL_MAX_BUFFER_LEN  4096
#endif

#endif /* DYNAMIXEL_SDK_INCLUDE_DYNAMIXEL_SDK_ROBOTISDEF_C_H_ */
