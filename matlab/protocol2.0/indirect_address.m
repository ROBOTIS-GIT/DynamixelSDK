%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright 2017 ROBOTIS CO., LTD.
%
% Licensed under the Apache License, Version 2.0 (the "License");
% you may not use this file except in compliance with the License.
% You may obtain a copy of the License at
%
%     http://www.apache.org/licenses/LICENSE-2.0
%
% Unless required by applicable law or agreed to in writing, software
% distributed under the License is distributed on an "AS IS" BASIS,
% WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
% See the License for the specific language governing permissions and
% limitations under the License.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Author: Ryu Woon Jung (Leon)

%
% *********     Indirect Address Example      *********
%
%
% Available DYNAMIXEL model on this example : All models using Protocol 2.0
% This example is designed for using a DYNAMIXEL PRO 54-200, and an USB2DYNAMIXEL.
% To use another DYNAMIXEL model, such as X series, see their details in E-Manual(emanual.robotis.com) and edit below variables yourself.
% Be sure that DYNAMIXEL PRO properties are already set as %% ID : 1 / Baudnum : 1 (Baudrate : 57600)
%

clc;
clear all;

lib_name = '';

if strcmp(computer, 'PCWIN')
  lib_name = 'dxl_x86_c';
elseif strcmp(computer, 'PCWIN64')
  lib_name = 'dxl_x64_c';
elseif strcmp(computer, 'GLNX86')
  lib_name = 'libdxl_x86_c';
elseif strcmp(computer, 'GLNXA64')
  lib_name = 'libdxl_x64_c';
elseif strcmp(computer, 'MACI64')
  lib_name = 'libdxl_mac_c';
end

% Load Libraries
if ~libisloaded(lib_name)
    [notfound, warnings] = loadlibrary(lib_name, 'dynamixel_sdk.h', 'addheader', 'port_handler.h', 'addheader', 'packet_handler.h', 'addheader', 'group_sync_write.h', 'addheader', 'group_sync_read.h');
end

%{
********* DYNAMIXEL Model *********
***** (Use only one definition at a time) ***** 
%}

 MY_DXL = 'X_SERIES'; % X330, X430, X540, 2X430  
% MY_DXL = 'PRO_SERIES'; % H54, H42, M54, M42, L54, L42
% MY_DXL = 'PRO_A_SERIES'; % PRO series with (A) firmware update.
% MY_DXL = 'P_SERIES'; % PH54, PH42, PM54
% MY_DXL = 'MX_SERIES'; % MX series with 2.0 firmware update.

% Control table address and data to Read/Write for my DYNAMIXEL, MY_DXL, in use. 
switch (MY_DXL)

    case {'X_SERIES','MX_SERIES'}
        ADDR_TORQUE_ENABLE                        = 64;
        ADDR_GOAL_POSITION                        = 116;
        ADDR_PRESENT_POSITION                     = 132;
        ADDR_LED                                  = 65;
        ADDR_MOVING                               = 122;
        ADDR_INDIRECTADDRESS_GOAL_POSITION        = 168; % Use 4 addresses from Indirect Address #1
        ADDR_INDIRECTADDRESS_PRESENT_POSITION     = 176; % Use 4 addresses from Indirect Address #5
        ADDR_INDIRECTDATA_GOAL_POSITION           = 224; % Use 4 addresses from Indirect Data #1
        ADDR_INDIRECTDATA_PRESENT_POSITION        = 228; % Use 4 addresses from Indirect Data #5
        DXL_MINIMUM_POSITION_VALUE                = 0; % DYNAMIXEL will rotate between this value
        DXL_MAXIMUM_POSITION_VALUE                = 4095; % and this value (note that the DYNAMIXEL would not move when the position value is out of movable range. Check e-manual about the range of the DYNAMIXEL you use.)
        MAX_LED_VALUE                             = 1; % Maxiumum value for DYNAMIXEL's red LED
        BAUDRATE                                  = 57600;
        
    case ('PRO_SERIES')
        ADDR_TORQUE_ENABLE                        = 562; 
        ADDR_GOAL_POSITION                        = 596;
        ADDR_PRESENT_POSITION                     = 611;
        ADDR_LED                                  = 563;
        ADDR_MOVING                               = 610;
        ADDR_INDIRECTADDRESS_GOAL_POSITION        = 49; % Use 4 addresses from Indirect Address #1
        ADDR_INDIRECTADDRESS_PRESENT_POSITION     = 59; % Use 4 addresses from Indirect Address #5
        ADDR_INDIRECTDATA_GOAL_POSITION           = 634; % Use 4 addresses from Indirect Data #1
        ADDR_INDIRECTDATA_PRESENT_POSITION        = 639; % Use 4 addresses from Indirect Data #5
        DXL_MINIMUM_POSITION_VALUE                = -150000; % Refer to the Minimum Position Limit of product eManual
        DXL_MAXIMUM_POSITION_VALUE                = 150000; % Refer to the Maximum Position Limit of product eManual
        MAX_LED_VALUE                             = 255; % Maxiumum value for DYNAMIXEL's red LED
        BAUDRATE                                  = 57600;
       
    case {'P_SERIES','PRO_A_SERIES'}
        ADDR_TORQUE_ENABLE                        = 512;
        ADDR_GOAL_POSITION                        = 564;
        ADDR_PRESENT_POSITION                     = 580;
        ADDR_LED                                  = 513;
        ADDR_MOVING                               = 570;
        ADDR_INDIRECTADDRESS_GOAL_POSITION        = 168; % Use 4 addresses from Indirect Address #1
        ADDR_INDIRECTADDRESS_PRESENT_POSITION     = 178; % Use 4 addresses from Indirect Address #5
        ADDR_INDIRECTDATA_GOAL_POSITION           = 634; % Use 4 addresses from Indirect Data #1
        ADDR_INDIRECTDATA_PRESENT_POSITION        = 639; % Use 4 addresses from Indirect Data #5
        DXL_MINIMUM_POSITION_VALUE                = -150000; % Refer to the Minimum Position Limit of product eManual
        DXL_MAXIMUM_POSITION_VALUE                = 150000; % Refer to the Maximum Position Limit of product eManual
        MAX_LED_VALUE                             = 255; % Maxiumum value for DYNAMIXEL's red LED
        BAUDRATE                                  = 57600;
       
end

% Data Byte Length
LEN_LED                                   = 1;
LEN_GOAL_POSITION                         = 4;
LEN_MOVING                                = 1;
LEN_PRESENT_POSITION                      = 4;

% Protocol version
PROTOCOL_VERSION                          = 2.0; % See which protocol version is used in the DYNAMIXEL

% Default setting
DXL_ID                                    = 1; % DYNAMIXEL ID: 1
DEVICENAME                                = '/dev/ttyUSB0'; % Check which port is being used on your controller
                                                            % ex) Windows: 'COM1'   Linux: '/dev/ttyUSB*' Mac: '/dev/tty.usbserial-*'

TORQUE_ENABLE                             = 1; % Value for enabling the torque
TORQUE_DISABLE                            = 0; % Value for disabling the torque
DXL_LED_ON                                = 1; % DYNAMIXEL LED will light between this value
DXL_LED_OFF                               = 0; % and this value
DXL_MOVING_STATUS_THRESHOLD               = 24; % DYNAMIXEL moving status threshold

ESC_CHARACTER                             = 'e'; % Key for escaping loop

COMM_SUCCESS                              = 0; % Communication Success result value
COMM_TX_FAIL                              = -1001; % Communication Tx Failed

% Initialize PortHandler Structs
% Set the port path
% Get methods and members of PortHandlerLinux or PortHandlerWindows
port_num = portHandler(DEVICENAME);

% Initialize PacketHandler Structs
packetHandler();

index = 1;
dxl_comm_result = COMM_TX_FAIL; % Communication result
dxl_addparam_result = false; % AddParam result
dxl_getdata_result = false; % GetParam result
dxl_goal_position = [DXL_MINIMUM_POSITION_VALUE DXL_MAXIMUM_POSITION_VALUE]; % Goal position
dxl_error = 0; % DYNAMIXEL error
dxl_moving = 0; % DYNAMIXEL moving status
dxl_led_value = [0 MAX_LED_VALUE]; 
dxl_present_position = 0; % Present position

% Open port
if (openPort(port_num))
    fprintf('Succeeded to open the port!\n');
else
    unloadlibrary(lib_name);
    fprintf('Failed to open the port!\n');
    input('Press any key to terminate...\n');
    return;
end

% Set port baudrate
if (setBaudRate(port_num, BAUDRATE))
    fprintf('Succeeded to change the baudrate!\n');
else
    unloadlibrary(lib_name);
    fprintf('Failed to change the baudrate!\n');
    input('Press any key to terminate...\n');
    return;
end

% Disable DYNAMIXEL Torque :
% Indirect address is not accessible when the torque is enabled in PRO / P series
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_DISABLE);
dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION);
dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION);
if dxl_comm_result ~= COMM_SUCCESS
    fprintf('%s\n', getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
elseif dxl_error ~= 0
    fprintf('%s\n', getRxPacketError(PROTOCOL_VERSION, dxl_error));
else
    fprintf('Torque OFF DYNAMIXEL.\n');
end


%Link Goal Position to Indirect address of Goal Position
for Re = 0:1:3 % Repeat 4 times
    %Goal Position Linking
    write2ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_INDIRECTADDRESS_GOAL_POSITION + 2*Re, ADDR_GOAL_POSITION + Re);
    dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION);
    dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION);
    if dxl_comm_result ~= 1 %COMM_SUCCESS is replaced as 1
        fprintf('%s\n', getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
    elseif dxl_error ~= 0
        fprintf('%s\n', getRxPacketError(PROTOCOL_VERSION, dxl_error));
    end
end

%Link Present Position to Indirect address of Present Position
for Re = 0:1:3 % Repeat 4 times
    write2ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_INDIRECTADDRESS_PRESENT_POSITION + 2*Re, ADDR_PRESENT_POSITION + Re);
    dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION);
    dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION);
    if dxl_comm_result ~= 1 %COMM_SUCCESS is replaced as 1
        fprintf('%s\n', getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
    elseif dxl_error ~= 0
        fprintf('%s\n', getRxPacketError(PROTOCOL_VERSION, dxl_error));
    end
end

% Enable DYNAMIXEL Torque
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE);
dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION);
dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION);
if dxl_comm_result ~= COMM_SUCCESS
    fprintf('%s\n', getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
elseif dxl_error ~= 0
    fprintf('%s\n', getRxPacketError(PROTOCOL_VERSION, dxl_error));
else
    fprintf('Torque ON DYNAMIXEL.\n');
end

% Switch Goal Position until input "e" comes 
while 1
    if input('Press any key to continue! (or input e to quit!)\n', 's') == ESC_CHARACTER
        break;
    end

    % Write Goal Position
    write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_INDIRECTDATA_GOAL_POSITION, typecast(int32(dxl_goal_position(index)), 'uint32'));
    dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION);
    dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION);
    if dxl_comm_result ~= COMM_SUCCESS
        fprintf('%s\n', getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
    elseif dxl_error ~= 0
        fprintf('%s\n', getRxPacketError(PROTOCOL_VERSION, dxl_error));
    end

    while 1
        %Read Present Position
        dxl_present_position = read4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_INDIRECTDATA_PRESENT_POSITION);
        dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION);
        dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION);
        if dxl_comm_result ~= COMM_SUCCESS
            fprintf('%s\n', getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
        elseif dxl_error ~= 0
            fprintf('%s\n', getRxPacketError(PROTOCOL_VERSION, dxl_error));
        end

        fprintf('[ID:%03d] GoalPos:%03d  PresPos:%03d\n', DXL_ID, dxl_goal_position(index), typecast(uint32(dxl_present_position), 'int32'));

        if ~(abs(dxl_goal_position(index) - typecast(uint32(dxl_present_position), 'int32')) > DXL_MOVING_STATUS_THRESHOLD)
            break;
        end
    end

    % Switch Goal position
    if index == 1
        index = 2;
    else
        index = 1;
    end
end

% Disable DYNAMIXEL Torque
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_DISABLE);
dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION);
dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION);
if dxl_comm_result ~= COMM_SUCCESS
    fprintf('%s\n', getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
elseif dxl_error ~= 0
    fprintf('%s\n', getRxPacketError(PROTOCOL_VERSION, dxl_error));
end

% Close port
closePort(port_num);

% Unload Library
unloadlibrary(lib_name);

close all;
clear all;