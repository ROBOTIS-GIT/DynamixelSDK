%{ 
Copyright 2017 ROBOTIS CO., LTD.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
%}

%{ 
*********     Read and Write Example      ******************
* Required Environment to run this example :
    - Protocol 2.0 supported DYNAMIXEL(X, P, PRO/PRO(A), MX 2.0 series)
    - DYNAMIXEL Starter Set (U2D2, U2D2 PHB, 12V SMPS)
* How to use the example :
    - Use proper DYNAMIXEL Model definition from line #44
    - Build and Run from proper architecture subdirectory.
    - For ARM based SBCs such as Raspberry Pi, use linux_sbc subdirectory to build and run.
    - https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_sdk/overview/

* Author: Ryu Woon Jung (Leon)

* Maintainer : Zerom, Will Son
*********************************************************** 
%}

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
    [notfound, warnings] = loadlibrary(lib_name, 'dynamixel_sdk.h', 'addheader', 'port_handler.h', 'addheader', 'packet_handler.h');
end

%{
********* DYNAMIXEL Model *********
***** (Use only one definition at a time) ***** 
%}

  My_DXL = 'X_SERIES'; % X330, X430, X540, 2X430  
% My_DXL = 'PRO_SERIES'; % H54, H42, M54, M42, L54, L42
% My_DXL = 'PRO_A_SERIES'; % PRO series with (A) firmware update.
% My_DXL = 'P_SERIES'; % PH54, PH42, PM54
% My_DXL = 'XL320';  % [WARNING] Operating Voltage : 7.4V
% My_DXL = 'MX_SERIES'; % MX series with 2.0 firmware update.

% Control table address and data to Read/Write for my DYNAMIXEL, My_DXL, in use. 
switch (My_DXL)

    case {'X_SERIES','MX_SERIES'}
        ADDR_TORQUE_ENABLE          = 64;
        ADDR_GOAL_POSITION          = 116;
        ADDR_PRESENT_POSITION       = 132;
        DXL_MINIMUM_POSITION_VALUE  = 0; % Dynamixel will rotate between this value
        DXL_MAXIMUM_POSITION_VALUE  = 4095; % and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
        BAUDRATE                    = 57600;

    case ('PRO_SERIES')
        ADDR_TORQUE_ENABLE          = 562;  % Control table address is different in DYNAMIXEL model
        ADDR_GOAL_POSITION          = 596;
        ADDR_PRESENT_POSITION       = 611;
        DXL_MINIMUM_POSITION_VALUE  = -150000;  % Refer to the Minimum Position Limit of product eManual
        DXL_MAXIMUM_POSITION_VALUE  = 150000;  % Refer to the Maximum Position Limit of product eManual
        BAUDRATE                    = 57600;
    
    case {'P_SERIES','PRO_A_SERIES'}
        ADDR_TORQUE_ENABLE          = 512;  % Control table address is different in DYNAMIXEL model
        ADDR_GOAL_POSITION          = 564;
        ADDR_PRESENT_POSITION       = 580;
        DXL_MINIMUM_POSITION_VALUE  = -150000;  % Refer to the Minimum Position Limit of product eManual
        DXL_MAXIMUM_POSITION_VALUE  = 150000;  % Refer to the Maximum Position Limit of product eManual
        BAUDRATE                    = 57600;
    case ('XL320')
        ADDR_TORQUE_ENABLE          = 24;
        ADDR_GOAL_POSITION          = 30;
        ADDR_PRESENT_POSITION       = 37;
        DXL_MINIMUM_POSITION_VALUE  = 0;  % Refer to the CW Angle Limit of product eManual
        DXL_MAXIMUM_POSITION_VALUE  = 1023;  % Refer to the CCW Angle Limit of product eManual
        BAUDRATE                    = 1000000;  % Default Baudrate of XL-320 is 1Mbps
end


% DYNAMIXEL Protocol Version (1.0 / 2.0)
% https://emanual.robotis.com/docs/en/dxl/protocol2/ 
PROTOCOL_VERSION            = 2.0;          


% Factory default ID of all DYNAMIXEL is 1
DXL_ID                      = 1; 

% Use the actual port assigned to the U2D2. 
% ex) Windows: "COM*", Linux: "/dev/ttyUSB*", Mac: "/dev/tty.usbserial-*" 
DEVICENAME                  = '/dev/ttyUSB0';       

% Common Control Table Address and Data 
ADDR_OPERATING_MODE         = 11;          
OPERATING_MODE              = 3;            % value for operating mode for position control                                
TORQUE_ENABLE               = 1;            % Value for enabling the torque
TORQUE_DISABLE              = 0;            % Value for disabling the torque
DXL_MOVING_STATUS_THRESHOLD = 20;           % Dynamixel moving status threshold

ESC_CHARACTER               = 'e';          % Key for escaping loop

COMM_SUCCESS                = 0;            % Communication Success result value
COMM_TX_FAIL                = -1001;        % Communication Tx Failed

% Initialize PortHandler Structs
% Set the port path
% Get methods and members of PortHandlerLinux or PortHandlerWindows
port_num = portHandler(DEVICENAME);

% Initialize PacketHandler Structs
packetHandler();

index = 1;
dxl_comm_result = COMM_TX_FAIL;           % Communication result
dxl_goal_position = [DXL_MINIMUM_POSITION_VALUE DXL_MAXIMUM_POSITION_VALUE];         % Goal position

dxl_error = 0;                              % Dynamixel error
dxl_present_position = 0;                   % Present position


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


% Enable Dynamixel Torque
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE);
dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION);
dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION);
if dxl_comm_result ~= COMM_SUCCESS
    fprintf('%s\n', getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
elseif dxl_error ~= 0
    fprintf('%s\n', getRxPacketError(PROTOCOL_VERSION, dxl_error));
else
    fprintf('Dynamixel has been successfully connected \n');
end


while 1
    if input('Press any key to continue! (or input e to quit!)\n', 's') == ESC_CHARACTER
        break;
    end

    % Write goal position
    write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_PRO_GOAL_POSITION, typecast(int32(dxl_goal_position(index)), 'uint32'));
    dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION);
    dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION);
    if dxl_comm_result ~= COMM_SUCCESS
        fprintf('%s\n', getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
    elseif dxl_error ~= 0
        fprintf('%s\n', getRxPacketError(PROTOCOL_VERSION, dxl_error));
    end

    while 1
        % Read present position
        dxl_present_position = read4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_PRO_PRESENT_POSITION);
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

    % Change goal position
    if index == 1
        index = 2;
    else
        index = 1;
    end
end


% Disable Dynamixel Torque
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE);
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
