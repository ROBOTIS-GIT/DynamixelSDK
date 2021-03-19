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
% *********     Bulk Read Example      *********
%
%
% Available DYNAMIXEL model on this example : MX or X series set to Protocol 1.0
% This example is designed for using two DYNAMIXEL MX-28, and an USB2DYNAMIXEL.
% To use another DYNAMIXEL model, such as X series, see their details in E-Manual(emanual.robotis.com) and edit below variables yourself.
% Be sure that DYNAMIXEL MX properties are already set as %% ID : 1 / Baudnum : 34 (Baudrate : 57600)
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
    [notfound, warnings] = loadlibrary(lib_name, 'dynamixel_sdk.h', 'addheader', 'port_handler.h', 'addheader', 'packet_handler.h', 'addheader', 'group_bulk_read.h');
end

%{
********* DYNAMIXEL Model *********
***** (Use only ONE definition at a time) ***** 
%}

 MY_DXL = 'X_SERIES'; % X430, X540, 2X430  
% MY_DXL = 'AX_SERIES'; % AX Series
% MY_DXL = 'MX_SERIES'; % MX series with 2.0 firmware update.

% Control table address and data to Read/Write for my DYNAMIXEL, MY_DXL, in use. 
switch (MY_DXL)

    case {'X_SERIES'}
        ADDR_MX_TORQUE_ENABLE       = 64;    
        ADDR_MX_GOAL_POSITION       = 116;
        ADDR_MX_PRESENT_POSITION    = 132;
        LEN_MX_GOAL_POSITION        = 4;
        LEN_MX_PRESENT_POSITION     = 4;
        LEN_MX_MOVING               = 1;
        DXL_MINIMUM_POSITION_VALUE  = 0;        
        DXL_MAXIMUM_POSITION_VALUE  = 4095;
        BAUDRATE                    = 57600;
       
    case {'AX_SERIES'}
        ADDR_MX_TORQUE_ENABLE       = 24;    
        ADDR_MX_GOAL_POSITION       = 30;
        ADDR_MX_PRESENT_POSITION    = 36;
        LEN_MX_GOAL_POSITION        = 2;
        LEN_MX_PRESENT_POSITION     = 2;
        LEN_MX_MOVING               = 1;
        DXL_MINIMUM_POSITION_VALUE  = 0;        
        DXL_MAXIMUM_POSITION_VALUE  = 1023;
        BAUDRATE                    = 1000000;
        
    case {'MX_SERIES'}
        ADDR_MX_TORQUE_ENABLE       = 24;    
        ADDR_MX_GOAL_POSITION       = 30;
        ADDR_MX_PRESENT_POSITION    = 36;
        LEN_MX_GOAL_POSITION        = 2;
        LEN_MX_PRESENT_POSITION     = 2;
        LEN_MX_MOVING               = 1;
        DXL_MINIMUM_POSITION_VALUE  = 0; % DYNAMIXEL will rotate between this value
        DXL_MAXIMUM_POSITION_VALUE  = 4095; % and this value (note that the DYNAMIXEL would not move when the position value is out of movable range. Check e-manual about the range of the DYNAMIXEL you use.)
        BAUDRATE                    = 57600;
       
end

% Protocol version
PROTOCOL_VERSION                = 1.0; % See which protocol version is used in the DYNAMIXEL

% Default setting
DXL1_ID                         = 1; % DYNAMIXEL#1 ID: 1
DXL2_ID                         = 2; % DYNAMIXEL#2 ID: 2
DEVICENAME                      = '/dev/ttyUSB0'; % Check which port is being used on your controller
                                                  % ex) Windows: 'COM1'   Linux: '/dev/ttyUSB0' Mac: '/dev/tty.usbserial-*'

TORQUE_ENABLE                   = 1; % Value for enabling the torque
TORQUE_DISABLE                  = 0; % Value for disabling the torque
DXL_MOVING_STATUS_THRESHOLD     = 10; % DYNAMIXEL moving status threshold

ESC_CHARACTER                   = 'e'; % Key for escaping loop

COMM_SUCCESS                    = 0; % Communication Success result value
COMM_TX_FAIL                    = -1001; % Communication Tx Failed

% Initialize PortHandler Structs
% Set the port path
% Get methods and members of PortHandlerLinux or PortHandlerWindows
port_num = portHandler(DEVICENAME);

% Initialize PacketHandler Structs
packetHandler();

% Initialize Groupbulkread Structs
group_num = groupBulkRead(port_num, PROTOCOL_VERSION);

index = 1;
dxl_comm_result = COMM_TX_FAIL; % Communication result
dxl_addparam_result = false; % AddParam result
dxl_getdata_result = false; % GetParam result
dxl_goal_position = [DXL_MINIMUM_POSITION_VALUE DXL_MAXIMUM_POSITION_VALUE]; % Goal position

dxl_error = 0; % DYNAMIXEL error
dxl1_present_position = 0; % Present position
dxl2_moving = 0;  % DYNAMIXEL moving status

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


% Enable DYNAMIXEL#1 Torque
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL1_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE);
dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION);
dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION);
if dxl_comm_result ~= COMM_SUCCESS
    fprintf('%s\n', getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
elseif dxl_error ~= 0
    fprintf('%s\n', getRxPacketError(PROTOCOL_VERSION, dxl_error));
else
    fprintf('DYNAMIXEL has been successfully connected \n');
end

% Enable DYNAMIXEL#2 Torque
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL2_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE);
dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION);
dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION);
if dxl_comm_result ~= COMM_SUCCESS
    fprintf('%s\n', getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
elseif dxl_error ~= 0
    fprintf('%s\n', getRxPacketError(PROTOCOL_VERSION, dxl_error));
else
    fprintf('DYNAMIXEL has been successfully connected \n');
end

% Add parameter storage for DYNAMIXEL#1 present position value
dxl_addparam_result = groupBulkReadAddParam(group_num, DXL1_ID, ADDR_MX_PRESENT_POSITION, LEN_MX_PRESENT_POSITION);
if dxl_addparam_result ~= true
    fprintf('[ID:%03d] groupBulkRead addparam failed', DXL1_ID);
    return;
end

% Add parameter storage for DYNAMIXEL#2 present moving value
dxl_addparam_result = groupBulkReadAddParam(group_num, DXL2_ID, ADDR_MX_MOVING, LEN_MX_MOVING);
if dxl_addparam_result ~= true
    fprintf('[ID:%03d] groupBulkRead addparam failed', DXL2_ID);
    return;
end


while 1
    if input('Press any key to continue! (or input e to quit!)\n', 's') == ESC_CHARACTER
        break;
    end

    % Write DYNAMIXEL#1 goal position
    if strcmp(MY_DXL,'X_SERIES')
        write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL1_ID, ADDR_MX_GOAL_POSITION, dxl_goal_position(index));
    else
        write2ByteTxRx(port_num, PROTOCOL_VERSION, DXL1_ID, ADDR_MX_GOAL_POSITION, dxl_goal_position(index));
    end
    dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION);
    dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION);
    if dxl_comm_result ~= COMM_SUCCESS
        fprintf('%s\n', getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
    elseif dxl_error ~= 0
        fprintf('%s\n', getRxPacketError(PROTOCOL_VERSION, dxl_error));
    end

    % Write DYNAMIXEL#2 goal position
    if strcmp(MY_DXL,'X_SERIES')
        write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL2_ID, ADDR_MX_GOAL_POSITION, dxl_goal_position(index));
    else
        write2ByteTxRx(port_num, PROTOCOL_VERSION, DXL2_ID, ADDR_MX_GOAL_POSITION, dxl_goal_position(index));
    end
    dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION);
    dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION);
    if dxl_comm_result ~= COMM_SUCCESS
        fprintf('%s\n', getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
    elseif dxl_error ~= 0
        fprintf('%s\n', getRxPacketError(PROTOCOL_VERSION, dxl_error));
    end

    while 1
        % Bulkread present position and moving status
        groupBulkReadTxRxPacket(group_num);
        dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION);
        if dxl_comm_result ~= COMM_SUCCESS
            fprintf('%s\n', getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
        end

        % Check if groupbulkread data of DYNAMIXEL#1 is available
        dxl_getdata_result = groupBulkReadIsAvailable(group_num, DXL1_ID, ADDR_MX_PRESENT_POSITION, LEN_MX_PRESENT_POSITION);
        if dxl_getdata_result ~= true
            fprintf('[ID:%03d] groupBulkRead getdata failed', DXL1_ID);
            return;
        end

        % Check if groupbulkread data of DYNAMIXEL#2 is available
        dxl_getdata_result = groupBulkReadIsAvailable(group_num, DXL2_ID, ADDR_MX_MOVING, LEN_MX_MOVING);
        if dxl_getdata_result ~= true
            fprintf('[ID:%03d] groupBulkRead getdata failed', DXL2_ID);
            return;
        end

        % Get DYNAMIXEL#1 present position value
        dxl1_present_position = groupBulkReadGetData(group_num, DXL1_ID, ADDR_MX_PRESENT_POSITION, LEN_MX_PRESENT_POSITION);

        % Get DYNAMIXEL#2 moving status value
        dxl2_moving = groupBulkReadGetData(group_num, DXL2_ID, ADDR_MX_MOVING, LEN_MX_MOVING);

        fprintf('[ID:%03d] Present Position : %d \t [ID:%03d] Is Moving : %d\n', DXL1_ID, dxl1_present_position, DXL2_ID, dxl2_moving);

        if ~(abs(dxl_goal_position(index) - dxl1_present_position) > DXL_MOVING_STATUS_THRESHOLD)
            break;
        end
    end

    % Change goal position
    if (index == 1)
      index = 2;
    else
      index = 1;
    end
end


% Disable DYNAMIXEL#1 Torque
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL1_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE);
dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION);
dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION);
if dxl_comm_result ~= COMM_SUCCESS
    fprintf('%s\n', getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
elseif dxl_error ~= 0
    fprintf('%s\n', getRxPacketError(PROTOCOL_VERSION, dxl_error));
end

% Disable DYNAMIXEL#2 Torque
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL2_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE);
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
