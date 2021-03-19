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

% Author: Ki Jong Gil (Gilbert)

%
% *********     Clear Multi-Turn Example      *********
%
%
% Available DYNAMIXEL model on this example : DYNAMIXEL X-series (firmware v42 or above)
% This example is designed for using a DYNAMIXEL XM430-W350-R, and an U2D2.
% To use another DYNAMIXEL model, such as MX series, see their details in E-Manual(emanual.robotis.com) and edit below "#define"d variables yourself.
% Be sure that DYNAMIXEL properties are already set as %% ID : 1 / Baudnum : 1 (Baudrate : 57600)
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
    [notfound, warnings] = loadlibrary(lib_name, 'dynamixel_sdk.h', 'addheader', 'port_handler.h', 'addheader', 'packet_handler.h');
end

%{
********* DYNAMIXEL Model *********
***** (Use only one definition at a time) ***** 
%}

  MY_DXL = 'X_SERIES'; % X330, X430, X540, 2X430  
% MY_DXL = 'PRO_SERIES'; % H54, H42, M54, M42, L54, L42
% MY_DXL = 'PRO_A_SERIES'; % PRO series with (A) firmware update.
% MY_DXL = 'P_SERIES'; % PH54, PH42, PM54
% MY_DXL = 'XL320';  % [WARNING] Operating Voltage : 7.4V, multi-turn X
% MY_DXL = 'MX_SERIES'; % MX series with 2.0 firmware update.

% Control table address and data to Read/Write for my DYNAMIXEL, MY_DXL, in use. 
switch (MY_DXL)
    
    case {'X_SERIES','MX_SERIES'}
        ADDR_TORQUE_ENABLE          = 64;
        ADDR_GOAL_POSITION          = 116;
        ADDR_PRESENT_POSITION       = 132;
        BAUDRATE                    = 57600;
        MAX_POSITION_VALUE          = 10000;

    case ('PRO_SERIES')
        ADDR_TORQUE_ENABLE          = 562;  % Control table address is different in DYNAMIXEL model
        ADDR_GOAL_POSITION          = 596;
        ADDR_PRESENT_POSITION       = 611;
        BAUDRATE                    = 57600;
        MAX_POSITION_VALUE          = 1000000;
    
    case {'P_SERIES','PRO_A_SERIES'}
        ADDR_TORQUE_ENABLE          = 512;  % Control table address is different in DYNAMIXEL model
        ADDR_GOAL_POSITION          = 564;
        ADDR_PRESENT_POSITION       = 580;
        BAUDRATE                    = 57600;
        MAX_POSITION_VALUE          = 1000000;
end

% Control table address
ADDR_OPERATING_MODE          = 11; % Control table address is different in DYNAMIXEL model

% Protocol version
PROTOCOL_VERSION             = 2.0; % See which protocol version is used in the DYNAMIXEL

% Default setting
DXL_ID                       = 1; % DYNAMIXEL ID: 1
DEVICENAME                   = '/dev/ttyUSB0'; % Check which port is being used on your controller
                                               % ex) Windows: 'COM1'   Linux: '/dev/ttyUSB0' Mac: '/dev/tty.usbserial-*'

TORQUE_ENABLE                = 1; % Value for enabling the torque
TORQUE_DISABLE               = 0; % Value for disabling the torque     
DXL_MOVING_STATUS_THRESHOLD  = 20; % DYNAMIXEL moving status threshold
EXT_POSITION_CONTROL_MODE    = 4; % Value for extended position control mode (operating mode)

ESC_CHARACTER                = 'e'; % Key for escaping loop
SPACE_ASCII_VALUE            = ' ';

COMM_SUCCESS                 = 0; % Communication Success result value
COMM_TX_FAIL                 = -1001; % Communication Tx Failed

% Initialize PortHandler Structs
% Set the port path
% Get methods and members of PortHandlerLinux or PortHandlerWindows
port_num = portHandler(DEVICENAME);

% Initialize PacketHandler Structs
packetHandler();

dxl_comm_result = COMM_TX_FAIL; % Communication result
       
dxl_error = 0; % DYNAMIXEL error
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

% Set operating mode to extended position control mode
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_OPERATING_MODE, EXT_POSITION_CONTROL_MODE);
dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION);
dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION);
if dxl_comm_result ~= COMM_SUCCESS
    fprintf('%s\n', getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
elseif dxl_error ~= 0
    fprintf('%s\n', getRxPacketError(PROTOCOL_VERSION, dxl_error));
else
    fprintf('Operating mode changed to extended position control mode. \n');
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
    fprintf('DYNAMIXEL has been successfully connected \n');
end

%Turns multiple time until it reaches Goal Position
while 1
    if input('\nPress any key to continue! (or input e to quit!)', 's') == ESC_CHARACTER
        break;
    end

    fprintf('  Press SPACE key to clear multi-turn information! (or press e to stop!)');
    
    % Write Goal Position
    write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_GOAL_POSITION, MAX_POSITION_VALUE);
    dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION);
    dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION);
    if dxl_comm_result ~= COMM_SUCCESS
        fprintf('%s\n', getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
    elseif dxl_error ~= 0
        fprintf('%s\n', getRxPacketError(PROTOCOL_VERSION, dxl_error));
    end

    while 1
        % Read Present Position
        dxl_present_position = read4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_PRESENT_POSITION);
        dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION);
        dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION);
        if dxl_comm_result ~= COMM_SUCCESS
            fprintf('%s\n', getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
        elseif dxl_error ~= 0
            fprintf('%s\n', getRxPacketError(PROTOCOL_VERSION, dxl_error));
        end

        fprintf('\r  [ID:%03d] GoalPos:%03d  PresPos:%03d', DXL_ID, MAX_POSITION_VALUE, dxl_present_position);
        ch = kbhit(0.2);
        if ch == SPACE_ASCII_VALUE
            fprintf('\n  Stop & Clear Multi-Turn Information! \n');
            
            % Write the present position to the goal position to stop moving
            write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_GOAL_POSITION,  typecast(uint32(dxl_present_position), 'int32'));
            dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION);
            dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION);
            if dxl_comm_result ~= COMM_SUCCESS
                fprintf('%s\n', getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
            elseif dxl_error ~= 0
                fprintf('%s\n', getRxPacketError(PROTOCOL_VERSION, dxl_error));
            end
            
            pause(0.5);
            
            % Clear Multi-Turn Information
            clearMultiTurn(port_num, PROTOCOL_VERSION, DXL_ID);
            dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION);
            dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION);
            if dxl_comm_result ~= COMM_SUCCESS
                fprintf('%s\n', getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
            elseif dxl_error ~= 0
                fprintf('%s\n', getRxPacketError(PROTOCOL_VERSION, dxl_error));
            end
            
            % Read Present Position
            dxl_present_position = read4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_PRESENT_POSITION);
            dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION);
            dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION);
            if dxl_comm_result ~= COMM_SUCCESS
                fprintf('%s\n', getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
            elseif dxl_error ~= 0
                fprintf('%s\n', getRxPacketError(PROTOCOL_VERSION, dxl_error));
            end
            
            fprintf('  Present Position has been reset. : %03d \n', dxl_present_position);
            break;
            
        elseif ch == ESC_CHARACTER
            fprintf('\n  Stopped!! \n');
            
            % Write the present position to the goal position to stop moving
            write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_GOAL_POSITION,  typecast(uint32(dxl_present_position), 'int32'));
            dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION);
            dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION);
            if dxl_comm_result ~= COMM_SUCCESS
                fprintf('%s\n', getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
            elseif dxl_error ~= 0
                fprintf('%s\n', getRxPacketError(PROTOCOL_VERSION, dxl_error));
            end
            break;
        end
                
        if ~(abs(MAX_POSITION_VALUE - typecast(uint32(dxl_present_position), 'int32')) > DXL_MOVING_STATUS_THRESHOLD)
            break;
        end
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

function ch = kbhit(m) 
% GETKEYWAIT - get a key within a time limit
%   CH = GETKEYWAIT(P) waits for a keypress for a maximum of P seconds. P
%   should be a positive number. CH is a double representing the key
%   pressed key as an ascii number, including backspace (8), space (32),
%   enter (13), etc. If a non-ascii key (Ctrl, Alt, etc.) is pressed, CH
%   will be NaN.
%   If no key is pressed within P seconds, -1 is returned, and if something
%   went wrong during excution 0 is returned. 
%
%  See also INPUT,
%           GETKEY (FileExchange)
% tested for Matlab 6.5 and higher
% version 2.1 (jan 2012)
% author : Jos van der Geest
% email  : jos@jasen.nl
% History
% 1.0 (2005) creation
% 2.0 (apr 2009) - expanded error check on input argument, changed return
% values when a non-ascii was pressed (now NaN), or when something went
% wrong (now 0); added comments ; slight change in coding
% 2.1 (jan 2012) - modified a few properties, included check is figure
%                  still exists (after comment on GETKEY on FEX by Andrew). 
% check input argument
    narginchk(1,1) ;
    if numel(m)~=1 || ~isnumeric(m) || ~isfinite(m) || m <= 0    
        error('Argument should be a single positive number.') ;
    end
    % set up the timer
    tt = timer ;
    tt.timerfcn = 'uiresume' ;
    tt.startdelay = m ;            
    % Set up the figure
    % May be the position property should be individually tweaked to avoid visibility
    callstr = 'set(gcbf,''Userdata'',double(get(gcbf,''Currentcharacter''))) ; uiresume ' ;
    fh = figure(...
        'name','Press a key', ...
        'keypressfcn',callstr, ...
        'windowstyle','modal',... 
        'numbertitle','off', ...
        'position',[0 0  1 1],...
        'userdata',-1) ; 
    try
        % Wait for something to happen or the timer to run out
        start(tt) ;    
        uiwait ;
        ch = get(fh,'Userdata') ;
        if isempty(ch) % a non-ascii key was pressed, return a NaN
            ch = NaN ;
        end
    catch
        % Something went wrong, return zero.
        ch = 0 ;
    end
    % clean up the timer ...
    stop(tt) ;
    delete(tt) ; 
    % ... and figure
    if ishandle(fh)
        delete(fh) ;
    end
end