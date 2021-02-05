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
% *********     Factory Reset Example      *********
%
%
% Available DYNAMIXEL model on this example : All models using Protocol 1.0
% This example is designed for using a DYNAMIXEL MX-28, and an USB2DYNAMIXEL.
% To use another DYNAMIXEL model, such as X series, see their details in E-Manual(emanual.robotis.com) and edit below variables yourself.
% Be sure that DYNAMIXEL PRO properties are already set as %% ID : 1 / Baudnum : 34 (Baudrate : 57600)
%

% Be aware that:
% This example resets all properties of DYNAMIXEL to default values, such as %% ID : 1 / Baudnum : 34 (Baudrate : 57600)
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
***** (Use only ONE definition at a time) ***** 
%}

% MY_DXL = 'X_SERIES'; % X430, X540, 2X430  
 MY_DXL = 'AX_SERIES'; % AX Series
% MY_DXL = 'MX_SERIES'; % MX series with 2.0 firmware update.

% Control table address and data to Read/Write for my DYNAMIXEL, MY_DXL, in use. 
switch (MY_DXL)

    case {'X_SERIES'}
        ADDR_BAUDRATE                = 8;
        BAUDRATE                    = 57600;
        FACTORYRST_DEFAULTBAUDRATE  = 57600; % DYNAMIXEL baudrate set by factoryreset
        NEW_BAUDNUM                 = 1; % New baudnum to recover DYNAMIXEL baudrate as it was

       
    case {'AX_SERIES'}
        ADDR_BAUDRATE               = 4;
        BAUDRATE                    = 1000000;
        FACTORYRST_DEFAULTBAUDRATE  = 1000000;
        NEW_BAUDNUM                 = 1;

        
    case {'MX_SERIES'}
        ADDR_BAUDRATE               = 4;
        BAUDRATE                    = 57600;
        FACTORYRST_DEFAULTBAUDRATE  = 57600;
        NEW_BAUDNUM                 = 34;       

       
end

% Protocol version
PROTOCOL_VERSION                = 1.0; % See which protocol version is used in the DYNAMIXEL

% Default setting
DXL_ID                          = 1; % DYNAMIXEL ID: 1
DEVICENAME                      = '/dev/ttyUSB0'; % Check which port is being used on your controller
                                                  % ex) Windows: 'COM1'   Linux: '/dev/ttyUSB0' Mac: '/dev/tty.usbserial-*'

OPERATION_MODE                  = 0; % Mode is unavailable in Protocol 1.0 Reset
COMM_SUCCESS                    = 0; % Communication Success result value
COMM_TX_FAIL                    = -1001; % Communication Tx Failed

% Initialize PortHandler Structs
% Set the port path
% Get methods and members of PortHandlerLinux or PortHandlerWindows
port_num = portHandler(DEVICENAME);

% Initialize PacketHandler Structs
packetHandler();

dxl_comm_result = COMM_TX_FAIL; % Communication result

dxl_error = 0; % DYNAMIXEL error
dxl_baudnum_read = 0; % Read baudnum

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


% Read present baudrate of the controller
fprintf('Now the controller baudrate is : %d\n', getBaudRate(port_num));

% Try factoryreset
fprintf('[ID:%03d] Try factoryreset : ', DXL_ID);
factoryReset(port_num, PROTOCOL_VERSION, DXL_ID, OPERATION_MODE);
dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION);
dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION);
if dxl_comm_result ~= COMM_SUCCESS
    fprintf('Aborted\n');
    fprintf('%s\n', getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
elseif dxl_error ~= 0
    fprintf('%s\n', getRxPacketError(PROTOCOL_VERSION, dxl_error));
end

% Wait for reset
fprintf('Wait for reset...\n');
pause(2);

fprintf('[ID:%03d] factoryReset Success!\n', DXL_ID);

% Set controller baudrate to dxl default baudrate
if (setBaudRate(port_num, FACTORYRST_DEFAULTBAUDRATE))
  fprintf('Succeed to change the controller baudrate to : %d\n', FACTORYRST_DEFAULTBAUDRATE);
else
  fprintf('Failed to change the controller baudrate\n');
  input('Press any key to terminate...\n');
  return;
end

% Read DYNAMIXEL baudnum
dxl_baudnum_read = read1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_BAUDRATE);
dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION);
dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION);
if dxl_comm_result ~= COMM_SUCCESS
    fprintf('%s\n', getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
elseif dxl_error ~= 0
    fprintf('%s\n', getRxPacketError(PROTOCOL_VERSION, dxl_error));
else
  fprintf('[ID:%03d] DYNAMIXEL baudnum is now : %d\n', DXL_ID, dxl_baudnum_read);
end

% Write new baudnum
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_BAUDRATE, NEW_BAUDNUM);
dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION);
dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION);
if dxl_comm_result ~= COMM_SUCCESS
    fprintf('%s\n', getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
elseif dxl_error ~= 0
    fprintf('%s\n', getRxPacketError(PROTOCOL_VERSION, dxl_error));
else
  fprintf('[ID:%03d] Set DYNAMIXEL baudnum to : %d\n', DXL_ID, NEW_BAUDNUM);
end

% Set port baudrate to BAUDRATE
if (setBaudRate(port_num, BAUDRATE))
  fprintf('Succeed to change the controller baudrate to : %d\n', BAUDRATE);
else
  fprintf('Failed to change the controller baudrate\n');
  input('Press any key to terminate...\n');
  return;
end

pause(0.2);

% Read DYNAMIXEL baudnum
dxl_baudnum_read = read1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_BAUDRATE);
dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION);
dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION);
if dxl_comm_result ~= COMM_SUCCESS
    fprintf('%s\n', getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
elseif dxl_error ~= 0
    fprintf('%s\n', getRxPacketError(PROTOCOL_VERSION, dxl_error));
else
  fprintf('[ID:%03d] DYNAMIXEL baudnum is now : %d\n', DXL_ID, dxl_baudnum_read);
end

% Close port
closePort(port_num);

% Unload Library
unloadlibrary(lib_name);

close all;
clear all;
