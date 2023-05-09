classdef servo < handle

    
    properties
        
        lib_name;
        port_num;
        
    end
    
    methods
        function obj = servo(inputArg1,inputArg2)

            addpath(genpath('C:\Users\samue\Documents\Git\DynamixelSDK\c\include\dynamixel_sdk\'))
            addpath(genpath('C:\Users\samue\Documents\Git\DynamixelSDK\c\build\win64'))
            addpath(genpath('C:\Users\samue\Documents\Git\DynamixelSDK\matlab\m_basic_function'))

            if strcmp(computer, 'PCWIN')
              obj.lib_name = 'dxl_x86_c';
            elseif strcmp(computer, 'PCWIN64')
              obj.lib_name = 'dxl_x64_c';
            elseif strcmp(computer, 'GLNX86')
              obj.lib_name = 'libdxl_x86_c';
            elseif strcmp(computer, 'GLNXA64')
              obj.lib_name = 'libdxl_x64_c';
            elseif strcmp(computer, 'MACI64')
              obj.lib_name = 'libdxl_mac_c';
            end

            % Load Libraries
            if ~libisloaded(obj.lib_name)
                [notfound, warnings] = loadlibrary(obj.lib_name, 'dynamixel_sdk.h', 'addheader', 'port_handler.h', 'addheader', 'packet_handler.h', 'addheader', 'group_bulk_read.h', 'addheader', 'group_bulk_write.h');
            end

            % Open port
            obj.port_num = calllib(obj.lib_name, 'portHandler', 'COM3'); %%%

            if (openPort(obj.port_num))
                fprintf('Succeeded to open the port!\n');
            else
                unloadlibrary(obj.lib_name);
                delete(obj)
                error('Failed to open the port!\n');
            end

            % Set port baudrate
            if (setBaudRate(obj.port_num, 1000000))
                fprintf('Succeeded to change the baudrate!\n');
            else
                delete(obj)
                error('Failed to change the baudrate!\n');

            end

            % Initialize PacketHandler Structs
            calllib(obj.lib_name, 'packetHandler');




        end

        function delete(obj)
                disp("destructor has been called")
                calllib(obj.lib_name, 'closePort', obj.port_num);
                unloadlibrary(obj.lib_name);

        end
        
        function enableTorque(obj,ID)
            
            PROTOCOL_VERSION = 2;
            ADDR_PRO_TORQUE_ENABLE       = 64;         % Control table address is different in Dynamixel model
            ADDR_PRO_PRESENT_POSITION    = 132;
            TORQUE_ENABLE               = 1;            % Value for enabling the torque
            COMM_SUCCESS                    = 0;            % Communication Success result value
            LEN_PRO_PRESENT_POSITION        = 4;



            % Initialize groupBulkWrite Struct
            groupwrite_num = calllib(obj.lib_name, 'groupBulkWrite', obj.port_num, PROTOCOL_VERSION);

            
            % Initialize Groupbulkread Structs
            groupread_num = calllib(obj.lib_name, 'groupBulkRead', obj.port_num, PROTOCOL_VERSION);

            %Enable torque
            calllib(obj.lib_name, 'write1ByteTxRx', obj.port_num, PROTOCOL_VERSION, ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE);
            dxl_comm_result = calllib(obj.lib_name, 'getLastTxRxResult', obj.port_num, PROTOCOL_VERSION);
            dxl_error = calllib(obj.lib_name, 'getLastRxPacketError', obj.port_num, PROTOCOL_VERSION);
            if dxl_comm_result ~= COMM_SUCCESS
                fprintf('%s\n', calllib(obj.lib_name, 'getTxRxResult', PROTOCOL_VERSION, dxl_comm_result));
            elseif dxl_error ~= 0
                fprintf('%s\n', calllib(obj.lib_name, 'getRxPacketError', protocol_version, dxl_error));
            else
                fprintf('Torque of ID %d has been successfully enabled \n', ID);
            end

            % Add parameter storage for Dynamixel#1 present position value
            dxl_addparam_result = calllib(obj.lib_name, 'groupBulkReadAddParam', groupread_num, ID, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION);
            if dxl_addparam_result ~= true
                fprintf('[ID:%d] groupBulkRead addparam failed', ID);
                return;
            else
                fprintf('[ID:%d] groupBulkRead addparam succeeded', ID);
            end

        end
    end
end

