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
                disp("closing port, unloading library")
                calllib(obj.lib_name, 'closePort', obj.port_num);
                unloadlibrary(obj.lib_name);

        end
        
        function enableOrDisableTorque(obj,ID,enable_bool)
            % Enable the Torque on servo with ID.

            %Definitions
            PROTOCOL_VERSION = 2;
            ADDR_PRO_TORQUE_ENABLE       = 64;         % Control table address is different in Dynamixel model
            ADDR_PRO_PRESENT_POSITION    = 132;
            TORQUE_ENABLE               = 1;            % Value for enabling the torque
            COMM_SUCCESS                    = 0;            % Communication Success result value
            LEN_PRO_PRESENT_POSITION        = 4;
            TORQUE_DISABLE              = 0;            % Value for disabling the torque

            if(enable_bool)
                %Enable torque
                calllib(obj.lib_name, 'write1ByteTxRx', obj.port_num, PROTOCOL_VERSION, ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE);
                dxl_comm_result = calllib(obj.lib_name, 'getLastTxRxResult', obj.port_num, PROTOCOL_VERSION);
                dxl_error = calllib(obj.lib_name, 'getLastRxPacketError', obj.port_num, PROTOCOL_VERSION);
                if dxl_comm_result ~= COMM_SUCCESS
                    fprintf('%s\n', calllib(obj.lib_name, 'getTxRxResult', PROTOCOL_VERSION, dxl_comm_result));
                elseif dxl_error ~= 0
                    fprintf('%s\n', calllib(obj.lib_name, 'getRxPacketError', PROTOCOL_VERSION, dxl_error));
                else
                    fprintf('Torque of ID %d has been successfully enabled \n', ID);
                end
            else
                %Disable torque
                calllib(obj.lib_name, 'write1ByteTxRx', obj.port_num, PROTOCOL_VERSION, ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE);
                dxl_comm_result = calllib(obj.lib_name, 'getLastTxRxResult', obj.port_num, PROTOCOL_VERSION);
                dxl_error = calllib(obj.lib_name, 'getLastRxPacketError', obj.port_num, PROTOCOL_VERSION);
                if dxl_comm_result ~= COMM_SUCCESS
                    fprintf('%s\n', calllib(obj.lib_name, 'getTxRxResult', PROTOCOL_VERSION, dxl_comm_result));
                elseif dxl_error ~= 0
                    fprintf('%s\n', calllib(obj.lib_name, 'getRxPacketError', PROTOCOL_VERSION, dxl_error));
                else
                    fprintf('Torque of ID %d has been successfully disabled \n', ID);
                end
            end

        end

        function success = setAngle(obj, ID, angle)
                %Set the angle of servo with ID to an angle between 0 and 2
                %pi in RADIANT. Torque has to be enabled.

                % Definitions
                PROTOCOL_VERSION = 2;
                ADDR_PRO_GOAL_POSITION       = 116;
                LEN_PRO_GOAL_POSITION           = 4;
                DXL_MINIMUM_POSITION_VALUE  = 0;
                DXL_MAXIMUM_POSITION_VALUE  = 4095;
                COMM_SUCCESS = 0;

                % Initialize groupBulkWrite Struct
                groupwrite_num = calllib(obj.lib_name, 'groupBulkWrite', obj.port_num, PROTOCOL_VERSION);           

                % Calculate the goal positoin from the angle and the
                % min/max values
                angle = rem(angle,2*pi);
                dxl_goal_position = (angle/(2*pi)) * DXL_MAXIMUM_POSITION_VALUE + DXL_MINIMUM_POSITION_VALUE;
            
                % Add parameter storage for Dynamixel#1 goal position
                % dxl_addparam_result = groupBulkWriteAddParam(groupwrite_num, DXL1_ID, ADDR_PRO_GOAL_POSITION, LEN_PRO_GOAL_POSITION, typecast(int32(dxl_goal_position(index2)), 'uint32'), LEN_PRO_GOAL_POSITION);
                dxl_addparam_result = calllib(obj.lib_name, 'groupBulkWriteAddParam', groupwrite_num, ID, ADDR_PRO_GOAL_POSITION, LEN_PRO_GOAL_POSITION,  typecast(int32(dxl_goal_position), 'uint32'), LEN_PRO_GOAL_POSITION);
                if dxl_addparam_result ~= true
                  fprintf(stderr, '[ID:%03d] groupBulkWrite addparam failed \n', ID);
                  success = 0;
                  return;
                end

                % Bulkwrite goal position
                % groupBulkWriteTxPacket(groupwrite_num);
                calllib(obj.lib_name, 'groupBulkWriteTxPacket', groupwrite_num);
                % dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION);
                dxl_comm_result = calllib(obj.lib_name, 'getLastTxRxResult', obj.port_num, PROTOCOL_VERSION);
                if dxl_comm_result ~= COMM_SUCCESS
                    % STRING = getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
                    fprintf('%s\n', calllib(obj.lib_name, 'getTxRxResult', PROTOCOL_VERSION, dxl_comm_result));
                else
                    fprintf('[ID:%03d] Successfully wrote goal position \n', ID);
                end

                % Clear bulkwrite parameter storage
                groupBulkWriteClearParam(groupwrite_num);

        end

        
    end
end

