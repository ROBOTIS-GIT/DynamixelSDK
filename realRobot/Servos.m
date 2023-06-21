classdef Servos < handle
% The 'Servos' class provides an interface for interacting with connected servos. It is
% designed to establish a connection to a specified port and list all the servos available at that port.
% The class allows the user to set and retrieve the angles of each connected servo. Additionally, it
% provides functionality to enable and disable the torque for each servo.
% The Servos class relies on the DynamixelSDK library, which is a C-based library. Throughout the class, 
% numerous 'calllibrary()' functions are utilized, which necessitate the correct path to the DynamixelSDK 
% library. To ensure the correct operation of the Servos class, it is essential that the absolute path to
% the DynamixelSDK library is accurately configured within the class constructor.

    
    properties
        lib_name;
        port_num;
        availableIDs = [];
    end
    
    methods
        function obj = Servos(PORT)
            % Hint: Get the correct Port string in your device manager e.g.
            % 'COM3', 'COM4'

            % Check if the function was called with an argument
          if nargin == 0
              % If not, use 'COM3' as a default port
              PORT = 'COM3';
          end

            %% Modify the following string to your absolute path to the DynamixelSDK
            absolute_path = 'C:\Users\samue\Documents\Git\DynamixelSDK\c\';
            %%

            %Definitions
            PROTOCOL_VERSION = 2;
            COMM_SUCCESS = 0;
            MAX_ID = 10;
            
            addpath(genpath([absolute_path, 'include\dynamixel_sdk\']))
            
            %Checks your OS and adds the correct built library
            %Pre-built for windows, you need to build yourself on Linux and
            %Mac. There is a Makefile in the linked folder.
            if strcmp(computer, 'PCWIN')
              obj.lib_name = 'dxl_x86_c';
              addpath(genpath([absolute_path, 'build\win32']))
            elseif strcmp(computer, 'PCWIN64')
              addpath(genpath([absolute_path, 'build\win64']))
              obj.lib_name = 'dxl_x64_c';
            elseif strcmp(computer, 'GLNX86')
              addpath(genpath([absolute_path, 'build\linux32']))
              obj.lib_name = 'libdxl_x86_c';
            elseif strcmp(computer, 'GLNXA64')
              addpath(genpath([absolute_path, 'build\linux64']))
              obj.lib_name = 'libdxl_x64_c';
            elseif strcmp(computer, 'MACI64')
              addpath(genpath([absolute_path, 'build\mac']))
              obj.lib_name = 'libdxl_mac_c';
            end


            % Load Libraries
            if ~libisloaded(obj.lib_name)
                [notfound, warnings] = loadlibrary(obj.lib_name, 'dynamixel_sdk.h', 'addheader', 'port_handler.h', 'addheader', 'packet_handler.h', 'addheader', 'group_bulk_read.h', 'addheader', 'group_bulk_write.h');
            end

            % Open port
            obj.port_num = calllib(obj.lib_name, 'portHandler', PORT);

            if (calllib(obj.lib_name, 'openPort', obj.port_num))
                fprintf('Succeeded to open the port!\n');
            else
                unloadlibrary(obj.lib_name);
                delete(obj)
                error('Failed to open the port!\n');
            end

            % Set port baudrate
            if (calllib(obj.lib_name, 'setBaudRate', obj.port_num, 1000000))
                fprintf('Succeeded to change the baudrate!\n');
            else
                delete(obj)
                error('Failed to change the baudrate!\n');

            end

            % Initialize PacketHandler Structs
            calllib(obj.lib_name, 'packetHandler');

            %Scan available IDs
            % Try to broadcast ping the Dynamixel
            calllib(obj.lib_name, 'broadcastPing', obj.port_num, PROTOCOL_VERSION);
            dxl_comm_result = calllib(obj.lib_name, 'getLastTxRxResult', obj.port_num, PROTOCOL_VERSION);
            if dxl_comm_result ~= COMM_SUCCESS
                fprintf('%s\n', calllib(obj.lib_name, 'getTxRxResult', PROTOCOL_VERSION, dxl_comm_result));
            end
            fprintf('Detected Dynamixel : \n');
            for id = 0 : MAX_ID
              if calllib(obj.lib_name, 'getBroadcastPingResult', obj.port_num, PROTOCOL_VERSION, id)
                fprintf('Available ID: %d \n', id);
                obj.availableIDs = [obj.availableIDs, id];
              end
            end
        end

        function delete(obj)
                disp("closing port, unloading library")
                calllib(obj.lib_name, 'closePort', obj.port_num);
                unloadlibrary(obj.lib_name);
        end

        function ID = checkIdAvailable(obj, id)
            if ismember(id, obj.availableIDs)
                ID = id;
            else
                error('ID %d is not available', id);
            end
        end
        
        function success = torqueEnableDisable(obj,ID,enable_bool)
            % Enable the Torque on servo with ID.

            ID = checkIdAvailable(obj, ID);

            %Definitions
            PROTOCOL_VERSION = 2;
            ADDR_PRO_TORQUE_ENABLE       = 64;         % Control table address is different in Dynamixel model
            TORQUE_ENABLE               = 1;            % Value for enabling the torque
            COMM_SUCCESS                    = 0;            % Communication Success result value
            TORQUE_DISABLE              = 0;            % Value for disabling the torque

            if(enable_bool)
                %Enable torque
                calllib(obj.lib_name, 'write1ByteTxRx', obj.port_num, PROTOCOL_VERSION, ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE);
                dxl_comm_result = calllib(obj.lib_name, 'getLastTxRxResult', obj.port_num, PROTOCOL_VERSION);
                dxl_error = calllib(obj.lib_name, 'getLastRxPacketError', obj.port_num, PROTOCOL_VERSION);
                if dxl_comm_result ~= COMM_SUCCESS
                    success = 0;
                    fprintf('%s\n', calllib(obj.lib_name, 'getTxRxResult', PROTOCOL_VERSION, dxl_comm_result));
                elseif dxl_error ~= 0
                    success = 0;
                    fprintf('%s\n', calllib(obj.lib_name, 'getRxPacketError', PROTOCOL_VERSION, dxl_error));
                else
                    success = 1;
                    fprintf('Torque of ID %d has been successfully enabled \n', ID);
                end
            else
                %Disable torque
                calllib(obj.lib_name, 'write1ByteTxRx', obj.port_num, PROTOCOL_VERSION, ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE);
                dxl_comm_result = calllib(obj.lib_name, 'getLastTxRxResult', obj.port_num, PROTOCOL_VERSION);
                dxl_error = calllib(obj.lib_name, 'getLastRxPacketError', obj.port_num, PROTOCOL_VERSION);
                if dxl_comm_result ~= COMM_SUCCESS
                    success = 0;
                    fprintf('%s\n', calllib(obj.lib_name, 'getTxRxResult', PROTOCOL_VERSION, dxl_comm_result));
                elseif dxl_error ~= 0
                    success = 0;
                    fprintf('%s\n', calllib(obj.lib_name, 'getRxPacketError', PROTOCOL_VERSION, dxl_error));
                else
                    success = 1;
                    fprintf('Torque of ID %d has been successfully disabled \n', ID);
                end
            end

        end

        function success = setAngle(obj, ID, angle)
                %Set the angle of servo with ID to an angle between 0 and 2
                %pi in RADIANT. Torque has to be enabled.

                ID = checkIdAvailable(obj, ID);

                % Definitions
                PROTOCOL_VERSION = 2;
                ADDR_PRO_GOAL_POSITION       = 116;
                LEN_PRO_GOAL_POSITION           = 4;
                DXL_MINIMUM_POSITION_VALUE  = 0;
                DXL_MAXIMUM_POSITION_VALUE  = 4095;
                COMM_SUCCESS = 0;

                % Check if the operating mode of the servo with the given
                % ID is set to postion control
                ADDR_PRO_OP_MODE = 11;
                VALUE = calllib(obj.lib_name, 'read1ByteTxRx', obj.port_num , PROTOCOL_VERSION, ID, ADDR_PRO_OP_MODE);
                if VALUE ~= 3
                    fprintf("Control mode of the Servo with ID %d is not set to position. Use setOperatingMode method first. \n", ID);
                    success = 0;
                    return
                end

                % Check if torque of the servo is enabled
                ADDR_PRO_TORQUE_ENABLE       = 64;         % Control table address is different in Dynamixel model
                VALUE = calllib(obj.lib_name, 'read1ByteTxRx', obj.port_num , PROTOCOL_VERSION, ID, ADDR_PRO_TORQUE_ENABLE);
                if VALUE ~= 1
                    fprintf("Torque of the Servo with with ID %d is not enabled. Use torqueEnableDisable method first. \n", ID);
                    success = 0;
                    return
                end


                % Initialize groupBulkWrite Struct
                groupwrite_num = calllib(obj.lib_name, 'groupBulkWrite', obj.port_num, PROTOCOL_VERSION);           

                while angle < 0
                    angle = angle + 2*pi;
                end

                angle = rem(angle, 2*pi);
                dxl_goal_position = (angle/(2*pi)) * DXL_MAXIMUM_POSITION_VALUE + DXL_MINIMUM_POSITION_VALUE;

                % Add parameter storage for Dynamixel#1 goal position
                dxl_addparam_result = calllib(obj.lib_name, 'groupBulkWriteAddParam', groupwrite_num, ID, ADDR_PRO_GOAL_POSITION, LEN_PRO_GOAL_POSITION,  typecast(int32(dxl_goal_position), 'uint32'), LEN_PRO_GOAL_POSITION);
                if dxl_addparam_result ~= true
                  fprintf(stderr, '[ID:%03d] groupBulkWrite addparam failed \n', ID);
                  success = 0;
                  return;
                end

                % Bulkwrite goal position
                calllib(obj.lib_name, 'groupBulkWriteTxPacket', groupwrite_num);
                dxl_comm_result = calllib(obj.lib_name, 'getLastTxRxResult', obj.port_num, PROTOCOL_VERSION);
                if dxl_comm_result ~= COMM_SUCCESS
                    fprintf('%s\n', calllib(obj.lib_name, 'getTxRxResult', PROTOCOL_VERSION, dxl_comm_result));
                else
                    fprintf('[ID:%03d] Successfully wrote goal position \n', ID);
                    success = 1;
                end

                % Clear bulkwrite parameter storage
                calllib(obj.lib_name, 'groupBulkWriteClearParam', groupwrite_num);
        end

        function [angle, success] = getAngle(obj,ID)
            

            
            %Receive the current Position of a servo in RAD

            ID = checkIdAvailable(obj, ID);

            %Definitions
            PROTOCOL_VERSION = 2;
            COMM_SUCCESS = 0;
            ADDR_PRO_PRESENT_POSITION    = 132;
            LEN_PRO_PRESENT_POSITION        = 4;

            % Initialize Groupbulkread Structs
            groupread_num = calllib(obj.lib_name, 'groupBulkRead', obj.port_num, PROTOCOL_VERSION);

            % Add parameter storage for Dynamixel present position value
            dxl_addparam_result = calllib(obj.lib_name, 'groupBulkReadAddParam', groupread_num, ID, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION);
            if dxl_addparam_result ~= true
                fprintf('[ID:%03d] groupBulkRead addparam failed \n', ID);
                success = 0;
                return;
            else
                % fprintf('[ID:%03d] groupBulkRead addparam succeeded \n', ID);
            end
                        
            % Bulkread present position
            % groupBulkReadTxRxPacket(groupread_num);
            calllib(obj.lib_name, 'groupBulkReadTxRxPacket', groupread_num);
            % dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION);
            dxl_comm_result = calllib(obj.lib_name, 'getLastTxRxResult', obj.port_num, PROTOCOL_VERSION);
            if dxl_comm_result ~= COMM_SUCCESS
                % fprintf('%s\n', getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
                fprintf('%s\n', calllib(obj.lib_name, 'getTxRxResult', PROTOCOL_VERSION, dxl_comm_result));
                success = 0;
                return
            end

            % Check if groupbulkread data of Dynamixel is available
            dxl_getdata_result = calllib(obj.lib_name, 'groupBulkReadIsAvailable', groupread_num, ID, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION);
            if dxl_getdata_result ~= true
                fprintf('[ID:%03d] groupBulkRead getdata failed \n', ID);
                success = 0;
                return;
            else
                % fprintf('[ID:%03d] groupBulkRead getdata succeeded \n', ID);
            end

              % Get Dynamixel#1 present position value
              dxl1_present_position = calllib(obj.lib_name, 'groupBulkReadGetData', groupread_num, ID, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION);
             

              %% This is a dirty fix, maybe rework
              if dxl1_present_position > 4294000000
                  dxl1_present_position = -(4294967295 - dxl1_present_position);
              end
                
                
              angle = dxl1_present_position * 0.087891 * pi/180;



              % fprintf('[ID:%03d] Current Angle : %d \n', ID, angle);
              success = 1;
        end
    
        function success = setOperatingMode(obj, ID, modeString)

            switch modeString

                case 'position'
                    mode = 3;
                case 'velocity'
                    mode = 1;
                otherwise
                    frpintf('Invalid Operating Mode. Use ''position'', or ''velocity''.');
                    success = 0;
                    return
            end

            
            PROTOCOL_VERSION = 2;
            ID = checkIdAvailable(obj, ID);
            ADDR_PRO_OP_MODE = 11;
            OP_MODE = mode;

            calllib(obj.lib_name, 'write1ByteTxRx', obj.port_num , PROTOCOL_VERSION, ID, ADDR_PRO_OP_MODE, OP_MODE);

            VALUE = calllib(obj.lib_name, 'read1ByteTxRx', obj.port_num , PROTOCOL_VERSION, ID, ADDR_PRO_OP_MODE);
            
            if VALUE == mode
                fprintf("Successfully set Operation mode of Servo with ID %d to %s \n", ID, modeString)
                success = 1;
            else
                fprintf("Could not set Operation mode of Servo with ID %d \n", ID);
                success = 0;
            end


        end

        function success = setVelocity(obj, ID, velocity)
            % velocity is given in rev/min
            


            ID = checkIdAvailable(obj, ID);
            PROTOCOL_VERSION = 2;
 
            switch ID
                case 1 
                    gear_ratio = 2.4570; % has to be approximated
                case 2
                    gear_ratio = 1;
                case 3
                    gear_ratio = 57/23;
                case 4
                    gear_ratio = 57/23;
                otherwise
                    fprintf("Faulty ID \n");
            end


            % Check if the operating mode of the servo with the given
            % ID is set to velocity control
            ADDR_PRO_OP_MODE = 11;
            VALUE = calllib(obj.lib_name, 'read1ByteTxRx', obj.port_num , PROTOCOL_VERSION, ID, ADDR_PRO_OP_MODE);
            if VALUE ~= 1
                success = 0;
                fprintf("Control mode of the Servo with ID %d is not set to velocity. Use setOperatingMode method first. \n", ID);
                return
            end

            % Check if torque of the servo is enabled
            ADDR_PRO_TORQUE_ENABLE       = 64;         % Control table address is different in Dynamixel model
            VALUE = calllib(obj.lib_name, 'read1ByteTxRx', obj.port_num , PROTOCOL_VERSION, ID, ADDR_PRO_TORQUE_ENABLE);
            if VALUE ~= 1
                success = 0;
                fprintf("Torque of the Servo with ID %d is not enabled. Use torqueEnableDisable method first. \n", ID);
                return
            end

            % Set the velocity
            ADDR_PRO_GOAL_VELOCITY      = 104;         % Control table address is different in Dynamixel model
            VELOCITY_VAL = 1/0.229 * velocity * gear_ratio; % Convert velocity to decimal
            %Floor VELOCITY_VAL
            VELOCITY_VAL = floor(VELOCITY_VAL);
            if VELOCITY_VAL <0
                VELOCITY_VAL = 4294967296 + VELOCITY_VAL;
            end
            



            calllib(obj.lib_name, 'write4ByteTxRx', obj.port_num , PROTOCOL_VERSION, ID, ADDR_PRO_GOAL_VELOCITY, VELOCITY_VAL);
            
            %Read the goal velocity
            VALUE = calllib(obj.lib_name, 'read4ByteTxRx', obj.port_num , PROTOCOL_VERSION, ID, ADDR_PRO_GOAL_VELOCITY);
            
            if VALUE == VELOCITY_VAL
                % fprintf("Successfully set velocity of Servo with ID %d \n", ID);
                success = 1;
            else
                fprintf("Could not set velocity of Servo with ID %d \n", ID);
                success = 0;
            end
            
        end

        function [velocity,success] = getVelocity(obj, ID)
            % get velocity in rev/min

            ID = checkIdAvailable(obj, ID);

            switch ID
                case 1 
                    gear_ratio = 2.4570; % has to be approximated
                case 2
                    gear_ratio = 1;
                case 3
                    gear_ratio = 57/23;
                case 4
                    gear_ratio = 57/23;
                otherwise
                    fprintf("Faulty ID \n");
            end



            PROTOCOL_VERSION = 2;
            ADDR_PRO_PRESENT_VELOCITY = 128;
            velocity = calllib(obj.lib_name, 'read4ByteTxRx', obj.port_num , PROTOCOL_VERSION, ID, ADDR_PRO_PRESENT_VELOCITY);
            
              if velocity > 4294000000
                  velocity = -(4294967295 - velocity);
              end
            
            velocity = (velocity * 0.229) / gear_ratio;

            success = 1;


        end

        function [success] = addVelocity(obj, ID, velocity)
            
            ID = checkIdAvailable(obj, ID);
            [present_velocity, success] = obj.getVelocity(ID);
            if success ~= 1
                return
            end
            
            new_velocity = present_velocity + velocity;

            success = obj.setVelocity(ID, new_velocity);
        
            

        end


    
    end
end

