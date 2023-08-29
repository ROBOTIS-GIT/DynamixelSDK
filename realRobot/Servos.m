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

        %Global Definitions
        PROTOCOL_VERSION = 2;
        COMM_SUCCESS = 0;

    end
    
    methods
        %Constructor
        function obj = Servos(PORT)
            % Hint: Get the correct Port string in your device manager e.g.
            % 'COM3', 'COM4'

            % Check if the constructor was called with a specified Port
            if nargin == 0
                % If not, use 'COM3' as a default port
                PORT = 'COM3';
            end

            %% Modify the following string to your absolute path to the DynamixelSDK
            absolute_path = 'C:\Users\samue\Documents\Git\DynamixelSDK\c\';
            addpath(genpath([absolute_path, 'include\dynamixel_sdk\']))
            %%

            %Local Definitions
            MAX_ID = 10;
            BAUDRATE = 1000000;
            
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
                disp(warnings);
                if isempty(notfound) && isempty(warnings)
                    fprintf("Succeeded to load the dynamixel library \n");
                else
                    fprintf("Failed to load the dynamixel library \n");
                end
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
            if (calllib(obj.lib_name, 'setBaudRate', obj.port_num, BAUDRATE))
                fprintf('Succeeded to change the baudrate!\n');
            else
                delete(obj)
                error('Failed to change the baudrate!\n');
            end

            % Initialize PacketHandler Structs
            calllib(obj.lib_name, 'packetHandler');

            %Scan available IDs
            % Try to broadcast ping the Dynamixel
            calllib(obj.lib_name, 'broadcastPing', obj.port_num, obj.PROTOCOL_VERSION);

            fprintf('Detected Dynamixel : \n');
            for id = 0 : MAX_ID
              if calllib(obj.lib_name, 'getBroadcastPingResult', obj.port_num, obj.PROTOCOL_VERSION, id)
                fprintf('Available ID: %d \n', id);
                obj.availableIDs = [obj.availableIDs, id];
              end
            end
        end

        %Destructor
        function delete(obj)
                disp("closing port, unloading library")
                calllib(obj.lib_name, 'closePort', obj.port_num);
                unloadlibrary(obj.lib_name);
        end

        function ID = checkIdAvailable(obj, id)
            % Throws an error if the ID of a Servo is not available

            if ismember(id, obj.availableIDs)
                ID = id;
            else
                error('ID %d is not available, check Button in ON Position and Connection to Robot Arm', id);
            end
        end
        
        function success = torqueEnableDisable(obj,ID,enable_bool)
            % Enable / Disable the Torque of a servo.

            ID = checkIdAvailable(obj, ID);

            %Local Definitions
            ADDR_PRO_TORQUE_ENABLE       = 64;         % Control table address is different in Dynamixel model
            TORQUE_ENABLE               = 1;            % Value for enabling the torque
            TORQUE_DISABLE              = 0;            % Value for disabling the torque

            % Enable / Disable torque
            if(enable_bool)
                calllib(obj.lib_name, 'write1ByteTxRx', obj.port_num, obj.PROTOCOL_VERSION, ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE);

            else
                calllib(obj.lib_name, 'write1ByteTxRx', obj.port_num, obj.PROTOCOL_VERSION, ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE);
            end
    
            switch enable_bool
                case 1 
                    enable_string = "enabled";
                case 0 
                    enable_string = "disabled";
            end

            % Check if torque is disabled/enabled correctly
            VALUE = calllib(obj.lib_name, 'read1ByteTxRx', obj.port_num , obj.PROTOCOL_VERSION, ID, ADDR_PRO_TORQUE_ENABLE);
            if VALUE == enable_bool
                success = 1;
                % fprintf("Torque of Servo with ID %d %s successfully \n", ID, enable_string);
            else
                success = 0;
                fprintf("Torque of Servo with ID %d could not be set \n", ID);
            end

        end

        function success = setOperatingMode(obj, ID, modeString)
            % Set the operating mode of a Servo

            ID = checkIdAvailable(obj, ID);

            %Local Definitions
            ADDR_PRO_OP_MODE = 11;

            % Check if torque of the servo is disabled
            ADDR_PRO_TORQUE_ENABLE       = 64;         % Control table address is different in Dynamixel model
            VALUE = calllib(obj.lib_name, 'read1ByteTxRx', obj.port_num , obj.PROTOCOL_VERSION, ID, ADDR_PRO_TORQUE_ENABLE);
            if VALUE ~= 0
                success = 0;
                fprintf("Failed to set Operating Mode of Servo with ID %d: Disable Torque first. \n", ID);
                return
            end

            switch modeString
                case 'position'
                    OP_MODE = 3;
                case 'velocity'
                    OP_MODE = 1;
                otherwise
                    frpintf('Invalid Operating Mode. Use ''position'', or ''velocity''.');
                    success = 0;
                    return
            end

            % Set the Operating Mode
            calllib(obj.lib_name, 'write1ByteTxRx', obj.port_num , obj.PROTOCOL_VERSION, ID, ADDR_PRO_OP_MODE, OP_MODE);
                  
            % Check if the Operating Mode was set successfully
            VALUE = calllib(obj.lib_name, 'read1ByteTxRx', obj.port_num , obj.PROTOCOL_VERSION, ID, ADDR_PRO_OP_MODE);
            if VALUE == OP_MODE
                % fprintf("Successfully set Operation mode of Servo with ID %d to %s \n", ID, modeString)
                success = 1;
            else
                fprintf("Failed to set Operation mode of Servo with ID %d \n", ID);
                success = 0;
            end

        end

        function [angle, success] = getAngle(obj,ID)
            %Receive the current Position of a servo in RAD. Can be multi
            %rotation and supports negative angles.

            ID = checkIdAvailable(obj, ID);

            %Local Definitions
            ADDR_PRO_PRESENT_POSITION    = 132;

            % Get the present position
            dxl1_present_position = calllib(obj.lib_name, 'read4ByteTxRx', obj.port_num , obj.PROTOCOL_VERSION, ID, ADDR_PRO_PRESENT_POSITION);
            if dxl1_present_position > 4294967294/2
                % The position becomes negative
                dxl1_present_position = -(4294967294 - dxl1_present_position);
            end
            
            % Convert position to RAD according to dynamixel intern factor
            % (0.087891)
            angle = dxl1_present_position * 0.088 * pi/180;

            success = 1;
        end
    
        function success = setVelocity(obj, ID, velocity)

            % Set a Servos velocity in rev/min. Uses the gear
            % ratios of the servo.
            

            ID = checkIdAvailable(obj, ID);

            % Local Definitions
            ADDR_PRO_GOAL_VELOCITY      = 104;
 
            switch ID
                case 1 
                    gear_ratio = 2.4570; % has to be approximated
                case 2
                    gear_ratio = 1;
                case 3
                    gear_ratio = 57/23; % Bevel Gear Ratio
                case 4
                    gear_ratio = 57/23; % Bevel Gear Ratio
                otherwise
                    fprintf("Faulty ID \n");
            end

            % Check if the operating mode of the servo with the given
            % ID is set to velocity control
            ADDR_PRO_OP_MODE = 11;
            VALUE = calllib(obj.lib_name, 'read1ByteTxRx', obj.port_num , obj.PROTOCOL_VERSION, ID, ADDR_PRO_OP_MODE);
            if VALUE ~= 1
                success = 0;
                fprintf("Control mode of the Servo with ID %d is not set to velocity. Use setOperatingMode method first. \n", ID);
                return
            end

            % Check if torque of the servo is enabled
            ADDR_PRO_TORQUE_ENABLE       = 64;         % Control table address is different in Dynamixel model
            VALUE = calllib(obj.lib_name, 'read1ByteTxRx', obj.port_num , obj.PROTOCOL_VERSION, ID, ADDR_PRO_TORQUE_ENABLE);
            if VALUE ~= 1
                success = 0;
                fprintf("Torque of the Servo with ID %d is not enabled. Use torqueEnableDisable method first. \n", ID);
                return
            end

            % Set the velocity. Uses gear_ratio and dynamixel internal factor (0.229)
            VELOCITY_VAL = 1/0.229 * velocity * gear_ratio; % Convert velocity to decimal
            %Floor VELOCITY_VAL since dynamixel accepts only integers here
            VELOCITY_VAL = floor(VELOCITY_VAL);
            if VELOCITY_VAL <0
                VELOCITY_VAL = 4294967296 + VELOCITY_VAL;
            end
            calllib(obj.lib_name, 'write4ByteTxRx', obj.port_num , obj.PROTOCOL_VERSION, ID, ADDR_PRO_GOAL_VELOCITY, VELOCITY_VAL);
            
            %Check if the goal velocity is set correctly.
            VALUE = calllib(obj.lib_name, 'read4ByteTxRx', obj.port_num , obj.PROTOCOL_VERSION, ID, ADDR_PRO_GOAL_VELOCITY);
            if VALUE == VELOCITY_VAL
                % fprintf("Successfully set velocity of Servo with ID %d \n", ID);
                success = 1;
            else
                fprintf("Could not set velocity of Servo with ID %d \n", ID);
                success = 0;
            end
        end
    end
end

