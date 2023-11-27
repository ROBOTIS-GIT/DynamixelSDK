classdef ServoChain < handle
% The 'ServoChain' class provides an interface for interacting with connected servos. It is
% designed to establish a connection to a specified port and list all the servos available at that port.
% The class allows the user to retrieve the angles of each connected servo and set their velocities.
% Additionally, it provides functionality to enable and disable the torque for each servo.
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
        function obj = ServoChain(PORT)
            % Hint: Get the correct Port string in your device manager e.g.
            % 'COM3', 'COM4'

            % Check if the constructor was called with a specified Port
            if nargin == 0
                % If not, use 'COM3' as a default port
                PORT = 'COM3';
            end

            %% Modify the following string to your absolute path to the DynamixelSDK
            absolute_path = 'C:\Users\samue\Documents\Git\Robotic-Arm-Prototype\DynamixelLib\c\';
            addpath(genpath([absolute_path, 'include\dynamixel_sdk\']))
            %%
            
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

            %Local Definitions
            MAX_ID = 10;
            BAUDRATE = 1000000;

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
            for ID = 0 : MAX_ID
              if calllib(obj.lib_name, 'getBroadcastPingResult', obj.port_num, obj.PROTOCOL_VERSION, ID)
                fprintf('Available ID: %d \n', ID);
                % Store the available IDs
                obj.availableIDs = [obj.availableIDs, ID];
              end
            end
        end

        %Destructor
        function delete(obj)
                disp("closing port, unloading library")
                calllib(obj.lib_name, 'closePort', obj.port_num);
                unloadlibrary(obj.lib_name);
        end

        function torqueEnableDisable(obj,ID,enable_bool)
            % Enable / Disable the Torque of a servo.

           checkIDAvailable(obj, ID);

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

        end

        function checkIDAvailable(obj, ID)
            % Throws an error if the ID of a Servo is not available

            if ~ismember(ID, obj.availableIDs)
                error('Servo ID %d is not available', ID);
            end
        end

        function setOperatingMode(obj, ID, modeString)
            % Set the operating mode of a Servo

            obj.checkIDAvailable(ID);
            switch modeString
                case 'position'
                    OP_MODE = 3;
                case 'velocity'
                    OP_MODE = 1;
                otherwise
                    error('Error setting servo operating mode: Invalid mode. Use ''position'', or ''velocity .');
            end

            %Local Definitions
            ADDR_PRO_OP_MODE = 11;

            % Set the Operating Mode
            calllib(obj.lib_name, 'write1ByteTxRx', obj.port_num , obj.PROTOCOL_VERSION, ID, ADDR_PRO_OP_MODE, OP_MODE);
        end

        function servoAngle = getServoAngle(obj,ID)
            %Receive the current Position of a servo in RAD. Can be multi
            %rotation and supports negative angles.

           checkIDAvailable(obj, ID);

            %Local Definitions
            ADDR_PRO_PRESENT_POSITION    = 132;

            % Get the present position
            % This should give a value in 4 byte (256^4) continuous range
            dxl1_present_position = calllib(obj.lib_name, 'read4ByteTxRx', obj.port_num , obj.PROTOCOL_VERSION, ID, ADDR_PRO_PRESENT_POSITION);
            
            % Define the conversion factor and midpoint
            conversionFactor = 0.087891;
            maxrange = 4294967295; % 4-byte
            midpoint = maxrange/2;
        
            % Check and Convert the position
            if dxl1_present_position == maxrange || dxl1_present_position == 0 % 0xFFFFFFFF || 0x00000000 in decimal
                servoAngle = 0;
            elseif dxl1_present_position > midpoint
                % Convert to a negative value
                servoAngle = (dxl1_present_position - maxrange) * conversionFactor;
            else
                % Convert to a positive value
                servoAngle = (dxl1_present_position) * conversionFactor;
            end
            % Convert to RAD
            servoAngle = deg2rad(servoAngle);

        end
    
        function setServoVelocity(obj, ID, servoVelocity)
            %Set the current Position of a servo in RAD/s. Supports
            %negative velocities.

            % Set a Servos velocity in rev/min
           checkIDAvailable(obj, ID);

            % Local Definitions
            ADDR_PRO_GOAL_VELOCITY      = 104;

            % Convert desired rev/s to dynamixel deciaml
            VELOCITY_VAL = (servoVelocity*60)/0.229; % Convert rad/s to decimal 

            %Round VELOCITY_VAL since dynamixel accepts only integers here
            VELOCITY_VAL = round(VELOCITY_VAL);

            % VELOCITY_VAL is a value in 4 byte (256^4) continuous range
            % A value of (256^4) / 2 is zero, values bigger are positive,
            % values smaller are negative.
            maxrange = 4294967295; % 4-byte
            if VELOCITY_VAL < 0
                VELOCITY_VAL = maxrange+VELOCITY_VAL;
            end

            
            calllib(obj.lib_name, 'write4ByteTxRx', obj.port_num , obj.PROTOCOL_VERSION, ID, ADDR_PRO_GOAL_VELOCITY, VELOCITY_VAL);
            
        end
    end
end

