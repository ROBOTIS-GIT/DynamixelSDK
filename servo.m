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
                if(notfound)
                    error(warnings)
                else
                    disp("lib loaded")
                end
            end

            % Open port
            obj.port_num = portHandler('COM3'); %%%

            if (openPort(obj.port_num))
                fprintf('Succeeded to open the port!\n');
            else
                unloadlibrary(obj.lib_name);
                fprintf('Failed to open the port!\n');
                input('Press any key to terminate...\n');
                return;
            end
        end

        function delete(obj)
                disp("destructor has been called")
                unloadlibrary(obj.lib_name);
                closePort(obj.port_num);

        end
        
        function outputArg = method1(obj,inputArg)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            outputArg = obj.Property1 + inputArg;
        end
    end
end

