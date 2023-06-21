classdef RealRobot < handle
    
    properties
        BevelGearObject
        YawJointZeroPos = -inf;
        ElbowJointZeroPos = -inf;
    end
    
    methods
        function obj = RealRobot(PORT)

            % Check if the function was called with a port argument
            if nargin < 1
                % If not, use 'COM3' as a default port
                PORT = 'COM3';
            end

            obj.BevelGearObject = NewBevelGear(PORT);
        end

        function success = setJointVelocity(obj,joint,velocity)


            % joint1 : bevel rotate y
            % joint2 : bevel rotate x
            % joint3 : yaw rotate z : ID 2
            % joint4 : elbow rotate x : ID 1
            
            switch joint
                case 1
                    success = obj.BevelGearObject.setVelocityAroundY(velocity);
                case 2
                    success = obj.BevelGearObject.setVelocityAroundX(velocity);
                case 3
                    success = obj.BevelGearObject.ServosObject.setVelocity(2, -velocity);
                case 4
                    success = obj.BevelGearObject.ServosObject.setVelocity(1, -velocity);
                otherwise
                    error("Invalid Joint: Choose Joint 1 to 4");
            end

            if success == 0
                fprintf("Error settting Joint Velocity \n");
            end

            
        
        end
        
        function success = setOperatingMode(obj, modeString)
            
            successLevel = 0;
            for ID = 1:4
                successLevel = successLevel + obj.BevelGearObject.ServosObject.setOperatingMode(ID,modeString);
            end

            if successLevel < 4
                fprintf("Error setting operating mode \n");
                success = 0;
            else
                fprintf("Successfully set Operation mode of the whole robot \n");
                success = 1;
            end
            
        end
    
        function success = torqueEnableDisable(obj,ID,enable_bool)

            success = obj.BevelGearObject.ServosObject.torqueEnableDisable(ID,enable_bool);

        end

        function success = robotTorqueEnableDisable(obj,enable_bool)


            successLevel = 0;
            for ID = 1:4
                successLevel = successLevel + obj.torqueEnableDisable(ID,enable_bool);
            end

            if successLevel < 4
                fprintf("Error enabling/disabling torque of whole robot \n");
                success = 0;
            else
                fprintf("Successfully enabled/disabled torque of whole robot \n");
                success = 1;
            end


            
        end

        function [angle,success] = getJointAngle(obj,joint)

            % joint1 : bevel rotate y
            % joint2 : bevel rotate x
            % joint3 : yaw rotate z : ID 2
            % joint4 : elbow rotate x : ID 1

            
            switch joint
                case 1
                    [angle, success] = obj.BevelGearObject.getRotationAroundY();
                case 2
                    [angle, success] = obj.BevelGearObject.getRotationAroundX();
                case 3
                    [angle,success] = obj.BevelGearObject.ServosObject.getAngle(2);
                    angle = - obj.YawJointZeroPos + angle;
                case 4
                    gear_ratio = 2.4570; % has to be approximated
                    [angle,success] = obj.BevelGearObject.ServosObject.getAngle(1);
                    angle = -(- obj.ElbowJointZeroPos + angle)/gear_ratio;
                otherwise
                    error("Invalid Joint: Choose Joint 1 to 4");
            end

            if success == 0
                fprintf("Error getting Joint Angle \n");
            end


        end
       
        function success = setZeroPositionToCurrentPosition(obj)

            [YawJointZeroPos, successYaw] = obj.BevelGearObject.ServosObject.getAngle(2);
            [ElbowJointZeroPos, successElbow] = obj.BevelGearObject.ServosObject.getAngle(1);
            successBevel = obj.BevelGearObject.setZeroPositionToCurrentPosition();

            obj.YawJointZeroPos = YawJointZeroPos;
            obj.ElbowJointZeroPos = ElbowJointZeroPos;

            if successYaw && successElbow && successBevel
                success = 1;
                fprintf("Successfully zero'd robot \n");
            else
                success = 0;
                fprintf("Error zeroing robot \n");
            end


        end
        
        function [elevation, success] = getBevelElevation(obj)


            [elevation, success] = obj.BevelGearObject.getElevation();

        end


    end
end
