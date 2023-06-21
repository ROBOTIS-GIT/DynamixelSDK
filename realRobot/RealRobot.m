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
                    success = obj.BevelGearObject.ServosObject.setVelocity(2, velocity);
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
        
        function [success] = goToZeroPosition(obj, enableTorque)

            if enableTorque == 1
                obj.robotTorqueEnableDisable(1)
            end

            
            if obj.YawJointZeroPos == -inf || obj.ElbowJointZeroPos == -inf
                success = 0;
                fprintf("Error: Zero position not defined \n");
                return
            end


            % Use a P-controller to move the joint angles to the zero
            % position


            Joint1_converged = 0;
            Joint2_converged = 0;
            Joint3_converged = 0;
            Joint4_converged = 0;


            while 1
                %Get current pos
                successLevel = 0;
                [Joint1_Angle, success] = obj.getJointAngle(1);
                successLevel = successLevel + 1;
                [Joint2_Angle, success] = obj.getJointAngle(2);
                successLevel = successLevel + 1;
                [Joint3_Angle, success] = obj.getJointAngle(3);
                successLevel = successLevel + 1;
                [Joint4_Angle, success] = obj.getJointAngle(4);
                successLevel = successLevel + 1;

                if successLevel < 4
                    fprintf("Error getting Angles of Servos")
                    success = 0;
                    return
                end

                %Set the velocity with a Gain
                P_Gain = 5;
                successLevel = 0;
                successLevel = successLevel + obj.setJointVelocity(1, -Joint1_Angle*P_Gain);
                successLevel = successLevel + obj.setJointVelocity(2, -Joint2_Angle*P_Gain);
                successLevel = successLevel + obj.setJointVelocity(3, -Joint3_Angle*P_Gain);
                successLevel = successLevel + obj.setJointVelocity(4, -Joint4_Angle*P_Gain);
                

                if successLevel < 4
                    fprintf("Error setting Angles of Servos")
                    obj.setJointVelocity(1, 0);
                    obj.setJointVelocity(2, 0);
                    obj.setJointVelocity(3, 0);
                    obj.setJointVelocity(4, 0);
                    success = 0;
                    return
                end



                %Check if the angles are within epsilon
                if abs(Joint1_Angle) < 0.02 && ~Joint1_converged
                    obj.setJointVelocity(1, 0);
                    fprintf("Joint 1 reset to zero pos. \n")
                    Joint1_converged = 1;
    
                end
                if abs(Joint2_Angle) <  0.02 && ~Joint2_converged
                    obj.BevelGearObject.ServosObject.setVelocity(4, 0);
                    fprintf("Joint 2 reset to zero pos. \n")
                    Joint2_converged = 1;   
                end
                if abs(Joint3_Angle) <  0.05 && ~Joint3_converged
                    obj.BevelGearObject.ServosObject.setVelocity(2, 0);
                    fprintf("Joint 3 reset to zero pos. \n")
                    Joint3_converged = 1;   
    
                end
                if abs(Joint4_Angle) <  0.05 && ~Joint4_converged
                    obj.BevelGearObject.ServosObject.setVelocity(1, 0);
                    fprintf("Joint 4 reset to zero pos. \n")
                    Joint4_converged = 1;   
    
                end

                if Joint1_converged && Joint2_converged && Joint3_converged && Joint4_converged
                    fprintf("All servos reset to zero pos. \n")

                    %Disable torque
                    obj.robotTorqueEnableDisable(0);


                    success = 1;
                    break
                end


            end



        end

   
    
    end
end
