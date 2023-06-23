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
        

            % Set maximum joint speed
            switch joint
                case 1
                    max_joint_speed = 2;
                case 2
                    max_joint_speed = 2;
                case 3
                    max_joint_speed = 10;
                case 4
                    max_joint_speed = 10;
            end

            % Ensure velocity does not exceed the maximum joint speed
            if abs(velocity) > max_joint_speed
                velocity = max_joint_speed * sign(velocity);
            end


            
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
                if enable_bool == 1
                    fprintf("Successfully enabled torque of whole robot \n");
                else
                    fprintf("Successfully disabled torque of whole robot \n");
                end
                success = 1;
            end


            
        end

        function [angle,success] = getJointAngle(obj,joint)

            % joint1 : bevel rotate y
            % joint2 : bevel rotate x
            % joint3 : yaw rotate z : ID 2
            % joint4 : elbow rotate x : ID 1

            if obj.YawJointZeroPos == -inf || obj.ElbowJointZeroPos == -inf
                success = 0;
                fprintf("Error getting Angle of Joint %d : Zero position not defined \n", joint);
                angle = 0;
                return
            end

            
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
                fprintf("Error getting Joint Angle of Joint %d \n", joint);
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

        function success = goToZeroPosition(obj, enableTorque,epsilon)


            disp("Returning to zero Position...")


            if nargin < 4
                epsilon_deg = 0.3;
                epsilon = deg2rad(epsilon_deg);
            end

            if enableTorque == 1
                obj.robotTorqueEnableDisable(1);
            end
    
            % Initialize errors and gains
            integralError = [0, 0, 0, 0];
            prevError = [0, 0, 0, 0];
            P_Gain = 10;  % Proportional gain
            I_Gain = 2;  % Integral gain
            D_Gain = 5;  % Derivative gain
    
            % Use a PID-controller to move the joint angles to the zero
            % position
            Joint1_converged = 0;
            Joint2_converged = 0;
            Joint3_converged = 0;
            Joint4_converged = 0;
    
            while 1
                % Get current positions
                successLevel = 0;
                [Joint1_Angle, success] = obj.getJointAngle(1);
                successLevel = successLevel + success;
                [Joint2_Angle, success] = obj.getJointAngle(2);
                successLevel = successLevel + success;
                [Joint3_Angle, success] = obj.getJointAngle(3);
                successLevel = successLevel + success;
                [Joint4_Angle, success] = obj.getJointAngle(4);
                successLevel = successLevel + success;
    
                % Update errors
                currentError = [Joint1_Angle, Joint2_Angle, Joint3_Angle, Joint4_Angle];
                integralError = integralError + currentError;
                derivativeError = currentError - prevError;
                prevError = currentError;
    
                if successLevel < 4
                    fprintf("Error getting Angles of Servos")
                    success = 0;
                    return
                end
    
                % Calculate and set the PID velocity for each joint
                successLevel = 0;
                for i = 1:4
                    PID_velocity = -(P_Gain * currentError(i) + I_Gain * integralError(i) + D_Gain * derivativeError(i));
                    successLevel = successLevel + obj.setJointVelocity(i, PID_velocity);
                end
    
                if successLevel < 4
                    fprintf("Error setting Angles of Servos")
                    for i = 1:4
                        obj.setJointVelocity(i, 0);
                    end
                    success = 0;
                    return
                end
    
                % Check if the angles are within epsilon
                for i = 1:4
                    if abs(currentError(i)) < epsilon && ~eval(['Joint' num2str(i) '_converged'])
                        obj.setJointVelocity(i, 0);
                        eval(['Joint' num2str(i) '_converged = 1;']);
                    end
                end
    
                if Joint1_converged && Joint2_converged && Joint3_converged && Joint4_converged
                    fprintf("All joints reset to zero pos. \n")
    
                    % Disable torque
                    obj.robotTorqueEnableDisable(0);
    
                    success = 1;
                    break
                end
            end
        end

        function success = setJointPosition(obj, joint, position, epsilon)
        
                %Blocking function of setting a Joint position

                if nargin < 4
                    epsilon_deg = 0.3;
                    epsilon = deg2rad(epsilon_deg);
                end
        
                % Initialize errors and gains
                integralError = [0, 0, 0, 0];
                prevError = [0, 0, 0, 0];
                P_Gain = 10;  % Proportional gain
                I_Gain = 2;  % Integral gain
                D_Gain = 5;  % Derivative gain
                    
                while 1
                    % Get the current joint angle
                    [joint_angle, success] = obj.getJointAngle(joint);
                
                    % If success is 0, that means there was an error in getting the joint angle
                    if success == 0
                        return
                    end
                    
                    % Calculate the error
                    currentError = position - joint_angle;
            
                    % Update integral and derivative errors
                    integralError = integralError + currentError;
                    derivativeError = currentError - prevError;
                    prevError = currentError;
            
                    % Check if the joint angle is within tolerance
                    if abs(currentError) < epsilon
                        obj.setJointVelocity(joint, 0);
                        fprintf("Joint %d set to desired pos. \n", joint);
                        success = 1;
                        break
                    end
                
                    % Calculate the control signal using a PID-controller
                    control_signal = P_Gain * currentError + I_Gain * integralError + D_Gain * derivativeError;
                
                    % Set the joint velocity
                    success = obj.setJointVelocity(joint, control_signal);

                    if success == 0
                        fprintf("Error setting Joint position. \n")
                        return
                    end
                
                end
        end

        
    end
end
