classdef RealRobot < handle
% The RealRobot class is an object-oriented MATLAB implementation that 
% represents a physical robot. This class models the robot's structure 
% and movements and offers a variety of functionalities to interact with the robot.
%
% 1. BevelGearObject: This property holds an instance of BevelGear class. 
%    It represents the physical Bevel gear mechanism of the robot.
%
% 2. YawJointZeroPos and ElbowJointZeroPos: These properties store the 
%    zero position of the Yaw and Elbow joints, which are used as 
%    references for further movements. They are initialized with a value 
%    of -inf, which is changed when the setZeroPositionToCurrentPosition method is called.
%
% 3. RealRobot(PORT): The constructor of the class. It initializes 
%    BevelGearObject with a specified port. If no port is specified, 
%    'COM3' is used as a default.
%
% 4. torqueEnableDisable(enable_bool): This method enables or disables 
%    the torque of the whole robot.
%
% 5. setOperatingMode(modeString): This method sets the operating mode of 
%    the robot.
%
% 6. setZeroPositionToCurrentPosition(): This method sets the zero position 
%    of the Yaw and Elbow joints to their current position.
%
% 7. getJointAngle(joint): This method retrieves the current joint angle 
%    for a specified joint.
%
% 8. getBevelElevation(): This method gets the current elevation of the bevel gear.
%
% 9. goToZeroPosition(precision): This method uses a PID controller to 
%    return the robot to its previously stored zero position.
%
% 10. setJointPosition(joint, position, precision): This method sets the 
%     position of a specific joint using a PID controller.
%
% 11. setJointVelocity(joint, velocity): This method sets the velocity 
%     of a specified joint.

    properties
        BevelGearObject = [];

        % The zero positions of the Yaw Joint (Joint 3) and Elbow Joint
        % (Joint 4). Need to be set first using the
        % setZeroPositionToCurrentPosition method.
        YawJointZeroPos = -inf;
        ElbowJointZeroPos = -inf;
    end
    
    methods
        function obj = RealRobot(PORT)
            % Check if the constructor was called with a specified Port
            if nargin < 1
                % If not, use 'COM3' as a default port
                PORT = 'COM3';
            end
            % Initialize the Bevel Gear object containing the Servos object
            obj.BevelGearObject = BevelGear(PORT);
        end

        function success = torqueEnableDisable(obj,enable_bool)
            % Enable / Disable the torque of the whole robot.

            successLevel = 0;
            for ID = 1:4
                successLevel = successLevel + obj.BevelGearObject.ServosObject.torqueEnableDisable(ID,enable_bool);
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

        function success = setOperatingMode(obj, modeString)
            % Set the Operating Mode of the whole Robot

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

        function success = setZeroPositionToCurrentPosition(obj)
            % Set the non-rotated (zero) position of the whole robot
            
            % Store the Yaw and Elbow Joint Zero position
            [YawJointZeroPos, successYaw] = obj.BevelGearObject.ServosObject.getAngle(2);
            [ElbowJointZeroPos, successElbow] = obj.BevelGearObject.ServosObject.getAngle(1);
            obj.YawJointZeroPos = YawJointZeroPos;
            obj.ElbowJointZeroPos = ElbowJointZeroPos;

            % The Zero position of the Bevel Gear is stored in the
            % underlying bevel gear object
            successBevel = obj.BevelGearObject.setZeroPositionToCurrentPosition();

            if successYaw && successElbow && successBevel
                success = 1;
                fprintf("Successfully zero'd robot \n");
            else
                success = 0;
                fprintf("Error zeroing robot \n");
            end
        end
       
        function [angle,success] = getJointAngle(obj,joint)

            % joint1 : bevel rotation around fixed y-Axis
            % joint2 : bevel rotation around dependent x-Axis
            % joint3 : yaw rotation around z-Axis
            % joint4 : elbow rotatoin around x-Axis

            % Check if Zero Position has been set
            if obj.YawJointZeroPos == -inf || obj.ElbowJointZeroPos == -inf
                success = 0;
                fprintf("Failed getting Angle of Joint %d : Zero position not defined \n", joint);
                angle = -inf;
                return
            end

            switch joint
                case 1
                    % Use the functionality in the bevel gear class to get
                    % the rotation around the fixed y-Axis  relative to
                    % its zero position
                    [angle, success] = obj.BevelGearObject.getRotationAroundY();
                case 2
                    % Use the functionality in the bevel gear class to get
                    % the rotation around the dependent x-Axis relative to
                    % its zero position
                    [angle, success] = obj.BevelGearObject.getRotationAroundX();
                case 3
                    % Get the rotation of the yaw joint relative to it's zero
                    % position
                    [angle,success] = obj.BevelGearObject.ServosObject.getAngle(2);
                    angle = angle - obj.YawJointZeroPos;
                case 4
                    % Get the rotation of the elbow joint relative to it's zero
                    % position
                    % Due to the belt transmission, the rotation has to be
                    % converted using the gear_ratio.
                    belt_gear_ratio = 2.4570;
                    [angle,success] = obj.BevelGearObject.ServosObject.getAngle(1);
                    % Signed swapped due to belt transmission
                    angle = -( angle - obj.ElbowJointZeroPos)/belt_gear_ratio;
                otherwise
                    success = 0;
                    fprintf("Invalid Joint: Choose Joint 1 to 4 \n");
                    return
            end

            if success == 0
                fprintf("Failed getting Joint Angle of Joint %d \n", joint);
            end
        end

        function [elevation, success] = getBevelElevation(obj)
            % Get the elevation of the bevel gear from the underlying bevel
            % gear object.

            [elevation, success] = obj.BevelGearObject.getElevation();
        end

        function success = goToZeroPosition(obj,precision)
            % Uses a PID controller for the joint position to return to the previously stored zero
            % position of the robot.

            % Check if Zero Position has been set
            if obj.YawJointZeroPos == -inf || obj.ElbowJointZeroPos == -inf
                success = 0;
                fprintf("Failed return to zero position: Zero position not defined \n");
                return
            end

            disp("Returning to zero Position...")

            % Use a default precision of 0.3 ° if no precision has been
            % defined
            if nargin < 4
                precision_deg = 0.3;
                precision = deg2rad(precision_deg);
            end

            % Enable The torque
            obj.torqueEnableDisable(1);

            % Initialize errors and gains for the PID controller. This
            % controller tries to move the joints to their stored zero
            % position by setting their velocity.
            integralError = [0, 0, 0, 0];
            prevError = [0, 0, 0, 0];
            P_Gain = 15;  % Proportional gain
            I_Gain = 0;  % Integral gain
            D_Gain = 5;  % Derivative gain
    
            % Store which joints have already converged to their zero
            % positoin within the precision
            Joint1_converged = 0;
            Joint2_converged = 0;
            Joint3_converged = 0;
            Joint4_converged = 0;

            % Initialize extra variables
            joint_angle_previous = [inf, inf, inf, inf];
            joint_not_changing_counter = [0, 0, 0, 0];
            I_gain_enabled = false;
    
            while 1
                % Get current joint positions
                successLevel = 0;
                [Joint1_Angle, success] = obj.getJointAngle(1);
                successLevel = successLevel + success;
                [Joint2_Angle, success] = obj.getJointAngle(2);
                successLevel = successLevel + success;
                [Joint3_Angle, success] = obj.getJointAngle(3);
                successLevel = successLevel + success;
                [Joint4_Angle, success] = obj.getJointAngle(4);
                successLevel = successLevel + success;

                % Check if all joint angles where received
                if successLevel < 4
                    fprintf("Error getting Angles of Servos")
                    success = 0;
                    return
                end
    
                % Calculate remaining errors
                currentError = [Joint1_Angle, Joint2_Angle, Joint3_Angle, Joint4_Angle];
                integralError = integralError + currentError;
                derivativeError = currentError - prevError;
                prevError = currentError;
        
                % Check if the joint angle is not changing much
                for i = 1:4
                    if abs(currentError(i) - joint_angle_previous(i)) < 0.15
                        joint_not_changing_counter(i) = joint_not_changing_counter(i) + 1;
                    else
                        joint_not_changing_counter(i) = 0; % Reset the counter if the change is more than 0.1 rad
                    end
                end
                joint_angle_previous = currentError;
        
                % If the joint angle is not changing for more than 3 timesteps for all joints, enable the I_gain
                if all(joint_not_changing_counter > 3) && ~I_gain_enabled
                    I_Gain = 3;
                    integralError = [0, 0, 0, 0]; % clear integralError
                    I_gain_enabled = true; % set the flag to avoid multiple activations
                    disp('I-Gain has been enabled');
                end
               
                % Calculate and set the PID velocity for each joint
                successLevel = 0;
                for i = 1:4
                    PID_velocity = -(P_Gain * currentError(i) + I_Gain * integralError(i) + D_Gain * derivativeError(i));
                    successLevel = successLevel + obj.setJointVelocity(i, PID_velocity);
                end
    
                % Check if all joint velocities where set
                if successLevel < 4
                    fprintf("Error setting Angles of Servos")
                    for i = 1:4
                        obj.setJointVelocity(i, 0);
                    end
                    success = 0;
                    return
                end
    
                % Set each joint as converged if their angle is within precision
                for i = 1:4
                    if abs(currentError(i)) < precision && ~eval(['Joint' num2str(i) '_converged'])
                        obj.setJointVelocity(i, 0);
                        eval(['Joint' num2str(i) '_converged = 1;']);
                    end
                end
                
                % If all joints converged disable the torque and return
                if Joint1_converged && Joint2_converged && Joint3_converged && Joint4_converged
                    fprintf("All joints reset to zero pos. \n")
    
                    % Disable torque
                    obj.torqueEnableDisable(0);
    
                    success = 1;
                    return
                end
            end
        end

        function success = setJointVelocity(obj,joint,velocity)
            % Set the velocity of a specified Joint in rev/min.

            % joint1 : bevel rotation around fixed y-Axis
            % joint2 : bevel rotation around dependent x-Axis
            % joint3 : yaw rotation around z-Axis
            % joint4 : elbow rotatoin around x-Ax
        
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
                    % Signed swapped due to belt transmission
                    success = obj.BevelGearObject.ServosObject.setVelocity(1, -velocity);
                otherwise
                    fprintf("Invalid Joint: Choose Joint 1 to 4 \n");
            end

            if success == 0
                fprintf("Error settting Joint Velocity \n");
            end     
        end
    end
end
