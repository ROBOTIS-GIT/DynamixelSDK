classdef RealRobot < handle

    properties
        
        ServoZeroPositions = [-inf, -inf, -inf, -inf];
        % [0] ShoulderServoOne --> located in x-direction
        % [1] ShoulderServoTwo
        % [2] YawServo
        % [3] ElbowServo

        %ServoChain Object
        ServoChain = [];
        % Assume following ID's (can be configured in Dynamixel Wizard)
        % ID 1 : ShoulderServoOne
        % ID 2 : ShoulderServoTwo
        % ID 3 : YawServo
        % ID 4 : ElbowServo

        %Transmission ratios
        i_shoulder = 5;
        i_elbow = 2.5;

        % Conifugre maximum absolut joint velocities
        maxJointVelocities = [2,2,10,10];
    end

    methods
        function obj = RealRobot()
            obj.ServoChain = ServoChain();
        end

        function torqueEnableDisable(obj,enable_bool)
            % Enable / Disable the torque of the whole robot.

            for ID = 1:4
                obj.ServoChain.torqueEnableDisable(ID,enable_bool);
            end
        end

        function setOperatingMode(obj, modeString)
            % Set the Operating Mode of the whole Robot

            for ID = 1:4
                obj.ServoChain.setOperatingMode(ID,modeString);
            end
        end

        function setZeroPositionToCurrentPosition(obj)
            
            % Store the current servo angles as zero angles
            for ID = 1:4
                obj.ServoZeroPositions(ID) = obj.ServoChain.getServoAngle(ID);
            end
        end
       
        function [jointAngles] = getJointAngles(obj)
            %Get the angles of the joints q in RAD

            % joint1 : bevel rotation around fixed y-Axis
            % joint2 : bevel rotation around dependent x-Axis
            % joint3 : yaw rotation around z-Axis
            % joint4 : elbow rotatoin around x-Axis

            % Check if Zero Position has been set
            if isInf(sum(obj.ServoZeroPositions))
                fprintf("Could not get Joint Angle, Zero position of the robot is not set. \n\n")
                return
            end

            % Get all servo angles phi in RAD
            servoAngles = [-inf, -inf, -inf, -inf];
            for ID = 1:4
                servoAngles(ID) = obj.ServoChain.getServoAngle(ID);
            end

            %Convert servoAngles phi to jointAngles q
            jointAngles = obj.convertServoAnglesToJointAngles(servoAngles);            

        end

        function [elevation] = getBevelElevation(obj)
            % Calculate the elevation of the bevel gear in spherical
            % coordinates in RAD from the jointAngles
            jointAngles = obj.getJointAngles();

            % Calculate the elevation using a trignometric formula
            elevation = pi/2 - acos(cos(jointAngles(2))*cos(jointAngles(1)));           
        end

        function goToZeroPosition(obj)
            % Uses a PID controller for the servo position to return to the previously stored zero
            % position.

            % Check if Zero Position has been set
            if isInf(sum(obj.ServoZeroPositions))
                fprintf("Could not return to zero position, Zero position of the robot is not set. \n\n")
                return
            end

            disp("Returning to zero Position...")

            % Use a default precision of 0.3 °
            precision_deg = 0.3;
            precision = deg2rad(precision_deg);

            % Enable The torque
            obj.torqueEnableDisable(1);

            % Initialize errors and gains for the PID controller. This
            % controller tries to move the servos to their stored zero
            % position by setting their servo.
            integralError = [0, 0, 0, 0];
            prevError = [0, 0, 0, 0];
            P_Gain = 15;  % Proportional gain
            I_Gain = 0;  % Integral gain
            D_Gain = 5;  % Derivative gain
    
            % Store which servo has already converged to their zero
            % angle
            servosConverged = [0, 0, 0, 0];

    
            while 1
                % Get current servo angles phi in RAD
                servoAngles = [-inf, -inf, -inf, -inf];
                for ID = 1:4
                    servoAngles(ID) = obj.ServoChain.getServoAngle(ID);
                end
   
                % Calculate remaining errors
                currentError = obj.ServoZeroPositions - servoAngles; % As they should become 0
                integralError = integralError + currentError;
                derivativeError = currentError - prevError;
                prevError = currentError;
               
                %Set velocites
                PID_velocities = P_Gain * currentError + I_Gain * integralError + D_Gain * derivativeError;
                for ID = 1:4
                    obj.ServoChain.setServoVelocity(ID, PID_velocities(ID))
                end
    
                % Set each joint as converged if their angle is within precision
                for ID = 1:4
                    if abs(currentError(ID)) < precision
                        servosConverged(ID) = 1;
                        obj.ServoChain.setServoVelocity(ID, 0);
                    end
                end
                
                % If all joints converged disable the torque and return
                if sum(jointsConverged) == 4
                    fprintf("All joints reset to zero pos. \n")
                    obj.torqueEnableDisable(0);
                    return
                end
            end
        end

        function setJointVelocities(obj,jointVelocities)
            % Set the angular velocities q_dot of all joints in rad/s.

            % joint1 : bevel rotation around fixed y-Axis
            % joint2 : bevel rotation around dependent x-Axis
            % joint3 : yaw rotation around z-Axis
            % joint4 : elbow rotatoin around x-Ax
       

            % Ensure velocities do not exceed the configued maximum joint speed
            for ID = 1:4
                if abs(jointVelocities(ID)) > obj.maxJointVelocities(ID)
                    jointVelocities(ID) = obj.maxJointVelocities(ID) * sign(jointVelocities(ID));
                end
            end

            % Convert joint velocities q_dot to servo velocities omega
            servoVelocity = obj.convertJointVelocitiesToServoVelocites(jointVelocities); 

            for ID = 1:4
                obj.ServoChain.setServoVelocity(ID, servoVelocity(ID))
            end

        end
        
        function jointAngles = convertServoAnglesToJointAngles(servoAngles)
            
            phi_1_0 = obj.ServoZeroPositions(1);
            phi_2_0 = obj.ServoZeroPositions(2);
            phi_3_0 = obj.ServoZeroPositions(3);
            phi_4_0 = obj.ServoZeroPositions(4);

            phi_1 = servoAngles(1);
            phi_2 = servoAngles(2);
            phi_3 = servoAngles(3);
            phi_4 = servoAngles(4);


            % Calculation according to Thesis
            delta_phi_1 = phi_1_0 - phi_1;
            delta_phi_2 = phi_2_0 - phi_2;
            q_1 = (delta_phi_1 - delta_phi_2)/obj.i_shoulder;
            q_2 = -(delta_phi_1 - delta_phi_2)/obj.i_shoulder;

            q_3 = phi_3 - phi_3_0;
            q_4 = -(phi_4 - phi_4_0)/obj.i_elbow;
            
            jointAngles = [q_1,q_2,q_3,q_4];

        end

        function servoVelocities = convertJointVelocitiesToServoVelocites(jointVelocities)
            
            q_1_dot = jointVelocities(1);
            q_2_dot = jointVelocities(2);
            q_3_dot = jointVelocities(3);
            q_4_dot = jointVelocities(4);
           
            % Calculation according to Thesis
            omega_3 = q_3_dot;
            omega_4 = -q_4_dot*obj.i_elbow;

            omega_1 = 0.5*(q_2_dot-q_1_dot) * obj.i_shoulder;
            omega_2 = 0.5*(q_2_dot+q_1_dot) * obj.i_shoudler;

            servoVelocities = [omega_1,omega_2,omega_3,omega_4];


        end
    end
end
