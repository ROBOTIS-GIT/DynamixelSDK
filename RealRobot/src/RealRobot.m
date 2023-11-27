classdef RealRobot < handle

    properties
        
        ServoZeroPositions = [-inf; -inf; -inf; -inf]; % RAD
        % [0] ShoulderServoOne --> located in x-direction  (front)
        % [1] ShoulderServoTwo (back)
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

        % Conifugre maximum absolut joint velocities % RAD/s
        q_dot_max = [0.03;0.03;0.1;0.1];
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
       
        function [jointAngles] = getQ(obj)
            %Get the angles of the joints q in RAD

            % joint1 : bevel rotation around fixed y-Axis
            % joint2 : bevel rotation around dependent x-Axis
            % joint3 : yaw rotation around z-Axis
            % joint4 : elbow rotatoin around x-Axis

            % Check if Zero Position has been set
            if isinf(sum(obj.ServoZeroPositions))
                fprintf("Could not get Joint Angle, Zero position of the robot is not set. \n\n")
                return
            end

            % Get all servo angles phi in RAD
            servoAngles = [-inf; -inf; -inf; -inf];
            for ID = 1:4
                servoAngles(ID) = obj.ServoChain.getServoAngle(ID);
            end

            %Convert servoAngles phi to jointAngles q
            jointAngles = obj.convertServoAnglesToJointAngles(servoAngles);            

        end

        function goToZeroPosition(obj)
            % Uses a PID controller for the servo position to return to the previously stored zero
            % position.



            % Check if Zero Position has been set
            if isinf(sum(obj.ServoZeroPositions))
                fprintf("Could not return to zero position, Zero position of the robot is not set. \n\n")
                return
            end

            disp("Returning to zero Position...")

            % Use a default precision of 1 °
            precision_deg = 0.3;
            precision = deg2rad(precision_deg);

            % Enable The torque
            obj.torqueEnableDisable(1);

            % Initialize errors and gains for the PID controller. This
            % controller tries to move the servos to their stored zero
            % position by setting their servo.
            integralError = [0; 0; 0; 0];
            prevError = [0; 0; 0; 0];
            P_Gain = 1;  % Proportional gain
            I_Gain = 0;  % Integral gain
            D_Gain = 0;  % Derivative gain

            jointsConverged = [0;0;0;0];
    
            while 1
                % Get current joint angles in RAD
                q = obj.getQ;
   
                % Calculate remaining errors
                currentError = q; % As they should become 0
                integralError = integralError + currentError;
                derivativeError = currentError - prevError;
                prevError = currentError;
               
                %Set velocites
                PID_velocities = P_Gain * currentError + I_Gain * integralError + D_Gain * derivativeError;
                obj.setJointVelocities(-PID_velocities);

                % Check if joints converged
                for i = 1:4
                    if q(i) <= precision
                        jointsConverged(i) = 1;
                    end
                end
                
                % If all joints converged disable the torque and return
                if sum(jointsConverged) == 4
                    obj.setJointVelocities([0;0;0;0])
                    obj.torqueEnableDisable(0)
                    fprintf("All joints reset to zero pos. \n")
                    break
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
                if abs(jointVelocities(ID)) > obj.q_dot_max(ID)
                    jointVelocities(ID) = obj.q_dot_max(ID) * sign(jointVelocities(ID));
                end
            end

            % Convert joint velocities q_dot to servo velocities omega
            servoVelocity = obj.convertJointVelocitiesToServoVelocites(jointVelocities); 

            for ID = 1:4
                obj.ServoChain.setServoVelocity(ID, servoVelocity(ID))
            end

        end
        
        function jointAngles = convertServoAnglesToJointAngles(obj,servoAngles)
            
            phi_1_0 = obj.ServoZeroPositions(1);
            phi_2_0 = obj.ServoZeroPositions(2);
            phi_3_0 = obj.ServoZeroPositions(3);
            phi_4_0 = obj.ServoZeroPositions(4);

            phi_1 = servoAngles(1);
            phi_2 = servoAngles(2);
            phi_3 = servoAngles(3);
            phi_4 = servoAngles(4);


            % Calculation according to Thesis (corrected)
            delta_phi_1 = phi_1_0 - phi_1;
            delta_phi_2 = phi_2_0 - phi_2;
            q_1 = (delta_phi_1 - delta_phi_2)/obj.i_shoulder;
            q_2 = (delta_phi_1 + delta_phi_2)/obj.i_shoulder;

            q_3 = phi_3 - phi_3_0;
            q_4 = (phi_4 - phi_4_0)/obj.i_elbow;
            
            jointAngles = [q_1;q_2;q_3;q_4];

        end

        function servoVelocities = convertJointVelocitiesToServoVelocites(obj,jointVelocities)
            
            q_1_dot = jointVelocities(1);
            q_2_dot = jointVelocities(2);
            q_3_dot = jointVelocities(3);
            q_4_dot = jointVelocities(4);
           
            % Calculation according to Thesis (corrected)
            omega_3 = q_3_dot;
            omega_4 = q_4_dot*obj.i_elbow;

            omega_1 = -0.5*(q_1_dot+q_2_dot) * obj.i_shoulder;
            omega_2 = +0.5*(q_1_dot-q_2_dot) * obj.i_shoulder;

            servoVelocities = [omega_1;omega_2;omega_3;omega_4];


        end
    end
end
