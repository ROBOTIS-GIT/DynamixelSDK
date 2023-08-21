classdef SimulatedRobot < handle
    % SimulatedRobot is a MATLAB class that represents a robot manipulator 
    % in a 3D environment. It provides methods for computing forward 
    % kinematics, Jacobian matrices, displaying the robot configuration 
    % in a 3D plot and updating the positions of its joints and links. 
    % The robot is modeled as a collection of rigid links connected by 
    % joints and additional frames can be defined as needed. 
    %
    % Each instance of the class has an array of Joint objects representing 
    % the robot's joints, an array of Link objects representing the physical 
    % connections between joints, and an array of Frame objects representing 
    % additional frames of the robot.
    %
    % The class provides two methods for forward kinematics, one numerical 
    % and one symbolic, which return the position of the end effector in 
    % global coordinates given the current joint angles. It also provides 
    % two methods for computing the Jacobian matrix, one numerical and one 
    % symbolic, which relate the joint velocities to the linear and angular 
    % velocity of the end effector. 
    %
    % The display method updates and displays all joints, links, and frames 
    % in a 3D plot, with the option to clear the previous plot and draw 
    % the coordinate frames.
    
    properties
        joints  % An array of Joint objects, defining the joints of the robot
        links   % An array of Link objects, defining the physical connections between joints
        frames % An array of Frame objects, defining additional frames of the robot
        axis_set = 0; % Store if the axis property of the plot has already been set once.
        % This saves compute compared to setting it repeatedly in the
        % display method.
    end

    methods
        % Constructor method for Robot. It takes two arguments, an array
        % of Joint objects and an array of Link objects
        function obj = SimulatedRobot(joints, links, frames)
            obj.joints = joints;
            obj.links = links;
            obj.frames = frames;
        end

        function [oxE] = forwardKinematicsNumeric(obj)
            %This method is written for the 4 joint robot configuration of the real robot. A
            %general method with n-joints is not available yet.
            
            %This method computes the Endeffector Position in the orignal
            %Frame (global coordinates) using the joint angles and given
            %dimensions. It does so by using rotational matrizes.

            %Get joint angles
            alpha = obj.joints(1).angle;
            beta = obj.joints(2).angle;
            gamma = obj.joints(3).angle;
            delta = obj.joints(4).angle;

            % Calculate the rotational matrices
            R1 = roty(alpha);
            R2 = rotx(beta);
            R3 = rotz(gamma);
            R4 = rotx(delta);

            % Distances
            xo1 = obj.joints(1).relativePosition;
            x12 = obj.joints(2).relativePosition;
            x23 = obj.joints(3).relativePosition;
            x34 = obj.joints(4).relativePosition;
            x4E = obj.frames(2).relativePosition;

            oxE  = R1 * (R2 * (R3 * (R4 * x4E + x34) + x23) + x12) + xo1;

        end

        function [sOxE] = forwardKinematicsSymbolic(obj)
            % This method computes the symbolic version of the forward kinematics function
            
            % Declare joint angles as symbolic variables
            syms sAlpha sBeta sGamma sDelta real;
            
            % Rotational matrices
            sR1 = roty(-sAlpha);
            sR2 = rotx(-sBeta);
            sR3 = rotz(-sGamma);
            sR4 = rotx(-sDelta);
            
            % Distances
            xo1 = obj.joints(1).relativePosition;
            x12 = obj.joints(2).relativePosition;
            x23 = obj.joints(3).relativePosition;
            x34 = obj.joints(4).relativePosition;
            x4E = obj.frames(2).relativePosition;
            
            % Symbolic forward kinematics
            sOxE  = sR1 * (sR2 * (sR3 * (sR4 * x4E + x34) + x23) + x12) + xo1;
        end

        function J = getJacobianSymbolic(obj)
            % This method computes the Jacobian matrix based on the
            % symbolic version of the forward kinematics function.
            
            % Declare joint angles as symbolic variables
            syms sAlpha sBeta sGamma sDelta real;
            
            % Get symbolic forward kinematics
            sOxE = obj.forwardKinematicsSymbolic;
            
            % Compute the Jacobian by partial derivation of the symbolic
            % forward kinematic equation
            J = jacobian(sOxE, [sAlpha, sBeta, sGamma, sDelta]);
        end

        function J = getJacobianNumeric(obj)
            % This method computes the Jacobian matrix numerically. Much
            % faster then the symbolic derivation.

            % This function computes the Jacobian matrix of the robot manipulator using a numerical method.
            %
            % The Jacobian matrix is a matrix of partial derivatives that relates the joint velocities of 
            % the robot manipulator to the linear and angular velocity of the end effector. Each column 
            % of the Jacobian matrix represents the derivative of the end effector's position with respect 
            % to one of the joint angles. Therefore, the size of the Jacobian matrix is 3xN for a 
            % manipulator with N joints, because each joint angle contributes one column to the Jacobian 
            % matrix, and the end effector's position is a 3D vector.
            %
            % The numerical method used to compute the Jacobian is based on the definition of the derivative. 
            % The derivative of a function at a point can be approximated as the change in the function value 
            % divided by the change in the input, for a small change in the input. In this case, the function 
            % is the forward kinematics function of the robot, which computes the end effector's position given 
            % the joint angles. The input is the vector of joint angles.
            %
            % For each joint angle, a small change 'delta_q' is added and subtracted from the current joint 
            % angle to compute 'q_plus' and 'q_minus'. Then, the forward kinematics function is called with 
            % 'q_plus' and 'q_minus' to compute 'oxE_plus' and 'oxE_minus', which are the positions of the 
            % end effector when the joint angle is slightly increased and slightly decreased, respectively. 
            % The derivative with respect to the joint angle is then computed as 
            % '(oxE_plus - oxE_minus) / (2 * delta_q)', which is the change in the end effector's position 
            % divided by the change in the joint angle.
            %
            % This process is repeated for each joint angle to compute each column of the Jacobian matrix. 
            % The result is a matrix that relates joint velocities to the velocity of the end effector, 
            % which can be used for tasks such as motion planning and control.
            %
            % Note: After the Jacobian matrix is computed, the joint angles are reset to their original 
            % values, because changing the joint angles can affect other parts of the robot model. This 
            % ensures that the function has no side effects.
            %
            % Note: This method of computing the Jacobian numerically can be faster than symbolic computation, 
            % especially if the forward kinematics function can be evaluated quickly. However, it might not 
            % be as precise as symbolic computation, especially if the forward kinematics function is not 
            % smooth or if 'delta_q' is not small enough. 

            % Get current joint angles
            alpha = obj.joints(1).angle;
            beta = obj.joints(2).angle;
            gamma = obj.joints(3).angle;
            delta = obj.joints(4).angle;
        
            % Vector of joint angles
            q = [alpha, beta, gamma, delta];
        
            % Small change in joint angles
            delta_q = 1e-6;
        
            % Initialize Jacobian matrix
            J = zeros(3, 4);
        
            % For each joint angle
            for i = 1:4
                % Add small change to joint i
                q_plus = q;
                q_plus(i) = q_plus(i) + delta_q;
        
                % Subtract small change from joint i
                q_minus = q;
                q_minus(i) = q_minus(i) - delta_q;
        
                % Update joint angles
                obj.joints(1).angle = q_plus(1);
                obj.joints(2).angle = q_plus(2);
                obj.joints(3).angle = q_plus(3);
                obj.joints(4).angle = q_plus(4);
        
                % Compute forward kinematics
                oxE_plus = obj.forwardKinematicsNumeric;
        
                % Update joint angles
                obj.joints(1).angle = q_minus(1);
                obj.joints(2).angle = q_minus(2);
                obj.joints(3).angle = q_minus(3);
                obj.joints(4).angle = q_minus(4);
        
                % Compute forward kinematics
                oxE_minus = obj.forwardKinematicsNumeric;
        
                % Compute derivative
                J(:, i) = (oxE_plus - oxE_minus) / (2 * delta_q);
            end
            % Reset joint angles to original values
            obj.joints(1).angle = q(1);
            obj.joints(2).angle = q(2);
            obj.joints(3).angle = q(3);
            obj.joints(4).angle = q(4);
        end

        function display(obj, draw_frames)
            % The display method updates and displays all joints and links

            % Get the current figure handle
            fig = findobj('type', 'figure');
            
            % If the figure does not exist, create a new one
            if isempty(fig)
                fig = figure;
            end
            
            hold on;
            grid on;
            for i = 1:length(obj.joints)
                if draw_frames
                    obj.joints(i).display();
                end
            end
            for i = 1:length(obj.links)
                obj.links(i).display();
            end
            for i = 1:length(obj.frames)
                if draw_frames
                    obj.frames(i).display();
                end
            end
            if obj.axis_set == 0
                view(-25, 40);
                axis equal;
                xlabel('X');
                ylabel('Y');
                zlabel('Z');
                title('Simulated Robot');
                obj.axis_set = 1;
            end
        end

    end
end

%% Definition of the standard rotational matrices

%% Definition of the standard rotational matrices

function rotx = rotx(alpha)
    rotx = [1 0 0; 0 cos(alpha) -sin(alpha); 0 sin(alpha) cos(alpha)];
end

function roty = roty(beta)
    roty = [cos(beta) 0 sin(beta); 0 1 0; -sin(beta) 0 cos(beta)];
end

function rotz = rotz(gamma)
    rotz = [cos(gamma) -sin(gamma) 0; sin(gamma) cos(gamma) 0; 0 0 1];
end

    
