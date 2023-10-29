classdef SimulatedRobot < handle
    % SimulatedRobot is a MATLAB class that represents a robot manipulator 
    % in a 3D environment. It provides methods for computing forward 
    % kinematics, Jacobian matrices and displaying the robot configuration 
    % in a 3D plot. 
    %
    % Each instance of the class has an array of Joint objects representing 
    % the robot's joints, an array of Link objects representing the physical 
    % connections between joints, and an array of Frame objects representing 
    % additional frames of the robot.

    properties
        joints  % An array of Joint objects, defining the joints of the robot
        links   % An array of Link objects, defining the physical connections between joints
        frames % An array of Frame objects, defining additional frames of the robot
        fig % The figure in which everything is visualized
    end

    methods
        function obj = SimulatedRobot()
            %% Setup Frames and Joints of the simulated robot
            orig_frame = CustomFrame([0; 0; 0], [], 'Origin');
            joint1 = CustomJoint([0; 0; 83.51], orig_frame, 'Joint 1', 'y');
            joint2 = CustomJoint([0;0;0], joint1, 'Joint 2', 'x');
            joint3 = CustomJoint([0;0;119.35], joint2, 'Joint 3', 'z');
            joint4 = CustomJoint([0;0;163.99], joint3, 'Joint 4', 'x');
            endeffector_frame = CustomFrame([0;0;218.86], joint4, 'Endeffector');
            
            %% Setup Links of the simulated robot
            numLinks = 5; % Number of links
            grayLevels = linspace(0.2, 0.8, numLinks);  % Define a range of grayscale values. Start from 0.2 (dark) to 0.8 (lighter)
            
            link1 = CustomLink(orig_frame, joint1, repmat(grayLevels(1), 1, 3));  % Link 1 with first grayscale value
            link2 = CustomLink(joint1, joint2, repmat(grayLevels(2), 1, 3));  % Link 2 with second grayscale value
            link3 = CustomLink(joint2, joint3, repmat(grayLevels(3), 1, 3));  % and so on...
            link4 = CustomLink(joint3, joint4, repmat(grayLevels(4), 1, 3));
            link5 = CustomLink(joint4, endeffector_frame, repmat(grayLevels(5), 1, 3));

            obj.joints =  [joint1, joint2, joint3, joint4];
            obj.links = [link1, link2, link3, link4, link5];
            obj.frames = [orig_frame, endeffector_frame];
        end

        function q = getQ(obj)
            q = vertcat(obj.joints.angle);
        end

        function setQ(obj, q)
            arrayfun(@(joint, angle) joint.setAngle(angle), obj.joints, q');
        end

        function draw(obj, draw_frames)
            % The display method updates and displays all joints and links
            % Activates hold on, no hold off
            if isempty(obj.fig)
                obj.fig = figure;
                hold on
                view(-30, 25);
                axis equal;
                xlabel('X');
                ylabel('Y');
                zlabel('Z');
                title('Simulated Robot');
                grid on;
              
                % Set fixed axis limits
                xlim([-400, 400]);
                ylim([-400, 400]);
                zlim([0, 600]);
            end

            if draw_frames
                arrayfun(@(x) x.draw, obj.joints);
                arrayfun(@(x) x.draw, obj.frames);
            end
            arrayfun(@(x) x.draw, obj.links); 
        end
    end

    methods (Static)

        function singularityBool = checkSingularity(q)
            % Check for singularity
            J = SimulatedRobot.getJacobianNumeric(q);
            pinvJ = pinv(J);

            if norm(J)*norm(pinvJ) > 25
                disp('Warning: Close to singularity');
                singularityBool = true;
            else
                singularityBool = false;
            end
        end

        function [elevation] = getShoulderElevation(q)
            % Calculate the elevation of the shoulder joint in spherical
            % coordinates in RAD from q
            elevation = pi/2 - acos(cos(q(2))*cos(q(1)));           
        end

        function [oxE] = forwardKinematicsNumeric(q)
            R = {SimulatedRobot.roty(q(1)), SimulatedRobot.rotx(q(2)), SimulatedRobot.rotz(q(3)), SimulatedRobot.rotx(q(4))};
            x = [    0         0         0         0         0
                     0         0         0         0         0
                83.5100         0  119.3500  163.9900  218.8600];
            oxE = R{1} * (R{2} * (R{3} * (R{4} * x(:,5) + x(:,4)) + x(:,3)) + x(:,2)) + x(:,1);
        end

        function J = getJacobianNumeric(q)
            % Computes the Jacobian matrix numerically, relating joint velocities to end-effector velocities.
            % Uses finite differences on forward kinematics by perturbing joint angles with 'delta_q'. 
            % Numerical methods may offer speed advantages over symbolic ones, but precision can vary.
            
            % Small change in joint angles
            delta_q = 1e-6;
            
            % Initialize Jacobian matrix
            J = zeros(3, 4);
            
            % For each joint angle
            parfor i = 1:4
                % Perturb joint angle i
                q_plus = q;
                q_plus(i) = q_plus(i) + delta_q;
                q_minus = q;
                q_minus(i) = q_minus(i) - delta_q;
                
                % Compute forward kinematics for q_plus
                oxE_plus = SimulatedRobot.forwardKinematicsNumeric(q_plus);
                
                % Compute forward kinematics for q_minus
                oxE_minus = SimulatedRobot.forwardKinematicsNumeric(q_minus);
                
                % Compute derivative
                J(:, i) = (oxE_plus - oxE_minus) / (2 * delta_q);
            end
        end

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
    end
end


    
