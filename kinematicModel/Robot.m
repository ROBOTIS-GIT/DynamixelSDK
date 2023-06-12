classdef Robot < handle
    properties
        joints  % An array of Joint objects, defining the joints of the robot
        links   % An array of Link objects, defining the physical connections between joints
        frames % An array of Frame objects, defining additional frames of the robot
    end

    methods
        % Constructor method for Robot. It takes two arguments, an array
        % of Joint objects and an array of Link objects
        function obj = Robot(joints, links, frames)
            obj.joints = joints;
            obj.links = links;
            obj.frames = frames;
        end

        function [oxE] = forwardKinematics(obj)
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

            % Rotational Matrices
            % Compare Technische Dynamik
            % Central_Exercise_01_Relative_Kinematics.
            % There the rotated Frame is on the left, therefore we have to
            % transpose the Rotational Matrix (or simply use a negative
            % sign before the angle).
            R1 = rotx(-alpha);
            R2 = roty(-beta);
            R3 = rotz(-gamma);
            R4 = rotx(-delta);

            % Distances
            xo1 = obj.joints(1).relativePosition;
            x12 = obj.joints(2).relativePosition;
            x23 = obj.joints(3).relativePosition;
            x34 = obj.joints(4).relativePosition;
            x4E = obj.frames(2).relativePosition;


            oxE  = R1 * (R2 * (R3 * (R4 * x4E + x34) + x23) + x12) + xo1;



        end

        
        % The display method updates and displays all joints and links
        function display(obj)
            figure;
            hold on;
            grid on;
            for i = 1:length(obj.joints)
                obj.joints(i).display();
            end
            for i = 1:length(obj.links)
                obj.links(i).display();
            end
            for i = 1:length(obj.frames)
                obj.frames(i).display();
            end
            view(45, 45);
            axis equal;
            xlabel('X');
            ylabel('Y');
            zlabel('Z');
            title('Robot Joints');
        end
    end
end

function rotx = rotx(alpha)
    
    rotx = [1 0 0; 0 cos(alpha) sin(alpha); 0 -sin(alpha) cos(alpha)];

end

function roty = roty(beta)
    
    roty = [cos(beta) 0 -sin(beta); 0 1 0; sin(beta) 0 cos(beta)];

end

function rotz = rotz(gamma)
    
    rotz = [cos(gamma) sin(gamma) 0; -sin(gamma) cos(gamma) 0; 0 0 1];

end


    
