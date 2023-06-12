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
