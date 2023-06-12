% The Robot class defines a robot manipulator model.
% The robot is composed of a series of Frames and Links. It includes
% methods for creating and displaying the complete robot structure.
classdef Robot < handle
    properties
        frames  % An array of Frame objects, defining the joints of the robot
        links   % An array of Link objects, defining the physical connections between joints
    end

    methods
        % Constructor method for Robot. It takes two arguments, an array
        % of Frame objects and an array of Link objects
        function obj = Robot(frames, links)
            obj.frames = frames;
            obj.links = links;
        end
        
        % The display method updates and displays all frames and links
        function display(obj)
            figure;
            hold on;
            grid on;
            for i = 1:length(obj.frames)
                obj.frames(i).display();
            end
            for i = 1:length(obj.links)
                obj.links(i).display();
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
