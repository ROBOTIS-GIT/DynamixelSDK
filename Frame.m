classdef Frame < handle
    properties
        position
        rotation
        parent
        children
    end
    methods
        function obj = Frame(position, rotation, parent)
            if nargin > 0
                obj.position = position;
                if isempty(parent)
                    obj.rotation = rotation;
                else
                    obj.rotation = parent.rotation * rotation;
                    parent.children = [parent.children; obj];
                end
                obj.parent = parent;
            end
            obj.children = [];
        end
        
        function rotate(obj, angle, axis)
        rot = rotate_about_axis(angle, axis);
        obj.rotation = rot * obj.rotation;
        
        % Adjust child frames
        for i = 1:length(obj.children)
            child = obj.children(i);
            child.position = obj.position + obj.rotation * (child.position - obj.position);
            child.rotate(angle, axis);
        end
    end
        
        function display(obj, label)
            draw_frame(obj.rotation, obj.position, label);
            for i = 1:length(obj.children)
                obj.children(i).display([label, ' child ', num2str(i)]);
            end
        end
    end
end

function rot = rotate_about_axis(angle, axis)
    if axis == 'x' || axis == 'X'
        rot = [1, 0, 0;
               0, cos(angle), -sin(angle);
               0, sin(angle), cos(angle)];
    elseif axis == 'y' || axis == 'Y'
        rot = [cos(angle), 0, sin(angle);
               0, 1, 0;
               -sin(angle), 0, cos(angle)];
    elseif axis == 'z' || axis == 'Z'
        rot = [cos(angle), -sin(angle), 0;
               sin(angle), cos(angle), 0;
               0, 0, 1];
    else
        error('Invalid rotation axis. Use ''x'', ''y'', or ''z''.');
    end
end

function draw_frame(frame, position, label)
    colors = ['r', 'g', 'b'];
    axis_labels = {'X', 'Y', 'Z'};
    
    % Plot each axis
    for i = 1:3
        quiver3(position(1), position(2), position(3), ...
                frame(1,i), frame(2,i), frame(3,i), ...
                'Color', colors(i), 'LineWidth', 2, 'MaxHeadSize', 0.3, 'AutoScale', 'off');
        
        % Plot color legend
        text(position(1) + frame(1,i), position(2) + frame(2,i), position(3) + frame(3,i), axis_labels{i}, ...
            'Color', colors(i), 'FontWeight', 'bold');
    end
    text(position(1), position(2), position(3), label);
end
