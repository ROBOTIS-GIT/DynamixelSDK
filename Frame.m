classdef Frame < handle
    properties
        position
        rotation
        parent
        children
        label
        xHandle
        yHandle
        zHandle
        textHandle
        xTextHandle % Added graphics handle for X label
        yTextHandle % Added graphics handle for Y label
        zTextHandle % Added graphics handle for Z label
    end
    methods
        function obj = Frame(position, rotation, parent, label)
            if nargin > 0
                obj.position = position;
                if isempty(parent)
                    obj.rotation = rotation;
                else
                    obj.rotation = parent.rotation * rotation;
                    parent.children = [parent.children; obj];
                end
                obj.parent = parent;
                obj.label = label;
            end
            obj.children = [];
        end
        
function rotate(obj, angle, axis)
    rot = obj.rotate_about_axis(angle, axis);
    oldRotation = obj.rotation;
    obj.rotation = rot * obj.rotation;

    for i = 1:length(obj.children)
        child = obj.children(i);
        child.position = obj.position + obj.rotation * inv(oldRotation) * (child.position - obj.position);
        child.rotation = obj.rotation * inv(oldRotation) * child.rotation;  % rotate child's frame relative to parent's rotation
    end
end

        
        function display(obj)
            fprintf('Frame: %s\n', obj.label);
            fprintf('Position: [%f, %f, %f]\n', obj.position);
            fprintf('Rotation:\n');
            disp(obj.rotation);
            obj.draw_frame(obj.rotation, obj.position, obj.label);
            if ~isempty(obj.parent)
                fprintf('%s has parent frame: %s\n', obj.label, obj.parent.label);
            else
                fprintf('%s has no parent frame\n', obj.label);
            end
            if ~isempty(obj.children)
                fprintf('%s has child frames: ', obj.label);
                for i = 1:length(obj.children)
                    if i == length(obj.children)
                        fprintf('%s\n', obj.children(i).label);
                    else
                        fprintf('%s, ', obj.children(i).label);
                    end
                end
            else
                fprintf('%s has no child frames\n', obj.label);
            end
        end
        
        function rot = rotate_about_axis(obj, angle, axis)
            switch lower(axis)
                case 'x'
                    axis_vec = obj.rotation(:, 1);
                case 'y'
                    axis_vec = obj.rotation(:, 2);
                case 'z'
                    axis_vec = obj.rotation(:, 3);
                otherwise
                    error('Invalid rotation axis. Use ''x'', ''y'', or ''z''.');
            end

            c = cos(angle);
            s = sin(angle);
            t = 1 - c;
            x = axis_vec(1);
            y = axis_vec(2);
            z = axis_vec(3);
            
            % Using Rodrigues' rotation formula
            rot = [t*x*x + c,   t*x*y - s*z, t*x*z + s*y;
                   t*x*y + s*z, t*y*y + c,   t*y*z - s*x;
                   t*x*z - s*y, t*y*z + s*x, t*z*z + c];
        end

        function draw_frame(obj, frame, position, label)
            colors = ['r', 'g', 'b'];
            axis_labels = {'X', 'Y', 'Z'};
            
            % Delete the previous plotted objects before replotting
            if ~isempty(obj.xHandle) && isvalid(obj.xHandle)
                delete(obj.xHandle);
            end
            if ~isempty(obj.yHandle) && isvalid(obj.yHandle)
                delete(obj.yHandle);
            end
            if ~isempty(obj.zHandle) && isvalid(obj.zHandle)
                delete(obj.zHandle);
            end
            if ~isempty(obj.textHandle) && isvalid(obj.textHandle)
                delete(obj.textHandle);
            end
            if ~isempty(obj.xTextHandle) && isvalid(obj.xTextHandle) % Added
                delete(obj.xTextHandle);
            end
            if ~isempty(obj.yTextHandle) && isvalid(obj.yTextHandle) % Added
                delete(obj.yTextHandle);
            end
            if ~isempty(obj.zTextHandle) && isvalid(obj.zTextHandle) % Added
                delete(obj.zTextHandle);
            end
            
            % Plot each axis and save the graphics handle
            obj.xHandle = quiver3(position(1), position(2), position(3), frame(1,1), frame(2,1), frame(3,1), 'Color', colors(1), 'LineWidth', 2, 'MaxHeadSize', 0.3, 'AutoScale', 'off');
            obj.yHandle = quiver3(position(1), position(2), position(3), frame(1,2), frame(2,2), frame(3,2), 'Color', colors(2), 'LineWidth', 2, 'MaxHeadSize', 0.3, 'AutoScale', 'off');
            obj.zHandle = quiver3(position(1), position(2), position(3), frame(1,3), frame(2,3), frame(3,3), 'Color', colors(3), 'LineWidth', 2, 'MaxHeadSize', 0.3, 'AutoScale', 'off');
            
            % Plot color legend and save the graphics handle
            obj.xTextHandle = text(position(1) + frame(1,1), position(2) + frame(2,1), position(3) + frame(3,1), axis_labels{1}, 'Color', colors(1), 'FontWeight', 'bold'); % Modified
            obj.yTextHandle = text(position(1) + frame(1,2), position(2) + frame(2,2), position(3) + frame(3,2), axis_labels{2}, 'Color', colors(2), 'FontWeight', 'bold'); % Modified
            obj.zTextHandle = text(position(1) + frame(1,3), position(2) + frame(2,3), position(3) + frame(3,3), axis_labels{3}, 'Color', colors(3), 'FontWeight', 'bold'); % Modified
            
            % Plot label and save the graphics handle
            obj.textHandle = text(position(1), position(2), position(3), label);
        end
    end
end