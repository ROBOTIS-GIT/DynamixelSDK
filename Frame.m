classdef Frame < handle
    properties
        relativePosition
        rotation
        parent
        children
        label
        xHandle
        yHandle
        zHandle
        textHandle
        xTextHandle
        yTextHandle
        zTextHandle
    end

    methods
        function obj = Frame(relativePosition, parent, label)
            if nargin > 0
                obj.relativePosition = relativePosition;
                if isempty(parent)
                    obj.rotation = eye(3);
                else
                    obj.rotation = parent.rotation;
                    parent.children = [parent.children; obj];
                end
                obj.parent = parent;
                obj.label = label;
            end
            obj.children = [];
        end
        
        function pos = getGlobalPosition(obj)
            if isempty(obj.parent)
                pos = obj.relativePosition;
            else
                pos = obj.parent.getGlobalPosition() + obj.parent.rotation * obj.relativePosition;
            end
        end

        function rotate(obj, angle, axis_label)

            switch lower(axis_label)
            case 'x'
                axis_vec = obj.rotation(:, 1);
            case 'y'
                axis_vec = obj.rotation(:, 2);
            case 'z'
                axis_vec = obj.rotation(:, 3);
            otherwise
                error('Invalid rotation axis_label. Use ''x'', ''y'', or ''z''.');
            end
            
            rotate_about_axis(obj, angle, axis_vec)


        end

          function [ref_position, ref_rotation, ref_frame] = display(obj, ref_frame)
            if nargin < 2
                % If no reference frame is given, use global frame
                ref_frame_label = 'global frame';
                ref_frame = [];
                ref_position = obj.getGlobalPosition;
                ref_rotation = obj.rotation;
            else
                % Compute position and rotation in the reference frame
                ref_frame_label = ref_frame.label;
                ref_position = ref_frame.rotation' * (obj.getGlobalPosition - ref_frame.getGlobalPosition);
                ref_rotation = ref_frame.rotation' * obj.rotation;
            end

            fprintf('Frame: %s\n', obj.label);
            fprintf('Position in %s FoR: [%f, %f, %f]\n', ref_frame_label, ref_position);
            fprintf('Rotation in %s FoR:\n', ref_frame_label);
            disp(ref_rotation);
            obj.draw_frame(obj.rotation, obj.getGlobalPosition(), obj.label);
            
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

        
        function rotMatrix = rotate_about_axis(obj, angle, axis_vec)

            c = cos(angle);
            s = sin(angle);
            t = 1 - c;
            x = axis_vec(1);
            y = axis_vec(2);
            z = axis_vec(3);
            
            % Using Rodrigues' rotation formula
            rotMatrix = [t*x*x + c,   t*x*y - s*z, t*x*z + s*y;
                   t*x*y + s*z, t*y*y + c,   t*y*z - s*x;
                   t*x*z - s*y, t*y*z + s*x, t*z*z + c];


            obj.rotation = rotMatrix * obj.rotation;
            
            for i = 1:length(obj.children)
                child = obj.children(i);
                child.rotate_about_axis(angle, axis_vec)
            end
        end

        function draw_frame(obj, frame, position, label)
            colors = ['r', 'g', 'b'];
            axis_labels = {'X', 'Y', 'Z'};
            
            max_robot_dimension = 500;  % Update this based on your robot size
            scale_factor = max_robot_dimension / 20;  % Scale factor for the frames
            
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
            obj.xHandle = quiver3(position(1), position(2), position(3), scale_factor*frame(1,1), scale_factor*frame(2,1), scale_factor*frame(3,1), 'Color', colors(1));
            obj.yHandle = quiver3(position(1), position(2), position(3), scale_factor*frame(1,2), scale_factor*frame(2,2), scale_factor*frame(3,2), 'Color', colors(2));
            obj.zHandle = quiver3(position(1), position(2), position(3), scale_factor*frame(1,3), scale_factor*frame(2,3), scale_factor*frame(3,3), 'Color', colors(3));
            
            % Plot color legend and save the graphics handle
            obj.xTextHandle = text(position(1) + scale_factor*frame(1,1), position(2) + scale_factor*frame(2,1), position(3) + scale_factor*frame(3,1), axis_labels{1}, 'Color', colors(1), 'FontWeight', 'bold'); 
            obj.yTextHandle = text(position(1) + scale_factor*frame(1,2), position(2) + scale_factor*frame(2,2), position(3) + scale_factor*frame(3,2), axis_labels{2}, 'Color', colors(2), 'FontWeight', 'bold'); 
            obj.zTextHandle = text(position(1) + scale_factor*frame(1,3), position(2) + scale_factor*frame(2,3), position(3) + scale_factor*frame(3,3), axis_labels{3}, 'Color', colors(3), 'FontWeight', 'bold'); 
            
            % Plot label and save the graphics handle
            obj.textHandle = text(position(1), position(2), position(3), label);
        end
    end
end

