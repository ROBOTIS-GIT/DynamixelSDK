classdef CustomFrame < handle

    properties
        relativePosition  % Position vector relative to the parent frame
        rotation          % Orientation of the frame represented as a 3x3 rotation matrix
        parent            % Reference to the parent Frame object
        children          % Array of references to the child Frame objects
        label             % String label for the frame

        % Graphics handles in arrays
        axisHandles       % Graphics handles for X, Y, Z axes
        axisTextHandles   % Graphics handles for X, Y, Z axis labels
        textHandle        % Graphics handle for the frame label
    end
    properties(Constant)
        AXES_COLORS = ['r', 'g', 'b'];
        AXIS_LABELS = {'X', 'Y', 'Z'};
        MAX_ROBOT_DIMENSION = 500; % Update based on your robot size
    end

    methods
        function obj = CustomFrame(relativePosition, parent, label)
            if nargin > 0
                obj.relativePosition = relativePosition;
                if isempty(parent)
                    obj.rotation = eye(3);
                else
                    obj.rotation = parent.rotation;
                end
                obj.parent = parent;
                obj.label = label;
                if ~isempty(parent)
                    parent.children = [parent.children; obj];
                end
            end
            obj.children = [];
            
            % Initialize graphics handle arrays with empty handles
            obj.axisHandles = [gobjects(1,1), gobjects(1,1), gobjects(1,1)];
            obj.axisTextHandles = [gobjects(1,1), gobjects(1,1), gobjects(1,1)];
        end

        function pos = getGlobalPosition(obj)
            if isempty(obj.parent)
                pos = obj.relativePosition;
            else
                pos = obj.parent.getGlobalPosition() + obj.parent.rotation * obj.relativePosition;
            end
        end

        function draw(obj)
            scale_factor = obj.MAX_ROBOT_DIMENSION / 10;
            globalPosition = obj.getGlobalPosition();
            
            for i = 1:3
                endPos = globalPosition + scale_factor * obj.rotation(:, i);
                
                % Draw or update the axis line
                obj.axisHandles(i) = CustomFrame.drawLineOrRefresh(obj.axisHandles(i), globalPosition, endPos, obj.AXES_COLORS(i));
    
                % Draw or update the axis label
                % obj.axisTextHandles(i) = CustomFrame.drawTextOrRefresh(obj.axisTextHandles(i), endPos, obj.AXIS_LABELS{i}, obj.AXES_COLORS(i));
            end
        end
    end

    methods(Static)
        function handle = drawLineOrRefresh(handle, startPos, endPos, color)
            if isempty(handle) || ~isvalid(handle) || isa(handle, 'matlab.graphics.GraphicsPlaceholder')
                handle = plot3([startPos(1), endPos(1)], ...
                               [startPos(2), endPos(2)], ...
                               [startPos(3), endPos(3)], ...
                               'Color', color, 'LineWidth', 2);
            else
                handle.XData = [startPos(1), endPos(1)];
                handle.YData = [startPos(2), endPos(2)];
                handle.ZData = [startPos(3), endPos(3)];
            end
        end
    
        function handle = drawTextOrRefresh(handle, position, label, color)
            if isempty(handle) || ~isvalid(handle)
                handle = text(position(1), position(2), position(3), label, 'Color', color, 'FontWeight', 'bold');
            else
                handle.Position = position;
            end
        end
    end
end
