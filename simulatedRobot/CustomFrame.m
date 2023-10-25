classdef CustomFrame < handle

    % CustomFrame - A class to visualize a coordinate frame in 3D space.
    % The Frame object holds information about its position and orientation.
    % It also contains a list of its child frames and its parent.
    % The frame's position is stored in the relativePosition
    % property and the frame's orientation is stored as a 3x3 rotation 
    % matrix in the rotation property. The orientation is represented in 
    % the origin frame, i.e., it's a global rotation.

    properties
        relativePosition  % Position vector relative to the parent frame, in parent frame (prf)
        rotation          % Orientation of the frame represented as a 3x3 rotation matrix in global frame (gAf)
        parent            % Reference to the parent Frame object
        children          % Array of references to the child Frame objects
        label             % String label for the frame

        % The graphic handles are used for modifying the existing plot of
        % the frame instead of redrawing it every time which is faster.
        xHandle           % Graphics handle for X axis
        yHandle           % Graphics handle for Y axis
        zHandle           % Graphics handle for Z axis
        xTextHandle       % Graphics handle for X axis label
        yTextHandle       % Graphics handle for Y axis label
        zTextHandle       % Graphics handle for Z axis label
        textHandle        % Graphics handle for the frame label

    end

    methods
        function obj = CustomFrame(relativePosition, parent, label)
           %Frame - Construct a Frame object.
            % Frame(relativePosition, parent, label) creates a Frame object
            % with the specified relative position, parent frame, and label.
            % The frame's orientation is initialized to match the parent
            % frame's orientation, or to the identity matrix if no parent 
            % frame is provided.
            if nargin > 0
                obj.relativePosition = relativePosition;
                if isempty(parent)
                    % gRf = identity
                    obj.rotation = eye(3);
                else
                    % gRf = gRp
                    obj.rotation = parent.rotation;
                    % Register this frame as a child of the specified
                    % parent frame
                    parent.children = [parent.children; obj];
                end
                obj.parent = parent;
                obj.label = label;
            end
            obj.children = [];
        end
        
        function pos = getGlobalPosition(obj)
            %getGlobalPosition - Calculate the frame's position in the global frame.
            % pos = getGlobalPosition(obj) returns a 3x1 vector specifying 
            % the frame's position in the global frame.

            % Recursevily iterate backwards thorugh the kinematic chain
            if isempty(obj.parent)
                % gPf = pPf
                pos = obj.relativePosition;
            else
                % gPf = gPp + gRp * pPf
                pos = obj.parent.getGlobalPosition() + obj.parent.rotation * obj.relativePosition;
            end
        end

        function draw(obj)
            colors = ['r', 'g', 'b'];
            axis_labels = {'X', 'Y', 'Z'};
            max_robot_dimension = 500;  % Update based on your robot size
            scale_factor = max_robot_dimension / 10;
            
            globalPosition = obj.getGlobalPosition;
            
            handles = {obj.xHandle, obj.yHandle, obj.zHandle, obj.xTextHandle, obj.yTextHandle, obj.zTextHandle, obj.textHandle};
            
            for i = 1:3
                % Calculate the end position of the line for the current axis
                endPos = globalPosition + scale_factor * obj.rotation(:, i);
                
                % Check if the handle exists and is valid
                if isempty(handles{i}) || ~isvalid(handles{i})
                    % Create a new line using plot3
                    handles{i} = plot3([globalPosition(1), endPos(1)], ...
                                       [globalPosition(2), endPos(2)], ...
                                       [globalPosition(3), endPos(3)], ...
                                       'Color', colors(i), LineWidth=2);
                else
                    % Update the existing line's data
                    handles{i}.XData = [globalPosition(1), endPos(1)];
                    handles{i}.YData = [globalPosition(2), endPos(2)];
                    handles{i}.ZData = [globalPosition(3), endPos(3)];
                end
        
                % Update the text data for axis labels
                % if isempty(handles{i+3}) || ~isvalid(handles{i+3})
                %     handles{i+3} = text(endPos(1), endPos(2), endPos(3), axis_labels{i}, 'Color', colors(i), 'FontWeight', 'bold');
                % else
                %     handles{i+3}.Position = endPos;
                % end
            end
        
            % Assign the updated handles back to the object properties
            [obj.xHandle, obj.yHandle, obj.zHandle, obj.xTextHandle, obj.yTextHandle, obj.zTextHandle, ~] = handles{:};
        end
    end
end