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
        textHandle        % Graphics handle for the frame label
        xTextHandle       % Graphics handle for X axis label
        yTextHandle       % Graphics handle for Y axis label
        zTextHandle       % Graphics handle for Z axis label
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

        function display(obj)

            % This method draws the frame in a 3D plot. It uses the 'quiver3' and
            % 'text' functions to draw the frame's axes and labels, respectively.
            % The 'rotation' and 'globalPosition' arguments provide the rotation
            % matrix and global position of the frame. The 'label' argument is the
            % label of the frame.
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

            globalPosition = obj.getGlobalPosition;
            
            % Plot each axis and save the graphics handle
            obj.xHandle = quiver3(globalPosition(1), globalPosition(2), globalPosition(3), scale_factor*obj.rotation(1,1), scale_factor*obj.rotation(2,1), scale_factor*obj.rotation(3,1), 'Color', colors(1));
            obj.yHandle = quiver3(globalPosition(1), globalPosition(2), globalPosition(3), scale_factor*obj.rotation(1,2), scale_factor*obj.rotation(2,2), scale_factor*obj.rotation(3,2), 'Color', colors(2));
            obj.zHandle = quiver3(globalPosition(1), globalPosition(2), globalPosition(3), scale_factor*obj.rotation(1,3), scale_factor*obj.rotation(2,3), scale_factor*obj.rotation(3,3), 'Color', colors(3));
            
            % Plot color legend and save the graphics handle
            obj.xTextHandle = text(globalPosition(1) + scale_factor*obj.rotation(1,1), globalPosition(2) + scale_factor*obj.rotation(2,1), globalPosition(3) + scale_factor*obj.rotation(3,1), axis_labels{1}, 'Color', colors(1), 'FontWeight', 'bold'); 
            obj.yTextHandle = text(globalPosition(1) + scale_factor*obj.rotation(1,2), globalPosition(2) + scale_factor*obj.rotation(2,2), globalPosition(3) + scale_factor*obj.rotation(3,2), axis_labels{2}, 'Color', colors(2), 'FontWeight', 'bold'); 
            obj.zTextHandle = text(globalPosition(1) + scale_factor*obj.rotation(1,3), globalPosition(2) + scale_factor*obj.rotation(2,3), globalPosition(3) + scale_factor*obj.rotation(3,3), axis_labels{3}, 'Color', colors(3), 'FontWeight', 'bold'); 
            
            % Plot label and save the graphics handle
            obj.textHandle = text(globalPosition(1), globalPosition(2), globalPosition(3), obj.label);
        end
    end
end