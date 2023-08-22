classdef CustomFrame < handle
    % CustomFrame - A class to represent a coordinate frame in 3D space.
    % The Frame object holds information about its position and orientation
    % relative to its parent frame. It also contains a list of its child
    % frames. The frame's position is stored in the relativePosition
    % property and the frame's orientation is stored as a 3x3 rotation 
    % matrix in the rotation property. The orientation is represented in 
    % the origin frame, i.e., it's a global rotation.
    properties
        relativePosition  % Position vector relative to the parent frame (pPf)
        rotation          % Orientation of the frame represented as a 3x3 rotation matrix in global frame (gRf)
        parent            % Reference to the parent Frame object (p)
        children          % Array of references to the child Frame objects (c)
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
        
        function rotate(obj, angle, axis_label)
            % This method rotates the frame around one of its axes by a certain
            % angle (in radians). The rotation axis is given by the 'axis_label'
            % parameter and is in the frame's local coordinates.
            
            % Determine the axis vector based on the axis_label
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
            
            % Compute the rotation matrix around the frames local axis using Rodrigues' rotation formula
            % Rodrigues' rotation formula is used to rotate a vector in 3D space.
            % Given an angle 'angle' and a rotation axis 'axis_vec', the formula
            % calculates a rotation matrix 'rotMatrix'. This matrix, when multiplied
            % with the original vector, gives the rotated vector.
            % The formula uses trigonometric functions (cos and sin) and the axis
            % vector components to compute the elements of 'rotMatrix'.
            % 'c' is cos(angle), 's' is sin(angle), and 't' is 1 - cos(angle).
            % 'x', 'y', and 'z' are the components of the axis vector 'axis_vec'.
            % The formula ensures a smooth and computationally efficient rotation.

            % Another approach would be to find the beginning of the
            % kinematic chain (global/fixed frame) and update the complete
            % forward kinematics using a composition of basis rotations.
            % However this would require storing the basic rotation angles of
            % every frame in the kinematic chain and computing the whole
            % chain even when only the last frames would be affected by the
            % rotation.

            % Quaternions could also be used.

            c = cos(angle);
            s = sin(angle);
            t = 1 - c;
            x = axis_vec(1);
            y = axis_vec(2);
            z = axis_vec(3);
            
            rotMatrix = [t*x*x + c,   t*x*y - s*z, t*x*z + s*y;
                         t*x*y + s*z, t*y*y + c,   t*y*z - s*x;
                         t*x*z - s*y, t*y*z + s*x, t*z*z + c];

            
            
            % Apply the rotation to the frame
            obj.rotation = rotMatrix * obj.rotation;
            
            % Apply the same rotation to all descendent frames recursively
            applyRotationToDescendants(obj, rotMatrix);
        end
        
        function applyRotationToDescendants(obj, rotMatrix)
            % Recursive function to apply the rotation to all descendant frames
            for i = 1:length(obj.children)
                child = obj.children(i);
                child.rotation = rotMatrix * child.rotation;
                applyRotationToDescendants(child, rotMatrix);
            end
        end

        function display(obj)
            % This method just plots the frame in a 3D plot
            obj.draw_frame(obj.rotation, obj.getGlobalPosition(), obj.label);
        end
    
        function [ref_position, ref_rotation] = getInfo(obj,display_info, ref_frame)
            % This method provides information about a frame. The position
            % and rotation is returned relative to a given ref_frame.

            % If the ref_frame argument is missing or empty, use global frame
            if nargin < 3 || isempty(ref_frame)
                ref_frame_label = 'global frame';
                ref_position = obj.getGlobalPosition;
                ref_rotation = obj.rotation;
            else
                % Compute position and rotation in the reference frame
                ref_frame_label = ref_frame.label;

                % The position of the frame represented in the reference
                % frame (rPf) is the vector you need to go from the origin
                % of the reference frame, in the coordinate systeme of the
                % reference frame, to reach the frames origin.

                %The position of the frame (rPf) is the global position of the frame (gPf) 
                % subtracted by the global position of the reference frame (gPr)
                % represented in the reference frame.

                % rPf = gRr' * (gPf - gPr) 
                ref_position = ref_frame.rotation' * (obj.getGlobalPosition - ref_frame.getGlobalPosition);

                % rRf = gRr' * gRf
                ref_rotation = ref_frame.rotation' * obj.rotation;
            end
                    
            if display_info
                fprintf('Frame: %s\n', obj.label);
                fprintf('Position in %s FoR: [%f, %f, %f]\n', ref_frame_label, ref_position);
                fprintf('Rotation in %s FoR:\n', ref_frame_label);
                disp(ref_rotation);
        
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
                fprintf('\n\n');
            end
    
            return
        end
      
        function draw_frame(obj, rotation, globalPosition, label)
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
            
            % Plot each axis and save the graphics handle
            obj.xHandle = quiver3(globalPosition(1), globalPosition(2), globalPosition(3), scale_factor*rotation(1,1), scale_factor*rotation(2,1), scale_factor*rotation(3,1), 'Color', colors(1));
            obj.yHandle = quiver3(globalPosition(1), globalPosition(2), globalPosition(3), scale_factor*rotation(1,2), scale_factor*rotation(2,2), scale_factor*rotation(3,2), 'Color', colors(2));
            obj.zHandle = quiver3(globalPosition(1), globalPosition(2), globalPosition(3), scale_factor*rotation(1,3), scale_factor*rotation(2,3), scale_factor*rotation(3,3), 'Color', colors(3));
            
            % Plot color legend and save the graphics handle
            obj.xTextHandle = text(globalPosition(1) + scale_factor*rotation(1,1), globalPosition(2) + scale_factor*rotation(2,1), globalPosition(3) + scale_factor*rotation(3,1), axis_labels{1}, 'Color', colors(1), 'FontWeight', 'bold'); 
            obj.yTextHandle = text(globalPosition(1) + scale_factor*rotation(1,2), globalPosition(2) + scale_factor*rotation(2,2), globalPosition(3) + scale_factor*rotation(3,2), axis_labels{2}, 'Color', colors(2), 'FontWeight', 'bold'); 
            obj.zTextHandle = text(globalPosition(1) + scale_factor*rotation(1,3), globalPosition(2) + scale_factor*rotation(2,3), globalPosition(3) + scale_factor*rotation(3,3), axis_labels{3}, 'Color', colors(3), 'FontWeight', 'bold'); 
            
            % Plot label and save the graphics handle
            obj.textHandle = text(globalPosition(1), globalPosition(2), globalPosition(3), label);
        end
    end
end
