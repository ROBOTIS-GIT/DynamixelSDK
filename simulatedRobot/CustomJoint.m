classdef CustomJoint < CustomFrame
    %Joint - A class to represent a joint in 3D space.
    % The Joint object inherits from Frame and holds an additional
    % information about its fixed rotation axis. It also provides
    % functionality to update the rotation of all frames in the kinematic
    % chain.

    properties
        rotationAxisLabel  % The axis label that this joint can rotate around
        angle = 0; % Joint Angle in the local coordinate system
    end

    methods
        function obj = CustomJoint(relativePosition, parent, label, rotationAxisLabel)
            %A joint is just a Frame with the ability to rotate around a fixed rotation axis.
            %Descendent Frames/Joints are rotated.
            %rotation axis label defines around which local axis the joint
            %is allowed to rotate. The rotation angle is stored.
            obj = obj@CustomFrame(relativePosition, parent, label); % Call the super class constructor
            obj.rotationAxisLabel = rotationAxisLabel;  % Initialize rotation axis label
        end

        function setAngle(obj, desiredAngle)
            % This method sets the Angle of the joint and udpates all
            % rotation matrizes of the kinematically dependent
            % joints/frames

            % Convert the angles to the range [0, 2*pi] to ensure consistency
            desiredAngle = mod(desiredAngle, 2*pi);

            % Set the angle
            obj.angle = desiredAngle;
                
            % Update the kinematic chain
            obj.updateRotation;
    
        end

        function updateRotation(obj)
            % Determine the corresponding basis rotation matrix
            switch lower(obj.rotationAxisLabel)
                        case 'x'
                            v_A_i = rotx(obj.angle);
                        case 'y'
                            v_A_i = roty(obj.angle);
                        case 'z'
                            v_A_i = rotz(obj.angle);
                        otherwise
                            error('Invalid rotation axis_label. Use ''x'', ''y'', or ''z''.');
            end
            
            % Update the Frames orientation relative to its parent
            if isempty(obj.parent)
                parentRotation = eye(3);
            else
                parentRotation = obj.parent.rotation;
            end
            obj.rotation = parentRotation * v_A_i;

            % Recursive function to apply the rotation to all descendant frames
            updateChildren(obj);
            function updateChildren(obj)
                % Update all children if there are multiple
                for i = 1:length(obj.children)
                    child = obj.children(i);
                    % Calculate the childs rotation if it is a joint: Only
                    % then it can have a different orientation then the
                    % parent.
                    if isa(child, 'CustomJoint')
                        switch lower(child.rotationAxisLabel)
                        case 'x'
                            v_A_i = rotx(child.angle);
                        case 'y'
                            v_A_i = roty(child.angle);
                        case 'z'
                            v_A_i = rotz(child.angle);
                        otherwise
                            error('Invalid rotation axis_label. Use ''x'', ''y'', or ''z''.');
                        end
                        child.rotation = child.parent.rotation * v_A_i;
                    else
                        child.rotation = child.parent.rotation;
                    end
                    updateChildren(child);
                end
            end
        end
    end
end


%% Definition of the basis rotation matrizes

function rotx = rotx(alpha)
    rotx = [1 0 0; 0 cos(alpha) -sin(alpha); 0 sin(alpha) cos(alpha)];
end

function roty = roty(beta)
    roty = [cos(beta) 0 sin(beta); 0 1 0; -sin(beta) 0 cos(beta)];
end

function rotz = rotz(gamma)
    rotz = [cos(gamma) -sin(gamma) 0; sin(gamma) cos(gamma) 0; 0 0 1];
end