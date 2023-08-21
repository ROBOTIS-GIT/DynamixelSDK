classdef CustomJoint < CustomFrame
    %Joint - A class to represent a joint in 3D space.
    % The Joint object inherits from Frame and holds an additional
    % information about its fixed rotation axis.

    properties
        rotationAxisLabel  % The axis label that this joint can rotate around
        % the axis label refers to the local coordinate system
        angle = 0;
    end

    methods
        function obj = CustomJoint(relativePosition, parent, label, rotationAxisLabel)
            %Joint - Construct a Joint object.
            % Joint(relativePosition, parent, label, rotationAxisLabel) creates a Joint object
            % with the specified relative position, parent frame, label, and rotationAxisLabel.
            % The frame's orientation is initialized to match the parent
            % frame's orientation, or to the identity matrix if no parent 
            % frame is provided. The rotationAxisLabel is initialized with
            % the input rotationAxisLabel and can't be changed afterwards.
            obj = obj@CustomFrame(relativePosition, parent, label); % Call the super class constructor
            obj.rotationAxisLabel = rotationAxisLabel;  % Initialize rotation axis label
        end

        function rotate(obj, angle)
            % This method rotates the joint a given angle around its
            % rotation axis.

            obj.angle = mod(obj.angle + angle, 2*pi);
            % The rotation axis is taken from the rotationAxisLabel property
            switch lower(obj.rotationAxisLabel)
            case 'x'
                axis_vec = obj.rotation(:, 1);
            case 'y'
                axis_vec = obj.rotation(:, 2);
            case 'z'
                axis_vec = obj.rotation(:, 3);
            otherwise
                error('Invalid rotation axis_label. Use ''x'', ''y'', or ''z''.');
            end
        
            obj.rotate_about_axis(angle, axis_vec);
        end

        function setAngle(obj, desiredAngle)
            % This method sets the Angle of the joint by rotating it.

            % Convert the angles to the range [0, 2*pi] to ensure consistency
            desiredAngle = mod(desiredAngle, 2*pi);
            obj.angle = mod(obj.angle, 2*pi);
        
            % Compute the difference between the desired angle and the current angle
            angleDiff = desiredAngle - obj.angle;
        
            % Call the rotate method with the angle difference to set the new angle
            obj.rotate(angleDiff);
        end

    end
end
