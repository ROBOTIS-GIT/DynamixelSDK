classdef CustomJoint < CustomFrame
    %Joint - A class to represent a joint in 3D space.
    % The Joint object inherits from Frame and holds an additional
    % information about its fixed rotation axis.

    properties
        rotationAxisLabel  % The axis label that this joint can rotate around
        % the axis label refers to the local coordinate system
        angle = 0; % Joint Angle in the local coordinate system
    end

    methods
        function obj = CustomJoint(relativePosition, parent, label, rotationAxisLabel)
            %A joint is just a Frame with a fixed rotation axis.
            %rotation axis label defines around which local axis the joint
            %is allowed to rotate. The rotation angle is stored.
            obj = obj@CustomFrame(relativePosition, parent, label); % Call the super class constructor
            obj.rotationAxisLabel = rotationAxisLabel;  % Initialize rotation axis label
        end

        function rotate(obj, angle)
            % This method rotates the joint a given angle around its
            % rotation axis.

            obj.angle = mod(obj.angle + angle, 2*pi);
       
            obj.rotate@CustomFrame(angle, obj.rotationAxisLabel);
        end

        function setAngle(obj, desiredAngle)
            % This method sets the Angle of the joint by rotating it.

            % Convert the angles to the range [0, 2*pi] to ensure consistency
            desiredAngle = mod(desiredAngle, 2*pi);
        
            % Compute the difference between the desired angle and the current angle
            angleDiff = desiredAngle - obj.angle;
        
            % Call the rotate method with the angle difference to set the new angle
            obj.rotate(angleDiff);
        end

    end
end
