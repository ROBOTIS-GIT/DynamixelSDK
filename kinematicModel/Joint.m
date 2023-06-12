classdef Joint < Frame
    %Joint - A class to represent a joint in 3D space.
    % The Joint object inherits from Frame and holds an additional
    % information about its fixed rotation axis.

    properties
        rotationAxisLabel  % The axis label that this joint can rotate around
        angle = 0;
    end

    methods
        function obj = Joint(relativePosition, parent, label, rotationAxisLabel)
            %Joint - Construct a Joint object.
            % Joint(relativePosition, parent, label, rotationAxisLabel) creates a Joint object
            % with the specified relative position, parent frame, label, and rotationAxisLabel.
            % The frame's orientation is initialized to match the parent
            % frame's orientation, or to the identity matrix if no parent 
            % frame is provided. The rotationAxisLabel is initialized with
            % the input rotationAxisLabel and can't be changed afterwards.
            obj = obj@Frame(relativePosition, parent, label); % Call the super class constructor
            obj.rotationAxisLabel = rotationAxisLabel;  % Initialize rotation axis label
        end

        function rotate(obj, angle)
            obj.angle = obj.angle + angle;
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
    end
end
