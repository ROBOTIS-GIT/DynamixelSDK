classdef CustomJoint < CustomFrame
    properties
        rotationAxisLabel  % Axis label that this joint rotates around
        angle = 0;        % Joint Angle in the local coordinate system
    end

    methods
        function obj = CustomJoint(relativePosition, parent, label, rotationAxisLabel)
            obj = obj@CustomFrame(relativePosition, parent, label);
            
            if ~any(strcmpi(rotationAxisLabel, {'x', 'y', 'z'}))
                error('Invalid rotation axis label. Use ''x'', ''y'', or ''z''.');
            end
            obj.rotationAxisLabel = rotationAxisLabel;
        end

        function setAngle(obj, desiredAngle)
            obj.angle = mod(desiredAngle + 2*pi, 4*pi) - 2*pi;  % Convert the angles to the range [-2pi, 2pi]
            obj.updateRotation;
        end


        function updateRotation(obj)
            v_A_i = obj.getRotationMatrixForAngle();
            
            if isempty(obj.parent)
                obj.rotation = eye(3) * v_A_i;
            else
                obj.rotation = obj.parent.rotation * v_A_i;
            end
        
            for child = obj.children
                child.rotation = child.parent.rotation;
                
                if isa(child, 'CustomJoint')
                    child.rotation = child.rotation * child.getRotationMatrixForAngle();
                    child.updateRotation();
                end
            end
        end

        function rotMatrix = getRotationMatrixForAngle(obj)
            switch lower(obj.rotationAxisLabel)
                case 'x'
                    rotMatrix = rotx(obj.angle);
                case 'y'
                    rotMatrix = roty(obj.angle);
                case 'z'
                    rotMatrix = rotz(obj.angle);
            end
        end
    end
end


%% Definition of the basis rotation matrices
function rotx = rotx(alpha)
    rotx = [1 0 0; 0 cos(alpha) -sin(alpha); 0 sin(alpha) cos(alpha)];
end

function roty = roty(beta)
    roty = [cos(beta) 0 sin(beta); 0 1 0; -sin(beta) 0 cos(beta)];
end

function rotz = rotz(gamma)
    rotz = [cos(gamma) -sin(gamma) 0; sin(gamma) cos(gamma) 0; 0 0 1];
end
