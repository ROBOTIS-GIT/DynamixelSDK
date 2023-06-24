classdef BevelGear < handle

% The BevelGear class is designed to control a bevel gear setup composed of two servos.
% It assumes that the bevel gear is in a fixed position and that the two servos
% are mounted in a way that allows the gear to rotate around two axes, X and Y.
% The Y-axis is assumed to be fixed, while the X-axis is dependent on the Y-axis.
%
% The servos are controlled via a Servos class object, which sends commands to 
% the servos through a specified port. The rotation of the bevel gear can be 
% controlled by setting the desired velocities around the X and Y axes. 
% The current rotations and the elevation (in spherical coordinates) can be obtained.
%
% The BevelGear class uses the concept of a "zero position", which is the
% position of the gear when it is not rotated. This position needs to be
% set before any other operation is performed on the gear.
%
% Note: the two servos must be calibrated to work in unison for the bevel gear
% to operate correctly. The class does not include methods to calibrate the 
% servos. The servos should be calibrated externally before using them with 
% this class.
%
% Example:
% 1. Initialize bevel gear with servos on port 'COM3': 
%       gear = BevelGear('COM3');
% 2. Set the zero position to the current position of the gear: 
%       gear.setZeroPositionToCurrentPosition();
% 3. Set the velocity around the Y axis (in rev/min): 
%       gear.setVelocityAroundY(5);
% 4. Get the current rotation around the X axis: 
%       [rotx, success] = gear.getRotationAroundX();
%

    properties (Access = public)
        ServosObject = [];
        composingServos = [];
        
        % The Angles of the composing servos that correspond with a
        % non-rotated bevel gear (in the zero position). Need to be set
        % first using the setZeroPositionToCurrentPosition method.
        zeroAngleFirstComposingServo = -inf;
        zeroAngleSecondComposingServo = -inf;

        % The velocity of the bevel gear around the fixed y - axis (joint
        % 1).
        velocityAroundX = 0
        % The velocity of the bevel gear around the dependent x - axis
        % (joint 2).
        velocityAroundY = 0

    end

    methods (Access = public)

        function obj = BevelGear(PORT)
               % Check if the constructor was called with a specified Port
              if nargin == 0
                  % If not, use 'COM3' as a default port
                  PORT = 'COM3';
              end
            % Initialize the Servos Object with the given Port
            obj.ServosObject = Servos(PORT);
            obj.setComposingServos(4,3); 
        end
        
        function obj = setComposingServos(obj,ID1, ID2) 
            % Configure which servos compose the bevel gear. Order matters.
            obj.ServosObject.checkIdAvailable(ID1);
            obj.ServosObject.checkIdAvailable(ID2);
            obj.composingServos = [ID1, ID2];
            fprintf("Composing Servos ID : %d %d \n", ID1, ID2);
        end

        function torqueEnableDisable(obj,enable_bool)
            % Enable / Disable the torque of the servos composing the bevel
            % gear.
            obj.ServosObject.torqueEnableDisable(obj.composingServos(1),enable_bool);
            obj.ServosObject.torqueEnableDisable(obj.composingServos(2),enable_bool);
        end

        function success = setZeroPositionToCurrentPosition(obj)
            % Configure the non-rotated (zero) position of the bevel gear
            % to the current physical position.
            
            [zeroAngleFirstServo, successOne] = obj.ServosObject.getAngle(obj.composingServos(1));
            [zeroAngleSecondServo, successTwo] = obj.ServosObject.getAngle(obj.composingServos(2));
            
            if successOne && successTwo
                success = 1;
                obj.zeroAngleFirstComposingServo  = zeroAngleFirstServo;
                obj.zeroAngleSecondComposingServo = zeroAngleSecondServo;
                fprintf("Successfully set Zero Position of Bevel Gear \n");
                return
            else
                success = 0;
                fprintf("Could not set Zero Position of Bevel Gear  \n");
                return
            end
       
        end

        function [roty, success] = getRotationAroundY(obj)
            %Get the rotation of the bevel gear around the fied y-axis
            %(joint 1)

            % Check if the zero position is set.
            if obj.zeroAngleFirstComposingServo == -inf || obj.zeroAngleSecondComposingServo == -inf
                fprintf("Failed getting the Rotation around X: Set the zero Position first! \n")
                success = 0;
                return
            end
            
            % Get the current angles of the composing servos
            angleFirstComposingServo = obj.ServosObject.getAngle(obj.composingServos(1));
            angleSecondComposingServo = obj.ServosObject.getAngle(obj.composingServos(2));

            % Calculate the difference of the current angles to the zero
            % position.
            firstServoRotation = obj.zeroAngleFirstComposingServo - angleFirstComposingServo;
            secondServoRotation = obj.zeroAngleSecondComposingServo - angleSecondComposingServo;

            % Calculate the rotation around y as the difference between the
            % composing servo rotations multiplied with the bevelFactor
            bevelFactor = 1/5;
            roty = (firstServoRotation - secondServoRotation) * bevelFactor;
            
            success = 1;
        end

        function [rotx, success] = getRotationAroundX(obj)
            %Get the rotation of the bevel gear around the dependent x-axis
            %(joint 2)

            % Check if the zero position is set.
            if obj.zeroAngleFirstComposingServo == -inf || obj.zeroAngleSecondComposingServo == -inf
                fprintf("Failed getting the Rotation around X: Set the zero Position first! \n")
                success = 0;
                return
            end

            % Get the current angles of the composing servos
            angleFirstComposingServo = obj.ServosObject.getAngle(obj.composingServos(1));
            angleSecondComposingServo = obj.ServosObject.getAngle(obj.composingServos(2));

             % Calculate the difference of the current angles to the zero
            % position.
            firstServoRotation = obj.zeroAngleFirstComposingServo - angleFirstComposingServo;
            secondServoRotation = obj.zeroAngleSecondComposingServo - angleSecondComposingServo;

            % Calculate the rotation around y as the negative sum between the
            % composing servo rotations multiplied with the bevelFactor
            bevelFactor = 1/5;
            rotx = -(secondServoRotation + firstServoRotation ) * bevelFactor;
            
            success = 1;
        end

        function success = setVelocityAroundY(obj,velocity)
            % Set Velocity around the fixed Y axis (Joint 1) in rev/min

            % Store the new velocity around y
            obj.velocityAroundY = velocity;

            % Set the servo velocities according to the desired velocities
            % around y and x
            successLevel = 0;
            successLevel = successLevel + obj.ServosObject.setVelocity(obj.composingServos(1), -velocity+obj.velocityAroundX);
            successLevel = successLevel + obj.ServosObject.setVelocity(obj.composingServos(2), velocity+obj.velocityAroundX);

            % Check if velocities where set correctly
            if successLevel ~= 2
                obj.ServosObject.setVelocity(obj.composingServos(1), 0);
                obj.ServosObject.setVelocity(obj.composingServos(2), 0);
                fprintf("Failed  Setting Bevel Gear Rotation \n");
                success = 0;
            else
                % fprintf("Bevel Gear Rotation Set Successfully \n");
                success = 1;
            end

        end

        function success = setVelocityAroundX(obj,velocity)
            % Set Velocity around the dependent X axis (Joint 2) in rev/min

            % Store the new velocity around x
            obj.velocityAroundX = velocity;

            % Set the servo velocities according to the desired velocities
            % around y and x
            successLevel = 0;
            successLevel = successLevel + obj.ServosObject.setVelocity(obj.composingServos(1), velocity-obj.velocityAroundY);
            successLevel = successLevel + obj.ServosObject.setVelocity(obj.composingServos(2), velocity+obj.velocityAroundY);
  
            % Check if velocities where set correctly
            if successLevel ~= 2
                obj.ServosObject.setVelocity(obj.composingServos(1), 0);
                obj.ServosObject.setVelocity(obj.composingServos(2), 0);
                fprintf("Failed Setting Bevel Gear Rotation \n");
                success = 0;
            else
                % fprintf("Bevel Gear Rotation Set Successfully \n");
                success = 1;
            end

        end

        function [elevation, success] = getElevation(obj)
            % Calculate the elevation of the bevel gear in spherical
            % coordinates in RAD.
            % Elevation limit should be 50° (tilted) - 90° (upright)
            
            % Get the current axis rotations
            [rotx, successX] = obj.getRotationAroundX();
            [roty, successY] = obj.getRotationAroundY();

            if successX && successY 
                success = 1;
            else
                success = 0;
                fprintf("Failed getting the elevation of the bevel gear.")
            end

            % Calculate the elevation using a trignometric formula
            elevation = pi/2 - acos(cos(rotx)*cos(roty));

        end    
    end
end

