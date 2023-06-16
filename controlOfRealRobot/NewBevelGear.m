classdef NewBevelGear < handle
    
    properties (Access = public)
        ServosObject = [];
        composingServos = [];
        angleAroundY = 0;
        angleAroundX = 0;
        zeroAngleFirstComposingServo = -inf;
        zeroAngleSecondComposingServo = -inf;
    end

    methods (Access = public)

        function obj = NewBevelGear(PORT)
               % Check if the function was called with an argument
              if nargin == 0
                  % If not, use 'COM3' as a default port
                  PORT = 'COM3';
              end
            obj.ServosObject = Servos(PORT);
            obj.setComposingServos(3,4); %Default Bevel Gears
        end

        function delete(obj)

            
        end
        
        function obj = setComposingServos(obj,ID1, ID2) 
            obj.ServosObject.checkIdAvailable(ID1);
            obj.ServosObject.checkIdAvailable(ID2);
            obj.composingServos = [ID1, ID2];
            fprintf("Composing Servos ID : %d %d \n", ID1, ID2);
        end

        function torqueEnableDisable(obj,enable)

            obj.ServosObject.torqueEnableDisable(obj.composingServos(1),enable);
            obj.ServosObject.torqueEnableDisable(obj.composingServos(2),enable);
        end


        %Rotate around the dependent X axis (Joint 2) with Velocity
        function success = setVelocityAroundX(obj,velocity)
            
            successLevel = 0;
            successLevel = successLevel + obj.ServosObject.setVelocity(obj.composingServos(1), velocity);
            successLevel = successLevel + obj.ServosObject.setVelocity(obj.composingServos(2), velocity);

            if successLevel ~= 2
                obj.ServosObject.setVelocity(obj.composingServos(1), 0);
                obj.ServosObject.setVelocity(obj.composingServos(2), 0);
                fprintf("Error Setting Bevel Gear Rotation \n");
                success = 0;
            else
                fprintf("Bevel Gear Rotation Set Successfully \n");
                success = 1;
            end

        end

        %Rotate around the Y axis (Joint 1) with Velocity
        function success = setVelocityAroundY(obj,velocity)
            
            successLevel = 0;
            successLevel = successLevel + obj.ServosObject.setVelocity(obj.composingServos(1), -velocity);
            successLevel = successLevel + obj.ServosObject.setVelocity(obj.composingServos(2), velocity);

            if successLevel ~= 2
                obj.ServosObject.setVelocity(obj.composingServos(1), 0);
                obj.ServosObject.setVelocity(obj.composingServos(2), 0);
                fprintf("Error Setting Bevel Gear Rotation \n");
                success = 0;
            else
                fprintf("Bevel Gear Rotation Set Successfully \n");
                success = 1;
            end

        end

        %Set the zero position for BevelGear
        function success = setZeroPositionToCurrentPosition(obj)
            
            [zeroAngleFirstServo, successOne] = obj.ServosObject.getAngle(obj.composingServos(1));
            [zeroAngleSecondServo, successTwo] = obj.ServosObject.getAngle(obj.composingServos(2));
            
            if successOne && successTwo
                success = 1;
                obj.angleAroundX = 0;
                obj.angleAroundY = 0;
                obj.zeroAngleFirstComposingServo  = zeroAngleFirstServo;
                obj.zeroAngleSecondComposingServo = zeroAngleSecondServo;
                fprintf("Successfully set Zero Position \n");
                return
            else
                success = 0;
                fprintf("Could not set Zero Position \n");
                return
            end
       
        end

        %Get the rotation around Y
        function [roty, success] = getRotationAroundY(obj)

            if obj.zeroAngleFirstComposingServo == -inf || obj.zeroAngleSecondComposingServo == -inf
                fprintf("Error getting the Rotation around X: Set the zero Position first! \n")
                success = 0;
                return
            end


            angleFirstComposingServo = obj.ServosObject.getAngle(obj.composingServos(1));
            angleSecondComposingServo = obj.ServosObject.getAngle(obj.composingServos(2));

            firstServoRotation = obj.zeroAngleFirstComposingServo - angleFirstComposingServo;
            secondServoRotation = obj.zeroAngleSecondComposingServo - angleSecondComposingServo;

            bevelFactor = 1/5;
            roty = (-firstServoRotation + secondServoRotation) * bevelFactor;
            
            success = 1;
            
                
        end

        %Get the rotation around X
        function [rotx, success] = getRotationAroundX(obj)

            if obj.zeroAngleFirstComposingServo == -inf || obj.zeroAngleSecondComposingServo == -inf
                fprintf("Error getting the Rotation around X: Set the zero Position first! \n")
                success = 0;
                return
            end


            angleFirstComposingServo = obj.ServosObject.getAngle(obj.composingServos(1));
            angleSecondComposingServo = obj.ServosObject.getAngle(obj.composingServos(2));

            firstServoRotation = obj.zeroAngleFirstComposingServo - angleFirstComposingServo;
            secondServoRotation = obj.zeroAngleSecondComposingServo - angleSecondComposingServo;

            bevelFactor = 1/5;
            rotx = -(firstServoRotation + secondServoRotation) * bevelFactor;
            
            success = 1;
            
                
        end

        function [elevation, success] = getElevation(obj)

            %Elevation limit should be 50° (tilted) - 90° (upright)

            if obj.zeroAngleFirstComposingServo == -inf || obj.zeroAngleSecondComposingServo == -inf
                fprintf("Error getting the Rotation around X: Set the zero Position first! \n")
                success = 0;
                return
            end

            angleFirstComposingServo = obj.ServosObject.getAngle(obj.composingServos(1));
            angleSecondComposingServo = obj.ServosObject.getAngle(obj.composingServos(2));

            firstServoRotation = obj.zeroAngleFirstComposingServo - angleFirstComposingServo;
            secondServoRotation = obj.zeroAngleSecondComposingServo - angleSecondComposingServo;

            bevelFactor = 1/5;
            rotx = -(firstServoRotation + secondServoRotation) * bevelFactor;
            roty = (-firstServoRotation + secondServoRotation) * bevelFactor;

            success = 1;
            elevation = pi/2 - acos(cos(rotx)*cos(roty));


        end

     
    end
end

