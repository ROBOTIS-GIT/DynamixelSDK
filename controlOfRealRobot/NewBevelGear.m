classdef NewBevelGear < handle
    
    properties (Access = private)
        ServosObject = [];
        composingServos = [];
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


        %Rotate around the X axis (Joint 1) with Velocity
        function setVelocityAroundX(obj,velocity)
            
            successLevel = 0;
            successLevel = successLevel + obj.ServosObject.setVelocity(obj.composingServos(1), velocity);
            successLevel = successLevel + obj.ServosObject.setVelocity(obj.composingServos(2), velocity);

            if successLevel ~= 2
                obj.ServosObject.setVelocity(obj.composingServos(1), 0);
                obj.ServosObject.setVelocity(obj.composingServos(2), 0);
                fprintf("Error Setting Bevel Gear Rotation \n");
            else
                fprintf("Bevel Gear Rotation Set Successfully \n");
            end

        end

        %Rotate around the Y axis (Joint 2) with Velocity
        function setVelocityAroundY(obj,velocity)
            
            successLevel = 0;
            successLevel = successLevel + obj.ServosObject.setVelocity(obj.composingServos(1), -velocity);
            successLevel = successLevel + obj.ServosObject.setVelocity(obj.composingServos(2), velocity);

            if successLevel ~= 2
                obj.ServosObject.setVelocity(obj.composingServos(1), 0);
                obj.ServosObject.setVelocity(obj.composingServos(2), 0);
                fprintf("Error Setting Bevel Gear Rotation \n");
            else
                fprintf("Bevel Gear Rotation Set Successfully \n");
            end

        end



     
    end
end

