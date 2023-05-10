classdef BevelGear < handle
    
    properties
        ServosObject = [];
        composingServos = [];
    end
    
    methods
        function obj = BevelGear(PORT)
            obj.ServosObject = Servos(PORT);
        end
        
        function obj = setComposingServos(obj,ID1, ID2) 
            obj.ServosObject.checkIdAvailable(ID1);
            obj.ServosObject.checkIdAvailable(ID2);
            obj.composingServos = [ID1, ID2]
        end

        function torqueEnableDisable(obj,enable)

            obj.ServosObject.torqueEnableDisable(obj.composingServos(1),enable);
            obj.ServosObject.torqueEnableDisable(obj.composingServos(2),enable);
        end

        function pointStraightUp(obj)
            %Torque must be enabled

            obj.ServosObject.setAngle(obj.composingServos(1),pi);
            obj.ServosObject.setAngle(obj.composingServos(2),pi);
        end

        function tiltRightLeft(obj, angle)
            angle = max(min(angle,2),-2);
            obj.ServosObject.setAngle(obj.composingServos(1),pi + angle);
            obj.ServosObject.setAngle(obj.composingServos(2),pi - angle);
        end

        function tiltFrontBack(obj, angle)
            angle = max(min(angle,2),-2);
            obj.ServosObject.setAngle(obj.composingServos(1),pi + angle);
            obj.ServosObject.setAngle(obj.composingServos(2),pi + angle);
        end
    end
end

