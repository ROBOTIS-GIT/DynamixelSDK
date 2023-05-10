classdef BevelGear < handle
    
    properties
        ServosObject = [];
        composingServos = [];
        rotationAroundY = 0;
        rotationAroundX = 0;
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
            obj.ServosObject.setAngle(obj.composingServos(1),pi);
            obj.ServosObject.setAngle(obj.composingServos(2),pi);
            obj.rotationAroundY = 0;
            obj.rotationAroundX = 0;
        end

        %Tilt back front
        function rotateAroundX(obj, angle)


            angle = min(max(angle,-pi/2 + 0.01),pi/2 - 0.01);

            obj.rotationAroundX = angle;
           

            obj.ServosObject.setAngle(obj.composingServos(1), pi - angle - obj.rotationAroundY);
            obj.ServosObject.setAngle(obj.composingServos(2), pi - angle + obj.rotationAroundY);
           
        end

        %Tilt right left
        function rotateAroundY(obj, angle)

            angle = min(max(angle,-pi/2 + 0.01),pi/2 - 0.01);

            obj.rotationAroundY = angle;

            obj.ServosObject.setAngle(obj.composingServos(1), pi - angle);
            obj.ServosObject.setAngle(obj.composingServos(2), pi + angle);
           
        end

    end
        methods(Static)

            function [roty, rotx] = getRotations(vec)
                %The goal is to find a function that calculates the euler rotation angles around y
                %and subsequent rotation around x' from a rotated vector vec.
                %The unrotated original vector is assumed to be [0,0,1]
                %pointing straight up with sceme [x,y,z]. 
    
                %E.g. getRotations([-1,0,0]) should give [pi/2, 0];
                %     getRotations([0, 1,0]) should give [0, pi/2];
                %     getRotations([0, 0, 1]) should give [0,0];
            
            
            end
    end
end

