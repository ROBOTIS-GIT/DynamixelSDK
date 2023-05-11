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

        %Set azimuth and elevation in RAD
        function setAzimuthAndElevation(obj,azimuth,elevation)

            %Elevation must be between pi/4 and pi/2 (zenit)
            elevation = min(max(elevation, pi/4), pi/2);

            azimuth_deg = rad2deg(azimuth);
            elevation_deg = rad2deg(elevation);
            
            v = BevelGear.getVectorFromAzimuthAndElevation(azimuth_deg, elevation_deg);


            [rot_y,rot_x] = BevelGear.solveForRotation([0,0,1],v);

            obj.rotateAroundY(rot_y);
            obj.rotateAroundX(rot_x);

        end

    end
        methods(Static)

            

            %% These functions all use DEG
            function v = rotateYX(u,rot_y,rot_x)

                v = rotx(-rot_x)*roty(-rot_y)*u';
                
            end

            function [rot_y,rot_x] = solveForRotation(u,v)
                % Define the error function to minimize
                errorFunc = @(rot) norm(v - rotx(-rot(2))*roty(-rot(1))*u');
                
                % Initial guess
                rot_init = [0 0];
                
                % Use fminsearch to find the rotations that minimize the error
                rot = fminsearch(errorFunc, rot_init);
                
                % Return the rotations
                rot_y = rot(1);
                rot_x = rot(2);
            end

            function v = getVectorFromAzimuthAndElevation(azimuth_deg, elevation_deg)
                % Convert degrees to radians
                azimuth = deg2rad(azimuth_deg);
                elevation = deg2rad(elevation_deg);
                
                % Convert Spherical coordinates (r, azimuth, elevation) to Cartesian coordinates (x, y, z)
                % Given that r = 1 for a unit vector
                x = cos(azimuth) * cos(elevation);
                y = sin(azimuth) * cos(elevation);
                z = sin(elevation);
                
                % Assemble the result vector
                v = [x; y; z];
            end
            %%



    end
end

