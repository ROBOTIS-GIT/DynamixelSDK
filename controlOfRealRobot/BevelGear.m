classdef BevelGear < handle
    
    properties (Access = private)
        ServosObject = [];
        composingServos = [];
        rotationAroundY = 0;
        rotationAroundX = 0;
    end

    methods (Access = public)

        function obj = BevelGear(PORT)
            obj.ServosObject = Servos(PORT);
        end

        function delete(obj)

            obj.pointStraightUp();

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

        function setAzimuthAndElevation(obj,azimuth,elevation)
             %Set azimuth and elevation in RAD


            %Elevation must be between pi/5 and pi/2 (zenit)
            elevation_limited = min(max(elevation, pi/5), pi/2);

            if(elevation_limited ~= elevation)
                fprintf("Elevation has been limited to: %.2f \n",rad2deg(elevation_limited));
                elevation = elevation_limited;
            end


            azimuth_deg = rad2deg(azimuth);
            elevation_deg = rad2deg(elevation);
            
            v = BevelGear.getVectorFromAzimuthAndElevation(azimuth_deg, elevation_deg);


            [rot_y,rot_x] = BevelGear.solveForRotation([0,0,1],v);

            obj.rotateAroundY(deg2rad(rot_y));
            obj.rotateAroundX(deg2rad(rot_x));

        end
        
        function setVector(obj, v)
            %Set the gear like a vector. Pointing straight up for v =
            %[0,0,1]


           [azimuth_deg, elevation_deg] = BevelGear.getAzimuthAndElevationFromVector(v);

           elevation = deg2rad(elevation_deg);

           elevation_limited = min(max(elevation, pi/5), pi/2);
            if(elevation_limited ~= elevation)
                fprintf("Elevation has been limited to: %.2f \n",rad2deg(elevation_limited));
                elevation = elevation_limited;
            end


            elevation_deg = rad2deg(elevation);


            v = BevelGear.getVectorFromAzimuthAndElevation(azimuth_deg, elevation_deg);


            [rot_y,rot_x] = BevelGear.solveForRotation([0,0,1],v);

            obj.rotateAroundY(deg2rad(rot_y));
            obj.rotateAroundX(deg2rad(rot_x));


        end

    end
    

    methods (Access = private)

        %Tilt back front
        function rotateAroundX(obj, angle)

            obj.rotationAroundX = angle;
           
            obj.ServosObject.setAngle(obj.composingServos(1), pi - 2*( angle - obj.rotationAroundY));
            obj.ServosObject.setAngle(obj.composingServos(2), pi - 2*( angle + obj.rotationAroundY));
           
        end

        %Tilt right left
        function rotateAroundY(obj, angle)


            obj.rotationAroundY = angle;

            obj.ServosObject.setAngle(obj.composingServos(1), pi - 2*angle);
            obj.ServosObject.setAngle(obj.composingServos(2), pi + 2*angle);
           
        end

    end

    methods(Static, Access = private)
            
            %% These functions all use DEG

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

            function [azimuth_deg, elevation_deg] = getAzimuthAndElevationFromVector(v)
                % Normalize the vector
                v = v / norm(v);
                
                % Get the coordinates
                x = -v(1);
                y = v(2);
                z = v(3);
                
                % Calculate the azimuth and elevation in radians
                azimuth = atan2(y, x);
                elevation = asin(z);
            
                % Convert to degrees
                azimuth_deg = rad2deg(azimuth);
                elevation_deg = rad2deg(elevation);
            end

            %%

    end
end

