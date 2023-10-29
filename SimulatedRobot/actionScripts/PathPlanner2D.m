classdef PathPlanner2D < handle
    properties
        robot         % Instance of SimulatedRobot
        height        % Given height z for the 2D slice
        pathX         % X coordinates of the drawn path
        pathY         % Y coordinates of the drawn path
        pathZ         % Z coordinate of the drawn path
        segments      % Segments of the workspace at the given height z
    end
    
    methods
        function obj = PathPlanner2D(simRobot, z)
            % Constructor
            obj.robot = simRobot;
            obj.height = z;
            obj.segments = obj.calculateWorkspaceSlice();
        end
        
        function [x, y, z] = getPath(obj)
            % Returns the X, Y, and Z coordinates of the drawn path
            if isempty(obj.pathX) || isempty(obj.pathY) || isempty(obj.pathZ)
                warning('Path has not been defined. Run drawPath first.');
                x = [];
                y = [];
                z = [];
                return;
            end
            x = obj.pathX;
            y = obj.pathY;
            z = obj.pathZ;
        end

        function drawPath(obj)
            % Open a new figure window with grid on and specified limits
            figure;

            fprintf('Draw the path using the mouse with left-click.\nFinish the drawing by pressing the "Enter" key.\n\n');


            % Determine axis limits based on the workspace slice
            allXs = cell2mat(cellfun(@(seg) seg(:, 1), obj.segments, 'UniformOutput', false));
            allYs = cell2mat(cellfun(@(seg) seg(:, 2), obj.segments, 'UniformOutput', false));
    
            margin = 100;
            minX = min(min(allXs)) - margin;
            maxX = max(max(allXs)) + margin;
            minY = min(min(allYs)) - margin;
            maxY = max(max(allYs)) + margin;

            axis([minX maxX minY maxY]);


            grid on;
            hold on;
            xlabel('X');
            ylabel('Y');
            title('Trajectory in 2D Plane');
            
            % Plot the workspace segments
            for i = 1:length(obj.segments)
                seg = obj.segments{i};
                plot(seg(:, 1), seg(:, 2), 'g-', 'LineWidth', 2);
            end

            % Initialize empty arrays to hold x and y coordinates
            obj.pathX = [];
            obj.pathY = [];

            % Use a while loop to continuously capture points until 'Enter' is pressed
            while true
                % Use ginput to capture a single point with a timeout of infinity
                [xi, yi, button] = ginput(1);

                % Break the loop if the 'Enter' key (which returns an empty button) is pressed
                if isempty(button)
                    break;
                end
                
                % Add the point to the path arrays
                obj.pathX = [obj.pathX; xi];
                obj.pathY = [obj.pathY; yi];
                obj.pathZ = [obj.pathZ; obj.height];
                
                % Plot the point immediately
                plot(xi, yi, 'o', 'MarkerSize', 6, 'MarkerFaceColor', 'r');
                
                % If there's more than one point, plot a line segment connecting the last two points
                if length(obj.pathX) > 1
                    plot(obj.pathX(end-1:end), obj.pathY(end-1:end), 'b-');
                end
            end

            hold off;
        end
        
    end

    methods (Access = private)
        function segments = calculateWorkspaceSlice(obj)
            % Calculate the intersection of the robot's workspace with the plane defined by z

            segments = {};
            for i = 1:size(obj.robot.boundaryK, 1)
                tri = obj.robot.boundaryVertices(obj.robot.boundaryK(i, :), :);  % Extract triangle vertices
                
                % Check if the triangle intersects the plane
                isAbove = tri(:, 3) > obj.height;
                isBelow = tri(:, 3) < obj.height;
                if any(isAbove) && any(isBelow)  % At least one vertex is above and one is below
                    % Triangle intersects the plane
                    
                    % Compute the two intersection points with the plane
                    intersectedLine = obj.computeIntersection(tri);
                    if ~isempty(intersectedLine)
                        segments{end+1} = intersectedLine;
                    end
                end
            end
            
            if isempty(segments)
                warning('The specified plane does not intersect the workspace.');
            end
        end
        
        function intersectedLine = computeIntersection(obj, tri)
            % Computes the intersection lines of a triangle with a plane at height z
            intersectedPts = [];
            
            for i = 1:3
                p1 = tri(i, :);
                p2 = tri(mod(i, 3) + 1, :);  % next vertex in the triangle
                
                if (p1(3) < obj.height && p2(3) > obj.height) || (p1(3) > obj.height && p2(3) < obj.height)
                    alpha = (obj.height - p1(3)) / (p2(3) - p1(3));  % Compute intersection ratio
                    intersectionPt = p1 + alpha * (p2 - p1);  % Intersection point in 3D
                    intersectedPts = [intersectedPts; intersectionPt(1:2)];  % Store only X and Y
                end
            end
            if size(intersectedPts, 1) == 2
                intersectedLine = intersectedPts;
            else
                intersectedLine = [];
            end
        end
    end
end
