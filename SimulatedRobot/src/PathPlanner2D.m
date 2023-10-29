classdef PathPlanner2D < handle
    properties (Access = private)
        robot         % Instance of SimulatedRobot
        height        % Given height z for the 2D slice
        pathX         % X coordinates of the drawn path
        pathY         % Y coordinates of the drawn path
        pathZ         % Z coordinate of the drawn path
        segments      % Segments of the workspace at the given height z
        hFig
    end
    
    methods
        function obj = PathPlanner2D(simRobot, z)
            % Constructor
            obj.robot = simRobot;
            obj.height = z;
            obj.segments = obj.calculateWorkspaceSlice();
        end
        
        function waypoint_list = getPath(obj)
            % Returns the X, Y, and Z coordinates of the drawn path
            if isempty(obj.pathX) || isempty(obj.pathY) || isempty(obj.pathZ)
                warning('Path has not been defined. Run drawPath first.');
                waypoint_list = [];
                return;
            end
            x = obj.pathX';
            y = obj.pathY';
            z = obj.pathZ';

            waypoint_list = [x;y; z];


        end

        function drawPath(obj)
            % Open a new figure window with grid on and specified limits
            obj.hFig = figure;

            fprintf('Draw the path using the mouse with left-click.\nFinish the drawing by right-clicking.\n\n');


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

             % Get current robot position in the XY-plane
            currentPosition = obj.robot.forwardKinematicsNumeric(obj.robot.getQ);
            currentX = currentPosition(1);
            currentY = currentPosition(2);
        
            % Set the starting point of the path to the current robot position
            obj.pathX = [currentX];
            obj.pathY = [currentY];
            obj.pathZ = [obj.height];
        
            % Plot the starting point on the figure
            plot(currentX, currentY, 'o', 'MarkerSize', 6, 'MarkerFaceColor', 'r');
        
            % Use a while loop to continuously capture points until right-click
            while true
                [xi, yi, button] = ginput(1);
            
                % If the user pressed right-click without having set any points, display a warning.
                if button == 3 && length(obj.pathX) < 2
                    disp("Create at least one waypoint using leftclick!");
                    continue;  % Skip the rest of the loop iteration
                end
            
                % Break the loop if right-clicked (button value is 3 for right-click)
                if button == 3
                    break;
                end
            
                % Add the point to the path arrays
                if button == 1 && (isempty(obj.pathX) || (xi ~= obj.pathX(end) || yi ~= obj.pathY(end)))  % Check if left-click and not a duplicate
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
            end
            close(obj.hFig);
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
