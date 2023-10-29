% drawPath.m

% Initialize the robot
robot = SimulatedRobot();

resolution = 0.1;
joint_limits = [-pi/4, pi/4; 
                -pi/4, pi/4; 
                -pi, pi; 
                -(5/6)*pi, (5/6)*pi];

% Create the robot instance and visualize its 3D boundary
robot.calculateWorkspace(resolution, joint_limits);

% Get segments at the desired z-slice
z = 100; % for example
segments = calculateWorkspaceSlice(robot, z);

% Open a new figure window with grid on and specified limits
figure;
axis([-500 500 -500 500]);
grid on;
hold on;
xlabel('X');
ylabel('Y');
title('Trajectory in 2D Plane');

% Plot the workspace segments
for i = 1:length(segments)
    seg = segments{i};
    plot(seg(:, 1), seg(:, 2), 'g-', 'LineWidth', 2);
end


% Initialize empty arrays to hold x and y coordinates
x = [];
y = [];

% Use a while loop to continuously capture points until 'Enter' is pressed
while true
    % Use ginput to capture a single point with a timeout of infinity
    [xi, yi, button] = ginput(1);

    % Break the loop if the 'Enter' key (which returns an empty button) is pressed
    if isempty(button)
        break;
    end

    % Check if the point is inside the workspace slice
    isInWorkspace = inpolygon(xi, yi, sliceX, sliceY);
    if isInWorkspace
        markerColor = 'r';
    else
        markerColor = 'm';  % Magenta color for points outside workspace
    end

    % Add the point to the x and y arrays
    x = [x; xi];
    y = [y; yi];

    % Plot the point immediately
    plot(xi, yi, 'o', 'MarkerSize', 6, 'MarkerFaceColor', markerColor);

    % If there's more than one point, plot a line segment connecting the last two points
    if length(x) > 1
        plot(x(end-1:end), y(end-1:end), 'b-');
    end
end



hold off;



function [segments] = calculateWorkspaceSlice(robot, z)
    % Calculate the intersection of the robot's workspace with the plane defined by z

    segments = {};
    for i = 1:size(robot.boundaryK, 1)
        tri = robot.boundaryVertices(robot.boundaryK(i, :), :);  % Extract triangle vertices
        
        % Check if the triangle intersects the plane
        isAbove = tri(:, 3) > z;
        isBelow = tri(:, 3) < z;
        if any(isAbove) && any(isBelow)  % At least one vertex is above and one is below
            % Triangle intersects the plane
            
            % Compute the two intersection points with the plane
            intersectedLine = computeIntersection(tri, z);
            if ~isempty(intersectedLine)
                segments{end+1} = intersectedLine;
            end
        end
    end

    if isempty(segments)
        warning('The specified plane does not intersect the workspace.');
    end
end

function intersectedLine = computeIntersection(tri, z)
    % Computes the intersection lines of a triangle with a plane at height z
    intersectedPts = [];
    
    for i = 1:3
        p1 = tri(i, :);
        p2 = tri(mod(i, 3) + 1, :);  % next vertex in the triangle
        
        if (p1(3) < z && p2(3) > z) || (p1(3) > z && p2(3) < z)  % Line segment intersects
            alpha = (z - p1(3)) / (p2(3) - p1(3));  % Compute intersection ratio
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

