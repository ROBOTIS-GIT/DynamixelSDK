clear()
clc
close

addpath('C:\Users\samue\Documents\Git\Robotic-Arm-Prototype\simulatedRobot')

%% Setup simulated robot
simulatedRobot = SimulatedRobot();


%% Initialize variables for visualization
ref_positions_array = [];  % Array to store end-effector positions


%% Move the robot to a non-singularity position
disp("Moving the robot to a non-singularity position.")
for i = 1:50

    simulatedRobot.joints(1).rotate(0.006);
    simulatedRobot.joints(2).rotate(0.006);
    simulatedRobot.joints(3).rotate(0.01);
    simulatedRobot.joints(4).rotate(0.01);

    
    
    % Get current end-effector position for trajectory visualization
    x_current = simulatedRobot.forwardKinematicsNumeric;
    % ref_positions_array = [ref_positions_array, x_current];
    
    % Display the robot
    simulatedRobot.display(0);
    
    % Plot the trajectory of the end-effector
    % plot3(ref_positions_array(1,:), ref_positions_array(2,:), ref_positions_array(3,:), 'k');
    

    drawnow;
end
disp("Starting control loop.")

% Number of points
n_points = 10;
% Radius of the circle in mm
radius = 150;  
% Center of the circle
center = [-radius/2, -radius/2, 500];

% Create an array of angular positions
theta = linspace(0, 2*pi, n_points);

% Use the parametric equations of a circle
x = center(1) + radius * cos(theta);
y = center(2) + radius * sin(theta);
z = center(3) * ones(size(theta));

% Create the positions array
positions_array = [x; y; z];

%% Use inverse kinematics to reach a goal position (Endeffector Position Control with PID)
% PID gains
Kp = 8;
Ki = 0;
Kd = 0.1;

% Display parameters
epsilon = 5; %mm

% Initialize error and integral terms
error_integral = zeros(3,1);
error_prev = zeros(3,1);
dt = 0.02;

distance_to_origin_array = [];

for k = 1:1
    for i = 1:size(positions_array, 2)
        plot_once = 1;
    
        x_desired = positions_array(:, i);
    
        while 1
            % Get current end-effector position
            x_current = simulatedRobot.forwardKinematicsNumeric;
        
        
            % Compute error
            error = x_desired - x_current;
            
            % Compute integral and derivative of error
            error_integral = error_integral + error;
            error_derivative = error - error_prev;
            
            % Compute control input (PID)
            u = Kp * error + Ki * error_integral + Kd * error_derivative;
            
            % Compute the Jacobian for the current robot configuration
            J = simulatedRobot.getJacobianNumeric();
    
            % Compute joint velocities
            pinvJ = pinv(J);
            q_dot = pinvJ * u;
        
            % Check for singularity
            if norm(J)*norm(pinvJ) > 25
                disp('Warning: Close to singularity');
                break
            end
            
            % Update joint angles based on computed joint velocities
            for j = 1:4
                angle = simulatedRobot.joints(j).angle;
                simulatedRobot.joints(j).setAngle(angle + q_dot(j)*dt);
            end
            
            % Update previous error
            error_prev = error;
            
            if i > 1 
                % Store the current end-effector position for trajectory visualization
                ref_positions_array = [ref_positions_array, x_current];
                % Plot the trajectory of the end-effector
                plot3(ref_positions_array(1,:), ref_positions_array(2,:), ref_positions_array(3,:), 'k');
            end
            
            
            % Display the robot
            simulatedRobot.display(0);
            
    
            % Calculate how far the end effector is away from [0;0;585.7100]
            distance_vec_desired = [0;0;500] - x_desired;
        
            % Plot the desired goal position
            % Goals with more then 200 mm from origin x-y are colored red
            if plot_once
                if norm(distance_vec_desired)>200 %distance_goal_origin_xy > 200
                    scatter3(x_desired(1), x_desired(2), x_desired(3), (epsilon^2) * pi, 'r', 'filled');
                else
                    scatter3(x_desired(1), x_desired(2), x_desired(3), (epsilon^2) * pi, 'g', 'filled');
                end
                plot_once = 0;
            end
    
            drawnow;
            
            % Break condition: stop if error is small
            if norm(error) < 5 %mm
                break;
            end

            % Calculate an array containing the current distance of the
            % endeffector to [0 0 500]
            if i > 1
                distance_vec = [0;0;500] - x_current;
                distance = norm(distance_vec);
                distance_to_origin_array =  [distance_to_origin_array   distance];
            end
            
            pause(dt)
    
        end
    end
end
