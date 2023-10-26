% This script shows an example of the roboticArm in the kinematic
% simulation. The arm moves to a non singularity position and then tries to
% reach a desired endeffector position given in global coordinates. It uses
% inverse kinematics together with a PID controller.
clear()
clc
close

addpath('C:\Users\samue\Documents\Git\Robotic-Arm-Prototype\SimulatedRobot')

%% Setup simulated robot
sr = SimulatedRobot();

%% Initialize variables for visualization
epsilon = 5;  % Radius for the scatter plot of the goal position

% Desired position
x_desired =  [-400, -400, 300]';

%% Set the robot to a non-singularity position
sr.setQ([0.3; 0.3; 0.5; 0.5])
sr.draw(0)

% Plot the desired goal position
scatter3(x_desired(1), x_desired(2), x_desired(3), (epsilon^2) * pi, 'm', 'filled');

%% Use inverse kinematics to reach a goal position (Endeffector Position Control with PID)
% PID gains
Kp = 3;
Ki = 0;
Kd = 0;

% Initialize error and integral terms
error_integral = zeros(3,1);
error_prev = zeros(3,1);

max_timesteps = 10000;
tcp_positions = zeros(3,max_timesteps);
outerTic = tic;
dt = 0.01;
for timesteps = 1:max_timesteps

    % Get current end-effector position
    x_current = SimulatedRobot.forwardKinematicsNumeric(sr.getQ);
    tcp_positions(:,timesteps) = x_current;

    % Compute error
    error = x_desired - x_current;
    
    % Compute integral and derivative of error
    error_integral = error_integral + error;
    error_derivative = error - error_prev;
    
    % Compute control input (PID)
    u = Kp * error + Ki * error_integral + Kd * error_derivative;
    
    % Compute the Jacobian for the current robot configuration
    J = SimulatedRobot.getJacobianNumeric(sr.getQ);

    % Compute joint velocities
    pinvJ = pinv(J);

    % Alternatively to the singulartiy check:
    % Filter out impossible workspace velocities using J * J'
    u = (J * J')/norm(J * J') * u;
    q_dot = pinvJ * u;

    % Check for singularity
    % if norm(J)*norm(pinvJ) > 25
    %     disp('Warning: Close to singularity');
    %     break
    % end
    
    % Update joint angles based on computed joint velocities
    q = sr.getQ;
    sr.setQ(q + q_dot*dt)
    
    % Update previous error
    error_prev = error;
        
    % Print the distance to the goal
    distance_to_goal = norm(error);
    fprintf('Distance to goal: %.0f mm \n', distance_to_goal);
    
    % Display the robot
    plot3(tcp_positions(1,1:timesteps), tcp_positions(2,1:timesteps), tcp_positions(3,1:timesteps), 'k');
    sr.draw(0);
    sr.frames(end).draw;
    drawnow limitrate
    
    % Break condition: stop if error is small
    if norm(error) < 5 %mm
        break;
    end

    % Wait if too fast
    if toc(outerTic) < timesteps*dt
        pause(timesteps*dt-toc(outerTic))
    end

end

