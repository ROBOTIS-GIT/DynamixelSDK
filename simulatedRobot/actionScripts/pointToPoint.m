% This script shows an example of the roboticArm in the kinematic
% simulation. The arm moves to a non singularity position and then tries to
% reach a desired endeffector position given in global coordinates. It uses
% inverse kinematics together with a PID controller.
clear()
clc
close

addpath('C:\Users\samue\Documents\Git\Robotic-Arm-Prototype\simulatedRobot')

%% Setup simulated robot
simulatedRobot = SimulatedRobot();
tcp_positions = [];  % Array to store end-effector trajectory

%% Initialize variables for visualization
epsilon = 5;  % Radius for the scatter plot of the goal position

% Desired position
x_desired =  [-400, -400, 300]';

%% Move the robot to a non-singularity position
simulatedRobot.moveInitPos(0);
simulatedRobot.display(0)
% Plot the desired goal position
scatter3(x_desired(1), x_desired(2), x_desired(3), (epsilon^2) * pi, 'g', 'filled');

%% Use inverse kinematics to reach a goal position (Endeffector Position Control with PID)
% PID gains
Kp = 1;
Ki = 0.01;
Kd = 0.1;

% Initialize error and integral terms
error_integral = zeros(3,1);
error_prev = zeros(3,1);

outerTic = tic;
dt = 0.01;
timesteps = 0;
while 1
    timesteps = timesteps +1;
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
    q = simulatedRobot.getQ;
    simulatedRobot.setQ(q + q_dot*dt)
    
    % Update previous error
    error_prev = error;
    
    % Store the current end-effector position for trajectory visualization
    tcp_positions = [tcp_positions, x_current];
    
    % Print the distance to the goal
    distance_to_goal = norm(error);
    fprintf('Distance to goal: %.0f mm \n', distance_to_goal);
    
    % Display the robot
    simulatedRobot.display(0);
    
    % Plot the trajectory of the end-effector
    plot3(tcp_positions(1,:), tcp_positions(2,:), tcp_positions(3,:), 'k');
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

