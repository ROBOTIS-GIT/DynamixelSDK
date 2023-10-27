%This script shows an example of the roboticArm in the kinematic
% simulation. The arm moves to a non singularity position and then tries to
% reach a desired endeffector position. It shall use the Nullspace operator
% to perform additional task (orientation of the endeffector, better
% configurations, avoid singularities, etc...)

clear()
clc
close

addpath('C:\Users\samue\Documents\Git\Robotic-Arm-Prototype\SimulatedRobot')

%% Setup simulated robot
sr = SimulatedRobot();

%% Set the robot to a non-singularity position
sr.setQ([0.3; 0.3; 0.5; 0.5])
sr.draw(0)

% Desired position and allowed deviation in [mm]
x_desired =  [200, 0, 300]';
epsilon = 5;
scatter3(x_desired(1), x_desired(2), x_desired(3), (epsilon^2) * pi, 'm', 'filled');

%% Control Loop
% P  gain
Kp = 1;

max_timesteps = 10000;
tcp_positions = zeros(3,max_timesteps);
outerTic = tic;

dt = 0.01; % 100 hz
for timesteps = 1:max_timesteps

    %% Get current end-effector position, Jacobian
    x_current = SimulatedRobot.forwardKinematicsNumeric(sr.getQ);
    tcp_positions(:,timesteps) = x_current;

    % Compute the Jacobian for the current robot configuration
    J = SimulatedRobot.getJacobianNumeric(sr.getQ);    
    
    % Compute pseudo inverse
    pinvJ = pinv(J);

    %% Compute task-space velocity u 

    % Compute error
    error = x_desired - x_current;
    
    % Compute control input (P)
    u = Kp * error;

    % warn for singularity
    if norm(J)*norm(pinvJ) > 50
        disp("Warning, close to singularity")
    end

    % limit u [mm/s]
    u_max = 100;
    u = u_max * u/norm(u);

    %% Compute joint-space velocity q_dot

    % Calculate Nullspace Operator
    N = (eye(4) - pinvJ * J);
    q = sr.getQ;

    % Desire a specific z-Axis of the TCP
    z_desired = [0;1;0];
    dHdQ_z = numericDiff(@H_z_desired, q, z_desired);

    % Secure minimum shoulder elevation if possible (does nothing unless close to
    % minimum elevation)
    elevation_min = deg2rad(50);
    dHdQ_elevation = numericDiff(@H_min_shoulder_elevation, q, elevation_min);

    % Combine both goals
    weight_z = 10;
    dHdQ = weight_z*dHdQ_z + dHdQ_elevation;

    % Update joint angles based on computed joint velocities with Nullspace
    q_dot = pinvJ * u - N*dHdQ';

    % Limit joint speeds [rad/s]
    q_dot_max = [0.2;0.2;0.4;0.4];
    q_dot = limitQdot(q_dot,q_dot_max);


    %% Simulation

    % Integrate q
    q = q+q_dot*dt;
    sr.setQ(q + q_dot*dt)

    % Display the robot
    plot3(tcp_positions(1,1:timesteps), tcp_positions(2,1:timesteps), tcp_positions(3,1:timesteps), 'k');
    sr.draw(0);
    sr.frames(end).draw;
    drawnow limitrate

    % Print the distance to the goal
    % distance_to_goal = norm(error);
    % fprintf('Distance to goal: %.0f mm \n', distance_to_goal);

    % Print shoulder elevation
    elevation = SimulatedRobot.getShoulderElevation(sr.getQ);
    fprintf('Shoulder Elevation: %.2f ° \n', elevation*180/pi);

    % Wait if simulation is faster than real time
    if toc(outerTic) < timesteps*dt
        pause(timesteps*dt-toc(outerTic))
    end
end




function H = H_z_desired(q, z_desired)
    z_desired = z_desired/norm(z_desired);
    g_A_tcp = SimulatedRobot.roty(q(1)) * SimulatedRobot.rotx(q(2)) * SimulatedRobot.rotz(q(3)) * SimulatedRobot.rotx(q(4));
    z_q = g_A_tcp * [0;0;1];
    z_q = z_q/norm(z_q);
    H = 1/2 * norm(z_q-z_desired)^2; % <= 1;
    % fprintf("H z = %.2f\n", H)

end

function H = H_min_shoulder_elevation(q, elevation_min)
    elevation = SimulatedRobot.getShoulderElevation(q);
    distance_to_min_elevation = elevation-elevation_min;
    a = 20;
    H = exp(-a*distance_to_min_elevation);

    % H = 1 for elevation = min_elevation, H --> inf for elevation <<
    % min_elevation, H --> 0 for elevation >> min_elevation

    % fprintf("H elevation = %.2f\n", H)
    
end

function dHdQ = numericDiff(H, q, varargin)
    % Define a small change in joint angles
    delta_q = 0.001;

    % Initialize dHdQ
    dHdQ = zeros(1,4); % Column vector initialization

    % For each joint angle
    for i = 1:4
        % Create a copy of q for perturbation
        q_plus = q;
        q_minus = q;
        
        % Add and subtract small change to joint i
        q_plus(i) = q_plus(i) + delta_q;
        q_minus(i) = q_minus(i) - delta_q;

        % Compute H(q)
        H_plus = H(q_plus, varargin{:});
        H_minus = H(q_minus, varargin{:});

        % Compute derivative
        dHdQ(1,i) = (H_plus - H_minus) / (2 * delta_q);
    end
end

function q_dot_limited = limitQdot(q_dot, q_dot_max)
    % This function limits the absolute value of q_dot 
    % based on q_dot_max while preserving the ratio among the q_dot values.

    % Ensure q_dot and q_dot_max have the same dimensions
    if size(q_dot) ~= size(q_dot_max)
        error('q_dot and q_dot_max must have the same dimensions.');
    end

    % Calculate the ratio of each element in q_dot to its limit in q_dot_max
    ratio = abs(q_dot) ./ q_dot_max;

    % Find the maximum ratio
    max_ratio = max(ratio);

    % If the maximum ratio is greater than 1, scale down the entire q_dot vector
    if max_ratio > 1
        q_dot_limited = q_dot / max_ratio;
    else
        q_dot_limited = q_dot;
    end

end
